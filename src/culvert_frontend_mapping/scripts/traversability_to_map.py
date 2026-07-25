#!/usr/bin/env python3

import math
import warnings

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


def decode_layer(name: str, array_msg) -> np.ndarray:
    data_np = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0, label1 = dims[0].label, dims[1].label
        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size or 1
            cols = dims[1].size or (len(data_np) // rows if rows else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout.")
            return data_np.reshape((rows, cols), order="C")
        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout.")
            return data_np.reshape((rows, cols), order="F")

    if dims:
        cols = dims[0].size or 1
        rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else len(data_np))
    else:
        cols = int(math.sqrt(len(data_np))) if len(data_np) else 0
        rows = cols

    if rows * cols != data_np.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout.")
    return data_np.reshape((rows, cols), order="C")


def unwrap_layer(array: np.ndarray, outer_start_index: int, inner_start_index: int) -> np.ndarray:
    shift = (-int(outer_start_index), -int(inner_start_index))
    if shift == (0, 0):
        return array
    return np.roll(array, shift=shift, axis=(0, 1))


def is_identity_quaternion(quaternion: Quaternion, atol: float = 1e-6) -> bool:
    return (
        abs(float(quaternion.x)) <= atol
        and abs(float(quaternion.y)) <= atol
        and abs(float(quaternion.z)) <= atol
        and abs(float(quaternion.w) - 1.0) <= atol
    )


def quaternion_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
    x_value = float(quaternion.x)
    y_value = float(quaternion.y)
    z_value = float(quaternion.z)
    w_value = float(quaternion.w)

    xx = x_value * x_value
    yy = y_value * y_value
    zz = z_value * z_value
    xy = x_value * y_value
    xz = x_value * z_value
    yz = y_value * z_value
    wx = w_value * x_value
    wy = w_value * y_value
    wz = w_value * z_value

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def local_height_range_map(elevation: np.ndarray, kernel_size: int) -> np.ndarray:
    if kernel_size % 2 == 0 or kernel_size < 1:
        raise ValueError("kernel_size must be a positive odd integer.")

    pad = kernel_size // 2
    padded = np.pad(elevation, pad_width=pad, mode="constant", constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    valid_mask = np.isfinite(windows)
    valid_count = np.sum(valid_mask, axis=(-2, -1))

    local_max = np.max(np.where(valid_mask, windows, -np.inf), axis=(-2, -1))
    local_min = np.min(np.where(valid_mask, windows, np.inf), axis=(-2, -1))
    height_range = np.abs(local_max - local_min).astype(np.float32)

    center_valid = np.isfinite(elevation)
    height_range[(valid_count < 2) | (~center_valid)] = np.nan
    return height_range


def local_nan_std(values: np.ndarray, kernel_size: int) -> np.ndarray:
    if kernel_size % 2 == 0 or kernel_size < 1:
        raise ValueError("kernel_size must be a positive odd integer.")

    pad = kernel_size // 2
    padded = np.pad(values, pad_width=pad, mode="constant", constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    valid_mask = np.isfinite(windows)
    valid_count = np.sum(valid_mask, axis=(-2, -1))

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        mean = np.nanmean(windows, axis=(-2, -1))
        mean_sq = np.nanmean(np.square(windows), axis=(-2, -1))

    variance = np.maximum(0.0, mean_sq - np.square(mean))
    std = np.sqrt(variance, dtype=np.float32)
    std[valid_count < 2] = np.nan
    std[~np.isfinite(values)] = np.nan
    return std.astype(np.float32)


def normalized_gradient_cost(elevation: np.ndarray, resolution: float, slope_full_deg: float) -> np.ndarray:
    cost = np.full(elevation.shape, np.nan, dtype=np.float32)
    valid_mask = np.isfinite(elevation)
    if not np.any(valid_mask):
        return cost

    filled = np.where(valid_mask, elevation, np.nanmedian(elevation[valid_mask]))
    grad_y, grad_x = np.gradient(filled, resolution, resolution)
    slope_rad = np.arctan(np.sqrt(np.square(grad_x) + np.square(grad_y)))
    slope_deg = np.degrees(slope_rad)

    cost[valid_mask] = np.clip((slope_deg[valid_mask] / max(slope_full_deg, 1e-3)) * 100.0, 0.0, 100.0)
    return cost


def normalize_cost(feature: np.ndarray, cost_zero_at: float, cost_full_at: float) -> np.ndarray:
    if cost_full_at <= cost_zero_at:
        raise ValueError("Expected cost_full_at > cost_zero_at.")

    cost = np.full(feature.shape, np.nan, dtype=np.float32)
    valid_mask = np.isfinite(feature)
    if not np.any(valid_mask):
        return cost

    values = feature[valid_mask]
    scaled = (values - cost_zero_at) / (cost_full_at - cost_zero_at)
    cost[valid_mask] = np.clip(scaled * 100.0, 0.0, 100.0)
    return cost


def median_filter_nan(values: np.ndarray, kernel_size: int) -> np.ndarray:
    if kernel_size % 2 == 0 or kernel_size < 1:
        raise ValueError("kernel_size must be a positive odd integer.")

    pad = kernel_size // 2
    padded = np.pad(values, pad_width=pad, mode="constant", constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        filtered = np.nanmedian(windows, axis=(-2, -1)).astype(np.float32)
    filtered[~np.isfinite(values)] = np.nan
    return filtered


def is_zero_time(stamp: Time) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


class TraversabilityToMap(Node):
    def __init__(self) -> None:
        super().__init__("traversability_to_map")

        self.declare_parameter("grid_map_topic", "/fastdem/mapping/gridmap")
        self.declare_parameter("map_topic", "/traversability_map")
        self.declare_parameter("local_map_topic", "/traversability_map_local")
        self.declare_parameter("mode", "terrain_fusion")
        self.declare_parameter("layer", "elevation")
        self.declare_parameter("confidence_layer", "")
        self.declare_parameter("uncertainty_layer", "")
        self.declare_parameter("water_cost_topic", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("fallback_frame_id", "odom")
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("kernel_size", 3)
        self.declare_parameter("median_filter_size", 3)
        self.declare_parameter("min_confidence", 0.20)
        self.declare_parameter("height_cost_zero_at_m", 0.02)
        self.declare_parameter("height_cost_full_at_m", 0.12)
        self.declare_parameter("slope_cost_full_at_deg", 18.0)
        self.declare_parameter("roughness_cost_zero_at_m", 0.01)
        self.declare_parameter("roughness_cost_full_at_m", 0.06)
        self.declare_parameter("uncertainty_cost_zero_at", 0.15)
        self.declare_parameter("uncertainty_cost_full_at", 0.80)
        self.declare_parameter("water_cost_scale", 1.0)
        self.declare_parameter("weight_height", 0.35)
        self.declare_parameter("weight_slope", 0.30)
        self.declare_parameter("weight_roughness", 0.20)
        self.declare_parameter("weight_water", 0.15)
        self.declare_parameter("weight_uncertainty", 0.10)
        self.declare_parameter("accumulate_global", True)
        self.declare_parameter("robot_clear_radius", 0.45)
        self.declare_parameter("global_fusion_alpha", 0.45)

        grid_topic = str(self.get_parameter("grid_map_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        local_map_topic = str(self.get_parameter("local_map_topic").value)
        self.mode = str(self.get_parameter("mode").value)
        self.layer = str(self.get_parameter("layer").value)
        self.confidence_layer = str(self.get_parameter("confidence_layer").value)
        self.uncertainty_layer = str(self.get_parameter("uncertainty_layer").value)
        self.water_cost_topic = str(self.get_parameter("water_cost_topic").value).strip()
        self.frame_override = str(self.get_parameter("frame_id").value).strip()
        self.fallback_frame_id = str(self.get_parameter("fallback_frame_id").value).strip()
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.median_filter_size = int(self.get_parameter("median_filter_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.height_cost_zero_at_m = float(self.get_parameter("height_cost_zero_at_m").value)
        self.height_cost_full_at_m = float(self.get_parameter("height_cost_full_at_m").value)
        self.slope_cost_full_at_deg = float(self.get_parameter("slope_cost_full_at_deg").value)
        self.roughness_cost_zero_at_m = float(self.get_parameter("roughness_cost_zero_at_m").value)
        self.roughness_cost_full_at_m = float(self.get_parameter("roughness_cost_full_at_m").value)
        self.uncertainty_cost_zero_at = float(self.get_parameter("uncertainty_cost_zero_at").value)
        self.uncertainty_cost_full_at = float(self.get_parameter("uncertainty_cost_full_at").value)
        self.water_cost_scale = float(self.get_parameter("water_cost_scale").value)
        self.weight_height = float(self.get_parameter("weight_height").value)
        self.weight_slope = float(self.get_parameter("weight_slope").value)
        self.weight_roughness = float(self.get_parameter("weight_roughness").value)
        self.weight_water = float(self.get_parameter("weight_water").value)
        self.weight_uncertainty = float(self.get_parameter("weight_uncertainty").value)
        self.accumulate_global = bool(self.get_parameter("accumulate_global").value)
        self.robot_clear_radius = float(self.get_parameter("robot_clear_radius").value)
        self.global_fusion_alpha = float(self.get_parameter("global_fusion_alpha").value)

        latch_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, latch_qos)
        self.local_map_pub = self.create_publisher(OccupancyGrid, local_map_topic, latch_qos)
        self.sub = self.create_subscription(GridMap, grid_topic, self.on_grid, 10)
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.latest_grid_msg = None
        self.latest_water_map = None

        if self.water_cost_topic:
            self.water_sub = self.create_subscription(
                OccupancyGrid, self.water_cost_topic, self.on_water_map, 10
            )
        else:
            self.water_sub = None

        self._warned_missing_layer = False
        self._warned_decode = False
        self._warned_frame_override = False
        self._warned_empty_frame = False
        self._warned_empty_source_frame = False
        self._warned_zero_stamp = False
        self._warned_rotated_pose = False
        self._warned_water_projection = False

        self.global_score_map = {}
        self.global_conf_map = {}
        self.global_min_mx = None
        self.global_max_mx = None
        self.global_min_my = None
        self.global_max_my = None

        self.get_logger().info(
            f"traversability_to_map: {grid_topic} mode='{self.mode}' elevation_layer='{self.layer}' "
            f"local='{local_map_topic}' global='{map_topic}' "
            f"weights[h={self.weight_height:.2f}, s={self.weight_slope:.2f}, "
            f"r={self.weight_roughness:.2f}, w={self.weight_water:.2f}, u={self.weight_uncertainty:.2f}]"
        )

    def on_grid(self, msg: GridMap) -> None:
        self.latest_grid_msg = msg
        self.process_grid(msg)

    def on_water_map(self, msg: OccupancyGrid) -> None:
        self.latest_water_map = msg
        if self.latest_grid_msg is not None:
            self.process_grid(self.latest_grid_msg)

    def process_grid(self, msg: GridMap) -> None:
        if len(msg.layers) != len(msg.data):
            self.get_logger().warning("Received malformed GridMap message.")
            return

        if self.layer not in msg.layers:
            if not self._warned_missing_layer:
                self.get_logger().warning(
                    f"GridMap has no layer '{self.layer}'. Available: {list(msg.layers)}"
                )
                self._warned_missing_layer = True
            return
        self._warned_missing_layer = False

        try:
            layer_index = msg.layers.index(self.layer)
            elevation = decode_layer(self.layer, msg.data[layer_index])
            elevation = unwrap_layer(elevation, msg.outer_start_index, msg.inner_start_index)
        except Exception as exc:
            if not self._warned_decode:
                self.get_logger().warning(f"Failed to decode GridMap layer '{self.layer}': {exc}")
                self._warned_decode = True
            return
        self._warned_decode = False

        confidence = self.decode_optional_layer(msg, self.confidence_layer)
        uncertainty = self.decode_optional_layer(msg, self.uncertainty_layer)

        local_cost_values = self.compute_occupancy(msg, elevation, confidence, uncertainty)
        occupancy = self.build_occupancy_from_grid(msg, local_cost_values)

        local_grid = self.to_grid_matrix(local_cost_values, self.unknown_value)
        occupancy.data = local_grid.flatten(order="C").tolist()
        self.local_map_pub.publish(occupancy)

        if self.accumulate_global:
            self.integrate_into_global_map(local_grid, occupancy)
            published = self.build_global_occupancy(occupancy)
        else:
            published = occupancy

        self.map_pub.publish(published)

    def decode_optional_layer(self, msg: GridMap, layer_name: str) -> np.ndarray | None:
        if not layer_name or layer_name not in msg.layers:
            return None
        try:
            layer_index = msg.layers.index(layer_name)
            layer_values = decode_layer(layer_name, msg.data[layer_index])
            return unwrap_layer(layer_values, msg.outer_start_index, msg.inner_start_index)
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to decode GridMap layer '{layer_name}': {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def build_occupancy_from_grid(self, msg: GridMap, occupancy_values: np.ndarray) -> OccupancyGrid:
        occupancy = OccupancyGrid()
        occupancy.header.stamp = msg.header.stamp
        if is_zero_time(occupancy.header.stamp):
            occupancy.header.stamp = self.get_clock().now().to_msg()
            if not self._warned_zero_stamp:
                self.get_logger().warning(
                    "Source GridMap stamp was zero; replacing with current node time.",
                    throttle_duration_sec=5.0,
                )
                self._warned_zero_stamp = True
        else:
            self._warned_zero_stamp = False

        source_frame = msg.header.frame_id.strip()
        target_frame = self.frame_override if self.frame_override else source_frame
        if self.frame_override and self.frame_override != source_frame and not self._warned_frame_override:
            self.get_logger().warning(
                f"Overriding GridMap frame '{source_frame}' with '{self.frame_override}'.",
                throttle_duration_sec=5.0,
            )
            self._warned_frame_override = True

        if not target_frame:
            target_frame = self.fallback_frame_id
            if source_frame:
                if not self._warned_empty_frame:
                    self.get_logger().warning(
                        f"GridMap frame was empty after override handling. Falling back to '{target_frame}'.",
                        throttle_duration_sec=5.0,
                    )
                    self._warned_empty_frame = True
            else:
                if not self._warned_empty_source_frame:
                    self.get_logger().warning(
                        f"Source GridMap frame_id is empty. Falling back to '{target_frame}'.",
                        throttle_duration_sec=5.0,
                    )
                    self._warned_empty_source_frame = True
        occupancy.header.frame_id = target_frame

        occupancy.info.resolution = float(msg.info.resolution)
        occupancy.info.width = int(msg.info.length_x / msg.info.resolution)
        occupancy.info.height = int(msg.info.length_y / msg.info.resolution)

        center_x = float(msg.info.pose.position.x)
        center_y = float(msg.info.pose.position.y)
        occupancy.info.origin.position.x = center_x - 0.5 * float(msg.info.length_x)
        occupancy.info.origin.position.y = center_y - 0.5 * float(msg.info.length_y)
        occupancy.info.origin.position.z = 0.0

        quaternion = msg.info.pose.orientation
        if is_identity_quaternion(quaternion):
            occupancy.info.origin.orientation.w = 1.0
        else:
            if not self._warned_rotated_pose:
                self.get_logger().warning(
                    "GridMap pose carries a non-identity orientation. OccupancyGrid remains axis-aligned; "
                    "origin orientation is reset to identity.",
                    throttle_duration_sec=5.0,
                )
                self._warned_rotated_pose = True
            _ = quaternion_to_rotation_matrix(quaternion)
            occupancy.info.origin.orientation.w = 1.0
        return occupancy

    def compute_occupancy(
        self,
        msg: GridMap,
        elevation: np.ndarray,
        confidence: np.ndarray | None,
        uncertainty: np.ndarray | None,
    ) -> np.ndarray:
        if self.mode != "terrain_fusion":
            raise ValueError("mode must be 'terrain_fusion'.")

        resolution = float(msg.info.resolution)
        height_range = local_height_range_map(elevation, self.kernel_size)
        slope_cost = normalized_gradient_cost(elevation, resolution, self.slope_cost_full_at_deg)
        roughness = local_nan_std(elevation, self.kernel_size)

        height_cost = normalize_cost(
            height_range, self.height_cost_zero_at_m, self.height_cost_full_at_m
        )
        roughness_cost = normalize_cost(
            roughness, self.roughness_cost_zero_at_m, self.roughness_cost_full_at_m
        )

        uncertainty_cost = np.full(elevation.shape, 0.0, dtype=np.float32)
        if uncertainty is not None:
            uncertainty_cost = normalize_cost(
                uncertainty,
                self.uncertainty_cost_zero_at,
                self.uncertainty_cost_full_at,
            )
        elif confidence is not None:
            uncertainty_cost = np.full(elevation.shape, np.nan, dtype=np.float32)
            valid_conf = np.isfinite(confidence)
            uncertainty_cost[valid_conf] = np.clip((1.0 - confidence[valid_conf]) * 100.0, 0.0, 100.0)

        water_cost = self.project_water_cost(msg, elevation.shape)

        total_weight = (
            self.weight_height
            + self.weight_slope
            + self.weight_roughness
            + self.weight_water
            + self.weight_uncertainty
        )
        occupancy_values = np.full(elevation.shape, np.nan, dtype=np.float32)
        valid = np.isfinite(elevation)
        if confidence is not None:
            valid &= np.isfinite(confidence) & (confidence >= self.min_confidence)

        component_stack = [
            (height_cost, self.weight_height),
            (slope_cost, self.weight_slope),
            (roughness_cost, self.weight_roughness),
            (water_cost, self.weight_water),
            (uncertainty_cost, self.weight_uncertainty),
        ]

        if np.any(valid):
            fused = np.zeros(elevation.shape, dtype=np.float32)
            effective_weight = np.zeros(elevation.shape, dtype=np.float32)
            for component, weight in component_stack:
                if weight <= 0.0:
                    continue
                component_valid = np.isfinite(component) & valid
                fused[component_valid] += component[component_valid] * weight
                effective_weight[component_valid] += weight

            valid_fused = valid & (effective_weight > 0.0)
            occupancy_values[valid_fused] = fused[valid_fused] / effective_weight[valid_fused]

        if total_weight > 0.0 and self.weight_water > 0.0 and np.any(np.isfinite(water_cost)):
            water_only = valid & ~np.isfinite(occupancy_values) & np.isfinite(water_cost)
            occupancy_values[water_only] = np.clip(water_cost[water_only], 0.0, 100.0)

        if self.median_filter_size > 1:
            occupancy_values = median_filter_nan(occupancy_values, self.median_filter_size)
        return occupancy_values

    def project_water_cost(self, msg: GridMap, shape: tuple[int, int]) -> np.ndarray:
        water_cost = np.zeros(shape, dtype=np.float32)
        if self.latest_water_map is None:
            return water_cost

        water_msg = self.latest_water_map
        if water_msg.header.frame_id != (self.frame_override or msg.header.frame_id):
            if not self._warned_water_projection:
                self.get_logger().warning(
                    "Water cost map frame does not match traversability frame. Ignoring water map until aligned.",
                    throttle_duration_sec=5.0,
                )
                self._warned_water_projection = True
            return water_cost

        resolution = float(msg.info.resolution)
        center_x = float(msg.info.pose.position.x)
        center_y = float(msg.info.pose.position.y)
        origin_x = center_x - 0.5 * float(msg.info.length_x)
        origin_y = center_y - 0.5 * float(msg.info.length_y)

        water_origin_x = float(water_msg.info.origin.position.x)
        water_origin_y = float(water_msg.info.origin.position.y)
        water_res = float(water_msg.info.resolution)
        water_width = int(water_msg.info.width)
        water_height = int(water_msg.info.height)
        water_grid = np.asarray(water_msg.data, dtype=np.int16).reshape((water_height, water_width))

        for row in range(shape[0]):
            world_y = origin_y + (row + 0.5) * resolution
            water_row = int(math.floor((world_y - water_origin_y) / water_res))
            if water_row < 0 or water_row >= water_height:
                continue
            for col in range(shape[1]):
                world_x = origin_x + (col + 0.5) * resolution
                water_col = int(math.floor((world_x - water_origin_x) / water_res))
                if water_col < 0 or water_col >= water_width:
                    continue
                cell = int(water_grid[water_row, water_col])
                if cell < 0:
                    continue
                water_cost[row, col] = np.clip(cell * self.water_cost_scale, 0.0, 100.0)

        self._warned_water_projection = False
        return water_cost

    @staticmethod
    def to_grid_matrix(values: np.ndarray, unknown_value: int) -> np.ndarray:
        out = np.full(values.shape, unknown_value, dtype=np.int8)
        valid_mask = np.isfinite(values)
        if np.any(valid_mask):
            out[valid_mask] = np.rint(np.clip(values[valid_mask], 0.0, 100.0)).astype(np.int8)
        return out.T[::-1, ::-1]

    def integrate_into_global_map(self, local_grid: np.ndarray, occupancy: OccupancyGrid) -> None:
        resolution = float(occupancy.info.resolution)
        origin_x = float(occupancy.info.origin.position.x)
        origin_y = float(occupancy.info.origin.position.y)
        height, width = local_grid.shape

        for gy in range(height):
            world_y = origin_y + (gy + 0.5) * resolution
            my = int(math.floor(world_y / resolution))
            for gx in range(width):
                cell_value = int(local_grid[gy, gx])
                if cell_value == self.unknown_value:
                    continue
                world_x = origin_x + (gx + 0.5) * resolution
                mx = int(math.floor(world_x / resolution))
                key = (mx, my)

                if key in self.global_score_map:
                    old_score = self.global_score_map[key]
                    old_conf = self.global_conf_map[key]
                    alpha = self.global_fusion_alpha
                    self.global_score_map[key] = (1.0 - alpha) * old_score + alpha * float(cell_value)
                    self.global_conf_map[key] = min(1.0, old_conf + alpha * 0.5)
                else:
                    self.global_score_map[key] = float(cell_value)
                    self.global_conf_map[key] = 1.0

                self.update_global_bounds(mx, my)

        if self.robot_clear_radius > 0.0:
            center_x = origin_x + 0.5 * width * resolution
            center_y = origin_y + 0.5 * height * resolution
            self.clear_robot_disc(
                center_x,
                center_y,
                resolution,
                self.robot_clear_radius,
            )

    def clear_robot_disc(self, center_x: float, center_y: float, resolution: float, radius: float) -> None:
        cell_radius = max(1, int(math.ceil(radius / resolution)))
        center_mx = int(math.floor(center_x / resolution))
        center_my = int(math.floor(center_y / resolution))
        radius_sq = radius * radius

        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                wx = (center_mx + dx + 0.5) * resolution
                wy = (center_my + dy + 0.5) * resolution
                if (wx - center_x) ** 2 + (wy - center_y) ** 2 > radius_sq:
                    continue
                key = (center_mx + dx, center_my + dy)
                self.global_score_map[key] = 0.0
                self.global_conf_map[key] = 1.0
                self.update_global_bounds(center_mx + dx, center_my + dy)

    def update_global_bounds(self, mx: int, my: int) -> None:
        if self.global_min_mx is None:
            self.global_min_mx = self.global_max_mx = mx
            self.global_min_my = self.global_max_my = my
            return
        self.global_min_mx = min(self.global_min_mx, mx)
        self.global_max_mx = max(self.global_max_mx, mx)
        self.global_min_my = min(self.global_min_my, my)
        self.global_max_my = max(self.global_max_my, my)

    def build_global_occupancy(self, template: OccupancyGrid) -> OccupancyGrid:
        occupancy = OccupancyGrid()
        occupancy.header = template.header
        occupancy.header.frame_id = template.header.frame_id
        occupancy.info.resolution = template.info.resolution
        occupancy.info.origin.orientation.w = 1.0

        if not self.global_score_map:
            occupancy.info.width = 1
            occupancy.info.height = 1
            occupancy.info.origin.position.x = template.info.origin.position.x
            occupancy.info.origin.position.y = template.info.origin.position.y
            occupancy.data = [self.unknown_value]
            return occupancy

        width = self.global_max_mx - self.global_min_mx + 1
        height = self.global_max_my - self.global_min_my + 1
        occupancy.info.width = width
        occupancy.info.height = height
        occupancy.info.origin.position.x = self.global_min_mx * occupancy.info.resolution
        occupancy.info.origin.position.y = self.global_min_my * occupancy.info.resolution

        grid = np.full((height, width), self.unknown_value, dtype=np.int8)
        for (mx, my), score in self.global_score_map.items():
            gx = mx - self.global_min_mx
            gy = my - self.global_min_my
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = int(np.clip(round(score), 0, 100))

        occupancy.data = grid.flatten(order="C").tolist()
        return occupancy

    def on_parameter_change(self, parameters):
        updated = False
        for parameter in parameters:
            if parameter.name == "mode" and parameter.type_ == Parameter.Type.STRING:
                self.mode = str(parameter.value)
                updated = True
            elif parameter.name == "layer" and parameter.type_ == Parameter.Type.STRING:
                self.layer = str(parameter.value)
                updated = True
            elif parameter.name == "confidence_layer" and parameter.type_ == Parameter.Type.STRING:
                self.confidence_layer = str(parameter.value)
                updated = True
            elif parameter.name == "uncertainty_layer" and parameter.type_ == Parameter.Type.STRING:
                self.uncertainty_layer = str(parameter.value)
                updated = True
            elif parameter.name == "water_cost_topic" and parameter.type_ == Parameter.Type.STRING:
                self.water_cost_topic = str(parameter.value).strip()
                updated = True
            elif parameter.name == "frame_id" and parameter.type_ == Parameter.Type.STRING:
                self.frame_override = str(parameter.value).strip()
                updated = True
            elif parameter.name == "fallback_frame_id" and parameter.type_ == Parameter.Type.STRING:
                self.fallback_frame_id = str(parameter.value).strip()
                updated = True
            elif parameter.name == "unknown_value" and parameter.type_ == Parameter.Type.INTEGER:
                self.unknown_value = int(parameter.value)
                updated = True
            elif parameter.name == "kernel_size" and parameter.type_ == Parameter.Type.INTEGER:
                self.kernel_size = int(parameter.value)
                updated = True
            elif parameter.name == "median_filter_size" and parameter.type_ == Parameter.Type.INTEGER:
                self.median_filter_size = int(parameter.value)
                updated = True
            elif parameter.name == "min_confidence" and parameter.type_ == Parameter.Type.DOUBLE:
                self.min_confidence = float(parameter.value)
                updated = True
            elif parameter.name == "height_cost_zero_at_m" and parameter.type_ == Parameter.Type.DOUBLE:
                self.height_cost_zero_at_m = float(parameter.value)
                updated = True
            elif parameter.name == "height_cost_full_at_m" and parameter.type_ == Parameter.Type.DOUBLE:
                self.height_cost_full_at_m = float(parameter.value)
                updated = True
            elif parameter.name == "slope_cost_full_at_deg" and parameter.type_ == Parameter.Type.DOUBLE:
                self.slope_cost_full_at_deg = float(parameter.value)
                updated = True
            elif parameter.name == "roughness_cost_zero_at_m" and parameter.type_ == Parameter.Type.DOUBLE:
                self.roughness_cost_zero_at_m = float(parameter.value)
                updated = True
            elif parameter.name == "roughness_cost_full_at_m" and parameter.type_ == Parameter.Type.DOUBLE:
                self.roughness_cost_full_at_m = float(parameter.value)
                updated = True
            elif parameter.name == "uncertainty_cost_zero_at" and parameter.type_ == Parameter.Type.DOUBLE:
                self.uncertainty_cost_zero_at = float(parameter.value)
                updated = True
            elif parameter.name == "uncertainty_cost_full_at" and parameter.type_ == Parameter.Type.DOUBLE:
                self.uncertainty_cost_full_at = float(parameter.value)
                updated = True
            elif parameter.name == "water_cost_scale" and parameter.type_ == Parameter.Type.DOUBLE:
                self.water_cost_scale = float(parameter.value)
                updated = True
            elif parameter.name == "weight_height" and parameter.type_ == Parameter.Type.DOUBLE:
                self.weight_height = float(parameter.value)
                updated = True
            elif parameter.name == "weight_slope" and parameter.type_ == Parameter.Type.DOUBLE:
                self.weight_slope = float(parameter.value)
                updated = True
            elif parameter.name == "weight_roughness" and parameter.type_ == Parameter.Type.DOUBLE:
                self.weight_roughness = float(parameter.value)
                updated = True
            elif parameter.name == "weight_water" and parameter.type_ == Parameter.Type.DOUBLE:
                self.weight_water = float(parameter.value)
                updated = True
            elif parameter.name == "weight_uncertainty" and parameter.type_ == Parameter.Type.DOUBLE:
                self.weight_uncertainty = float(parameter.value)
                updated = True
            elif parameter.name == "accumulate_global" and parameter.type_ == Parameter.Type.BOOL:
                self.accumulate_global = bool(parameter.value)
                updated = True
            elif parameter.name == "robot_clear_radius" and parameter.type_ == Parameter.Type.DOUBLE:
                self.robot_clear_radius = float(parameter.value)
                updated = True
            elif parameter.name == "global_fusion_alpha" and parameter.type_ == Parameter.Type.DOUBLE:
                self.global_fusion_alpha = float(parameter.value)
                updated = True

        try:
            if self.mode != "terrain_fusion":
                raise ValueError("mode must be 'terrain_fusion'.")
            if self.kernel_size % 2 == 0 or self.kernel_size < 1:
                raise ValueError("kernel_size must be a positive odd integer.")
            if self.median_filter_size % 2 == 0 or self.median_filter_size < 1:
                raise ValueError("median_filter_size must be a positive odd integer.")
            if self.height_cost_full_at_m <= self.height_cost_zero_at_m:
                raise ValueError("height_cost_full_at_m must be greater than height_cost_zero_at_m.")
            if self.roughness_cost_full_at_m <= self.roughness_cost_zero_at_m:
                raise ValueError("roughness_cost_full_at_m must be greater than roughness_cost_zero_at_m.")
            if self.uncertainty_cost_full_at <= self.uncertainty_cost_zero_at:
                raise ValueError("uncertainty_cost_full_at must be greater than uncertainty_cost_zero_at.")
            if self.slope_cost_full_at_deg <= 0.0:
                raise ValueError("slope_cost_full_at_deg must be > 0.")
            if not (0.0 <= self.min_confidence <= 1.0):
                raise ValueError("Expected 0 <= min_confidence <= 1.")
            if not (0.0 <= self.global_fusion_alpha <= 1.0):
                raise ValueError("Expected 0 <= global_fusion_alpha <= 1.")
            if self.robot_clear_radius < 0.0:
                raise ValueError("robot_clear_radius must be >= 0.")
            if self.water_cost_scale < 0.0:
                raise ValueError("water_cost_scale must be >= 0.")
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if updated and self.latest_grid_msg is not None:
            self.process_grid(self.latest_grid_msg)
        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = TraversabilityToMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
