#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import numpy as np
import rclpy
from culvert_mapping_interfaces.msg import (
    DirtySubmapList,
    OptimizedSubmapPoseArray,
    SubmapGrid,
    SubmapGridArray,
)
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def quaternion_to_matrix_xyzw(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(position: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_matrix_xyzw(quat_xyzw)
    transform[:3, 3] = position
    return transform


def pose_msg_to_matrix(pose: Pose) -> np.ndarray:
    position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=np.float64,
    )
    return pose_to_matrix(position, quat)


def pose_close(a: np.ndarray, b: np.ndarray, trans_tol: float = 1e-4, rot_tol_deg: float = 0.01) -> bool:
    translation_close = np.linalg.norm(a[:3, 3] - b[:3, 3]) <= trans_tol
    rotation_a = a[:3, :3]
    rotation_b = b[:3, :3]
    trace = float(np.trace(rotation_a.T @ rotation_b))
    trace = max(-1.0, min(3.0, trace))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5))))
    return translation_close and angle <= rot_tol_deg


def bresenham(start: Tuple[int, int], end: Tuple[int, int]) -> Iterable[Tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x = x0
    y = y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


@dataclass
class SubmapCache:
    submap_id: int
    version: int = 0
    is_frozen: bool = False
    dirty: bool = False
    anchor_keyframe_id: int = -1
    resolution: float = 0.1
    keyframe_indices: List[int] | None = None
    initial_pose: np.ndarray | None = None
    current_pose: np.ndarray | None = None
    free_cells: np.ndarray | None = None
    occupied_cells: np.ndarray | None = None
    contributions: Dict[Tuple[int, int], int] | None = None
    pose_version: int = 0


class OnlineGlobalMapBuilder(Node):
    def __init__(self) -> None:
        super().__init__("online_global_map_builder")

        self.declare_parameter("submap_grid_topic", "/submap_manager/submap_grids")
        self.declare_parameter(
            "optimized_submap_pose_topic", "/culvert_pgo_node/optimized_submap_poses"
        )
        self.declare_parameter("dirty_submap_topic", "/culvert_pgo_node/dirty_submap_list")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_publish_rate", 1.0)
        self.declare_parameter("include_active_submap", True)
        self.declare_parameter("hit_score", 4)
        self.declare_parameter("miss_score", 1)
        self.declare_parameter("occupied_threshold", 2)
        self.declare_parameter("free_threshold", -1)
        self.declare_parameter("robot_clear_radius", 0.35)

        self.submap_grid_topic = str(self.get_parameter("submap_grid_topic").value)
        self.optimized_submap_pose_topic = str(self.get_parameter("optimized_submap_pose_topic").value)
        self.dirty_submap_topic = str(self.get_parameter("dirty_submap_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.include_active_submap = bool(self.get_parameter("include_active_submap").value)
        self.hit_score = max(1, int(self.get_parameter("hit_score").value))
        self.miss_score = max(1, int(self.get_parameter("miss_score").value))
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.robot_clear_radius = max(0.0, float(self.get_parameter("robot_clear_radius").value))

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, latch_qos)
        self.submap_grid_sub = self.create_subscription(
            SubmapGridArray, self.submap_grid_topic, self.on_submap_grids, 10
        )
        self.optimized_pose_sub = self.create_subscription(
            OptimizedSubmapPoseArray, self.optimized_submap_pose_topic, self.on_optimized_poses, 10
        )
        self.dirty_sub = self.create_subscription(
            DirtySubmapList, self.dirty_submap_topic, self.on_dirty_list, 10
        )

        publish_rate = max(0.2, float(self.get_parameter("map_publish_rate").value))
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.submaps: Dict[int, SubmapCache] = {}
        self.pending_dirty_ids: set[int] = set()
        self.global_scores: Dict[Tuple[int, int], int] = {}
        self.last_publish_warn = self.get_clock().now()
        self.latest_optimization_epoch = 0

        self.get_logger().info(
            "online_global_map_builder: submaps=%s poses=%s map=%s"
            % (self.submap_grid_topic, self.optimized_submap_pose_topic, self.map_topic)
        )

    def on_submap_grids(self, msg: SubmapGridArray) -> None:
        for submap_msg in msg.submaps:
            if not self.include_active_submap and not submap_msg.is_frozen:
                continue
            cache = self.submaps.get(submap_msg.submap_id)
            initial_pose = pose_msg_to_matrix(submap_msg.initial_pose)
            free_cells = np.column_stack(
                [
                    np.asarray(submap_msg.free_cell_x, dtype=np.int32),
                    np.asarray(submap_msg.free_cell_y, dtype=np.int32),
                ]
            )
            occupied_cells = np.column_stack(
                [
                    np.asarray(submap_msg.occupied_cell_x, dtype=np.int32),
                    np.asarray(submap_msg.occupied_cell_y, dtype=np.int32),
                ]
            )
            if free_cells.size == 0:
                free_cells = np.zeros((0, 2), dtype=np.int32)
            if occupied_cells.size == 0:
                occupied_cells = np.zeros((0, 2), dtype=np.int32)

            changed = (
                cache is None
                or cache.version != submap_msg.version
                or cache.resolution != float(submap_msg.resolution)
                or cache.is_frozen != bool(submap_msg.is_frozen)
                or cache.anchor_keyframe_id != int(submap_msg.anchor_keyframe_id)
                or not np.array_equal(cache.free_cells, free_cells)
                or not np.array_equal(cache.occupied_cells, occupied_cells)
                or not pose_close(cache.initial_pose, initial_pose)  # type: ignore[arg-type]
            )

            if cache is None:
                cache = SubmapCache(submap_id=submap_msg.submap_id)
                self.submaps[submap_msg.submap_id] = cache

            cache.version = int(submap_msg.version)
            cache.is_frozen = bool(submap_msg.is_frozen)
            cache.dirty = bool(submap_msg.dirty) or changed
            cache.anchor_keyframe_id = int(submap_msg.anchor_keyframe_id)
            cache.resolution = float(submap_msg.resolution)
            cache.keyframe_indices = [int(v) for v in submap_msg.keyframe_indices]
            cache.initial_pose = initial_pose
            cache.free_cells = free_cells
            cache.occupied_cells = occupied_cells
            cache.contributions = cache.contributions or {}
            if changed:
                self.pending_dirty_ids.add(cache.submap_id)

    def on_optimized_poses(self, msg: OptimizedSubmapPoseArray) -> None:
        self.latest_optimization_epoch = int(msg.optimization_epoch)
        for submap_msg in msg.submaps:
            cache = self.submaps.get(submap_msg.submap_id)
            if cache is None:
                cache = SubmapCache(submap_id=submap_msg.submap_id)
                self.submaps[submap_msg.submap_id] = cache
            optimized_pose = pose_msg_to_matrix(submap_msg.optimized_pose)
            changed = (
                cache.pose_version != int(submap_msg.version)
                or cache.current_pose is None
                or not pose_close(cache.current_pose, optimized_pose)
                or bool(submap_msg.dirty)
            )
            cache.pose_version = int(submap_msg.version)
            cache.current_pose = optimized_pose
            cache.dirty = bool(submap_msg.dirty) or changed
            if changed:
                self.pending_dirty_ids.add(cache.submap_id)

    def on_dirty_list(self, msg: DirtySubmapList) -> None:
        self.latest_optimization_epoch = max(
            self.latest_optimization_epoch, int(msg.optimization_epoch)
        )
        for submap_id in msg.submap_ids:
            self.pending_dirty_ids.add(int(submap_id))

    def effective_pose(self, cache: SubmapCache) -> np.ndarray | None:
        return cache.current_pose if cache.current_pose is not None else cache.initial_pose

    def transform_cell(self, pose: np.ndarray, cell_xy: np.ndarray, resolution: float) -> Tuple[int, int]:
        local_point = np.array(
            [
                (float(cell_xy[0]) + 0.5) * resolution,
                (float(cell_xy[1]) + 0.5) * resolution,
                0.0,
                1.0,
            ],
            dtype=np.float64,
        )
        world = pose @ local_point
        return int(math.floor(world[0] / resolution)), int(math.floor(world[1] / resolution))

    def compute_contributions(self, cache: SubmapCache) -> Dict[Tuple[int, int], int]:
        pose = self.effective_pose(cache)
        if pose is None or cache.free_cells is None or cache.occupied_cells is None:
            return {}

        contributions: Dict[Tuple[int, int], int] = {}
        for cell_xy in cache.free_cells:
            key = self.transform_cell(pose, cell_xy, cache.resolution)
            contributions[key] = contributions.get(key, 0) - self.miss_score
        for cell_xy in cache.occupied_cells:
            key = self.transform_cell(pose, cell_xy, cache.resolution)
            contributions[key] = contributions.get(key, 0) + self.hit_score
        return contributions

    def apply_dirty_updates(self) -> None:
        if not self.pending_dirty_ids:
            return

        for submap_id in list(self.pending_dirty_ids):
            cache = self.submaps.get(submap_id)
            if cache is None or cache.initial_pose is None:
                continue
            if cache.contributions:
                for key, score in cache.contributions.items():
                    self.global_scores[key] = self.global_scores.get(key, 0) - score
                    if self.global_scores.get(key, 0) == 0:
                        self.global_scores.pop(key, None)

            cache.contributions = self.compute_contributions(cache)
            for key, score in cache.contributions.items():
                self.global_scores[key] = self.global_scores.get(key, 0) + score
            cache.dirty = False

        self.pending_dirty_ids.clear()

    def clear_robot_disc(self, scores: Dict[Tuple[int, int], int]) -> None:
        if self.robot_clear_radius <= 0.0 or not self.submaps:
            return
        latest_submap = max(self.submaps.values(), key=lambda item: item.submap_id)
        pose = self.effective_pose(latest_submap)
        if pose is None:
            return
        center_x = float(pose[0, 3])
        center_y = float(pose[1, 3])
        cell_radius = max(1, int(math.ceil(self.robot_clear_radius / latest_submap.resolution)))
        center_mx = int(math.floor(center_x / latest_submap.resolution))
        center_my = int(math.floor(center_y / latest_submap.resolution))
        radius_sq = self.robot_clear_radius * self.robot_clear_radius
        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                wx = (center_mx + dx + 0.5) * latest_submap.resolution
                wy = (center_my + dy + 0.5) * latest_submap.resolution
                if (wx - center_x) ** 2 + (wy - center_y) ** 2 <= radius_sq:
                    scores[(center_mx + dx, center_my + dy)] = min(
                        scores.get((center_mx + dx, center_my + dy), 0), self.free_threshold
                    )

    def rebuild_global_map(self) -> OccupancyGrid | None:
        if not self.submaps:
            return None
        self.apply_dirty_updates()
        if not self.global_scores:
            return None

        scores = dict(self.global_scores)
        self.clear_robot_disc(scores)

        keys = list(scores.keys())
        min_mx = min(item[0] for item in keys)
        max_mx = max(item[0] for item in keys)
        min_my = min(item[1] for item in keys)
        max_my = max(item[1] for item in keys)
        width = max_mx - min_mx + 1
        height = max_my - min_my + 1

        resolution = min(submap.resolution for submap in self.submaps.values())
        grid = np.full((height, width), -1, dtype=np.int8)
        for (mx, my), score in scores.items():
            gx = mx - min_mx
            gy = my - min_my
            if score >= self.occupied_threshold:
                grid[gy, gx] = 100
            elif score <= self.free_threshold:
                grid[gy, gx] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_mx * resolution
        msg.info.origin.position.y = min_my * resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten(order="C").tolist()
        return msg

    def on_timer(self) -> None:
        msg = self.rebuild_global_map()
        if msg is None:
            if self.get_clock().now() - self.last_publish_warn > Duration(seconds=5.0):
                self.get_logger().warning("waiting for submaps and optimized poses before publishing /map")
                self.last_publish_warn = self.get_clock().now()
            return
        self.map_pub.publish(msg)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OnlineGlobalMapBuilder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
