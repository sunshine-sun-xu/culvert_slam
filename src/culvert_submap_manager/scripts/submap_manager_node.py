#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np
import rclpy
import yaml
from culvert_mapping_interfaces.msg import DirtySubmapList, SubmapGrid, SubmapGridArray
from geometry_msgs.msg import Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


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


def pose_msg(position: np.ndarray, quat_xyzw: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quat_xyzw[0])
    pose.orientation.y = float(quat_xyzw[1])
    pose.orientation.z = float(quat_xyzw[2])
    pose.orientation.w = float(quat_xyzw[3])
    return pose


def angular_distance_deg(quat_a: np.ndarray, quat_b: np.ndarray) -> float:
    dot = abs(float(np.dot(quat_a, quat_b)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


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


@dataclass(frozen=True)
class KeyframeAsset:
    keyframe_id: int
    patch_name: str
    patch_path: Path
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass
class KeyframeRecord:
    index: int
    stamp_ns: int
    local_position: np.ndarray
    local_quaternion: np.ndarray
    local_transform: np.ndarray
    points: np.ndarray


@dataclass
class SubmapRecord:
    submap_id: int
    version: int
    is_frozen: bool
    dirty: bool
    anchor_index: int
    keyframe_indices: List[int]
    initial_pose: Pose
    free_cells: np.ndarray
    occupied_cells: np.ndarray


def parse_pose_line(line: str, line_number: int, patches_dir: Path) -> KeyframeAsset:
    tokens = line.split()
    if len(tokens) != 8:
        raise ValueError(
            f"poses.txt line {line_number} must have 8 fields: "
            "patch_name tx ty tz qw qx qy qz"
        )

    patch_name = tokens[0]
    tx, ty, tz = (float(tokens[i]) for i in range(1, 4))
    qw, qx, qy, qz = (float(tokens[i]) for i in range(4, 8))

    try:
        keyframe_id = int(Path(patch_name).stem)
    except ValueError:
        keyframe_id = line_number - 1

    return KeyframeAsset(
        keyframe_id=keyframe_id,
        patch_name=patch_name,
        patch_path=patches_dir / patch_name,
        position=(tx, ty, tz),
        orientation_xyzw=(qx, qy, qz, qw),
    )


def load_keyframes(source_map_dir: Path) -> list[KeyframeAsset]:
    poses_path = source_map_dir / "poses.txt"
    patches_dir = source_map_dir / "patches"

    if not source_map_dir.exists():
        raise FileNotFoundError(f"source_map_dir does not exist: {source_map_dir}")
    if not poses_path.exists():
        raise FileNotFoundError(f"poses.txt does not exist: {poses_path}")
    if not patches_dir.exists():
        raise FileNotFoundError(f"patches directory does not exist: {patches_dir}")

    keyframes: list[KeyframeAsset] = []
    with poses_path.open("r", encoding="utf-8") as poses_file:
        for line_number, raw_line in enumerate(poses_file, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            keyframe = parse_pose_line(line, line_number, patches_dir)
            if not keyframe.patch_path.exists():
                raise FileNotFoundError(
                    f"patch referenced by poses.txt line {line_number} does not exist: "
                    f"{keyframe.patch_path}"
                )
            keyframes.append(keyframe)

    keyframes.sort(key=lambda item: item.keyframe_id)
    return keyframes


def chunk_keyframes(
    keyframes: list[KeyframeAsset],
    keyframes_per_submap: int,
    overlap_keyframes: int,
) -> list[list[KeyframeAsset]]:
    if keyframes_per_submap <= 0:
        raise ValueError("keyframes_per_submap must be > 0")
    if overlap_keyframes < 0:
        raise ValueError("overlap_keyframes must be >= 0")
    if overlap_keyframes >= keyframes_per_submap:
        raise ValueError("overlap_keyframes must be smaller than keyframes_per_submap")

    step = keyframes_per_submap - overlap_keyframes
    chunks: list[list[KeyframeAsset]] = []
    start = 0
    while start < len(keyframes):
        chunk = keyframes[start : start + keyframes_per_submap]
        if chunk:
            chunks.append(chunk)
        if start + keyframes_per_submap >= len(keyframes):
            break
        start += step
    return chunks


def pose_bounds(keyframes: list[KeyframeAsset], padding: float) -> dict[str, list[float]]:
    xs = [item.position[0] for item in keyframes]
    ys = [item.position[1] for item in keyframes]
    zs = [item.position[2] for item in keyframes]
    return {
        "min": [min(xs) - padding, min(ys) - padding, min(zs)],
        "max": [max(xs) + padding, max(ys) + padding, max(zs)],
    }


def mean_position(keyframes: list[KeyframeAsset]) -> list[float]:
    count = float(len(keyframes))
    return [
        sum(item.position[0] for item in keyframes) / count,
        sum(item.position[1] for item in keyframes) / count,
        sum(item.position[2] for item in keyframes) / count,
    ]


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as yaml_file:
        yaml.safe_dump(data, yaml_file, sort_keys=False, allow_unicode=False)


def link_or_copy_patch(src: Path, dst: Path, asset_mode: str) -> None:
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    if asset_mode == "copy":
        shutil.copy2(src, dst)
    elif asset_mode == "symlink":
        os.symlink(src, dst)
    elif asset_mode == "reference":
        return
    else:
        raise ValueError("asset_mode must be one of: symlink, copy, reference")


class SubmapManager(Node):
    def __init__(self) -> None:
        super().__init__("submap_manager")

        self.declare_parameter("online_enable", True)
        self.declare_parameter("body_cloud_topic", "/fastlio2/body_cloud")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("submap_grid_topic", "~/submap_grids")
        self.declare_parameter("submap_content_dirty_topic", "~/submap_content_dirty")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("sensor_frame", "base_link")
        self.declare_parameter("dynamic_keyframe_enable", True)
        self.declare_parameter("keyframe_meter_gap", 0.5)
        self.declare_parameter("keyframe_deg_gap", 10.0)
        self.declare_parameter("keyframe_meter_gap_min", 0.5)
        self.declare_parameter("keyframe_meter_gap_max", 2.0)
        self.declare_parameter("keyframe_deg_gap_min", 10.0)
        self.declare_parameter("keyframe_deg_gap_max", 30.0)
        self.declare_parameter("submap_resolution_m", 0.1)
        self.declare_parameter("keyframes_per_submap", 20)
        self.declare_parameter("overlap_keyframes", 5)
        self.declare_parameter("point_z_min", -0.6)
        self.declare_parameter("point_z_max", 1.2)
        self.declare_parameter("point_range_min", 0.3)
        self.declare_parameter("point_range_max", 20.0)
        self.declare_parameter("point_stride", 2)
        self.declare_parameter("hit_score", 4)
        self.declare_parameter("miss_score", 1)
        self.declare_parameter("occupied_threshold", 2)
        self.declare_parameter("free_threshold", -1)
        self.declare_parameter("publish_rate", 2.0)

        self.declare_parameter("source_map_dir", "/home/hust-craic/culvert_ws/tmp_pgo_map")
        self.declare_parameter("output_dir", "/home/hust-craic/culvert_ws/submaps")
        self.declare_parameter("backend_name", "fastlio2_pgo_hba_localizer")
        self.declare_parameter("submap_size_m", 10.0)
        self.declare_parameter("asset_mode", "symlink")
        self.declare_parameter("clean_output", False)
        self.declare_parameter("auto_build", False)

        self.online_enable = bool(self.get_parameter("online_enable").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.sensor_frame = str(self.get_parameter("sensor_frame").value)
        self.dynamic_keyframe_enable = bool(self.get_parameter("dynamic_keyframe_enable").value)
        self.keyframe_meter_gap = float(self.get_parameter("keyframe_meter_gap").value)
        self.keyframe_deg_gap = float(self.get_parameter("keyframe_deg_gap").value)
        self.keyframe_meter_gap_min = float(self.get_parameter("keyframe_meter_gap_min").value)
        self.keyframe_meter_gap_max = float(self.get_parameter("keyframe_meter_gap_max").value)
        self.keyframe_deg_gap_min = float(self.get_parameter("keyframe_deg_gap_min").value)
        self.keyframe_deg_gap_max = float(self.get_parameter("keyframe_deg_gap_max").value)
        self.resolution = float(self.get_parameter("submap_resolution_m").value)
        self.keyframes_per_submap = int(self.get_parameter("keyframes_per_submap").value)
        self.overlap_keyframes = int(self.get_parameter("overlap_keyframes").value)
        self.point_z_min = float(self.get_parameter("point_z_min").value)
        self.point_z_max = float(self.get_parameter("point_z_max").value)
        self.point_range_min = float(self.get_parameter("point_range_min").value)
        self.point_range_max = float(self.get_parameter("point_range_max").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.hit_score = max(1, int(self.get_parameter("hit_score").value))
        self.miss_score = max(1, int(self.get_parameter("miss_score").value))
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)

        self.keyframes: List[KeyframeRecord] = []
        self.frozen_submaps: List[SubmapRecord] = []
        self.active_submap: SubmapRecord | None = None
        self.next_submap_id = 0
        self.active_start_index = 0
        self.submap_epoch = 0
        self.latest_degeneration_score = 0.0
        self.submap_dirty_ids: set[int] = set()

        self.build_srv = self.create_service(Trigger, "~/build_index", self.on_build_index)
        self.submap_pub = self.create_publisher(
            SubmapGridArray, str(self.get_parameter("submap_grid_topic").value), 10
        )
        self.dirty_pub = self.create_publisher(
            DirtySubmapList, str(self.get_parameter("submap_content_dirty_topic").value), 10
        )

        publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_online_submaps)

        if self.online_enable:
            cloud_topic = str(self.get_parameter("body_cloud_topic").value)
            odom_topic = str(self.get_parameter("odom_topic").value)
            self.cloud_sub = Subscriber(self, PointCloud2, cloud_topic)
            self.odom_sub = Subscriber(self, Odometry, odom_topic)
            self.sync = ApproximateTimeSynchronizer([self.cloud_sub, self.odom_sub], 20, 0.1)
            self.sync.registerCallback(self.on_sync)
            self.get_logger().info(
                f"submap_manager online: cloud={cloud_topic} odom={odom_topic} "
                f"submaps={self.get_parameter('submap_grid_topic').value}"
            )
        else:
            self.cloud_sub = None
            self.odom_sub = None
            self.sync = None

        if bool(self.get_parameter("auto_build").value):
            self.build_submap_index()

    def on_sync(self, cloud_msg: PointCloud2, odom_msg: Odometry) -> None:
        points = self.extract_points(cloud_msg)
        if points.size == 0:
            return

        position = np.array(
            [
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                odom_msg.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        quat = np.array(
            [
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        if not self.should_create_keyframe(position, quat):
            return

        stamp_ns = int(odom_msg.header.stamp.sec) * 1_000_000_000 + int(odom_msg.header.stamp.nanosec)
        self.keyframes.append(
            KeyframeRecord(
                index=len(self.keyframes),
                stamp_ns=stamp_ns,
                local_position=position,
                local_quaternion=quat,
                local_transform=pose_to_matrix(position, quat),
                points=points,
            )
        )
        self.maybe_freeze_submaps()
        self.refresh_active_submap()

    def should_create_keyframe(self, position: np.ndarray, quat: np.ndarray) -> bool:
        if not self.keyframes:
            return True
        last = self.keyframes[-1]
        distance = float(np.linalg.norm(position - last.local_position))
        angular = angular_distance_deg(quat, last.local_quaternion)
        if not self.dynamic_keyframe_enable:
            return distance >= self.keyframe_meter_gap or angular >= self.keyframe_deg_gap

        degeneration = float(np.clip(self.latest_degeneration_score, 0.0, 1.0))
        meter_gap = self.keyframe_meter_gap_min + (
            self.keyframe_meter_gap_max - self.keyframe_meter_gap_min
        ) * degeneration
        deg_gap = self.keyframe_deg_gap_min + (
            self.keyframe_deg_gap_max - self.keyframe_deg_gap_min
        ) * degeneration
        return distance >= meter_gap or angular >= deg_gap

    def maybe_freeze_submaps(self) -> None:
        if self.keyframes_per_submap <= 0 or self.overlap_keyframes >= self.keyframes_per_submap:
            return
        step = self.keyframes_per_submap - self.overlap_keyframes
        while len(self.keyframes) - self.active_start_index >= self.keyframes_per_submap:
            end = self.active_start_index + self.keyframes_per_submap
            indices = list(range(self.active_start_index, end))
            submap = self.build_submap(indices, self.next_submap_id, is_frozen=True)
            self.frozen_submaps.append(submap)
            self.submap_dirty_ids.add(submap.submap_id)
            self.next_submap_id += 1
            self.active_start_index += step
            self.get_logger().info(
                f"frozen submap {submap.submap_id}: keyframes {indices[0]}..{indices[-1]}"
            )

    def refresh_active_submap(self) -> None:
        if self.active_start_index >= len(self.keyframes):
            self.active_submap = None
            return
        active_id = self.next_submap_id
        indices = list(range(self.active_start_index, len(self.keyframes)))
        self.active_submap = self.build_submap(indices, active_id, is_frozen=False)
        self.submap_dirty_ids.add(active_id)
        self.publish_online_submaps()

    def build_submap(self, keyframe_indices: List[int], submap_id: int, is_frozen: bool) -> SubmapRecord:
        anchor = self.keyframes[keyframe_indices[0]]
        inv_anchor = np.linalg.inv(anchor.local_transform)
        cell_scores: Dict[Tuple[int, int], int] = {}

        for keyframe_index in keyframe_indices:
            keyframe = self.keyframes[keyframe_index]
            rel_transform = inv_anchor @ keyframe.local_transform
            sensor_origin = rel_transform[:2, 3]
            origin_cell = (
                int(math.floor(sensor_origin[0] / self.resolution)),
                int(math.floor(sensor_origin[1] / self.resolution)),
            )
            homogeneous = np.ones((keyframe.points.shape[0], 4), dtype=np.float64)
            homogeneous[:, :3] = keyframe.points
            local_points = (rel_transform @ homogeneous.T).T[:, :3]

            for point in local_points:
                end_cell = (
                    int(math.floor(point[0] / self.resolution)),
                    int(math.floor(point[1] / self.resolution)),
                )
                ray = list(bresenham(origin_cell, end_cell))
                for free_cell in ray[:-1]:
                    cell_scores[free_cell] = cell_scores.get(free_cell, 0) - self.miss_score
                cell_scores[end_cell] = cell_scores.get(end_cell, 0) + self.hit_score

        free_cells: List[Tuple[int, int]] = []
        occupied_cells: List[Tuple[int, int]] = []
        for key, score in cell_scores.items():
            if score >= self.occupied_threshold:
                occupied_cells.append(key)
            elif score <= self.free_threshold:
                free_cells.append(key)

        self.submap_epoch += 1
        return SubmapRecord(
            submap_id=submap_id,
            version=self.submap_epoch,
            is_frozen=is_frozen,
            dirty=True,
            anchor_index=anchor.index,
            keyframe_indices=keyframe_indices,
            initial_pose=pose_msg(anchor.local_position, anchor.local_quaternion),
            free_cells=np.asarray(free_cells, dtype=np.int32)
            if free_cells
            else np.zeros((0, 2), dtype=np.int32),
            occupied_cells=np.asarray(occupied_cells, dtype=np.int32)
            if occupied_cells
            else np.zeros((0, 2), dtype=np.int32),
        )

    def extract_points(self, msg: PointCloud2) -> np.ndarray:
        points: List[Tuple[float, float, float]] = []
        for idx, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx % self.point_stride != 0:
                continue
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if z < self.point_z_min or z > self.point_z_max:
                continue
            xy_range = math.hypot(x, y)
            if xy_range < self.point_range_min or xy_range > self.point_range_max:
                continue
            points.append((x, y, z))
        if not points:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(points, dtype=np.float64)

    def submap_to_msg(self, submap: SubmapRecord) -> SubmapGrid:
        msg = SubmapGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.submap_id = int(submap.submap_id)
        msg.version = int(submap.version)
        msg.is_frozen = bool(submap.is_frozen)
        msg.dirty = bool(submap.dirty)
        msg.resolution = float(self.resolution)
        msg.anchor_keyframe_id = int(submap.anchor_index)
        msg.keyframe_indices = [int(index) for index in submap.keyframe_indices]
        msg.initial_pose = submap.initial_pose
        msg.free_cell_x = submap.free_cells[:, 0].astype(np.int32).tolist()
        msg.free_cell_y = submap.free_cells[:, 1].astype(np.int32).tolist()
        msg.occupied_cell_x = submap.occupied_cells[:, 0].astype(np.int32).tolist()
        msg.occupied_cell_y = submap.occupied_cells[:, 1].astype(np.int32).tolist()
        return msg

    def publish_online_submaps(self) -> None:
        if not self.online_enable:
            return
        submaps = list(self.frozen_submaps)
        if self.active_submap is not None:
            submaps.append(self.active_submap)
        if not submaps:
            return

        array = SubmapGridArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.global_frame
        array.submaps = [self.submap_to_msg(submap) for submap in submaps]
        self.submap_pub.publish(array)

        if self.submap_dirty_ids:
            dirty = DirtySubmapList()
            dirty.header = array.header
            dirty.optimization_epoch = int(self.submap_epoch)
            for submap in submaps:
                if submap.submap_id in self.submap_dirty_ids:
                    dirty.submap_ids.append(int(submap.submap_id))
                    dirty.versions.append(int(submap.version))
            self.dirty_pub.publish(dirty)
            self.submap_dirty_ids.clear()

        for submap in self.frozen_submaps:
            submap.dirty = False
        if self.active_submap is not None:
            self.active_submap.dirty = False

    def on_build_index(self, _request, response):
        try:
            count = self.build_submap_index()
        except Exception as exc:  # noqa: BLE001 - service should report all build failures.
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Built {count} submaps."
        return response

    def build_submap_index(self) -> int:
        source_map_dir = Path(str(self.get_parameter("source_map_dir").value)).expanduser()
        output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        backend_name = str(self.get_parameter("backend_name").value)
        global_frame = str(self.get_parameter("global_frame").value)
        sensor_frame = str(self.get_parameter("sensor_frame").value)
        submap_size_m = float(self.get_parameter("submap_size_m").value)
        submap_resolution_m = float(self.get_parameter("submap_resolution_m").value)
        keyframes_per_submap = int(self.get_parameter("keyframes_per_submap").value)
        overlap_keyframes = int(self.get_parameter("overlap_keyframes").value)
        asset_mode = str(self.get_parameter("asset_mode").value)
        clean_output = bool(self.get_parameter("clean_output").value)

        keyframes = load_keyframes(source_map_dir)
        if not keyframes:
            raise ValueError(f"No keyframes found in {source_map_dir / 'poses.txt'}")

        if clean_output and output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        chunks = chunk_keyframes(keyframes, keyframes_per_submap, overlap_keyframes)
        submaps: list[dict[str, Any]] = []
        padding = max(0.0, 0.5 * submap_size_m)

        for submap_id, chunk in enumerate(chunks):
            submap_name = f"submap_{submap_id:04d}"
            submap_dir = output_dir / submap_name
            patch_dir = submap_dir / "patches"
            patch_dir.mkdir(parents=True, exist_ok=True)

            patch_records = []
            for keyframe in chunk:
                dst = patch_dir / keyframe.patch_name
                link_or_copy_patch(keyframe.patch_path, dst, asset_mode)
                patch_records.append(
                    {
                        "keyframe_id": keyframe.keyframe_id,
                        "patch_name": keyframe.patch_name,
                        "source_path": str(keyframe.patch_path),
                        "local_path": str(dst.relative_to(submap_dir))
                        if asset_mode != "reference"
                        else "",
                        "position": list(keyframe.position),
                        "orientation_xyzw": list(keyframe.orientation_xyzw),
                    }
                )

            first = chunk[0]
            last = chunk[-1]
            meta = {
                "schema_version": 2,
                "submap_id": submap_id,
                "submap_name": submap_name,
                "status": "frozen",
                "dirty": False,
                "source_backend": backend_name,
                "source_map_dir": str(source_map_dir),
                "global_frame": global_frame,
                "sensor_frame": sensor_frame,
                "submap_size_m": submap_size_m,
                "submap_resolution_m": submap_resolution_m,
                "keyframe_range": [first.keyframe_id, last.keyframe_id],
                "keyframe_count": len(chunk),
                "center_position": mean_position(chunk),
                "pose_bounds": pose_bounds(chunk, padding),
                "initial_pose_odom": {
                    "keyframe_id": first.keyframe_id,
                    "position": list(first.position),
                    "orientation_xyzw": list(first.orientation_xyzw),
                },
                "optimized_pose_map": {
                    "version": 0,
                    "position": list(first.position),
                    "orientation_xyzw": list(first.orientation_xyzw),
                },
                "anchor_pose": {
                    "keyframe_id": first.keyframe_id,
                    "position": list(first.position),
                    "orientation_xyzw": list(first.orientation_xyzw),
                },
                "patches": patch_records,
                "derived_layers": {
                    "elevation": "",
                    "traversability": "",
                    "slope": "",
                    "roughness": "",
                },
                "history": [],
            }
            write_yaml(submap_dir / "meta.yaml", meta)
            submaps.append(
                {
                    "submap_id": submap_id,
                    "submap_name": submap_name,
                    "meta_path": str((submap_dir / "meta.yaml").relative_to(output_dir)),
                    "keyframe_range": [first.keyframe_id, last.keyframe_id],
                    "keyframe_count": len(chunk),
                    "center_position": meta["center_position"],
                    "pose_bounds": meta["pose_bounds"],
                    "status": "frozen",
                    "dirty": False,
                    "pose_version": 0,
                }
            )

        index = {
            "schema_version": 2,
            "source_backend": backend_name,
            "source_map_dir": str(source_map_dir),
            "global_frame": global_frame,
            "sensor_frame": sensor_frame,
            "asset_mode": asset_mode,
            "keyframes_total": len(keyframes),
            "submaps_total": len(submaps),
            "keyframes_per_submap": keyframes_per_submap,
            "overlap_keyframes": overlap_keyframes,
            "submap_size_m": submap_size_m,
            "submap_resolution_m": submap_resolution_m,
            "submaps": submaps,
        }
        write_yaml(output_dir / "index.yaml", index)

        self.get_logger().info(
            f"Built {len(submaps)} submaps from {len(keyframes)} keyframes into {output_dir}"
        )
        return len(submaps)


def build_once_from_args() -> None:
    source_map_dir = Path("/home/hust-craic/culvert_ws/tmp_pgo_map")
    output_dir = Path("/home/hust-craic/culvert_ws/submaps")
    if "--source-map-dir" in sys.argv:
        source_map_dir = Path(sys.argv[sys.argv.index("--source-map-dir") + 1]).expanduser()
    if "--output-dir" in sys.argv:
        output_dir = Path(sys.argv[sys.argv.index("--output-dir") + 1]).expanduser()

    rclpy.init(args=None)
    node = SubmapManager()
    try:
        node.set_parameters(
            [
                Parameter("source_map_dir", value=str(source_map_dir)),
                Parameter("output_dir", value=str(output_dir)),
            ]
        )
        count = node.build_submap_index()
        print(f"Built {count} submaps into {output_dir}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    if "--build-once" in sys.argv:
        build_once_from_args()
        return

    rclpy.init(args=None)
    node = SubmapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
