# Culvert WS

## Overview
- `fastlio2`: frontend odom and cloud source.
- `culvert_pgo`: keyframe pose graph optimization and `map -> odom`.
- `culvert_submap_manager`: submap lifecycle and asset management.
- `culvert_frontend_mapping`: online global map and traversability pipeline.
- `FastDEM`: elevation mapping backend.
- `grid_map`: grid map dependency.
- `culvert_slam_tf`: static/dynamic TF bridge.

## Navigation Topics
- `/map`: global occupancy map for Nav2 global costmap.
- `/traversability_map_local`: local traversability map for Nav2 local costmap.
- `/traversability_map`: global traversability map.
- `/fastdem/mapping/gridmap`: FastDEM output.
- `/culvert_pgo_node/optimized_path`: optimized keyframe path.
- `/culvert_pgo_node/optimized_submap_poses`: optimized submap poses.
- `/culvert_pgo_node/dirty_submap_list`: dirty submap updates.

## TF
- `map -> odom`: dynamic TF from `culvert_pgo`.
- `odom -> base_link -> lidar`: frontend TF chain.
- `culvert_slam_tf` keeps the standard TF bridge available.

## Nav2
- Global costmap uses `/map`.
- Local costmap uses `/traversability_map_local`.
- Nav2 params: [src/culvert_frontend_mapping/config/nav2_mppi_elevation.yaml](/home/hust-craic/culvert_ws/src/culvert_frontend_mapping/config/nav2_mppi_elevation.yaml)

## Main Launches
- Full stack: [src/culvert_frontend_mapping/launch/full_navigation_stack.launch.py](/home/hust-craic/culvert_ws/src/culvert_frontend_mapping/launch/full_navigation_stack.launch.py)
- Mapping only: [src/culvert_frontend_mapping/launch/frontend_elevation_mapping.launch.py](/home/hust-craic/culvert_ws/src/culvert_frontend_mapping/launch/frontend_elevation_mapping.launch.py)
- Nav2 only: [src/culvert_frontend_mapping/launch/nav2_mppi_navigation.launch.py](/home/hust-craic/culvert_ws/src/culvert_frontend_mapping/launch/nav2_mppi_navigation.launch.py)
- Submap manager: [src/culvert_submap_manager/launch/submap_manager.launch.py](/home/hust-craic/culvert_ws/src/culvert_submap_manager/launch/submap_manager.launch.py)
- PGO: [src/culvert_pgo/launch/culvert_pgo.launch.py](/home/hust-craic/culvert_ws/src/culvert_pgo/launch/culvert_pgo.launch.py)

## Run
```bash
source install/setup.bash
ros2 launch culvert_frontend_mapping full_navigation_stack.launch.py
```

## Notes
- `FastDEM` is used for elevation and traversability products.
- `online_global_map_builder` publishes `/map` from submaps and optimized poses.
- RViz fixed frame and TF time alignment matter for map displays.
