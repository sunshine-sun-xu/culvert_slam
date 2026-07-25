# culvert_submap_manager

Backend-agnostic submap asset manager for the culvert mapping stack.

## Purpose

This package keeps submap management decoupled from a specific backend optimizer.
The first adapter consumes the map asset format already produced by the
`FAST_LIO_ROS2_edited/FASTLIO2_ROS2/pgo` backend:

```text
map_dir/
  map.pcd
  poses.txt
  patches/
    0.pcd
    1.pcd
```

`poses.txt` lines use:

```text
patch_name tx ty tz qw qx qy qz
```

The manager groups keyframe patches into submap folders and writes stable
metadata. FastDEM elevation/traversability reconstruction can be added on top of
this asset layer without coupling to the PGO implementation.

## Output Layout

```text
submaps/
  index.yaml
  submap_0000/
    meta.yaml
    patches/
      0.pcd -> source patch
      ...
```

## Run

```bash
ros2 launch culvert_submap_manager submap_manager.launch.py
ros2 service call /submap_manager/build_index std_srvs/srv/Trigger {}
```

For a one-shot build at launch, set `auto_build: true` in the config.

## Design References

- Cartographer-style separation of submaps and pose graph.
- `elevation_mapping` style separation between local grid map products and map assets.
- FAST-LIO-SLAM style `patches + poses` map asset export.
