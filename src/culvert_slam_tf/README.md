# culvert_slam_tf

This package keeps the TF tree for the culvert SLAM backend clean and standard:

- `map -> odom`: dynamic transform published by the backend bridge
- `body -> lidar`: static extrinsic
- `body -> imu`: static extrinsic

Expected full chain:

`map -> odom -> body -> lidar`

The local costmap should use:

- `global_frame = odom`
- `robot_base_frame = body`
