std_msgs/Header header

int32 submap_id
uint32 version
bool dirty
bool is_frozen

int32 anchor_keyframe_id

# Initial pose supplied by the submap manager and optimized pose estimated
# from the backend keyframe solution.
geometry_msgs/Pose initial_pose
geometry_msgs/Pose optimized_pose

# Rigid correction from initial_pose to optimized_pose.
geometry_msgs/Pose delta_pose
