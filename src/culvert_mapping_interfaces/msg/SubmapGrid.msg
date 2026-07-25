std_msgs/Header header

int32 submap_id
uint32 version
bool is_frozen
bool dirty

float32 resolution
int32 anchor_keyframe_id
int32[] keyframe_indices

# Initial submap pose in the odom/local frame before backend optimization.
geometry_msgs/Pose initial_pose

# Local 2D occupancy evidence stored in the submap frame.
int32[] free_cell_x
int32[] free_cell_y
int32[] occupied_cell_x
int32[] occupied_cell_y
