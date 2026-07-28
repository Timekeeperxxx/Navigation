int32 order
int64 traj_id
bool emergency_stop
# True only when this local spline ends at the active global goal. Local
# lookahead splines and emergency-stop splines must leave this false.
bool terminal_goal
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

# Canonical Unitree B2 body-yaw samples. Values are unwrapped and linearly
# interpolated at yaw_dt; an empty array is accepted only for legacy senders.
float64[] yaw_pts
float64 yaw_dt
