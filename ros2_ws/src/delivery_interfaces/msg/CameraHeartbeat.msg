# Industrial camera heartbeat (≥2 Hz, RELIABLE).
# Publisher: Xavier cam_front_ros_node
# Subscriber: NUC camera_ingress_node
std_msgs/Header header
string camera_name
uint64 seq
float32 uptime_sec
float32 fps_actual
uint64 frames_total
uint64 frames_dropped
bool link_ok
string device_id
int32 error_code
string error_msg
