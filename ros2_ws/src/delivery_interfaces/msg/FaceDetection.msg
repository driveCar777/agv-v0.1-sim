std_msgs/Header header
string camera_name
# person_id empty if unknown / stranger
string person_id
string person_name
float32 confidence
# bbox in pixels: x, y, w, h
float32[] bbox_xywh
# optional landmarks flattened [x0,y0,x1,y1,...]
float32[] landmarks_2d
bool liveness_ok
string track_id
