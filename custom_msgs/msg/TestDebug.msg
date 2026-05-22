std_msgs/Header header
float64 sample_time

float64[2] raw_p
float64[2] raw_v
float64[2] raw_a
float64[2] estimated_p
float64[2] estimated_v
float64[2] estimated_a
float64 dt

bool has_raw
bool estimator_initialized
bool detected
bool tracked
bool predicted_only

bool has_control
float64 u_yaw
float64 u_pitch
bool fire
bool reload

string tracking_mode
string control_source
bool is_pixel
bool webcam_enabled
float64 picam_age
float64 webcam_age
float64 source_age
