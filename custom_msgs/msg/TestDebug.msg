std_msgs/Header header
float64 sample_time

float64[2] raw_p
float64[2] raw_v
float64[2] estimated_p
float64[2] estimated_v
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
