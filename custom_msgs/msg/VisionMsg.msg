std_msgs/Header header

# Boolean
bool detected
bool tracked

# State value pixel (x, y)
float32[] p
float32[] v
float32[] a

# bbox width and height
float32[] bbox

# Precision, validity
float32 confidence
float32[] covariance

# Current Vision Mode
string source_mode
