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

# image center
float32[] img_center

# Precision, validity
float32 confidence
float32[] covariance

# Current Camera Type
string camera
