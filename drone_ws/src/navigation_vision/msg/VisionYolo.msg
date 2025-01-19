# Header for timestamp and frame information
std_msgs/Header header

# Number of detections
uint32 num_detections

# Detection boxes [x, y, width, height] for each object
Box[] boxes           # Array of boxes, each containing [x,y,w,h]

# Class information
int32[] classes          # class IDs for each detection
float32[] confidences    # confidence scores for each detection (optional)

# Additional information
string[] class_names     # names of the classes (optional)
