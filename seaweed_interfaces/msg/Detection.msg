# Array of object detections from YOLO inference
std_msgs/Header header               # Timestamp and frame information

# Image information
uint32 image_width                   # Source image width in pixels
uint32 image_height                  # Source image height in pixels

# Detection results
BoundingBox[] detections            # Array of detected objects with bounding boxes
float64 inference_time              # Time taken for inference (seconds)
