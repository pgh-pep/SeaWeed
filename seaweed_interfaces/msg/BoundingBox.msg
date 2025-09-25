# Single detected object
string class_name          # Human-readable class name (e.g., "person", "car")
int32 class_id            # Class ID from model
float64 confidence        # Detection confidence score [0.0, 1.0]

# Bounding box coordinates (in pixels)
float64 x                 # Top-left x coordinate
float64 y                 # Top-left y coordinate
float64 width             # Bounding box width
float64 height            # Bounding box height

# Optional: normalized coordinates [0.0, 1.0]
float64 x_normalized      # Normalized x center
float64 y_normalized      # Normalized y center
float64 width_normalized  # Normalized width
float64 height_normalized # Normalized height
