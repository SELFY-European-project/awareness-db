# Unique identifier for considered sensor. 
string                  id

# Type of considered sensor.
# These are samples values that we may change.
int8 CAMERA = 0
int8 LIDAR  = 1
int8 RADAR  = 2
int8 CITS   = 3
int8 GROUND_TRUTH = 4
int8                    type

# Position of sensor.
geometry_msgs/Pose      pose

# Perception region expressed as a list of points.
geometry_msgs/Point[]   perception_region_shape
