# Unique identifier for considered object.
string                              id

# Optional identifier of the object in the case where
# the object is described in a CAM message (i.e. the vehicle
# is the emitting agent, describing itself).
string                              v2x_id

# Measurement time.
# In case of fusion, it may be an arbitrary time chosen by
# fusion algorithm based on raw measurements from sensors.
time                               measurement_time

# Classification of object (type and confidence).
Classification                      classification

# List of identifiers of sensors that have reported this object.
string[]                            reported_by

# Pose of the object with covariance for uncertainty.
geometry_msgs/PoseWithCovariance    pose_with_covariance

# Twist of the object with covariance for uncertainty.
geometry_msgs/TwistWithCovariance   twist_with_covariance

# Bounding box (full size)
geometry_msgs/Vector3                extent

# Helper: 3D oriented bounding box as a list of points.
geometry_msgs/Point[] 	            oriented_bb

