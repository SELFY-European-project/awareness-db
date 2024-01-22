# Unique identifier for considered sensors. 
string          sensor1
string          sensor2

# Global correlation score
float32         global_score

# Percent of candidate objects over all objects (candidate = successful + failed fusion) : how similar the sensors perception fields are
float32         candidate_ratio

# Percent of sucessful fusion over candidate objects : how consistent the sensors are
float32         successful_fusion_ratio

# Mean correlation score for merged objects : how well calibrated the sensors are
float32         mean_matching_score
