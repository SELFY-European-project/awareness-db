# Contains the reference time for the positions of the objects.
# (but which time is it exactly?)
# (each object has its own measurement time)
Header              header

# List of sensors used to obtain this objects listing.
Sensor[]            sensors

# List of raw objects from all sensors.
Object[]            raw_objects

# List of objects following fusion.
PostFusionObject[]  fused_objects

# Details on failed fusions.
FailedFusion[]  failed_fusions

# Cross-correlation between sensors.
SensorCorrelation[] sensors_correlation
