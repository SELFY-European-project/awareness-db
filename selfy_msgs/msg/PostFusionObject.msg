# IDs of fused objects (all data available in raw_objects).
# May be a single ID if object is seen by a single sensor.
string[] object_ids

# Resulting object. 
# Remark: should we have a rule to let object empty in case
# we have a single ID in object_ids? (object should be the same
# as the one in raw_objects in this case)
Object object

# Cross-correlation data for this object.
ObjectCorrelation[] correlation
