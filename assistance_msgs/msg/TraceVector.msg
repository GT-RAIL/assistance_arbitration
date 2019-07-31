# A message type for a trace vector. Execution and Intervention traces can be
# converted to consistent (but different from each other) vector representations
# that can be shared with the rest of the ROS system if required

# The timestamp of the vector
time stamp

# Float vector for the data
float64[] data

# A string array (optional) corresponding to the data fields in the vector. It
# can be useful to populate this to help with parsing trace vectors, but it is
# not necessary
string[] fields
