# A representation of pose in free space, composed of position and orientation.

# This contains the sequence ID, timestamp and frame ID this data is associated with.
Header header

# This contains the 3D position of body in free space in Cartesian representation.
float64 pos_x
float64 pos_y
float64 pos_z

# This represents the 3D orientation of body in free space in Euler angle representation.
float64 rot_x
float64 rot_y
float64 rot_z
