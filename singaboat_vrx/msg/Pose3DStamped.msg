# A representation of pose in free space, composed of position and orientation.

# This contains the sequence ID, timestamp and frame ID this data is associated with.
Header header

# This contains the 3D pose (2D position + 1D orientation) of body in free space.
float64 pos_x
float64 pos_y
float64 rot_z
