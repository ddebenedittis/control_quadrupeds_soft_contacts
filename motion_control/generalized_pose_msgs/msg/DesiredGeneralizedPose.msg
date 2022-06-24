# A representation of all the quantities that define the desired pose of the robot base and of the swing feet, called desired generalized pose.

# Base linear quantities
geometry_msgs/Vector3 base_acc
geometry_msgs/Vector3 base_vel
geometry_msgs/Vector3 base_pos

# Base angular quantities
geometry_msgs/Vector3 base_angvel
geometry_msgs/Quaternion base_quat

# Swing feet linear quantities
float64[] feet_acc
float64[] feet_vel
float64[] feet_pos

# List of feet names in contact with the ground
string[] contact_feet