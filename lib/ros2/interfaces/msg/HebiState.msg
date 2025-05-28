# HEBI Actuator State Message
# Used to record feedback from HEBI actuators

# Header (w/ timestamp)
std_msgs/Header header

# Actuator IDs
string[] families
string[] names

# Position feedback (deg)
float64[] target_pos
float64[] actual_pos

# Velocity feedback (deg/s)
float64[] target_vel
float64[] actual_vel

# Effort/Torque feedback (Nm)
float64[] target_trq
float64[] actual_trq

# Additional sensor data
float64[] deflection      # deg
float64[] deflection_vel  # deg/s

float64[] voltage  # V
float64[] current  # A

float64[] motor_temp  # C
