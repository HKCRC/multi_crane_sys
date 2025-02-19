# TowerCraneMsg.msg
# This message represents a single tower crane.
# The parameters are:
# 0: crane_id (float64) - Unique identifier for the crane.
# 1: crane_x (float64) - X-coordinate of the crane's base.
# 2: crane_y (float64) - Y-coordinate of the crane's base.
# 3: crane_z (float64) - Z-coordinate of the crane's tower.
# 4: boom_length (float64) - Length of the crane's boom.
# 5: boom_angle (float64) - Horizontal angle of the crane's boom (in radians).
# 6: trolley_radius (float64) - Trolley radius for the tower crane.
# 7: hook_height (float64) - Height of the crane's hook.

uint8 crane_id
string crane_type
float64 crane_x
float64 crane_y
float64 crane_z
float64 boom_length
float64 boom_angle
float64 trolley_radius
float64 hook_height
