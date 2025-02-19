# LuffingJibCraneMsg.msg
# This message represents a single luffing jib crane.
# The parameters are:
# 0: crane_id (float64) - Unique identifier for the crane.
# 1: crane_x (float64) - X-coordinate of the crane's base.
# 2: crane_y (float64) - Y-coordinate of the crane's base.
# 3: crane_z (float64) - Z-coordinate of the crane's tower.
# 4: boom_length (float64) - Length of the crane's boom.
# 5: boom_horAngle (float64) - Horizontal angle of the crane's boom (in radians).
# 6: boom_verAngle (float64) - Vertical angle of the crane's boom (in radians).
# 7: hook_height (float64) - Height of the crane's hook.

uint8 crane_id
string crane_type
float64 crane_x
float64 crane_y
float64 crane_z
float64 boom_length
float64 boom_hor_angle
float64 boom_ver_angle
float64 hook_height

