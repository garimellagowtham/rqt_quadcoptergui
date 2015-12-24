# Gui Commands Message
bool command              # Status of the component
uint8 commponent_name     # Name of the component

uint8 enable_log = 0      # Enable logging of quad and controller data
uint8 enable_tracking = 1 # Enable logging of quad and controller data
uint8 enable_control = 2  # Enable Control of quad (request sdk control)
uint8 arm_quad = 6        # Arm the Quadcopter
uint8 land_quad = 7       # Land the quadcopter
uint8 disarm_quad = 8     # Disarm Quadcopter
