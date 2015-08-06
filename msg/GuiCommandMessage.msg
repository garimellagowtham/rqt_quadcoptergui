# Gui Commands Message
bool command           # Status of the component
uint8 commponent_name    # Name of the component

uint8 enable_log = 0     # Enable logging of quad and controller data
uint8 enable_integrator = 1     # Enable or Disable Integrator for controller
uint8 traj_on = 2        # Follow a trajectory or not
uint8 enable_ctrl = 3    # Enable Position Controller
uint8 enable_joy = 4     # Enable Joy Stick
uint8 enable_cam = 5     # Enable Camera
uint8 arm_quad = 6       # Arm the Quadcopter
uint8 land_quad = 7      # Land the quadcopter
uint8 disarm_quad = 8    # Disarm Quadcopter
uint8 target_capture = 9 # Capture the target object
