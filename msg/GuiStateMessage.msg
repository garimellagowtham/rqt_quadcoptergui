# Gui State Message
bool status                          # Status of the component
uint8 commponent_id                  # Name of the component

uint8 log_status = 0                 # Status of logging of quad and controller data
uint8 integrator_status = 1        # Status of Integrator for controller
uint8 trajectory_tracking_status = 2 # Status of Trajectory tracking controller
uint8 controller_status = 3          # Status of Controller
uint8 joystick_status = 4            # Status of Joystick
uint8 camera_controlstatus = 5       # Status of camera
