# Gui State Message
bool status                          # Status of the component
uint8 commponent_id                  # Name of the component

uint8 log_status = 0                 # Status of logging of quad and controller data
uint8 tracking_status = 1            # Status of quadcopter tracking
uint8 control_status = 2             # Enable Control of quad (request sdk control)
