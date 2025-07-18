# UAVState.msg - Current state of the UAV finite state machine

# State definitions
uint8 IDLE = 0
uint8 TAKEOFF = 1
uint8 ACTION_IN_PROGRESS = 2
uint8 HOVER = 3
uint8 LANDING = 4
uint8 EMERGENCY = 5

# Current state
uint8 state

# State-specific information
string state_description
builtin_interfaces/Time state_start_time
float64 state_duration
geometry_msgs/Point target_position
