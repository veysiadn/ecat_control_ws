# The timestamp is the time at which data is received from the slaves.
std_msgs/Header header

uint8  com_status
uint8[]  slave_com_status
int32[]  actual_pos
int32[]  actual_vel
int16[]  actual_tor
uint16[] status_word
uint8[]  op_mode_display
uint16[] error_code
uint8  current_lifecycle_state
uint8  left_limit_switch_val
uint8  right_limit_switch_val
uint8  emergency_switch_val
uint32  period_ns
uint32  jitter_ns
uint32  exec_ns
