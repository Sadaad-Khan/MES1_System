# Standard CAN frame
builtin_interfaces/Time timestamp
uint32 can_id
bool is_extended
bool is_error_frame
bool is_remote_frame
uint8 dlc
uint8[8] data
