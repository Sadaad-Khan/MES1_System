# CAN bus health status
builtin_interfaces/Time timestamp
string interface_name
string state                # UP/DOWN/ERROR_ACTIVE/ERROR_PASSIVE/BUS_OFF
uint32 tx_errors
uint32 rx_errors
float32 bus_load
bool device_connected
