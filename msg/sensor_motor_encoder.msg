#
#  Motor encoder topic
#
uint64 timestamp					# time since system start (microseconds)
uint8 motor_id
float32 motor_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 motor_abs_angle
# int64 multi_turns
