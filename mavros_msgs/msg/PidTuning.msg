# PID tuning information.
#
# See:
# https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING

std_msgs/Header header

uint8 axis
float32 desired
float32 achieved
float32 ff
float32 p
float32 i
float32 d
