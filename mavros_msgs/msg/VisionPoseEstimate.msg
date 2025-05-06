# Vision Position Estimate with reset capabilities
#
# See:
# https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE

std_msgs/Header header

geometry_msgs/PoseStamped pose
uint8 reset_counter
