# This is a message that holds data to describe the control input of a set of joints.
# The state of each joint is defined by:
#  * the ID of the joint,
#  * the position of the joint (rad),
#  * the speed of the joint (rad/s),
#  * the accelerate of the joint (rad/s^2).
#  * the torque enable of the joint (0: unloading torque or 1: loading torque),
# Each joint is uniquely identified by its ID.
# This message consists of a multiple arrays, one for each part of the control input.
# All arrays in this message should have the same size, or be empty.

std_msgs/Header header
uint32 seq

uint8[] id

float64[] pos
float64[] speed
float64[] acc
bool[] torque