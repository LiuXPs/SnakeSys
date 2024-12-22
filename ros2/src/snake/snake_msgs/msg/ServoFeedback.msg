# This is a message that holds data to describe the state of a set of joints.
# The state of each joint is defined by:
#  * the ID of the joint,
#  * the position of the joint (rad),
#  * the speed of the joint (rad/s),
#  * the load of the joint (Nm),
#  * the voltage of the joint (V),
#  * the temperature of the joint (Celsius degree)
#  * the move state of the joint (0 or 1),
#  * the current of the joint (A).
# Each joint is uniquely identified by its ID.
# This message consists of a multiple arrays, one for each part of the joint state.
# All arrays in this message should have the same size, or be empty.

std_msgs/Header header
uint32 seq

uint8[] id

float64[] position
float64[] speed
float64[] load
float64[] voltage
float64[] temperature
float64[] move
float64[] current
