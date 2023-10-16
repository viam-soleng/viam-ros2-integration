#
# example file to be used in the /etc/viam/setup.bash file
#

# source required ROS variables
# this exposes all variables to viam module process
. /etc/turtlebot4/setup.bash
. /opt/ros/humble/setup.bash

# set environment variables as needed
export VIAM_NODE_NAME="viam"
export VIAM_ROS_NAMESPACE=""