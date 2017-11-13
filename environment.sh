#!/bin/bash

echo "Activating ROS..."
source /opt/ros/kinetic/setup.bash
echo "...done."

echo "Setting up PYTHONPATH."
export PYTHONPATH=/home/ubuntu/multi_robot_patrolling/catkin_ws/src:$PYTHONPATH

echo "Setup ROS_HOSTNAME."
export ROS_HOSTNAME=$HOSTNAME.local
export PATROLLING_ROOT=$HOME/multi_robot_patrolling

echo "Building machines file..."
make -C  $PATROLLING_ROOT
echo "...done"
echo "Activating development."
source $PATROLLING_ROOT/catkin_ws/devel/setup.bash

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.

