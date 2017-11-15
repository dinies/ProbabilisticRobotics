# thin_localizer #

Particle Filter based localizer

## Prerequisites

This package needs [ROS kinetic (Ubuntu 16.04)](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS indigo (Ubuntu 14.04)](http://wiki.ros.org/indigo/Installation/Ubuntu)

## Compilation

compile it with catkin.

	cd <catkin_workspace>/src
	ln -s <path-to-prob-rob-16-17>/applications/cpp/16_thin_localizer .
	cd ..
	catkin_make

## Test
launch the simulator from the test directory

       rosrun stage_ros stageros dis-B1-2011-09-27.world

launch the localizer, providing the map image

       rosrun thin_localizer thin_localizer_node dis-B1-2011-09-27.png

This will open a window showing the state of the localizer
move the robot with a joystick or some other teleoperation means, e.g.

     sudo apt-get install ros-kinetic-teleop-twist-keyboard
     rosrun teleop_twist_keyboard teleop_twist_keyboard.py

