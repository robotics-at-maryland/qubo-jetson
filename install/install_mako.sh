#!/bin/bash
# clones a ROS package that encaspulates the AVT Vimba drivers,
# then moves it to the appropriate place in the catkin_ws
# installing the mako node

git clone git@github.com:astuff/avt_vimba_camera.git
mv avt_vimba_camera ../../
