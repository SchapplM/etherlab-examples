#!/bin/bash -e
# ROS-Workspace kompilieren

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03  
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
cd catkin_ws
catkin_make install
