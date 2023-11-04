#!/bin/bash

# Source the ROS setup script and set up the environment
#source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
source /catkin_ws/install/setup.bash

# Run your ROS nodes or desired commands
addgroup --gid $GID $USER
useradd -m -s /bin/bash -u $UID -g $GID $USER
# runuser -u $USER /bin/bash
# source /catkin_ws/devel/setup.bash
echo "source /catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
runuser -u $USER /bin/bash
# exec su - $USER
# su - $USER -c "/bin/bash -c 'source /catkin_ws/devel/setup.bash; exec /bin/bash'"

# roslaunch kortex_gazebo spawn_kortex_robot.launch
# rosrun turtlesim turtlesim_node