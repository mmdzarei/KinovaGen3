# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables to avoid interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install necessary packages and ROS Noetic
RUN apt-get update && apt-get install -y \
    gnupg2 \
    lsb-release \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    && apt-get clean && rm -rf /var/lib/apt/lists/

RUN apt-get update && apt-get install -y v4l-utils

RUN apt install -y ffmpeg

RUN apt-get install -y python3-rosdep
RUN apt-get install -y python3-rospy
# Initialize ROS
RUN rosdep init && rosdep update

# Source ROS setup.bash in every shell session
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create a workspace directory
RUN mkdir -p /catkin_ws/src

# Set the default working directory
# WORKDIR /catkin_ws


# for kinova
RUN apt install -y python3 python3-pip && \
apt install -y git && \
apt-get install ros-noetic-catkin && \
echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
python3 -m pip install conan==1.59  && \
conan config set general.revisions_enabled=1  && \
conan profile new default --detect > /dev/null && \
conan profile update settings.compiler.libcxx=libstdc++11 default
# RUN mkdir -p catkin_workspace/src
WORKDIR /catkin_ws/src
RUN git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
WORKDIR /catkin_ws
RUN apt install wget
# RUN sh \
#     -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
#         > /etc/apt/sources.list.d/ros-latest.list'
# RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# RUN apt-get -y update
RUN apt-get -y install python3-catkin-tools
RUN pip3 install -U catkin_tools
# WORKDIR /root/catkin_ws
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
# catkin_make"
# # # RUN ["/bin/bash", "-c", "source /devel/setup.bash"]
# # # Source the ROS environment setup file
# # RUN /bin/bash -c "source /path/to/catkin_ws/devel/setup.bash"
# RUN /bin/bash -c "source catkin_ws/devel/setup.bash"

RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash"]

RUN rosdep update
# RUN export ROS_DISTRO=noetic

# RUN rosdep install --from-paths /catkin_ws/src --ignore-src -y
RUN rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro noetic -y

# RUN echo "source devel/setup.bash" >> /root/.bashrc

# RUN ["/bin/bash", "-c", "source devel/setup.bash"]

WORKDIR /catkin_ws
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# RUN catkin_make

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \ 
catkin_make "


# Set the entry point script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# CMD ["/bin/bash", "source catkin_ws/devel/setup.bash"]

