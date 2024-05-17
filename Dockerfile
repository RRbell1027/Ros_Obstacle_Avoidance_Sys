FROM osrf/ros:iron-desktop

RUN apt-get update

# RUN apt-get install -y 
RUN apt-get install -y ros-iron-joint-state-publisher-gui \
                       ros-iron-gazebo-ros-pkgs \
                       ros-iron-rplidar-ros \
                       ros-iron-xacro \
                       ifconfig \
                       vim
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/.bashrc

RUN echo "done!"
