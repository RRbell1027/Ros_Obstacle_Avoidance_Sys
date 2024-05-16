FROM osrf/ros:iron-desktop

RUN apt-get update

# RUN apt-get install -y ros-iron-gazebo-ros-pkgs
RUN apt-get install -y ros-iron-joint-state-publisher-gui
RUN apt-get install -y vim
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/.bashrc

RUN echo "done!"
