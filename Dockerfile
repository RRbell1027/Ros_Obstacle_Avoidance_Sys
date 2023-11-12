FROM osrf/ros:iron-desktop-full

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip

COPY my_bot car_ws/src/my_bot

RUN apt-get install -y ros-iron-gazebo-ros-pkgs
RUN apt-get install -y ros-iron-joint-state-publisher-gui
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/.bashrc

RUN echo "done!"
