FROM osrf/ros:iron-desktop-full

RUN apt-get update && apt-get clean
RUN apt-get install -y git && apt-get install -y python3-pip

RUN mkdir -p /car_ws/src/
RUN git clone https://github.com/joshnewans/articubot_one.git /car_ws/src/
COPY my_avoid_sys car_ws/src

RUN apt-get install -y ros-iron-gazebo-ros-pkgs
RUN apt-get install -y ros-iron-joint-state-publisher-gui
RUN apt-get install -y ros-iron-slam-toolbox

RUN echo ". /opt/ros/iron/setup.bash" >> /etc/.bashrc

RUN echo "done!"