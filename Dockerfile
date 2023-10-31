FROM osrf/ros:iron-desktop-full

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip

COPY run_docker.bash run_docker.bash
COPY my_bot car_ws/src/my_bot

RUN echo "done!"