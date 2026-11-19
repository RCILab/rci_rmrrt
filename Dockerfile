# RMRRT on ROS Noetic with MoveIt, the robot packages of the scenarios and RViz.
#   docker build -t rmrrt .                         # JOBS: parallel compile jobs, about 1 GB of memory each
#   docker build -t rmrrt --build-arg JOBS=8 .
#   docker run -it --rm rmrrt
FROM ros:noetic

ARG JOBS=2
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-catkin-tools python3-vcstool \
      ros-noetic-moveit ros-noetic-franka-description ros-noetic-xacro ros-noetic-rviz \
      libyaml-cpp-dev libboost-program-options-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/rmrrt_ws
COPY . src/rci_rmrrt
RUN vcs import src < src/rci_rmrrt/dependencies.repos \
    && source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status -j${JOBS} \
    && echo "source /root/rmrrt_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
