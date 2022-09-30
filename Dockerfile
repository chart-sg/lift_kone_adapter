FROM ghcr.io/sharp-rmf/rmf-deployment/builder-rmf
ARG GIT_VER=main
WORKDIR /opt/rmf/src

SHELL ["/bin/bash", "-c"]
RUN echo 'Checking out GIT version of repo:' {$GIT_VER}
RUN git clone https://github.com/sharp-rmf/kone-ros-api.git && cd kone-ros-api && git checkout $GIT_VER

WORKDIR /opt/rmf

RUN apt-get update && apt-get install -y python3-pip && \
  pip3 install websocket websockets websocket-client requests

# install
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  apt-get update && rosdep install -y \
  --from-paths \
  src/kone-ros-api \
  --ignore-src \
  && rm -rf /var/lib/apt/lists/*

# Build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  colcon build --packages-select kone_ros_api

RUN sed -i '$iros2 run kone_ros_api koneNode_v2' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
