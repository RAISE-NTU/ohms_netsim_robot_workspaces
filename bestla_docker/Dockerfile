# Use the official OSRF ROS2 Humble desktop image as the base image
FROM osrf/ros:humble-desktop

# Switch to bash shell for consistency with ROS setup sourcing
SHELL ["/bin/bash", "-c"]

# 1. Update packages and install necessary utilities (including python3-pip)
RUN apt-get update && \
    apt-get install -y nano curl zip unzip tar iproute2 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# 2. Clone and set up vcpkg
WORKDIR /root
RUN git clone https://github.com/Microsoft/vcpkg.git && \
    cd vcpkg && \
    ./bootstrap-vcpkg.sh && \
    ./vcpkg integrate install && \
    ./vcpkg install octomap

# 3. Install ROS Humble octomap packages
RUN apt-get update && \
    apt-get install -y ros-humble-octomap* && \
    rm -rf /var/lib/apt/lists/*

# 4. Install rotop using pip
RUN pip3 install rotop

# 5. Source ROS setup 
RUN source /opt/ros/humble/setup.bash

# 6. Copy the atlas_ws folder into the container
COPY bestla_ws /bestla_ws

# 7. Set the default command to an interactive bash shell
CMD ["bash"]

