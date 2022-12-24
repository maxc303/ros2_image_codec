# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM osrf/ros:humble-desktop-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install other tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    clang-format \
    clang-tidy \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# install ffmpeg libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    libavutil-dev \
    libavcodec-dev \
    libavformat-dev \
    && rm -rf /var/lib/apt/lists/*

# test tool
RUN git clone https://github.com/catchorg/Catch2.git && \
    cd Catch2 && \
    cmake -Bbuild -H. -DBUILD_TESTING=OFF && \
    sudo cmake --build build/ --target install