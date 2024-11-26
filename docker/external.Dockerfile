FROM ros:humble

# Enable Nvidia Driver Capabilities
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display

RUN apt-get update
RUN apt-get install vim libeigen3-dev wget ros-humble-rviz2 -y

RUN wget https://github.com/google/or-tools/releases/download/v9.10/or-tools_amd64_ubuntu-22.04_cpp_v9.10.4067.tar.gz
RUN tar -xvzf or-tools_amd64_ubuntu-22.04_cpp_v9.10.4067.tar.gz
RUN rm or-tools_amd64_ubuntu-22.04_cpp_v9.10.4067.tar.gz
WORKDIR /or-tools_x86_64_Ubuntu-22.04_cpp_v9.10.4067
RUN make test

WORKDIR /inspection_ws
