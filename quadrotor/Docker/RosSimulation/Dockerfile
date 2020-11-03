FROM ros:base
RUN DEBIAN_FRONTEND=noninteractive apt-get update  
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-desktop-full freecad netgen
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/netgen
