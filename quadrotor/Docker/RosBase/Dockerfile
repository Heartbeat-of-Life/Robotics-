#locale  # check for UTF-8
FROM ubuntu:focal
RUN apt-get update &&  apt-get install -y tzdata gnupg2 zsh gawk vim mesa-utils sudo wget
RUN apt-get update &&  apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale  # verify settings
RUN DEBIAN_FRONTEND=noninteractive apt-get update &&  apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-ros-base
RUN apt-get install -y python3-pip && pip3 install -U argcomplete
COPY ros-entrypoint.sh /usr/bin
RUN chmod a+x /usr/bin/ros-entrypoint.sh
ENTRYPOINT ["/usr/bin/ros-entrypoint.sh"]
