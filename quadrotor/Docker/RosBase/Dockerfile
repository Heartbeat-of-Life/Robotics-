#locale  # check for UTF-8
FROM ubuntu:focal
RUN apt-get update &&  apt-get install -y tzdata gnupg2 zsh gawk vim mesa-utils sudo wget
RUN apt-get update &&  apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale  # verify settings

RUN apt-get update &&  apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |  apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN mkdir /opt/ros2_foxy && cd /opt/ros2_foxy && wget https://github.com/ros2/ros2/releases/download/release-foxy-20201009/ros2-foxy-20201009-linux-focal-arm64.tar.bz2  && tar xf ros2-foxy-20201009-linux-focal-arm64.tar.bz2

#RUN mkdir /opt/ros2_foxy && cd /opt/ros2_foxy && tar xf ros2-foxy-20201009-linux-focal-arm64.tar.bz2
RUN apt-get update
RUN  apt-get install -y python3-rosdep
RUN rosdep init
RUN rosdep update
RUN apt-get install -y git 
COPY ros-entrypoint.sh /usr/bin
RUN cd /opt/ros2_foxy && rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
RUN apt-get install -y libpython3-dev python3-pip
RUN pip3 install -U argcomplete
RUN chmod a+x /usr/bin/ros-entrypoint.sh
ENTRYPOINT ["/usr/bin/ros-entrypoint.sh"]
