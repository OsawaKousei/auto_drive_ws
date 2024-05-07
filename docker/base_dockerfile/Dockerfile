FROM n622jwith/ros2_basic_base:latest

# gui tools
RUN apt-get update && \
    apt-get install -y libqt5gui5 && \
    rm -rf /var/lib/apt/lists/*
ENV QT_DEBUG_PLUGINS=1

# install rqt
RUN apt-get update &&\
    apt-get install -y ros-humble-rqt-* &&\
    rm -rf /var/lib/apt/lists/*

# install gazebo
RUN apt-get update &&\
    apt-get install -y lsb-release wget gnupg &&\
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null &&\
    apt-get update &&\
    apt-get install -y ignition-fortress &&\
    rm -rf /var/lib/apt/lists/*

# install ros-gazebo-bridge
RUN apt-get update &&\
    apt-get install -y ros-humble-ros-ign &&\
    rm -rf /var/lib/apt/lists/*

# install slamtoolbox
RUN apt-get update &&\
    apt-get install -y ros-humble-slam-toolbox &&\
    rm -rf /var/lib/apt/lists/*

# install nav2
RUN apt-get update &&\
    apt-get install -y ros-humble-navigation2 &&\
    apt-get install -y ros-humble-nav2-bringup &&\
    rm -rf /var/lib/apt/lists/*
