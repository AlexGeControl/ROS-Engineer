FROM ubuntu:18.04

# ------ PART 0: set environment variables ------

# set up environment:
ENV DEBIAN_FRONTEND noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# ------ PART 1: set up CN sources ------

# Ubuntu:
COPY ${PWD}/image/etc/apt/sources.list /etc/apt/sources.list
RUN rm -f /etc/apt/sources.list.d/*

# Python: 
COPY ${PWD}/image/etc/pip.conf /root/.pip/pip.conf

# ------ PART 2: set up apt-fast -- NEED PROXY DUE TO UNSTABLE CN CONNECTION ------

# install:
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        # PPA utilities:
        software-properties-common \
        # certificates management:
        dirmngr gnupg2 \
        # download utilities:
        axel aria2 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 1EE2FF37CA8DA16B && \
    add-apt-repository ppa:apt-fast/stable && \
    apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated apt-fast && \
    rm -rf /var/lib/apt/lists/*

# CN config:
COPY ${PWD}/image/etc/apt-fast.conf /etc/apt-fast.conf

# ------ PART 3: add external repositories ------

# ROS:
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
COPY ${PWD}/image/etc/apt/sources.list.d/ /etc/apt/sources.list.d/

# libsparse:
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

# ------ PART 4: install packages ------

RUN apt-fast update --fix-missing && \
    apt-fast install -y --no-install-recommends --allow-unauthenticated \
        # package utils:
        sudo dpkg pkg-config \
        # security:
        openssh-server pwgen ca-certificates \
        # network utils:
        curl wget iputils-ping net-tools \
        # command line:
        vim grep sed patch \
        # io:
        pv zip unzip bzip2 \
        # version control:
        git mercurial subversion \
        # daemon & services:
        supervisor nginx \
        # potential image & rich text IO:
        lxde \
        xvfb dbus-x11 x11-utils libxext6 libsm6 x11vnc \
        gtk2-engines-pixbuf gtk2-engines-murrine pinta ttf-ubuntu-font-family \
        mesa-utils libgl1-mesa-dri libxrender1 \
        # c++:
        gcc g++ cmake build-essential libglib2.0-dev  \
        # python 2:
        python-pip python-dev python-tk \
        # ROS melodic:
        python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
        ninja-build \
        ros-melodic-desktop-full \
        ros-melodic-rosbridge-server \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-ecl-threads \
        # common:
        # a. numerical optimization -- integer programming:
        coinor-libcoinutils-dev \
        coinor-libcbc-dev \
        # a. numerical optimization -- eigen3
        libeigen3-dev \
        # a. numerical optimization -- ceres:
        libdw-dev libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
        # a. numerical optimization -- ipopt:
        gfortran liblapack-dev libmetis-dev \
        # b. image:
        libsdl1.2-dev \
        libsdl-image1.2-dev \
        # c. point-cloud:
        libpcl-dev \
        # imu:
        ros-melodic-imu-complementary-filter ros-melodic-imu-filter-madgwick ros-melodic-rviz-imu-plugin \
        # lidar:
        ros-melodic-sick-scan \
        ros-melodic-laser-pipeline \
        # localization:
        ros-melodic-amcl \
        # localization fusion:
        ros-melodic-robot-localization \
        # slam:
        # a. gmapping:
        ros-melodic-openslam-gmapping \
        ros-melodic-libg2o \
        # b. cartographer:
        lua5.3 liblua5.3-dev libluabind-dev \
        # perception:
        ros-melodic-opengm \
        ros-melodic-libdlib \
        # navigation:
        ros-melodic-costmap-2d \
        ros-melodic-move-base \
        ros-melodic-global-planner \
        ros-melodic-move-base-msgs \
        ros-melodic-cob-map-accessibility-analysis \
        # GUI tools:
        gnome-themes-standard \
        terminator \
        firefox \
        # ROS bag visualizer:
        ros-melodic-plotjuggler \ 
        # ROS map editor:
        gimp && \
    apt-fast autoclean && \
    apt-fast autoremove && \
    rm -rf /var/lib/apt/lists/*

# ------ PART 5: offline installers ------

# load installers:
COPY ${PWD}/installers /tmp/installers
WORKDIR /tmp/installers

# install ceres:
RUN tar zxf ceres-solver-1.14.0.tar.gz && \
    mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver-1.14.0 && \
    make -j8 && make test && make install

# install CppAD:
RUN git clone https://github.com/coin-or/CppAD.git -o CppAD && cd CppAD && \
    mkdir build && cd build && \
    # config:
    cmake .. && make -j8 check && \
    # install:
    make install

# install ipopt:
RUN chmod u+x coinbrew && \
    ./coinbrew fetch Ipopt --no-prompt && \
    ./coinbrew build Ipopt --prefix=/usr/local --test --no-prompt --parallel-jobs=8 --verbosity=3 && \
    ./coinbrew install Ipopt --no-prompt

# install tini:
RUN dpkg -i tini.deb && \
    apt-get clean

RUN rm -rf /tmp/installers

# ------ PART 7: set up VNC servers ------

COPY image /

WORKDIR /usr/lib/

RUN git clone https://github.com/novnc/noVNC.git -o noVNC

WORKDIR /usr/lib/noVNC/utils

RUN git clone https://github.com/novnc/websockify.git -o websockify

WORKDIR /usr/lib/webportal

RUN pip install --upgrade pip && pip install -r requirements.txt

EXPOSE 80 5901 9001

# ------ PART 7: set up ROS environments ------

# activate ros environment:
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# initialize rosdep
# 
# NOTE:
# be careful about DNS resolution problem caused by https://raw.githubusercontent.com
# get the latest IP address of the site from Baidu and Google search engine
# 
RUN rosdep fix-permissions && \
    rosdep init && \
    rosdep update

# for ROS:
EXPOSE 11311

# ------------------ DONE -----------------------
WORKDIR /workspace

ENTRYPOINT ["/startup.sh"]
