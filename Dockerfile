FROM ros:melodic

SHELL ["/bin/bash", "-c"]

# Install git
RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
		cmake g++ python3-pip wget git \
        python3-colcon-common-extensions \
        python3-catkin-tools

# Install latest asio
RUN wget http://launchpadlibrarian.net/714460716/libasio-dev_1.28.1-0.2_all.deb && \
    dpkg -i libasio-dev_1.28.1-0.2_all.deb

# Install latest cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v4.1.1/cmake-4.1.1-linux-x86_64.tar.gz && \
    tar -zxvf cmake-4.1.1-linux-x86_64.tar.gz && \
    mv cmake-4.1.1-linux-x86_64 /opt/cmake-4.1.1 && \
    ln -sf /opt/cmake-4.1.1/bin/* /usr/bin/

# Install Fast-DDS from source
RUN mkdir -p ./Fast-DDS/src && cd ./Fast-DDS/src \
    # Clone Repos
    && git clone https://github.com/eProsima/foonathan_memory_vendor.git \
    && git clone https://github.com/eProsima/Fast-CDR.git fastcdr \
    && git clone https://github.com/eProsima/Fast-DDS.git fastdds \
    # Build using colcon
    && cd .. \
    && colcon build --packages-up-to fastdds

# Install Fast-DDS-Gen from source
RUN apt install openjdk-11-jdk -y && \
    cd /Fast-DDS/src && \
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen && \
    cd fastddsgen && \
    ./gradlew assemble 

## Copy the entire repository into a catkin workspace so we can build a ROS1 package
COPY . /catkin_ws/src/ros_dds_bridge

# Setup environment, create workspace, generate IDL sources (if available), and build
RUN if [ -f /Fast-DDS/install/setup.bash ]; then . /Fast-DDS/install/setup.bash; fi && \
    if [ -f /opt/ros/melodic/setup.bash ]; then . /opt/ros/melodic/setup.bash; fi && \
    mkdir -p /catkin_ws && \
    cd /catkin_ws/src/ros_dds_bridge && \
    mv ./types ./src && \
    cd ./src && \
    if [ -f /Fast-DDS/src/fastddsgen/scripts/fastddsgen ]; then /Fast-DDS/src/fastddsgen/scripts/fastddsgen /catkin_ws/src/ros_dds_bridge/src/types/geometry_msgs/PoseWithCovarianceStamped.idl || true; else echo "fastddsgen not found; skipping IDL generation"; fi && \
    cd /catkin_ws && \
    catkin build -DCMAKE_POLICY_VERSION_MINIMUM=3.10

# Safe entrypoint: attempt to source ROS and Fast-DDS setup files if they exist,
# but don't fail the container if they are missing. Then start an interactive bash.
ENTRYPOINT ["/bin/bash", "-lc", "if [ -f /opt/ros/melodic/setup.bash ]; then source /opt/ros/melodic/setup.bash; fi; if [ -f /Fast-DDS/install/setup.bash ]; then source /Fast-DDS/install/setup.bash; fi; exec /bin/bash"]