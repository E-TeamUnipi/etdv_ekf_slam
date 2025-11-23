FROM ros:humble

# Install dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-foxglove-bridge \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Ensure bash is the default shell
SHELL ["/bin/bash", "-c"]

# Create workspace
WORKDIR /ros2_ws/src

# Copy only the package directories (not Dockerfile, docker-compose, etc)
COPY pacsim/ ./pacsim/
COPY circular_controller/ ./circular_controller/
# Se hai altri pacchetti, aggiungili qui:
# COPY altro_pacchetto/ ./altro_pacchetto/

WORKDIR /ros2_ws
COPY run.sh ./run.sh

# Build
RUN source /opt/ros/humble/setup.bash && colcon build

# Persistent environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]