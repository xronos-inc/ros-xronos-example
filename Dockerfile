FROM ros:jazzy

WORKDIR /
RUN git clone --recurse-submodules --depth 1 https://github.com/xronos-inc/xronos.git

WORKDIR /xronos/cpp-sdk

RUN cmake -B build
RUN cmake --build build --target xronos-sdk -j$(nproc)

WORKDIR /root/ros2_ws/src

COPY publisher/ publisher/

WORKDIR /root/ros2_ws/

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/jazzy/setup.bash && \
  rosdep install -i --from-path src --rosdistro jazzy -y && \
  colcon build --packages-select ros_xronos_example_publisher || \
  echo "build failed"
