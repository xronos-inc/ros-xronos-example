FROM ros:jazzy

WORKDIR /root/ros2_ws/src

COPY . publisher/

WORKDIR /root/ros2_ws/

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=build source /opt/ros/jazzy/setup.bash && \
  rosdep install -i --from-path src --rosdistro jazzy -y && \
  colcon build --packages-select ros_xronos_example_publisher

COPY ./run.sh run.sh

ENTRYPOINT [ "./run.sh" ]
