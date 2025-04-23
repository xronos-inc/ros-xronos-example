# Xronos/ROS 2 interoperability

The Xronos and ROS 2 frameworks can coexist within the same process, and
programs that primarily use Xronos can integrate in larger composition of ROS
nodes by delegating interprocess communication to `rclcpp`.

The `without-xronos` directory shows sample code closely based on [this
tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) that simply demonstrates
the tutorial running in Docker without Xronos.

The `with-xronos` directory shows sample code that lets `xronos` and `rclcpp`
coexist within the same process, delegating interprocess communication to
`rclcpp`.

## License

This example is based on an example from [the ROS 2 documentation](https://github.com/ros2/ros2_documentation/blob/jazzy/LICENSE), which is licensed under the Creative Commons Attribution 4.0 International license.

Changes are licensed under the BSD 3-clause license.
