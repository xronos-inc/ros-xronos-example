// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xronos/sdk.hh"

using namespace std::chrono_literals;

class MinimalPublisherRosNode : public rclcpp::Node {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

public:
  MinimalPublisherRosNode() : Node("minimal_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }
  void publish(std::string msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    RCLCPP_INFO(this->get_logger(), "ROS publisher node publishes: '%s'",
                message.data.c_str());
    this->publisher_->publish(message);
  }
};

class MinimalPublisher : public xronos::sdk::Reactor {
  using xronos::sdk::Reactor::Reactor;

  xronos::sdk::PeriodicTimer timer_{"timer", context(), 500ms};
  std::shared_ptr<MinimalPublisherRosNode> publisher_node_;
  int count_{0};

  // the following are for managing the lifecycle of the ROS node
  std::thread rclcpp_thread_;
  std::promise<void> shutdown_promise_;
  int argc_;
  char **argv_;

public:
  MinimalPublisher(std::string_view name, xronos::sdk::Context context,
                   int argc, char *argv[])
      : xronos::sdk::Reactor(name, context), argc_(argc), argv_(argv) {}

private:
  class OnStartupReaction : public xronos::sdk::Reaction<MinimalPublisher> {
    using xronos::sdk::Reaction<MinimalPublisher>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    void handler() final {
      try {
        rclcpp::init(self().argc_, self().argv_);
        auto publisher_node = std::make_shared<MinimalPublisherRosNode>();
        self().publisher_node_ = publisher_node;

        // Both the Xronos runtime and ROS 2 runtime involve
        // indefinitely-running API calls, so they are composed by starting
        // `rclcpp::spin` in a separate thread.
        std::shared_future<void> shutdown_future(
            self().shutdown_promise_.get_future());
        self().rclcpp_thread_ = std::thread([shutdown_future,
                                             publisher_node]() {
          rclcpp::spin_until_future_complete(publisher_node, shutdown_future);
          rclcpp::shutdown();
        });
      } catch (const std::exception &e) {
        std::cerr << "Error creating ROS 2 node: " << e.what() << std::endl;
        return;
      }
      std::cout << "ROS 2 started." << std::endl;
    }
  };
  class OnShutdownReaction : public xronos::sdk::Reaction<MinimalPublisher> {
    using xronos::sdk::Reaction<MinimalPublisher>::Reaction;
    Trigger<void> shutdown_trigger{self().shutdown(), context()};
    void handler() final {
      std::cout << "Shutting down ROS 2..." << std::endl;
      self().shutdown_promise_.set_value();
      self().rclcpp_thread_.join();
      std::cout << "ROS 2 shutdown complete." << std::endl;
    }
  };
  class OnTimerReaction : public xronos::sdk::Reaction<MinimalPublisher> {
    using xronos::sdk::Reaction<MinimalPublisher>::Reaction;

    Trigger<void> timer_trigger{self().timer_, context()};

    void handler() final {
      std::cout << "Xronos runtime passes " << self().count_
                << " to ROS publisher at "
                   "time "
                << self().get_time() << std::endl;
      self().publisher_node_->publish("Hello, world! " +
                                      std::to_string(self().count_++));
    }
  };

  void assemble() final {
    // startup reaction must be added first so that rclcpp is initialized
    // before handling the timer event at time zero
    add_reaction<OnStartupReaction>("on_startup");
    add_reaction<OnTimerReaction>("on_timer");
    add_reaction<OnShutdownReaction>("on_shutdown");
  }
};

auto main(int argc, char *argv[]) -> int {
  xronos::sdk::Environment env{};
  MinimalPublisher publisher{"publisher", env.context(), argc, argv};
  env.execute();
  return 0;
}
