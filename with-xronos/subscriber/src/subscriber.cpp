#include <future>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xronos/sdk.hh"

class MinimalSubscriberRosNode : public rclcpp::Node {
public:
  MinimalSubscriberRosNode() : Node("minimal_subscriber") {
    auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "ROS subscriber heard: '%s'",
                  msg->data.c_str());
    };
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class MinimalSubscriber : public xronos::sdk::Reactor {
  using xronos::sdk::Reactor::Reactor;
  std::thread rclcpp_thread_;
  std::promise<void> shutdown_promise_;
  int argc_;
  char **argv_;

public:
  MinimalSubscriber(std::string_view name, xronos::sdk::Context &context,
                    int argc, char *argv[])
      : xronos::sdk::Reactor(name, context), argc_(argc), argv_(argv) {}

private:
  class OnStartupReaction : public xronos::sdk::Reaction<MinimalSubscriber> {
    using xronos::sdk::Reaction<MinimalSubscriber>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    void handler() final {
      rclcpp::init(self().argc_, self().argv_);
      // Both the Xronos runtime and ROS 2 runtime involve indefinitely-running
      // API calls, so they are composed by starting `rclcpp::spin` in a
      // separate thread.
      std::shared_future<void> shutdown_future(
          self().shutdown_promise_.get_future());
      self().rclcpp_thread_ = std::thread([shutdown_future]() {
        rclcpp::spin_until_future_complete(
            std::make_shared<MinimalSubscriberRosNode>(), shutdown_future);
        rclcpp::shutdown();
      });
      std::cout << "ROS 2 started." << std::endl;
    }
  };

  class OnShutdownReaction : public xronos::sdk::Reaction<MinimalSubscriber> {
    using xronos::sdk::Reaction<MinimalSubscriber>::Reaction;
    Trigger<void> shutdown_trigger{self().shutdown(), context()};
    void handler() final {
      std::cout << "Shutting down ROS 2..." << std::endl;
      self().shutdown_promise_.set_value();
      self().rclcpp_thread_.join();
      std::cout << "ROS 2 shutdown complete." << std::endl;
    }
  };

  void assemble() final {
    add_reaction<OnStartupReaction>("on_startup");
    add_reaction<OnStartupReaction>("on_shutdown");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberRosNode>());
  rclcpp::shutdown();
  return 0;
}
