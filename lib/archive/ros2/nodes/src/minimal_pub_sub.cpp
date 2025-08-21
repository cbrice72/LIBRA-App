/******************************************************************************
 * @file   minimal_pub_sub.cpp
 * @brief  Minimal working example of a ROS2 pub/sub node; all-in-one file.
 *
 * @author Christian Brice
 ******************************************************************************/

// C++ Standard Library Headers
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

// Other Library Headers
#include "rclcpp/rclcpp.hpp"        // ROS2 Core
#include "std_msgs/msg/string.hpp"  // ROS2 Messages

// Project Headers
//   (none)

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Minimal working example of a ROS2 publish/subscribe node.
 */
class MinimalPubSub : public rclcpp::Node {
  public:
    /**
     * @brief Conditional constructor, initializing one of two data
     *        communication mechanisms.
     *
     * @param is_publisher Whether to initialize the node as a publisher or a
     *                     subscriber.
     */
    MinimalPubSub(bool is_publisher)
        : Node("minimal_pub_sub"), is_publisher_(is_publisher) {
        if (is_publisher_) {
            // Create publisher object
            pub_ = this->create_publisher<std_msgs::msg::String>("test", 10);
            // Specify how often publishing should occur, and via which function
            timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalPubSub::timer_callback, this));
            // ROS-like terminal output (for debugging)
            RCLCPP_INFO(this->get_logger(), "Node initialized as publisher");
        } else {
            // Create subscriber object, and specify which function handles it
            sub_ = this->create_subscription<std_msgs::msg::String>(
                "test", 10, std::bind(&MinimalPubSub::topic_callback, this, _1));
            // ROS-like terminal output (for debugging)
            RCLCPP_INFO(this->get_logger(), "Node initialized as subscriber");
        }
    }

  private:
    // --- Helper Functions ---

    /**
     * @brief Publisher callback: will always publish the same string.
     */
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from MinimalPubSub!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
                    message.data.c_str());
        pub_->publish(message);
    }

    /**
     * @brief Subscriber callback: prints the received string
     *
     * @param msg The received std_msgs/String object.
     */
    void topic_callback(const std_msgs::msg::String& msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    // --- Data Members ---

    bool is_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
    bool is_publisher = false;  // default to "subscriber"

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-p" || arg == "--publisher") {
            is_publisher = true;
            break;
        } else if (arg == "-s" || arg == "--subscriber") {
            // Do nothing -- is_publisher is already false
            break;
        } else if (arg == "-h" || arg == "--help") {
            printf("Select the node's communication mode via "
                   "\"-p\"/\"--publisher\" or \"-s\"/\"--subscriber\".");
            return 0;
        }
    }

    // Standard ROS2 node initialization, running, and teardown
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPubSub>(is_publisher);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
