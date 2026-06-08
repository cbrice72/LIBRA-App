# Custom ROS2 Interface Usage Example

## Header File (`.h`/`.hpp`)

Although not required, if you plan on sending messages frequently, you may want to declare interface type member variable(s) in your class in order to lower the overhead resulting from message object creation.

```cpp
# include <rclcpp/rclcpp.hpp>
# include <libra_app/msg/hebi_state.hpp>

using msgHebiState = libra_app::msg::HebiState;  // for convenience

class Foo : public rclcpp::Node {
  public:
    Foo:Foo();

    void Foo::PublishState();

  private:
    const int num_actuators_{5};

    rclcpp::Publisher<msgHebiState>::SharedPtr state_pub_;
    msgHebiState state_msg_;  // reused for efficiency
}
```

## Source File (`.cpp`)

```cpp
/**
 * @brief Standard constructor.
 */
Foo::Foo()
    : rclcpp::Node("foo_node") {
    ...
    
    // Initialize ROS2 components
    state_pub_ =
        this->create_publisher<msgHebiState>(
            "foo/state",         // topic name
            QoS(KeepLast(10)));  // quality of service

    state_msg_.header.frame_id = "foo_actuators";
    state_msg_.families = ["Foo"];
    state_msg_.names = ["A", "B", "C"];

    // [optional] Resize message vectors to match number of actuators
    state_msg_.target_pos.resize(num_actuators_);
    ...
}

/**
 * @brief Publishes Foo actuator state(s).
 */
void Foo::PublishState() {
    // Update header timestamp
    state_msg_.header.stamp = this->get_clock()->now();

    // Populate the message and publish it
    for (int i = 0; i < num_actuators_; ++i) {
        // Positions (convert from rad to deg)
        state_msg_.target_pos[i] = 42;
        ...
    }

    state_pub_->publish(state_msg_);
}
```
