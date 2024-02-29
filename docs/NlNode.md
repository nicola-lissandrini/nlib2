# NlNode: A Guide to Advanced ROS Node Creation

## Overview

The `NlNode` class is a convenience class designed for the rapid development of ROS (Robot Operating System) nodes. It provides a structured way to handle initialization, parameter management, subscriptions, publications, and custom callbacks within a ROS environment.

## Features

- **Parameter Management:** Simplifies the process of declaring, getting, and setting ROS parameters with centralized parameter server support.
- **Automatic ROS Initialization:** Handles the necessary ROS initialization steps, including setting up node options and command-line arguments processing.
- **Subscription and Publication Simplification:** Offers templated methods to add subscriptions and publishers with minimal boilerplate, supporting Quality of Service (QoS) settings.
- **Dynamic Topic Resolution:** Integrates with a centralized configuration to dynamically resolve topic names for subscriptions and publications, enabling flexible topic remapping.
- **Custom Callbacks:** Supports adding custom callbacks for parameter changes and periodic clock events, facilitating easy integration of time-based operations or parameter-driven behaviors.

## Getting Started

### Creating a Node

To create a new node, inherit from `NlNode` specifying your class as the template parameter. Implement your node's specific functionalities within this derived class.

```cpp
class MyNode : public NlNode<MyNode> {
public:
    MyNode(int argc, char **argv) : NlNode(argc, argv, "my_node_name") {
        // Additional initialization
    }

    void myCallback(const std_msgs::msg::String &msg) {
        // Handle message
    }
};
```

### Initializing Parameters

Parameters are initialized in the `initParams()` method. Use this method to declare and initialize parameters your node will use, including any ROS-specific configurations.

```cpp
void initParams() {
    auto paramValue = this->_nlParams->declareAndGet<std::string>("my_param", "default_value");
    // Use paramValue as needed
}
```

### Subscribing to Topics

Add subscriptions to topics using the `addSubscription` method, specifying the message type, topic name, QoS settings, and the callback method.

```cpp
void initROS() {
    this->addSubscription<std_msgs::msg::String>("sub_name", "topic_name", rclcpp::QoS(10), &MyNode::myCallback);
}
```

### Publishing Messages

First, add a publisher for your message type, then use the `publish` method to send messages.

```cpp
// Adding a publisher in initROS
this->addPublisher<std_msgs::msg::String>("pub_name", "topic_to_publish", rclcpp::QoS(10));

// Publishing a message
std_msgs::msg::String msg;
msg.data = "Hello, world!";
this->publish("pub_name", msg);
```

### Running Your Node

The `spin` method starts the node, causing it to begin executing its operations, such as listening for incoming messages and calling callbacks.

```cpp
int main(int argc, char **argv) {
    MyNode node(argc, argv);
    return node.spin();
}
```

## Conclusion

The `NlNode` class framework offers a robust starting point for developing ROS nodes by abstracting away common initialization and setup tasks, allowing developers to focus on the unique functionalities of their nodes.