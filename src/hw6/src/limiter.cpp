// limiter.cpp
//
// Bill Smart
//
// An example of a C++ subscriber node.

// Include the basic ROS functionality and the message definition.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cmath>

// We need this in order to set up the subscriber.  This is the C++ way of
// letting us specify callback parameters.
using std::placeholders::_1;

// As usual, we create a class that inherits from Node. a
class OscopeSubscriber : public rclcpp::Node {
public:
	OscopeSubscriber() :Node("oscope_subscriber") {

		this->declare_parameter<float>("limit", 0.7);
		this->get_parameter("limit", limit_);

		publisher_ = this->create_publisher<std_msgs::msg::Float32>("limited_data", 10);
		subscriber_ = this->create_subscription<std_msgs::msg::Float32>("oscope", 
								10, std::bind(&OscopeSubscriber::callback, this, _1));
	}


private:
	// Variables for the subscriber.  The idiom is to end member variables with an underscore.
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

	double limit_ = 0.7;
	// The callback for the subscriber.  The message is passed via a shared pointer.  This means
	// you have to use the -> accessor, rather than the . (dot) accessor.
	void callback(const std_msgs::msg::Float32::SharedPtr msg) {
		// Just announce that we got the message.
		auto new_msg = std_msgs::msg::Float32();
		if (abs(msg->data) > this->limit_){
			if ((msg->data) >= 0)
				new_msg.data = this->limit_;
			else new_msg.data = -1 * this->limit_;
		}
		else new_msg.data = msg->data;

		publisher_->publish(new_msg);
		// RCLCPP_INFO(this->get_logger(), "Got %f", new_msg.data);
	}
};


// This is the entry point for the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node.
	auto subscriber = std::make_shared<OscopeSubscriber>();

	// Give control to the ROS event handler.
	rclcpp::spin(subscriber);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}