/*Example of publishing in ROS 2 for ROB499 HW2

oscope.cpp

Cory Hungate

This has been modified from sample code provided for the class by Bill Smart. 
I have kept some of the original comments that I find helpful as well as 
added my own
*/


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>
#include <cmath>

// constexpr double pi = 3.14159265358979323846;

using namespace std::chrono_literals;


// Create a class that inherits from Node.
class OscopePublisher : public rclcpp::Node {
public:
	
	OscopePublisher() :Node("oscope_publisher"), count_(0) {
		
		publisher_ = this->create_publisher<std_msgs::msg::Float32>("oscope", 10);

		timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / sampling_rate_), std::bind(&OscopePublisher::timer_callback, this));
	}

private:
	// A variable to hold the current count.  The C++ idiom is to make private class variables
	// end in an underscore.
	double count_ = 0;
	double frequency_ = 1;
	double sampling_rate_ = 100;
	
	// Variables for the publisher and timer.  These shoudl be shared pointers.
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// The callback that the timer uses.
	void timer_callback() {
		// Make a new message, and set the data element. We can use the auto
		// keyword here, since the type is clear.  Note that we're assigning
		// the data element and post-incrementing the count_ variable.
		auto message = std_msgs::msg::Float32();
		message.data = sin(this->frequency_ * 2 * M_PI * this->count_);

		// Publish the message.
		publisher_->publish(message);

		this->count_ += 1/this->sampling_rate_;

		// Record in the log that we published the message. turning this off but leaving it in
		// RCLCPP_INFO(this->get_logger(), "Published %f %f", message.data, this->count_);
	}
};


// This is the entry point of the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and assign a shared pointer to it to the publisher variable.
	auto publisher = std::make_shared<OscopePublisher>();

	// Give control to the ROS event handler.
	rclcpp::spin(publisher);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}

