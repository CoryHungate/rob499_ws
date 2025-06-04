// data_sender.cpp
//
// Cory Hungate
//
// A reimplementation of the data_sender node from HW3. This node publishes 
//a custom message called TestPacket on the topic 'data'. 


#include <rclcpp/rclcpp.hpp>

#include <rob499_msgs/msg/test_packet.hpp>
#include <rob499_msgs/srv/send_data.hpp>

#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


// Service node that inherits from Node, as usual.
class SendDataService : public rclcpp::Node {
public:
	// Use an alias to simplify the syntax.
	using SendData = rob499_msgs::srv::SendData;
	using TestPacket = rob499_msgs::msg::TestPacket;

	SendDataService() :Node("send_data_service") {

		publisher_ = this->create_publisher<TestPacket>("data", 10);
		service_ = this->create_service<SendData>("send_data", std::bind(&SendDataService::service_callback, this, _1, _2));
		
		//if I have the default send_data_=true, then I want data to begin publishing immediately
		if (send_data_) {
			timer_ = this->create_wall_timer(0.5s, std::bind(&SendDataService::timer_callback, this));
		}
	}	

	

private:

	bool send_data_ = true;
	int counter_ = 0;

	rclcpp::Service<SendData>::SharedPtr service_;
	rclcpp::Publisher<TestPacket>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// A callback to handle the service request.  The has parameters for the request and the response.
	void service_callback(
		const std::shared_ptr<SendData::Request> request,
		const std::shared_ptr<SendData::Response> response) {

		this->send_data_ = request->send;	

		if (this->send_data_ == true){
			if (this->timer_ == nullptr)
				this->timer_ = this->create_wall_timer(0.5s, std::bind(&SendDataService::timer_callback, this));
			response->message = "Data Publishing Started";
			RCLCPP_INFO(this->get_logger(), "Data is now publishing");
		}

		else{
			this->timer_.reset();
			response->message = "Data Publishing Stopped";
			RCLCPP_INFO(this->get_logger(), "Data is no longer publishing");
		}

	}

	void timer_callback(){
		auto msg = TestPacket();
		msg.send_time = this->now();
		msg.payload = {1,2,3,4,5}; 

		publisher_->publish(msg);

		//if you want to visualize the incoming data, just uncomment the next 2 lines
		// RCLCPP_INFO(this->get_logger(), "Receiver message #%i at time %.2f", 
		// 			this->counter_, rclcpp::Time(msg.send_time).seconds());

		this->counter_ += 1;

	}
};


int main(int argc, char **argv) {
	
	rclcpp::init(argc, argv);

	auto service = std::make_shared<SendDataService>();

	rclcpp::spin(service);

	rclcpp::shutdown();

	return 0;
}