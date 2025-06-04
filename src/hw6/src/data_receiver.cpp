// data_receiver.cpp
//
// Cory Hungate
//
// A reimplementation of the data_receiver node from HW3. This node subscribes 
//to the topic 'data', measures the latency time from receipt->read, and keeps 
//an average latency of the last 10 'data' published


// Include the standard ROS stuff plus some extras for custom messages etc...
#include <rclcpp/rclcpp.hpp>

#include <rob499_msgs/srv/enable_logging.hpp>
#include <rob499_msgs/msg/test_packet.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>
#include <string>
#include <fstream>      
#include <iostream>     
#include <memory>   

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

//this is the size of the array that keeps track of the averages, 
//because it's used frequently, decided to make it global for easy change
const int AVG_ARR_SIZE = 10;


class EnableLoggingService : public rclcpp::Node {
public:
	// aliases (aliasi?) to make readability easier
	using EnableLogging = rob499_msgs::srv::EnableLogging;
	using TestPacket = rob499_msgs::msg::TestPacket;
	using Float32 = std_msgs::msg::Float32;

	EnableLoggingService() :Node("data_receiver") {

		//need 2 publishers, one for the "raw" value, and another for the average value
		raw_publisher_ = this->create_publisher<Float32>("raw_latency", 10);
		avg_publisher_ = this->create_publisher<Float32>("average_latency", 10);
		
		subscriber_ = this->create_subscription<TestPacket>("data", 10, 
						std::bind(&EnableLoggingService::sub_callback, this, _1));
		service_ = this->create_service<EnableLogging>("enable_logging", 
						std::bind(&EnableLoggingService::service_callback, this, _1, _2));
	}

	//destructor in case the node fails
	~EnableLoggingService() {
    	close_file();
	}

	
private:
	//creating the pointers to the various pubs, subs, etc.
	rclcpp::Service<EnableLogging>::SharedPtr service_;
	rclcpp::Publisher<Float32>::SharedPtr raw_publisher_;
	rclcpp::Publisher<Float32>::SharedPtr avg_publisher_;
	rclcpp::Subscription<TestPacket>::SharedPtr subscriber_;
	std::ofstream csv_file_;

	bool enable_logging_ = false;
	double buffer_[AVG_ARR_SIZE];
	std::string file_name_;
	int counter_ = 0;
	double avg_latency_;

	//callback that handles the service call
	void service_callback(
		const std::shared_ptr<EnableLogging::Request> request,
		const std::shared_ptr<EnableLogging::Response> response) {

		this->enable_logging_ = request->enabled;	
		this->file_name_ = request->filename;

		if (request->enabled){
			//just in case
			close_file();

			//open a new file
			csv_file_.open(this->file_name_, std::ios::out | std::ios::trunc);

			//if the file failed to open
			if(!csv_file_.is_open()) {
				response->message = "failed to open file " + request->filename;
				this->enable_logging_ = false;
				RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
				return;
			}

			//if the file opened correctly
			this->enable_logging_ = true;
			response->message = "Data logging enabled";
			RCLCPP_INFO(this->get_logger(), "Data logging has begun");

		}

		else{
			close_file();
			response->message = "Data logging disabled";
			RCLCPP_INFO(this->get_logger(), "Data logging has stopped");
		}
		RCLCPP_INFO_STREAM(this->get_logger(), "responding with " << response->message);
	}


	void sub_callback(const TestPacket::SharedPtr msg) {
		
		//get the current time, the sent time, and the difference between the two
		rclcpp::Time current_time = this->now();
		rclcpp::Time sent_time(msg->send_time);		//note the strange syntax here...
		rclcpp::Duration latency = current_time - sent_time;

		// the other variables I will be using for this callback...
		double latency_sec = latency.seconds();
		int index = this->counter_ % AVG_ARR_SIZE;
		int buffer_size;
		double buffer_sum = 0;

		this->buffer_[index] = latency_sec;
		this->counter_ += 1;

		this->counter_ < AVG_ARR_SIZE ? buffer_size = this->counter_ : buffer_size = AVG_ARR_SIZE;

		for (int i = 0; i < buffer_size; i++) buffer_sum += this->buffer_[i];

		this->avg_latency_ = buffer_sum/buffer_size;
		
		if (csv_file_.is_open())
			csv_file_ << latency_sec << "," << avg_latency_ << '\n'; 

		//publishig the actual values
		Float32 latency_msg;
		latency_msg.data = static_cast<float>(latency_sec);
		raw_publisher_->publish(latency_msg);

		Float32 avg_msg;
		avg_msg.data = static_cast<float>(avg_latency_);
		avg_publisher_->publish(avg_msg);

		// RCLCPP_INFO(this->get_logger(), "Latency: %.6f\tAverage Latency %.6f", latency_sec, this->avg_latency_);
	}

	void close_file(){
		if (csv_file_.is_open()) 
			csv_file_.close();
		this->enable_logging_ = false;
	}
	

};


// This is the entry point of the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node.
	auto service = std::make_shared<EnableLoggingService>();

	// Give control over to the ROS event handler.
	rclcpp::spin(service);

	// Once the event handler is done, make sure we're completely shut down.
	rclcpp::shutdown();

	return 0;
}