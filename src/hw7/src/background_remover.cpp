// background_remover.cpp
//
// Cory Hungate
//
// ROS2 node to remove the background clutter from pointcloud data of a table


#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/point_cloud2.hpp>


#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h> 

using std::placeholders::_1;

class BackGroundRemover : public rclcpp::Node {
public:
	// Simplify the syntax a bit.
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	BackGroundRemover() :Node("background_remover") {

		// Create a publisher and subscriber for the modified point cloud and point cloud.
		publisher_ = this->create_publisher<PointCloud2>("sans_background", 1);
		subscriber_ = this->create_subscription<PointCloud2>("/astra_ros/devices/default/point_cloud", 1, std::bind(&BackGroundRemover::callback, this, _1));
	}

private:
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	void callback(const PointCloud2::SharedPtr msg) {

		// Translate the ROS PointCloud2 to a PCL PointCloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pcl_cloud);

		// Make another PCL PointCloud for the result.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		//cropping in a bounding box
		pcl::PointCloud<pcl::PointXYZ>::Ptr table_cropped(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.02); // Adjust this value
		seg.setInputCloud(pcl_cloud);

		seg.segment(*inliers, *coefficients);
		
		// pcl::ExtractIndices<pcl::PointXYZ> extract;
		// extract.setInputCloud(pcl_cloud);
		// extract.setIndices(inliers);
		// extract.setNegative(false);  
		// extract.filter(*pcl_cloud_filtered);

		// Log the plane equation for debugging
    // RCLCPP_INFO(this->get_logger(),
    //     "Plane: %.2fx + %.2fy + %.2fz + %.2f = 0",
    //     coefficients->values[0], coefficients->values[1],
    //     coefficients->values[2], coefficients->values[3]);

		// Extract all points within X meters of the plane
		float a = coefficients->values[0];
		float b = coefficients->values[1];
		float c = coefficients->values[2];
		float d = coefficients->values[3];

    	float plane_threshold = 0.20;  // 5 cm buffer above/below table

		for (const auto& point : pcl_cloud->points) {
			float distance = std::abs(a * point.x + b * point.y + c * point.z + d) /
							std::sqrt(a * a + b * b + c * c);
			if (distance < plane_threshold) {
				pcl_cloud_filtered->points.push_back(point);
			}
		}

		pcl::CropBox<pcl::PointXYZ> crop;
		crop.setInputCloud(pcl_cloud_filtered);

		// Set bounding box dimensions (in meters); tweak these for your table
		crop.setMin(Eigen::Vector4f(-2.0, -2.0, -2.0, 1.0));  // min X, Y, Z
		crop.setMax(Eigen::Vector4f(2.0, 2.0, 2.0, 1.0));    // max X, Y, Z

		crop.filter(*table_cropped);



		// Convert PCL PointCloud to a ROS PointCloud2.
		PointCloud2 ros_cloud;
		pcl::toROSMsg(*table_cropped, ros_cloud);

		// Set the header information.
		ros_cloud.header = msg->header;

		// And, publish it out.
		this->publisher_->publish(ros_cloud);
		}		
		
};


// This is the entry point for the executable.
int main(int argc, char **argv) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<BackGroundRemover>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
