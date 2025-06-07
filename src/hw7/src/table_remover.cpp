// background_remover.cpp
//
// Cory Hungate
//
// ROS2 node to remove the background clutter from pointcloud data of a table
// this is basically the same node as background_remover


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
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <Eigen/Dense>

using std::placeholders::_1;

class TableRemover : public rclcpp::Node {
public:
    // Simplify the syntax a bit.
    using PointCloud2 = sensor_msgs::msg::PointCloud2;

    TableRemover() :Node("table_remover") {

        // Create a publisher and subscriber for the modified point cloud and point cloud.
        publisher_ = this->create_publisher<PointCloud2>("sans_table", 1);
        subscriber_ = this->create_subscription<PointCloud2>("sans_background", 1, std::bind(&TableRemover::callback, this, _1));
        table_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("table_marker", 1);
    }

private:
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr table_marker_pub_;

    void callback(const PointCloud2::SharedPtr msg) {

        // Translate the ROS PointCloud2 to a PCL PointCloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Make another PCL PointCloud for the result.
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_table_removed(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.06); // Adjust this value
        seg.setInputCloud(pcl_cloud);

        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(pcl_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);  
        extract.filter(*pcl_table_removed);

        if (coefficients->values.size() >= 4) {

            float a = coefficients->values[0];
            float b = coefficients->values[1];
            float c = coefficients->values[2];
            float d = coefficients->values[3];		//this ends up going unused

            // Extract only the inlier points representing the table plane
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_table_plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract_table;
            extract_table.setInputCloud(pcl_cloud);
            extract_table.setIndices(inliers);
            extract_table.setNegative(false);  // Get only the table points
            extract_table.filter(*pcl_table_plane);

            // Compute centroid of the table plane points
			//full disclosure, this is a chatGPT recommendation. I don't know how else I would have found this
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_table_plane, centroid);

            // Making a marker for the table plane. going to use a cube and make it thin
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "table_plane";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker position to the centroid of the table plane points
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];

            // Find orientation 
            tf2::Vector3 normal(a, b, c);
            tf2::Vector3 up(0.0, 0.0, 1.0);
            tf2::Quaternion q;
            q.setRotation(up.cross(normal), acos(up.dot(normal)));
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // mostly guessing at these values... I could probably find them more exactly from
			//the find_plane data, but I don't feel like it...
            marker.scale.x = 1.5;  // Table length
            marker.scale.y = 1.0;  // Table width
            marker.scale.z = 0.01; // Thin

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;

            marker.lifetime = rclcpp::Duration::from_seconds(0); 

            table_marker_pub_->publish(marker);
        }

        // Convert PCL PointCloud to a ROS PointCloud2.
        PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_table_removed, ros_cloud);

        // Set the header information.
        ros_cloud.header = msg->header;

        // And, publish it out.
        this->publisher_->publish(ros_cloud);
    }        
};


// This is the entry point for the executable.
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TableRemover>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
