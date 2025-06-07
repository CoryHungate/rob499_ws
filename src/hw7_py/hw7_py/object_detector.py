#!/usr/bin/env python3

# object_detector.py
#
# Cory Hungate
#
# node that subscribes to the filtered pointcloud2 data and counts the number of objects on the table. 
# note: I do not have a parameter callback, so you can only change those values here or in the launch file

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, Header, String
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pcl2

import numpy as np
from sklearn.cluster import DBSCAN
import struct

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        #parameters for 'epsilon - cluster distance' and minimum number of samples
        self.declare_parameter('eps', 0.03)  
        self.declare_parameter('min_samples', 15)

        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.current_objects = 0

        self.sub = self.create_subscription(PointCloud2, '/sans_table', self.cloud_callback, 10)
        self.pub_count = self.create_publisher(Int32, '/tabletop_objects', 10)
        self.pub_centroids = self.create_publisher(PointCloud2, '/objects', 10)
        self.pub_data = self.create_publisher(String, '/data', 10)
        self.pub_marker = self.create_publisher(Marker, '/object_count_marker', 10)


    def cloud_callback(self, msg: PointCloud2):

        points = []
        for pt in pcl2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([pt[0], pt[1], pt[2]])

        #if no points in the nparray, publish that there are 
        #Note, I'm trying out this new DataClass syntax I found (Class(field=)).. a bit less readable but nice and short!
        if not points:
            self.pub_count.publish(Int32(data=0))
            return
        
        #importing the pointcloud2 array from above into a numpy array
        points_np = np.array(points)

        #DBSCAN is looking for clusters and then creating an array of labels
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points_np)
        labels = db.labels_

        #DBSCAN assigns the label '-1' to a "noise cluster", so I want a set of unique labels that arent noise
        unique_objs = set(labels) - {-1}
        count = len(unique_objs)

        #publishing the data, but only printing the data if the count has changed
        self.pub_count.publish(Int32(data=count))

        # Compute centroids from the unique object set and then publishing their location
        centroids = []
        for obj in unique_objs:
            cluster_points = points_np[labels == obj]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        self.publish_centroids(centroids, msg.header)

        old_count = self.current_objects
        self.current_objects = count
        if old_count != self.current_objects:
            self.pub_data.publish(String(data = f'There are currently {count} objects'))
        
        #creating my marker 
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # or use a fixed frame like 'quori/head'
        marker.header.stamp = msg.header.stamp
        marker.ns = "object_count"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = -0.5
        marker.pose.position.z = 2.0  
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.3  
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"Objects: {count}"
        marker.lifetime.sec = 1  # Auto-deletes after 1s unless refreshed

        self.pub_marker.publish(marker)

    def publish_centroids(self, centroids, header_in):

        header = Header()
        header.stamp = header_in.stamp
        header.frame_id = header_in.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        points_data = [
            struct.pack('fff', float(x), float(y), float(z))
            for x, y, z in centroids
        ]

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(centroids)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(centroids)
        cloud.is_dense = True
        cloud.data = b''.join(points_data)

        self.pub_centroids.publish(cloud)

    

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()