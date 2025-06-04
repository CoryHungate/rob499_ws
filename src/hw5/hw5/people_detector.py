#!/usr/bin/env python3

# people_detector.py
#
# Cory Hungate
#
# node that subscribes to the filtered laser range data and uses DBSCAN to identify 
# likely people and retrurn each as a point in a point cloud

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from sklearn.cluster import DBSCAN
import struct

class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')

        #parameters for 'epsilon - cluster distance' and minimum number of samples
        self.declare_parameter('eps', 0.7)  
        self.declare_parameter('min_samples', 5)

        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value

        self.sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/people_detected', 10)

        self.get_logger().info(f"people_detector node started. Listening to {'scan_filtered'}")

    def scan_callback(self, msg: LaserScan):
        points = []

        angle = msg.angle_min
        for r in msg.ranges:
            if np.isfinite(r):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment

        if not points:
            return

        points_np = np.array(points)
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points_np)
        labels = db.labels_

        # -1 label is "noise"
        unique_labels = set(labels) - {-1}
        centroids = []
        for label in unique_labels:
            cluster_points = points_np[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        self.publish_pointcloud2(centroids, msg.header)

    def publish_pointcloud2(self, centroids, header_in):
        header = Header()
        header.stamp = header_in.stamp
        header.frame_id = header_in.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points_data = []
        for c in centroids:
            # note: z = 0 since it's 2D LIDAR
            x, y = c
            points_data.append(struct.pack('fff', x, y, 0.0))

        data = b''.join(points_data)

        #god damn there are a lot of pieces to the LaserScan data class...
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(centroids)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        pc2.data = data

        self.pub.publish(pc2)

def main(args=None):
    rclpy.init(args=args)
    node = PeopleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()