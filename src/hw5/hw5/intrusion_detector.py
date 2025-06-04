#!/usr/bin/env python3

# intrusion_detector.py
#
# Cory Hungate
#
# node that subscribes to the person_detector node, examines whether or not the 
# point is within 1m of the robot, and republishes the point if it is

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

class IntruderDetector(Node):
    def __init__(self):
        super().__init__('intruder_detector')

        # Parameters
        self.declare_parameter('distance_threshold', 1.0)  
        self.distance_threshold = self.get_parameter('distance_threshold').value

        # Subscriber to people detection
        self.sub_people = self.create_subscription(
            PointCloud2, '/people_detected', self.people_callback, 10
        )

        # Publisher for detected intruders
        self.pub_intruder = self.create_publisher(PointCloud2, 'intruder_detected', 10)

        self.get_logger().info("IntruderDetector node started.")

    def people_callback(self, msg: PointCloud2):
        # Convert PointCloud2 data into an array of points (x, y, z)
        points = self.convert_pointcloud2_to_numpy(msg)

        # Filter out points that are farther than 1 meter
        intruders = []
        for point in points:
            x, y, z = point
            distance = np.sqrt(x**2 + y**2)

            # If the person is within 1 meter, add to the intruder list
            if distance <= self.distance_threshold:
                intruders.append(point)

        if intruders:
            # Publish the filtered intruder points
            self.publish_intruder_pointcloud(intruders, msg.header)

    def convert_pointcloud2_to_numpy(self, msg: PointCloud2):
        # Converts PointCloud2 to numpy array (assuming it's 2D)
        points = []

        # Deserialize data into points
        for i in range(msg.width):
            offset = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, offset)
            points.append([x, y, z])
        return np.array(points)

    def publish_intruder_pointcloud(self, intruders, header_in):
        # Prepare PointCloud2 message
        header = header_in
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        points_data = []
        for point in intruders:
            points_data.append(struct.pack('fff', point[0], point[1], point[2]))

        data = b''.join(points_data)

        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(intruders)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        pc2.data = data

        self.pub_intruder.publish(pc2)


def main(args=None):
    rclpy.init(args=args)

    node = IntruderDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
