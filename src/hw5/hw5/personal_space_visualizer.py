import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class PersonalSpaceMarker(Node):
    def __init__(self):
        super().__init__('personal_space_marker')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_transform()
        
        self.marker_pub = self.create_publisher(Marker, '/personal_space_marker', 1)
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.get_logger().info("Publishing personal space marker...")
    
    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ramsis/base_laser_scanner'  # or 'odom' if you're using that as your global frame
        t.child_frame_id = 'base_link'

        # Fixed position of base_link relative to map
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
    
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "personal_space"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0  # 1m radius -> diameter = 2m
        marker.scale.y = 2.0
        marker.scale.z = 0.01  # Thin layer
        marker.color.a = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PersonalSpaceMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()