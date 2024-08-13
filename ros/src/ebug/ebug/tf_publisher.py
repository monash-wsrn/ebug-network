import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.br = TransformBroadcaster(self)

        # Subscription to the odometry or pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ebug03/pose',  # Change this to the actual topic you're using
            self.pose_callback,
            10
        )
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.publish_transform()

    def publish_transform(self):
        if self.current_pose is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set the transform based on the robot's pose
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = self.current_pose.position.z
        
        t.transform.rotation.x = self.current_pose.orientation.x
        t.transform.rotation.y = self.current_pose.orientation.y
        t.transform.rotation.z = self.current_pose.orientation.z
        t.transform.rotation.w = self.current_pose.orientation.w
        
        # Publish the transform
        self.br.sendTransform(t)
        self.get_logger().info("Publishing transform: %s" % str(t))

def main():
    rclpy.init()
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
