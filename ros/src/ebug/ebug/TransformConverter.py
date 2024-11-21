import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform

import math


# CAM_0 = (0, 0,     0) RPY Radians (FRONT)
# CAM_1 = (0, 0,  PI/2) RPY Radians (RIGHT)
# CAM_2 = (0, 0,    PI) RPY Radians (BACK)
# CAM_3 = (0, 0, 3PI/2) RPY Radians (LEFT)

CAM_OFFSET = 0.025
# CAM_ROTATION = [
#     0.0,                    # Camera 0: Forward
#     math.pi / 2.0,          # Camera 1: Right
#     math.pi,                # Camera 2: Behind
#     3.0 * math.pi / 2.0     # Camera 3: Left
# ]
CAM_ROTATION = [
    0.0                  # Camera 0: Forward
]

"""
This code inverse the cam->tag transform
"""

class TransformConverter(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(TFMessage, 'tf_detections', self.listener_callback, 100)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 100)

        self.covariance = mat6diag(1e-3)


    def listener_callback(self, tf_det:TFMessage):
        for t in tf_det.transforms:
            try:
                # Get tag position
                tag_in_map = self.tf_buffer.lookup_transform(
                    'map',
                    t.child_frame_id,
                    rclpy.time.Time())
                
                # Camera measurements
                cam_z = t.transform.translation.z + CAM_OFFSET
                cam_x = t.transform.translation.x
                
                cam_roll, cam_pitch, cam_yaw  = quat2rpy(t.transform.rotation)

                # Tag measurements
                tag_x = tag_in_map.transform.translation.x
                tag_y = tag_in_map.transform.translation.y
                _, _, tag_yaw = quat2rpy(tag_in_map.transform.rotation)
                self.get_logger().info(f"- Tag yaw: {math.degrees(tag_yaw):.1f}°")

                     
                # Robot's position from camera view
                robot_angle = normalise_angle(tag_yaw + math.pi + cam_pitch)
            
                # Now calculate true distance with corrected measurements
                true_distance = math.sqrt(cam_x**2 + cam_z**2)

                dx = true_distance * math.cos(robot_angle)
                dy = true_distance * math.sin(robot_angle)

                # Calculate robot position
                robot_x = tag_x - dx
                robot_y = tag_y - dy
                self.get_logger().info(f"- Roll, Pitch, Yaw: ({math.degrees(cam_roll):.1f}, {math.degrees(cam_pitch):.1f}, {math.degrees(cam_yaw):.1f})")
                self.get_logger().info(f"- Cam: ({cam_x:.3f}, {cam_z:.3f})")             
                self.get_logger().info(f"- Robot Angle: {math.degrees(robot_angle):.1f}°")
                self.get_logger().info(f"- change position: ({dx:.3f}, {dy:.3f})")
                self.get_logger().info(f"- Final robot position: ({robot_x:.3f}, {robot_y:.3f})")


                # Create and publish pose message
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # Set position
                msg.pose.pose.position.x = robot_x
                msg.pose.pose.position.y = robot_y
                msg.pose.pose.position.z = 0.0

                # Set orientation (quaternion)
                q = rpy2quat(0.0, 0.0, robot_angle)
                msg.pose.pose.orientation.x = q[0]
                msg.pose.pose.orientation.y = q[1]
                msg.pose.pose.orientation.z = q[2]
                msg.pose.pose.orientation.w = q[3]

                # Set covariance based on distance
                distance = math.sqrt(cam_x * cam_x + cam_z * cam_z)

                base_covariance = 0.01  # Very certain when close
                distance_factor = distance * distance  # Quadratic scaling with distance
                scaled_covariance = base_covariance * distance_factor
                msg.pose.covariance = [scaled_covariance, 0.0,  0.0,  0.0,  0.0,  0.0,    
                       0.0, scaled_covariance, 0.0,  0.0,  0.0,  0.0,    
                       0.0, 0.0,  99.0, 0.0,  0.0,  0.0,    
                       0.0, 0.0,  0.0,  99.0, 0.0,  0.0,    
                       0.0, 0.0,  0.0,  0.0,  99.0, 0.0,    
                       0.0, 0.0,  0.0,  0.0,  0.0,  scaled_covariance]
                self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Transform lookup failed: {str(e)}')

def normalise_angle(angle):
    """Keep angle between -π and π"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]


# Function to get the rpy (radians) of a quaternion
def quat2rpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

# Function to get the quaternion of a rpy (radians)
def rpy2quat(roll, pitch, yaw):    
    # https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
    qx = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    qy = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
    qz = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
    qw = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    return [qx, qy, qz, qw]
                

def main():
    rclpy.init()
    node = TransformConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()