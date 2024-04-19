import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry

from message_filters import Subscriber, ApproximateTimeSynchronizer

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class CameraController(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.declare_parameter('all_cameras', 'disable')
        self.all_cameras = self.get_parameter('all_cameras').get_parameter_value().bool_value

        self.declare_parameter('cameras', ['cam_0'])
        self.cameras = self.get_parameter('cameras').get_parameter_value().string_array_value
        

        self.selected = self.cameras[0]
        self.synchronizers = []

        self.pub_image = self.create_publisher(Image, 'image_raw', 10)
        self.pub_cinfo = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.sub_odom = self.create_subscription(Odometry, 'filtered_odom', self.odom_callback, 10)

        for cam_id in self.cameras:
            image = Subscriber(self, Image, f'{cam_id}/image_raw')
            cinfo = Subscriber(self, CameraInfo, f'{cam_id}/camera_info')

            sync = ApproximateTimeSynchronizer([image, cinfo], 10, 0.1, allow_headerless=True)
            sync.registerCallback(lambda x, y, cid=cam_id : self.camera_callback(cid, x, y))

            self.synchronizers.append(sync)

            self.get_logger().info(f"Connected USB Camera (ID: {cam_id})")


    def camera_callback(self, cam_id: str, image: Image, cinfo: CameraInfo):
        self.get_logger().info(f"All Cameras: {self.all_cameras}, Cam ID: {cam_id}, Selected: {self.selected}")
        if self.all_cameras or (cam_id == self.selected):
            self.pub_image.publish(image)
            self.pub_cinfo.publish(cinfo)


    def odom_callback(self, odom: Odometry):        
        # TODO Use odom to identify which camera in the cameras array
        # would be best positioned to see an april tag.
        
        self.selected = self.cameras[0]





## Boilerplate

def main():
    rclpy.init()
    node = CameraController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()