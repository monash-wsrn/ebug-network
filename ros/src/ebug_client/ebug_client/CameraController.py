import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import CameraInfo, Image

class CameraController(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.declare_parameter('cameras', ['cam_0'])
        self.cameras = self.get_parameter('cameras').get_parameter_value().string_array_value
        
        self.pub_image = self.create_publisher(Image, 'image_rect', 10)
        self.pub_cinfo = self.create_publisher(CameraInfo, 'camera_info', 10)

        qos_profile = QoSProfile(depth=10)
        for cam_id in self.cameras:
            self.create_subscription(Image, f'{cam_id}/image_raw', lambda x : self.image_callback(cam_id, x), qos_profile)
            self.create_subscription(CameraInfo, f'{cam_id}/camera_info', lambda x : self.cinfo_callback(cam_id, x), qos_profile)


    def image_callback(self, cam_id: str, image: Image):
        self.pub_image.publish(image)
        
    def cinfo_callback(self, cam_id: str, cinfo: CameraInfo):
        self.pub_cinfo.publish(cinfo)





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