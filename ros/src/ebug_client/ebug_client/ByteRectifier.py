import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image, CompressedImage

class ByteRectifier(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

    
        qos_profile = QoSProfile(depth=10)
        self.sub = self.create_subscription(Image, "image_raw", self.image_callback, qos_profile)
        self.pub = self.create_publisher(CompressedImage, "image_compressed", qos_profile)


    def image_callback(self, image: Image):
        cimage = CompressedImage()

        cimage.header = image.header
        cimage.data = image.data
        cimage.format = 'jpeg'

        self.pub.publish(cimage)




## Boilerplate

def main():
    rclpy.init()
    node = ByteRectifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()