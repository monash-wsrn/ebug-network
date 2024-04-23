import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class ByteRectifier(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.sub_image = self.create_subscription(Image, 'image_raw/uncompressed', self.img_callback, 10)
        self.pub_image = self.create_publisher(Image, 'image_raw/rectified', 10)


    def img_callback(self, img: Image):        
        if ( str(img.encoding).lower() == "yuv422" ):
            img.step = img.width * 2

        self.pub_image.publish(img)



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