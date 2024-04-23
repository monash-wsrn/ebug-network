import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image

import simplejpeg as jpeg
import numpy as np

class ByteRectifier(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.sub_image = self.create_subscription(CompressedImage, 'image_raw/compressed', self.img_callback, 10)
        self.pub_image = self.create_publisher(Image, 'image_raw/uncompressed', 10)

        self.WIDTH = 640
        self.HEIGHT = 480


    def img_callback(self, img: CompressedImage):        
        data = memoryview(img.data) 
        raw = jpeg.decode_jpeg(data, 'BGR', True, True, self.HEIGHT, self.WIDTH, 1, None, False)

        result = Image()
        result.header = img.header
        result.width = int(self.WIDTH)
        result.height = int(self.HEIGHT)
        result.encoding = 'bgr8'
        result.is_bigendian = False
        result.step = int(raw.size / self.HEIGHT)
        result.data = raw.data
        
        self.pub_image.publish(result)




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