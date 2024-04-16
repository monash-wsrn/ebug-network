import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
import sys
from ebug_interfaces.msg import RobotPose
from ebug_interfaces.msg import ControlCommand

# https://medium.com/@rndonovan1/running-pygame-gui-in-a-docker-container-on-windows-cc587d99f473
# ENV DISPLAY=host.docker.internal:0.0
class PyGameDisplay(Node):
    # Initialise Pygame Variables
    ARENA_WIDTH = 200   # cm
    ARENA_HEIGHT = 200  # cm
    SCALE = int(os.getenv('DISPLAY_SCALE', "3"))
    
    # Initialise Robot Params
    ROBOT_RADIUS = 7.5  # cm (15 cm diameter)
    NUM_LEDS = 16       # 16 leds circullarly arranged on each robot

    # Initialise Colours
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GREY = (128, 128, 128)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    def __init__(self):
        super().__init__('pygame_display')
        self.subscription = self.create_subscription(RobotPose, 'global_poses', self.render, 10)
        #self.subscription = self.create_subscription(ControlCommand, 'controls', self.render, 10)

        # Define robot pose dictionary to fill
        self.robot_poses = {}     
        
        # Initialize Pygame
        pygame.init()

        self.surface = pygame.Surface((self.ARENA_WIDTH, self.ARENA_HEIGHT))
        self.window_size = (self.ARENA_WIDTH * self.SCALE, self.ARENA_HEIGHT * self.SCALE)
        self.window = pygame.display.set_mode(self.window_size)
        
        pygame.display.set_caption("Pygame Display")
        self.clock = pygame.time.Clock()

        self.surface.fill(self.WHITE)
        self.redraw()


    def render(self, payload:RobotPose):

        # Update robot pose dictionary with new data
        self.robot_poses[payload.robot_id] = payload.pose.pose.pose #[position vector3, orientation vector3]

        self.surface.fill(self.GREY)

        for key, value in self.robot_poses.items():
            ### render pygame window

            # Grab robot position, direction and LED colours
            x = int(value.position.x)
            y = int(value.position.y)
            theta = angle(value)
            #LED = 

            # Draw robot body
            pygame.draw.circle(self.surface, self.BLACK, (x, y), self.ROBOT_RADIUS)
            
            # Draw LED's evenly space around ring
            for i in range(0, self.NUM_LEDS):
                led_x = x + self.ROBOT_RADIUS/2 * math.cos(theta + i * (2*math.pi/self.NUM_LEDS))
                led_y = y + self.ROBOT_RADIUS/2 * math.sin(theta + i * (2*math.pi/self.NUM_LEDS))
                pygame.draw.circle(self.surface, self.RED, (int(led_x), int(led_y)), 1)
            
            #pygame.draw.line(self.surface, self.BLUE, (int(x - 20/2*cos())), (), width=1)
            ###
        
        # Refresh display
        self.redraw()

    def redraw(self):
        scaled = pygame.transform.scale(self.surface, self.window_size)
        self.window.blit(scaled, (0, 0))
        pygame.display.update()
        

    def run(self):
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            rclpy.spin_once(self)
            self.clock.tick(30)



def main(args=None):
    rclpy.init(args=args)

    pygame_display = PyGameDisplay()
    pygame_display.run()

    pygame_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




def angle(pose):
    roll, pitch, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return yaw

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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