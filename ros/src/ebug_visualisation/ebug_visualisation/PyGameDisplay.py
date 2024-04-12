import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
import sys
from ebug_interfaces.msg import RobotPose

# https://medium.com/@rndonovan1/running-pygame-gui-in-a-docker-container-on-windows-cc587d99f473
# ENV DISPLAY=host.docker.internal:0.0
class PyGameDisplay(Node):
    # Initialise Pygame Variables
    ARENA_WIDTH = 2000  # mm
    ARENA_HEIGHT = 2000 # mm
    SCALE = int(os.getenv('DISPLAY_SCALE', "5"))
    
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GREY = (128, 128, 128)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    def __init__(self):
        super().__init__('pygame_display')
        self.subscription =  self.create_subscription(RobotPose, 'global_poses', self.render, 10)

        # Define robot pose dictionary to fill
        self.robot_poses = {}     
        
        # Initialize Pygame
        pygame.init()

        self.surface = pygame.Surface((self.ARENA_WIDTH, self.ARENA_HEIGHT))
        self.window = pygame.display.set_mode((self.ARENA_WIDTH * self.SCALE, self.ARENA_HEIGHT * self.SCALE))
        
        pygame.display.set_caption("Pygame Display")
        self.clock = pygame.time.Clock()

        # TODO
        self.surface.fill(self.GREY)
        pygame.draw.circle(self.surface, self.RED, (30, 30), 15)
        
        self.redraw()


    def render(self, payload:RobotPose):

        # Update robot pose dictionary with new data
        self.robot_poses[payload.robot_id] = payload.pose.pose.pose #[position vector3, orientation vector3]

        self.surface.fill(self.GREY)

        for key, value in self.robot_poses.items():

            x = value.position.x
            y = value.position.y
            theta = value.orientation.z

            ### render pygame window
            pygame.draw.circle(self.surface, self.BLACK, (int(value.position.x), int(value.position.y)), 15)
            #pygame.draw.line(self.surface, self.BLACK, (int(x - 15/2*cos())), (), width=1)
            ###
        
        # Refresh display
        self.redraw()

    def redraw(self):
        scaled = pygame.transform.scale(self.surface, self.window.get_size())
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