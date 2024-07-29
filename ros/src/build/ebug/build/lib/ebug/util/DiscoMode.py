import os
import math

# Screen dimensions (in m)
ARENA_LEFT = float(os.getenv('ARENA_LEFT', "-1.08"))            # m
ARENA_RIGHT = float(os.getenv('ARENA_RIGHT', "1.08"))           # m

ARENA_TOP = float(os.getenv('ARENA_TOP', "0.70"))               # m
ARENA_BOTTOM = float(os.getenv('ARENA_BOTTOM', "-0.70"))        # m

BUFFER_SPACE = float(os.getenv('BORDER_BUFFER', "0.20"))        # m

MAX_FORWARD_SPEED = float(os.getenv('MAX_FORWARD_SPEED', "0.100000"))   # m /s 
MAX_ANGULAR_SPEED = float(os.getenv('MAX_ANGULAR_SPEED', "0.785398"))   # rads /s

def party(main_boid, other_boids):

    current_angle = pose_angle(main_boid)

    linear_velocity = MAX_FORWARD_SPEED
    angular_velocity = MAX_ANGULAR_SPEED

    if 0 <= current_angle < 2.1:
        led_colour = (255, 0, 0)
    elif 2.1<= current_angle < 4.2:
        led_colour = (0, 255, 0)
    else:
        led_colour = (0, 0, 255)

    return (linear_velocity, angular_velocity, led_colour)

class Vec2():
    def __init__(self, x_ = 0.0, y_ = 0.0):
        self.x = x_
        self.y = y_
        
    def from_pose(pose):
        return Vec2(pose.position.x, pose.position.y)
    
    def from_angle(radians):
        return Vec2( math.cos(radians), math.sin(radians) )
    
    
    def sub(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    
    def add(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    
    def scale(self, scale):
        return Vec2(self.x * scale, self.y * scale)
    
    def divide(self, denom):
        if (denom == 0.0):
            return Vec2(0.0, 0.0)
        return self.scale(1.0 / denom)
    
    def inversed(self):
        return self.scale(-1.0)
    
    def length(self):
        return math.sqrt((self.x ** 2) + (self.y ** 2))
    
    def normalised(self):
        return self.divide(self.length())
    
    def angle(self):
        return math.atan2(self.x, self.y)


# Function to get the Euler Z angle of a boid
def pose_angle(pose):
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

    # Calculate yaw/z component from quaternion
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)