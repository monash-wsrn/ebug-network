import os
import math

# Screen dimensions (in m)
ARENA_LEFT = float(os.getenv('ARENA_LEFT', "-1.08"))            # m
ARENA_RIGHT = float(os.getenv('ARENA_RIGHT', "1.08"))           # m

ARENA_TOP = float(os.getenv('ARENA_TOP', "0.70"))               # m
ARENA_BOTTOM = float(os.getenv('ARENA_BOTTOM', "-0.70"))        # m

BUFFER_SPACE = float(os.getenv('BORDER_BUFFER', "0.20"))        # m


V_UPPER = ARENA_TOP - BUFFER_SPACE
V_LOWER = ARENA_BOTTOM + BUFFER_SPACE

H_UPPER = ARENA_RIGHT - BUFFER_SPACE
H_LOWER = ARENA_LEFT + BUFFER_SPACE


# Boid parameters
MAX_FORWARD_SPEED = float(os.getenv('MAX_FORWARD_SPEED', "0.50"))       # m /s 
MAX_ANGULAR_SPEED = float(os.getenv('MAX_ANGULAR_SPEED', "50.0"))       # m /s 
SEPARATION_DISTANCE = float(os.getenv('SEPARATION_DISTANCE', "0.20"))   # m
VIEW_DISTANCE = float(os.getenv('VIEW_DISTANCE', "0.40"))               # m

ALIGNMENT_FACTOR = float(os.getenv('ALIGNMENT_FACTOR', "0.01"))
COHESION_FACTOR = float(os.getenv('COHESION_FACTOR', "0.01"))
SEPARATION_FACTOR = float(os.getenv('SEPARATION_FACTOR', "1.0"))


# Function to implement Boid rules
def next(main_boid, other_boids):
    led_colour = [255, 255, 255]

    aggregate_cohesion = Vec2(0.0, 0.0)
    aggregate_alignment = Vec2(0.0, 0.0)
    aggregate_separation = Vec2(0.0, 0.0)
    
    count_cohesion = 0.0
    count_alignment = 0.0
    count_separation  = 0.0
    
    main_pos = Vec2.from_pose(main_boid)
    for other_boid in other_boids:
        other_pos = Vec2.from_pose(other_boid) 
        
        diff = other_pos.sub(main_pos)
        dist = diff.length()
        
        if dist <= VIEW_DISTANCE:
            # Cohesion
            norm = diff.normalised()
            aggregate_cohesion= aggregate_cohesion.add(norm)
            count_cohesion += 1
            
            # Alignment
            other_dir = Vec2.from_angle(pose_angle(other_boid))
            aggregate_alignment = aggregate_alignment.add(other_dir.normalised())
            count_alignment += 1
            
            # Separation
            if (dist <= SEPARATION_DISTANCE):
                aggregate_separation = aggregate_separation.add(norm.inversed())
                count_separation += 1
        
    print(aggregate_cohesion.x, aggregate_cohesion.y)
    
    aggregate_cohesion = aggregate_cohesion.divide(count_cohesion)
    aggregate_alignment = aggregate_alignment.divide(count_alignment)
    aggregate_separation = aggregate_separation.divide(count_separation)
    
    aggregate_cohesion = aggregate_cohesion.scale(COHESION_FACTOR)
    aggregate_alignment = aggregate_alignment.scale(ALIGNMENT_FACTOR)
    aggregate_separation = aggregate_separation.scale(SEPARATION_FACTOR)
    
    if (count_cohesion > 0):
        led_colour = [0, 255, 0]
    if (count_alignment > 0):
        led_colour = [0, 0, 255]
    if (count_separation > 0):
        led_colour = [255, 0, 0]
    
    boundary = Vec2(0.0, 0.0)
    
    if ((main_pos.x < H_LOWER) or
        (main_pos.x > H_UPPER) or
        (main_pos.y < V_LOWER) or 
        (main_pos.y > V_UPPER)):
        boundary = main_pos.normalised().inversed()
    
    
    direction = boundary.add(aggregate_cohesion).add(aggregate_alignment).add(aggregate_separation)
    angular_delta  = direction.angle() * min(direction.length(), 1.0) * MAX_ANGULAR_SPEED

    # Return (Linear Velocity Forward, Angular Velocity Yaw)
    return (MAX_FORWARD_SPEED, angular_delta, led_colour)


    
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