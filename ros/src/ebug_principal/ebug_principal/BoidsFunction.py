import os
import math

# Screen dimensions (in mm)
ARENA_WIDTH = int(os.getenv('ARENA_WIDTH', "2000"))         # mm
ARENA_HEIGHT = int(os.getenv('ARENA_HEIGHT', "2000"))       # mm
BUFFER_SPACE = float(os.getenv('BORDER_BUFFER', "0.10"))    # proportion

LOWER_WIDTH = int(ARENA_WIDTH * BUFFER_SPACE)
UPPER_WIDTH = ARENA_WIDTH - LOWER_WIDTH

LOWER_HEIGHT = int(ARENA_HEIGHT * BUFFER_SPACE)
UPPER_HEIGHT = ARENA_HEIGHT - LOWER_HEIGHT

# Boid parameters
MAX_FORWARD_SPEED = float(os.getenv('MAX_FORWARD_SPEED', "0.50"))         # m /s 
MAX_ANGULAR_SPEED = float(os.getenv('MAX_ANGULAR_SPEED', "5.00"))         # degrees /s 
SEPARATION_DISTANCE = float(os.getenv('SEPARATION_DISTANCE', "0.125"))    # mm
VIEW_DISTANCE = float(os.getenv('VIEW_DISTANCE', "0.300"))                # mm

ALIGNMENT_FACTOR = float(os.getenv('ALIGNMENT_FACTOR', "0.01"))
COHESION_FACTOR = float(os.getenv('COHESION_FACTOR', "0.001"))
SEPARATION_FACTOR = float(os.getenv('SEPARATION_FACTOR', "1.0"))


# Function to implement Boid rules
def next(main_boid, other_boids):# Function to implement Boid rules
    cohesion = [0, 0]
    alignment = [0, 0]
    separation = [0, 0]
    count_cohesion = 0
    count_alignment = 0
    count_separation = 0
    led_colour = (255, 255, 255)
    
    for other_boid in other_boids:
        if distance(main_boid, other_boid) < VIEW_DISTANCE:
            cohesion[0] += other_boid.position.x # find average x position of all other boids
            cohesion[1] += other_boid.position.y # find average y position of all other boids
            count_cohesion += 1

            alignment[0] += math.cos(angle(other_boid))
            alignment[1] += math.sin(angle(other_boid))
            count_alignment += 1

            if distance(main_boid, other_boid) < SEPARATION_DISTANCE:
                separation[0] += (main_boid.position.x - other_boid.position.x)
                separation[1] += (main_boid.position.y - other_boid.position.y)
                count_separation += 1
    
    resultant_angle = angle(main_boid)
    if count_cohesion > 0:
        cohesion[0] /= count_cohesion
        cohesion[1] /= count_cohesion
        target_angle = angle_between(main_boid, cohesion[0], cohesion[1]) # TODO Build pose from cohesion ??
        resultant_angle = (1 - COHESION_FACTOR) * angle(main_boid) + COHESION_FACTOR * target_angle
        led_colour = (0, 255, 0)

    if count_alignment > 0:
        alignment[0] /= count_alignment
        alignment[1] /= count_alignment
        target_angle = math.atan2(alignment[1], alignment[0])
        resultant_angle = (1 - ALIGNMENT_FACTOR) * angle(main_boid) + ALIGNMENT_FACTOR * target_angle
        led_colour = (0, 0, 255)

    if count_separation > 0:
        separation[0] /= count_separation
        separation[1] /= count_separation
        target_angle = math.atan2(separation[1], separation[0])
        resultant_angle = (1 - SEPARATION_FACTOR) * angle(main_boid) + SEPARATION_FACTOR * target_angle
        led_colour = (255, 0, 0)


    # TODO set desired linear velocity (this should just be constant ??)
    linear_velocity = MAX_FORWARD_SPEED
    resultant_angle = clamp(resultant_angle)
    x_comp = math.cos(resultant_angle)
    y_comp = math.sin(resultant_angle) 
    
     # negative is towards bottom, positive is towards up
    if (main_boid.position.x < 50 and x_comp < 0) or (main_boid.position.x > 550 and x_comp > 0):
        resultant_angle = math.pi - resultant_angle
    
    
    resultant_angle = clamp(resultant_angle)
    x_comp = math.cos(resultant_angle)
    y_comp = math.sin(resultant_angle) 
    if (main_boid.position.y < 50 and y_comp < 0) or (main_boid.position.y > 550 and y_comp > 0):
        resultant_angle = 2*math.pi - resultant_angle
    
    
    resultant_angle = clamp(resultant_angle)
    # TODO compare resultant_angle with angle(main_boid) to determine angular velocity
    # Maybe keep a constant linear velocity and only worry about signing it
    angular_velocity = clamp(resultant_angle - angle(main_boid))

    if (angular_velocity > math.pi):
        angular_velocity -= 2*math.pi

    # Return (Linear Velocity Forward, Angular Velocity Yaw)
    return (linear_velocity, angular_velocity, led_colour)


def clamp(angle):
    return angle % (2*math.pi)

# Function to calculate distance between two boids
def distance(pose1, pose2):
    return math.sqrt(
            ((pose1.position.x - pose2.position.x) ** 2)
          + ((pose1.position.y - pose2.position.y) ** 2) )


# Function to calculate angle between two boids
def angle_between(pose1, other_x, other_y):
    return math.atan2(other_y - pose1.position.y, other_x - pose1.position.x)


# Function to get the Euler Z angle of a boid
def angle(pose):
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

    # Calculate yaw/z component from quaternion
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return clamp(yaw_z)


def sign(num):
    return -1 if num < 0 else 1