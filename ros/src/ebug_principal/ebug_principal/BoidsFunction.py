import math

# Screen dimensions (in mm)
ARENA_WIDTH = 2000      # mm
ARENA_HEIGHT = 2000     # mm
BUFFER_SPACE = 0.10     # proportion


LOWER_WIDTH = int(ARENA_WIDTH * BUFFER_SPACE)
UPPER_WIDTH = ARENA_WIDTH - LOWER_WIDTH

LOWER_HEIGHT = int(ARENA_HEIGHT * BUFFER_SPACE)
UPPER_HEIGHT = ARENA_HEIGHT - LOWER_HEIGHT

# Boid parameters
MAX_SPEED = 50              # mm /s 
SEPARATION_DISTANCE = 100   # mm
ALIGNMENT_FACTOR = 0.01
COHESION_FACTOR = 0.01
SEPARATION_FACTOR = 0.01

# Function to calculate distance between two boids
def distance(pose1, pose2):
    return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 
          + (pose1.position.y - pose2.position.y) ** 2)

# Function to calculate angle between two boids
def angle_between(pose1, other_x, other_y):
    return math.atan2(other_y - pose1.position.y, other_x - pose1.position.x)

# Function to get the Euler Z angle of a boid
def angle(pose):
    roll, pitch, yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return yaw

def sign(num):
    return -1 if num < 0 else 1

# Function to implement Boid rules
def next(main_boid, other_boids):
    cohesion = [0, 0]
    alignment = [0, 0]
    separation = [0, 0]
    count_cohesion = 0
    count_alignment = 0
    count_separation = 0
    
    for other_boid in other_boids:
        cohesion[0] += other_boid.position.x # find average x position of all other boids
        cohesion[1] += other_boid.position.y # find average y position of all other boids
        count_cohesion += 1
        
        alignment[0] += math.cos(angle(other_boid))  # TODO get angle??? other_boid.orientation is a Quaternion
        alignment[1] += math.sin(angle(other_boid))  # TODO get angle??? other_boid.orientation is a Quaternion
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

    if count_alignment > 0:
        alignment[0] /= count_alignment
        alignment[1] /= count_alignment
        target_angle = math.atan2(alignment[1], alignment[0])
        resultant_angle = (1 - ALIGNMENT_FACTOR) * angle(main_boid) + ALIGNMENT_FACTOR * target_angle

    if count_separation > 0:
        separation[0] /= count_separation
        separation[1] /= count_separation
        target_angle = math.atan2(separation[1], separation[0])
        resultant_angle = (1 - SEPARATION_FACTOR) * angle(main_boid) + SEPARATION_FACTOR * target_angle


    # TODO set desired linear velocity (this should just be constant ??)
    linear_velocity = MAX_SPEED

    x_comp = math.cos(resultant_angle) # negative is towards left, positive is towards right
    y_comp = math.sin(resultant_angle) # negative is towards bottom, positive is towards up
    if      (main_boid.position.x < LOWER_WIDTH and x_comp < 0) \
        or  (main_boid.position.x > UPPER_WIDTH and x_comp > 0):
        resultant_angle = math.acos(-x_comp)
        linear_velocity = 0
    
    if      (main_boid.position.y < LOWER_HEIGHT and y_comp < 0) \
        or  (main_boid.position.y > UPPER_HEIGHT and y_comp > 0):
        resultant_angle = math.asin(-y_comp)
        linear_velocity = 0
    
    # TODO compare resultant_angle with angle(main_boid) to determine angular velocity
    # Maybe keep a constant linear velocity and only worry about signing it
    angular_velocity = 1 * sign(resultant_angle - angle(main_boid))

    led_colour = (255, 0, 0)
    # Return (Linear Velocity Forward, Angular Velocity Yaw)
    return (linear_velocity, angular_velocity, led_colour)


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