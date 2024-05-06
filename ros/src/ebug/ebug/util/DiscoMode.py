import os
import math

def party(main_boid, other_boids):

    current_angle = pose_angle(main_boid)

    linear_velocity = 0
    angular_velocity = 1

    if 0 <= current_angle < 2.1:
        led_colour = (255, 0, 0)
    elif 2.1<= current_angle < 4.2:
        led_colour = (0, 255, 0)
    else:
        led_colour = (0, 0, 255)

    return (linear_velocity, angular_velocity, led_colour)

# Function to get the Euler Z angle of a boid
def pose_angle(pose):
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

    # Calculate yaw/z component from quaternion
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return clamp(yaw_z)