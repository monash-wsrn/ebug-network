import numpy as np
from .obstacle import Obstacle
import numpy as np

class NaivePlanner:
    
    def __init__(self,robot_id):

        self.robot_id = robot_id

    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y, goal_th,x,y,th):
        
        e_th = goal_th - th
        e_th = np.arctan2(np.sin(e_th), np.cos(e_th))
        dist = np.sqrt((goal_x - x)**2 + (goal_y-y)**2)
        if dist > 0.10:
            if abs(e_th) > 0.05:
                if e_th > 0:
                    v_desired, w_desired = 0.0, 2.5

                else:
                    v_desired, w_desired = 0.0, -2.5

            else:
                    v_desired, w_desired = 0.2, 0.0

        else:
            v_desired, w_desired = 0.0, 0.0

        return v_desired, w_desired
