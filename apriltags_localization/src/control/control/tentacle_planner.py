import numpy as np
from .obstacle import Obstacle
import math

class TentaclePlanner:
    
    def __init__(self,robot_id, obstacles=[],dt=0.1,steps=2,alpha=1,beta=0.1):

        self.robot_id = robot_id
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.20, 0.0), (-0.20, 0.0), (0.05, 0.0), (-0.05, 0.0), (0.0, 3.0), (0.0, -3.0), (0.0, 2.2), (0.0, -2.2), (0.1, 3.0), (0.1, -3.0)]
        
        self.alpha = alpha
        self.beta = beta
        
        self.obstacles = obstacles

        self.e_th = 0.0
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y, goal_th, x,y,th):
        
        for _ in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
            if (self.check_collision(x,y)):
                return np.inf, np.inf



        e_th = goal_th-th
        self.e_th = np.arctan2(np.sin(e_th), np.cos(e_th))



        dist = math.sqrt(((goal_x-x)**2 + (goal_y-y)**2))
        
        cost = self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)

        return cost, dist
    
    def check_collision(self,x,y):

        
        for obj in self.obstacles:

            if obj.check_collisions(x,y):

                return True

        return False
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y, goal_th,x,y,th):
        
        costs =[]
        dists = []
        for v,w in self.tentacles:
            res = self.roll_out(v,w,goal_x,goal_y, goal_th,x,y,th)
            costs.append(res[0])
            dists.append(res[1])
            
        
        best_idx = np.argmin(costs)

        if dists[best_idx] < 0.1 and abs(self.e_th)<0.1:
            return (0.0, 0.0)

        return self.tentacles[best_idx]