import numpy as np

class Obstacle():

    def __init__(self, name:str, pos_x: float, pos_y:float, covar:list[4] = [], prohibited_rad: float = 0, confidence_intervals = 0.999):

        self.name = name
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.covar = covar
        self.prohibited_rad = prohibited_rad


    def check_collisions(self, x, y):
        # need to take covariance into account

        dist = np.sqrt(((x-self.pos_x)**2)+((y-self.pos_y)**2))

        if dist < self.prohibited_rad:

            return True

        return False



    

