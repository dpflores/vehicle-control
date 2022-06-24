# Set of controllers for lateral and longitudinal vehicle control

import numpy as np

class PIDController():
    
    def __init__(self,kp=0.5,ki=0.5,kd=0.1,Ts=0.01, limMin=-1.0, limMax=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts

        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        self.limMax = limMax
        self.limMin = limMin    
        self.prev_error = 0.0

    def control(self, error):
        

        # PROPORTIONAL
        self.proportional = self.kp*error

        # INTEGRAL
        self.integral += self.ki*self.Ts*(error)
        
        # limiting the integral part
        if (self.integral>self.limMax):     
            self.integral = self.limMax
        elif (self.integral<self.limMin):
            self.integral = self.limMin

        # DERIVATIVE
        self.derivative = self.kd * (error-self.prev_error)/self.Ts

        # OUTPUT
        u = self.proportional + self.integral + self.derivative

        # limiting output
        if (u>self.limMax):
            u = self.limMax
        elif (u<self.limMin):
            u = self.limMin

        # UPDATE
        self.prev_error = error

        return u

class StanleyController():
    def __init__(self,k=1, ks=0.01):
        self.k = k
        self.ks = ks
        
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def control(self, current_position, yaw, v, path0, path1):
        """
        Stranley Control.
        :current_position: [p0, p1], list of current 2D head position
        :yaw: current yaw angle
        :v: current forward velocity
        :path0: list of initial point of local path
        :path1: list of final point of local path
        """
        x = current_position[0]
        y = current_position[1]
        x0 = path0[0]
        y0 = path0[1]
        x1 = path1[0]
        y1 = path1[1]
        
        # path line equation
        line_coef = np.polyfit([x0, x1], [y0, y1], 1)
        a = -line_coef[0]; b = 1; c = -line_coef[1]
        
        # path angle
        path_angle = np.arctan2(y1-y0, x1-x0)
        # crosstrack error 
        crosstrack_error = (a*x + b*y + c)/np.sqrt(a**2 + b**2) # does not differe if its to the left or right

        angle_cross_track = np.arctan2(y-y0, x-x0) # gets the angle to the cross head of vehicle
        angle_path2ct = path_angle - angle_cross_track # gives the angle between path and the line form path start to front of vehicle
        angle_path2ct = self.normalize_angle(angle_path2ct) # keep -pi to pi

        if angle_path2ct > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = - abs(crosstrack_error)

        crosstrack_steer = np.arctan2(self.k * crosstrack_error , (v + self.ks)) 

        # heading error 
        heading = path_angle - yaw
        heading = self.normalize_angle(heading)  # keep -pi to pi

        
        steer = heading + crosstrack_steer

        return steer


class PurePursuit():

    def __init__(self, L=2, Kdd=0.5, ks = 0.01) -> None:
        self.L = L
        self.Kdd = Kdd
        self.ks =ks
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def control(self, current_position, yaw, v, path0, path1):
        """
        Pure Pursuit Control.
        :current_position: [p0, p1], list of current 2D head position
        :yaw: current yaw angle
        :v: current forward velocity
        :path0: list of initial point of local path
        :path1: list of final point of local path
        """
        x = current_position[0]
        y = current_position[1]
        
        xr = x - self.L*np.cos(yaw) # x position of rear axle
        yr = y - self.L*np.sin(yaw) # y position of rear axle
        
        x0 = path0[0]
        y0 = path0[1]
        x1 = path1[0]
        y1 = path1[1]

        alpha = np.arctan2(y1-yr, x1-xr) - yaw  # Alpha (angle between the look ahead line and vehicle)
        alpha = self.normalize_angle(alpha) # keep -pi to pi
        ld = self.Kdd * v  # lookahead distance

        steer = np.arctan(2*self.L*np.sin(alpha)/(ld + self.ks))
        return steer