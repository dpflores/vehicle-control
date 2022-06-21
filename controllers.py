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
    def __init__(self,k=1, ks=0.5):
        self.k = k
        self.ks = ks
        

    def control(self, heading, e, v):

        steer = heading + np.arctan(self.k*e/(self.ks + v))
        return steer