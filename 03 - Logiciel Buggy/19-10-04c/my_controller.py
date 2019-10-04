import numpy as np
from my_math import  *

# pid
class pid:
    def __init__(self,kp=1.0, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.0):
        # settings
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max  = integral_max
        self.output_max  = output_max
        self.alpha = alpha
        # state
        self.error = 0.0
        self.filtered_error = 0.0
        self.last_filtered_error = 0.0
        self.derivative = 0.0
        self.integral_window = np.zeros((30)) # sliding window depth of 30 consecutive errors (0.5 second)
        self.integral_error = 0.0
        self.output = 0.0
   
    def compute(self,error):
        # update
        self.error = error
        self.last_filtered_error = self.filtered_error
        self.filtered_error = self.filtered_error*(1.0-self.alpha) + self.alpha*self.error # EWMA
        self.derivative =  self.filtered_error -  self.last_filtered_error
        self.integral_window[:-1] = self.integral_window[1:] # slice & slide
        self.integral_window[-1] = self.error
        self.integral_error = np.sum(self.integral_window)
        # windup (staturate)
        self.integral_error = constraint(self.integral_error, -self.integral_max, self.integral_max )
        # pid
        p = self.error * self.kp
        i = self.integral_error * self.ki
        d = self.derivative * self.kd
        # saturate and output
        self.output = constraint(p+i+d, -self.output_max, self.output_max)
        return self.output
