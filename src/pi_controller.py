#! /usr/bin/env python

class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error
        control_effort = self.kp * error + self.ki * self.integral

        if control_effort > 1:
            return 1
        elif control_effort < 0: 
            return 0
        
        return control_effort