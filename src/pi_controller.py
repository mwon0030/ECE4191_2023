#! /usr/bin/env python

class PIController:
    def __init__(self, kp, ki, upper_limit, lower_limit):
        self.kp = kp
        self.ki = ki
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error
        control_effort = self.kp * error + self.ki * self.integral

        if control_effort > self.upper_limit:
            return self.upper_limit
        elif control_effort < self.lower_limit: 
            return self.lower_limit
        
        return control_effort
    
    def update_angle(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error
        control_effort = self.kp * error + self.ki * self.integral
        print('error: ', error)
        print('control effort: ', control_effort)
        if control_effort > self.upper_limit:
            return self.upper_limit
        elif control_effort < self.lower_limit: 
            return self.lower_limit
        
        return control_effort