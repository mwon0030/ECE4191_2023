#! /usr/bin/env python

import time

class PIController:
    def __init__(self, kp, ki, kt, upper_limit, lower_limit):
        self.kp = kp
        self.ki = ki
        self.kt = kt
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_u = 0 
        self.prev_v = 0
    
    def update(self, setpoint, feedback ):
        error = setpoint - feedback
        self.integral += error
        control_effort = self.kp * error + self.ki * self.integral

        if control_effort > self.upper_limit:
            return self.upper_limit
        elif control_effort < self.lower_limit: 
            return self.lower_limit
        
        return control_effort
    
    def update_angle(self, setpoint, feedback, prev_time):
        error = setpoint - feedback
        dt = time.time() - prev_time
        self.integral += error * dt
        es =  self.prev_u - self.prev_v
        control_effort = self.kp * error + self.ki * self.integral + self.kt * es
        
        self.prev_v = control_effort
        
        print('integral: ', self.integral)
        print('error: ', error)
        print('control effort: ', control_effort)
        print('es: ', es)
        
        if control_effort > self.upper_limit:
            self.prev_u = self.upper_limit
            return self.prev_u
        elif control_effort < self.lower_limit:
            self.prev_u = self.lower_limit 
            return self.prev_u
        else:
            self.prev_u = control_effort
            return self.prev_u
        
    
    def update_angle2(self, setpoint, feedback, prev_time):
        error = setpoint - feedback
        dt = time.time() - prev_time
        self.integral += error * dt
        control_effort = self.kp * error + self.ki * self.integral
        
        print('integral: ', self.integral)
        print('error: ', error)
        print('control effort: ', control_effort)
        
        if control_effort > self.upper_limit:
            return self.upper_limit
        elif control_effort < self.lower_limit:
            return self.lower_limit
        else:
            return control_effort