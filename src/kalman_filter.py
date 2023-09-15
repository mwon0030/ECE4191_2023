#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
import time

class KalmanFilter():
  def __init__(self) -> None:
    self.P = np.zeros((3,3))
    self.F = np.eye(3)
    self.left_motor_speed = 0
    self.right_motor_speed = 0
    wheel_rad = 2.715
    self.wheel_circum = 2 * np.pi * wheel_rad
    self.linear_velocity = 0
    self.angular_velocity = 0
    self.wheels_width = 21.3 # the distance between the left and right wheels
    self.dt = time.time()
    self.state = np.zeros((3,1))
    
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
  
  def left_motor_cb(self, data) -> None:
    self.left_motor_speed = data.data
    
  def right_motor_cb(self, data) -> None:
    self.right_motor_speed = data.data
  
  def convert_wheel_speeds(self) -> float:
    # Convert encoder readings to m/s
    left_speed = self.left_motor_speed * self.wheel_circum
    right_speed = self.right_motor_speed * self.wheel_circum
    
    # Computer linear and angular velocity
    linear_velocity = (left_speed + right_speed) / 2.0
    angular_velocity = (right_speed - left_speed) / self.wheels_width
    
    return linear_velocity, angular_velocity
    
  def drive(self) -> None:
    linear_velocity, angular_velocity = self.convert_wheel_speeds()
    
    self.dt = time.time() - self.dt
    self.state[0] += np.cos(self.state[2]) * linear_velocity * self.dt
    self.state[1] += np.sin(self.state[2]) * linear_velocity * self.dt
    self.state[2] += self.dt*angular_velocity
    
  def derivative_drive(self):
    F = np.eye(3)
    
    linear_velocity, angular_velocity = self.convert_wheel_speeds()
    th = self.state[2]
    
    
  
  def predict(self) -> None:
    F = 