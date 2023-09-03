#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time
import numpy as np

class Localisation():
  def __init__(self):
    self.length = 25 # in cm
    self.width = 21
    self.max_arena_size = 120
    self.wheel_rad = 2.714
    self.wheel_circum = 2 * np.pi * self.wheel_rad
    self.wheel_width = 21.5 # the distance between the left and right wheels
    
    self.front_left_dist = 200
    self.front_right_dist = 200
    self.left_dist = 200
    self.right_dist = 200
    self.left_motor_speed = 0
    self.right_motor_speed = 0
    self.x = 0
    self.y = 0
    self.th = 0
    
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)
    self.left_motor_sub = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor_sub = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    
    self.prev_time = time.time()
  
  def ds_front_left_cb(self, data):
    self.front_left_dist = round(data.data, 4)
    
  def ds_front_right_cb(self, data):
    self.front_right_dist = round(data.data, 4)
    
  def ds_left_cb(self, data):
    self.left_dist = round(data.data, 4)
    
  def ds_right_cb(self, data):
    self.right_dist = round(data.data, 4)
    
  def left_motor_cb(self, data):
    self.left_motor_speed = data.data
  
  def right_motor_cb(self, data):
    self.right_motor_speed = data.data
  
  def localise_motor(self):
    self.time = time.time() - self.prev_time
    self.th = self.th + (-(self.left_motor_speed * self.wheel_circum * self.time + self.right_motor_speed * self.wheel_circum * self.time))/self.wheel_width
    self.x = self.x + ((self.left_motor_speed * self.wheel_circum * self.time - self.right_motor_speed * self.wheel_circum * self.time)/2) * np.sin(self.th)
    self.y = self.y + ((self.left_motor_speed * self.wheel_circum * self.time - self.right_motor_speed * self.wheel_circum * self.time)/2) * np.cos(self.th)
    self.prev_time = time.time()
    print('x: ', self.x, '   y: ', self.y, '     th: ', self.th, '     time: ', self.time)
    rospy.sleep(0.07)
  
  def localise_sensor(self):
    for _ in range(5):
      self.x = ((self.left_dist + self.width/2) + (self.max_arena_size - self.right_dist - self.width/2))/2
      self.y = ((self.max_arena_size - self.front_left_dist - self.length/2) + (self.max_arena_size - self.front_right_dist - self.length/2))/2
      print('x: ', self.x, '   y: ', self.y)
      rospy.sleep(0.1)
    self.prev_time = time.time()

if __name__ == '__main__':
  rospy.init_node('localisation')
  localiser = Localisation()
  localiser.localise_sensor()
  
  while not rospy.is_shutdown():
    localiser.localise_motor()
      