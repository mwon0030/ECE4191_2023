#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Float32MultiArray

class Collision():
  def __init__(self):
    # Initialise variables
    self.front_left_dist = 200 # in cm
    self.front_right_dist = 200
    self.left_dist = 200
    self.right_dist = 200
    self.back_dist = 200
    self.threshold = 5
    self.arena_size = [120, 120]
    self.length = 25
    self.width = 21
    self.obstacle_flag = False
    self.near_wall = False
    
    self.x = 0
    self.y = 0
    self.th = 0
    
    # Publisher for obstacle detection
    self.obstacle_detect_pub = rospy.Publisher('obstacle_detect', Bool, queue_size=1)
    
    # Subscribers for sensors
    self.ds_front_left_sub = rospy.Subscriber('/ds_front_left', Float32, self.ds_front_left_cb)
    self.ds_front_right_sub = rospy.Subscriber('/ds_front_right', Float32, self.ds_front_right_cb)
    self.ds_left_sub = rospy.Subscriber('/ds_left', Float32, self.ds_left_cb)
    self.ds_right_sub = rospy.Subscriber('/ds_right', Float32, self.ds_right_cb)
    self.ds_back = rospy.Subscriber('/ds_back', Float32, self.ds_back_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)

    
    
  def ds_front_left_cb(self, data):
    self.front_left_dist = round(data.data, 4)
    
  def ds_front_right_cb(self, data):
    self.front_right_dist = round(data.data, 4)
    
  def ds_left_cb(self, data):
    self.left_dist = round(data.data, 4)
    
  def ds_right_cb(self, data):
    self.right_dist = round(data.data, 4)
    
  def ds_back_cb(self, data):
    self.back_dist = round(data.data, 4)

  def state_cb(self, data):
    self.x = data.data[0]
    self.y = data.data[1]
    self.th = data.data[2]
    
    
  def obstacle_detect(self):
    # self.check_sensor_diff = self.front_left_dist - self.front_right_dist
    # self.check_arena_size1 = (self.front_left_dist + self.front_right_dist)/2 + self.length + self.back_dist
    # # self.check_arena_size2 = self.left_dist + self.right_dist + self.width
    
    # if self.check_sensor_diff >= 5: # Check if distance between two front sensors are not close to each other
    #   self.obstacle_flag = True
    #   self.obstacle_detect_pub.publish(self.obstacle_flag)
    
    # # check collision that is flat 
    # elif self.check_arena_size1 <= self.arena_size[0] - self.threshold: 
    #   self.obstacle_flag = True
    #   self.obstacle_detect_pub.publish(self.obstacle_flag)
    
    # # diagonal obstacle
    # elif self.check_arena_size1 >= self.arena_size[0] + self.threshold: 
    #   self.obstacle_flag = True
    #   self.obstacle_detect_pub.publish(self.obstacle_flag)

    # else:
    #   self.obstacle_flag = False
    #   self.obstacle_detect_pub.publish(self.obstacle_flag)
      
    # elif self.check_arena_size2 <= self.arena_size[0] - self.threshold:
    #   self.obstacle_detect_pub.publish(True)

    dist_threshold = 15

    # near wall 
    if self.x < 45 or self.x > 100: 
      self.near_wall = True 
    elif self.y < 20 or self.y > 100: 
      self.near_wall = True 
    else: 
      self.near_wall = False

    if (self.front_left_dist < dist_threshold or self.front_right_dist < dist_threshold) and not self.near_wall: 
      self.obstacle_detect_pub.publish(True)
    else:
      self.obstacle_detect_pub.publish(False)
      


if __name__ == "__main__":
  rospy.init_node('objection_detection')
  obstacle_detection = Collision()
  
  while not rospy.is_shutdown():
    try:
      obstacle_detection.obstacle_detect()
    except rospy.ROSInterruptException:
      break