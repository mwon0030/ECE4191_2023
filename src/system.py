#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray
from distance_sensor_cluster import DistanceSensorCluster
from distance_sensor_info import DistanceSensorInfo
from motor_control import MotorControl
from obstacle_detect import obstacle_detect

class System():
  def __init__(self):
    self.front_left_sensor_dist = 200 
    self.front_right_sensor_dist = 200
    self.left_sensor_dist = 200
    self.right_sensor_dist = 200
    self.x = 0
    self.y = 0
    self.th = 0
    self.is_turning = False
    self.flag_0 = 0
    self.flag_1 = 0
    
    self.Kp = 0.1
    self.dist_threshold = 1
    self.angle_threshold = np.pi/60
    
    self.front_left_sensor = rospy.Subscriber('/distance_sensor_0', Float32, self.front_left_sensor_cb)
    self.front_right_sensor = rospy.Subscriber('/distance_sensor_1', Float32, self.front_right_sensor_cb)
    self.left_sensor = rospy.Subscriber('/distance_sensor_2', Float32, self.left_sensor_cb)
    self.right_sensor = rospy.Subscriber('/distance_sensor_3', Float32, self.right_sensor_cb)
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    
    self.turning_pub = rospy.Publisher('turning', Bool, queue_size=1)
    self.set_left_motor_speed_pub = rospy.Publisher('set_left_motor_speed', Float32, queue_size=1)
    self.set_right_motor_speed_pub = rospy.Publisher('set_right_motor_speed', Float32, queue_size=1)
  
    
  def front_left_sensor_cb(self, data):
    self.front_left_sensor_dist = data.data
    
  def front_right_sensor_cb(self, data):
    self.front_right_sensor_dist = data.data
    
  def left_sensor_cb(self, data):
    self.left_sensor_dist = data.data
    
  def right_sensor_cb(self, data):
    self.right_sensor_dist = data.data
    
  def left_motor_cb(self, data):
    self.left_motor_speed = data.data
    
  def right_motor_cb(self, data):
    self.right_motor_speed = data.data
    
  def state_cb(self, data):
    self.x = data.data[0]
    self.y = data.data[1]
    self.th = data.data[2]

  def drive(self, dist_error):
    # angle_error = self.angle_from_goal(goal_location)
    # if angle_error > self.angle_threshold:
    #   self.turn(angle_error + self.th)
    self.set_left_motor_speed_pub.publish(self.Kp * dist_error)
    self.set_right_motor_speed_pub.publish(self.Kp * dist_error)
    # elif direction == 1:
    #   self.set_left_motor_speed_pub.publish(self.Kp * dist_error)
    #   self.set_right_motor_speed_pub.publish(self.Kp * dist_error)
    
  def turn(self, goal_angle):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    angle_error = self.angle_from_goal(goal_location)
    goal_angle = angle_error + self.th
    diff = abs(goal_angle - self.th)
    print(diff)
    if goal_angle > self.th and diff > self.angle_threshold:
      self.set_left_motor_speed_pub.publish(-self.Kp * diff)
      self.set_right_motor_speed_pub.publish(self.Kp * diff)
      
    elif goal_angle < self.th and diff > self.angle_threshold:
      self.set_left_motor_speed_pub.publish(self.Kp * diff)
      self.set_right_motor_speed_pub.publish(-self.Kp * diff)
      
    else:
      self.set_left_motor_speed_pub.publish(0)
      self.set_right_motor_speed_pub.publish(0)
      self.flag_0 = 0
      self.flag_1 = 0
      self.is_turning = False
      self.turning_pub.publish(self.is_turning)
      rospy.sleep(15)
  
  def distance_from_goal(self, goal_location):
    x_diff = abs(float(goal_location[0]) - self.x)
    y_diff = abs(float(goal_location[1]) - self.y)
    distance_to_goal = np.hypot(x_diff, y_diff)
    return distance_to_goal
  
  def angle_from_goal(self, goal_location):
    x_diff = float(goal_location[0]) - self.x
    y_diff = float(goal_location[1]) - self.y
    angle_to_goal = self.clamp_angle(np.arctan2(y_diff, x_diff) - self.th)
    return angle_to_goal
  
  def clamp_angle(self, rad_angle, min_value=-np.pi, max_value=np.pi):
    if min_value > 0:
      min_value *= -1
    angle = (rad_angle + max_value) % (2 * np.pi) + min_value
    return angle
  
  def path_planning(self, goal_location):
    tot_error = self.distance_from_goal(goal_location)
    if tot_error >= np.hypot(self.dist_threshold, self.dist_threshold):
      if ((self.th >= np.pi/2 - self.angle_threshold) and (self.th <= np.pi/2 + self.angle_threshold)) or ((self.th >= -np.pi/2 - self.angle_threshold) and (self.th <= -np.pi/2 + self.angle_threshold)) or self.flag_0: # When pi/2 or -pi/2
        x_error = abs(goal_location[0] - self.x)
        y_error = abs(goal_location[1] - self.y)
        if y_error >= self.dist_threshold and not self.flag_0:
          self.drive(y_error)
          
        elif x_error >= self.dist_threshold:
          self.flag_0 = 1
          self.turn(goal_location)
          
      elif self.th >= (np.pi - self.angle_threshold) or self.th <= (-np.pi + self.angle_threshold) or (self.th >= -self.angle_threshold and self.th <= self.angle_threshold) or self.flag_1: # When 0, pi or -pi
        x_error = abs(goal_location[0] - self.x)
        y_error = abs(goal_location[1] - self.y)
        print(x_error)
        if x_error >= self.dist_threshold and not self.flag_1:
          self.drive(x_error)
        
        elif y_error >= self.dist_threshold:
          self.flag_1 = 1
          self.turn(goal_location)
    else:
      print("Goal has been reached!")
      rospy.sleep(5)
    
if __name__ == "__main__":
  rospy.init_node('system')
  robot = System()
  rospy.sleep(1)
  goal_location = [60, 60]
  while not rospy.is_shutdown():
    try:
      robot.path_planning(goal_location)
    except rospy.ROSInterruptException:
      break