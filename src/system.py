#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray
from pi_controller import PIController
from obstacle_detect import obstacle_detect

class System():
  def __init__(self):
    self.Kp = 0.05
    # self.Ki = 0.01
    # self.pi_controller = PIController(self.Kp, self.Ki, 1, -1)
    
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
    self.obstacle_detected = False
    
    # self.Kp = 0.3
    self.dist_threshold = 3
    self.angle_threshold = np.pi/22
    self.candidate_angles = [-np.pi, -np.pi/2, 0, np.pi/2, np.pi]
    
    self.front_left_sensor = rospy.Subscriber('/distance_sensor_0', Float32, self.front_left_sensor_cb)
    self.front_right_sensor = rospy.Subscriber('/distance_sensor_1', Float32, self.front_right_sensor_cb)
    self.left_sensor = rospy.Subscriber('/distance_sensor_2', Float32, self.left_sensor_cb)
    self.right_sensor = rospy.Subscriber('/distance_sensor_3', Float32, self.right_sensor_cb)
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    self.obstacle_detector = rospy.Subscriber('/obstacle_detect', Bool, self.obstacle_detect_cb)
    
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

  def obstacle_detect_cb(self, data): 
    self.obstacle_detected = data.data

  def drive(self):
    # self.set_left_motor_speed_pub.publish(self.Kp * dist_error)
    # self.set_right_motor_speed_pub.publish(self.Kp * dist_error)
    self.set_left_motor_speed_pub.publish(0.2)
    self.set_right_motor_speed_pub.publish(-0.2)
    
    
  def turn(self, goal_angle):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    # print('angle error: ', angle_error)
    # print('goal angle: ', goal_angle)
    diff = abs(goal_angle - self.th)
    # print('diff: ', diff)
    while diff > self.angle_threshold:
      # motor_speed_control_signal = self.pi_controller.update_angle(goal_angle, self.th)
      # motor_speed_control_signal = self.Kp * diff
      motor_speed_control_signal = 0.2
      if goal_angle > self.th:
        self.set_left_motor_speed_pub.publish(-motor_speed_control_signal)
        self.set_right_motor_speed_pub.publish(-motor_speed_control_signal)
        # print('motor control signal: ', motor_speed_control_signal)
        
      elif goal_angle < self.th:
        self.set_left_motor_speed_pub.publish(motor_speed_control_signal)
        self.set_right_motor_speed_pub.publish(motor_speed_control_signal)
        # print('motor control signal: ', motor_speed_control_signal)

      # angle_error = self.angle_from_goal(goal_location)
      diff = abs(goal_angle - self.th)
      # print('angle: ', self.th)
      # print('angle error: ', angle_error)
      # print('diff: ', diff)
      # rospy.sleep(0.05)
        
    self.set_left_motor_speed_pub.publish(0)
    self.set_right_motor_speed_pub.publish(0)
    self.is_turning = False
    self.turning_pub.publish(self.is_turning)
    rospy.sleep(3)

  def distance_from_goal(self, goal_location):
    x_diff = abs(float(goal_location[0]) - self.x)
    y_diff = abs(float(goal_location[1]) - self.y)
    distance_to_goal = np.hypot(x_diff, y_diff)
    return distance_to_goal
  
  def angle_from_goal(self, goal_location):
    x_diff = float(goal_location[0]) - self.x - self.dist_threshold
    y_diff = float(goal_location[1]) - self.y - self.dist_threshold
    angle_to_goal = self.clamp_angle(np.arctan2(y_diff, x_diff) - self.th)
    return angle_to_goal
  
  def clamp_angle(self, rad_angle, min_value=-np.pi, max_value=np.pi):
    if min_value > 0:
      min_value *= -1
    angle = (rad_angle + max_value) % (2 * np.pi) + min_value
    return angle
  
  def path_planning(self, goal_locations):
    for point in goal_locations:
      if not self.obstacle_detected:
        tot_error = self.distance_from_goal(point)
        while tot_error >= 5:
          x_error = abs(goal_location[0] - self.x)
          y_error = abs(goal_location[1] - self.y)
          
          if x_error >= self.dist_threshold and y_error > self.dist_threshold:
            self.dist_threshold = 9
          else:
            self.dist_threshold = 3
          
          if x_error >= self.dist_threshold:
            angle_error = self.angle_from_goal(goal_location)
            if angle_error >= self.angle_threshold:
              goal_angle = angle_error + self.th
              self.turn(goal_angle)
            else:
              self.drive()
              
          elif y_error >= self.dist_threshold:
            angle_error = self.angle_from_goal(goal_location)
            if angle_error >= self.angle_threshold:
              goal_angle = angle_error + self.th
              self.turn(goal_angle)
            else:
              self.drive()
            
        tot_error = self.distance_from_goal(point)
      
      else: 
        if self.left_sensor_dist > self.right_sensor_dist: 
           # turn left
          if (self.th >= (5/6) * np.pi): # When theta is pi 
            self.turn(-np.pi/2) 
          else: 
            self.turn(self.th + np.pi/2)

          # look in the right direction and travel forward until it becomes clear 
          while self.right_sensor_dist < self.x: 
            self.drive()
          
          # when clear turn back perform right turn 
          if (self.th <= (-5/6) * np.pi): # When theta is -pi
            self.turn(-np.pi/2)
          else:
            self.turn(self.th - np.pi/2)

          # self.th left_angle right_angle 
          # 90 180 0 
          # 0 90 -90
          # 180 -90 90 
          # -180 -0 90 
          # -90 0 -180

        else: 
          # turn right 
          if (self.th <= (-5/6) * np.pi): # When theta is -pi
            self.turn(-np.pi/2)
          else:
            self.turn(self.th - np.pi/2)

          # look in the right direction and travel forward until it becomes clear 
          while self.left_sensor_dist < self.x: 
            self.drive()
          
          # when clear turn back perform left turn 
          if (self.th >= (5/6) * np.pi): # When theta is pi 
            self.turn(-np.pi/2) 
          else: 
            self.turn(self.th + np.pi/2)

      self.set_left_motor_speed_pub.publish(0)
      self.set_right_motor_speed_pub.publish(0)
      print("Goal reached!")
      rospy.sleep(10)

  def path_planning_1(self, goal_location): 
    tot_error = self.distance_from_goal(goal_location)
    while tot_error >= 5:
      if not self.obstacle_detected:
        diff_angles = [abs(angle - self.th) for angle in self.candidate_angles]
        candidate_angle_idx = diff_angles.index(min(diff_angles))
        if candidate_angle_idx % 2:
          x_error = abs(goal_location[0] - self.x)
          y_error = abs(goal_location[1] - self.y)
          print('y error: ', y_error)
          if y_error >= self.dist_threshold:
            self.drive()
            
          elif x_error >= self.dist_threshold:
            self.turn(goal_location)
            # self.adjust()
        else:
          x_error = abs(goal_location[0] - self.x)
          print('x error: ', x_error)
          y_error = abs(goal_location[1] - self.y)
          if x_error >= self.dist_threshold:
            self.drive()
            
          elif y_error >= self.dist_threshold:
            self.turn(goal_location)

      else: 
        if self.left_sensor_dist > self.right_sensor_dist: # turn left 

          if (self.th >= (5/6) * np.pi): # When theta is pi 
            self.turn(-np.pi/2) 
          else: 
            self.turn(self.th + np.pi/2)

          # look in the right direction and travel forward until it becomes clear 
          while self.right_sensor_dist < self.x: 
            self.drive()
          
          # when clear turn back perform right turn 
          if (self.th <= (-5/6) * np.pi): # When theta is -pi
            self.turn(-np.pi/2)
          else:
            self.turn(self.th - np.pi/2)

          # self.th left_angle right_angle 
          # 90 180 0 
          # 0 90 -90
          # 180 -90 90 
          # -180 -0 90 
          # -90 0 -180

        else: # turn right 
          if (self.th <= (-5/6) * np.pi): # When theta is -pi
            self.turn(-np.pi/2)
          else:
            self.turn(self.th - np.pi/2)

          # look in the right direction and travel forward until it becomes clear 
          while self.left_sensor_dist < self.x: 
            self.drive()
          
          # when clear turn back perform left turn 
          if (self.th >= (5/6) * np.pi): # When theta is pi 
            self.turn(-np.pi/2) 
          else: 
            self.turn(self.th + np.pi/2)

  
    
if __name__ == "__main__":
  rospy.init_node('system')
  robot = System()
  rospy.sleep(1)
  goal_location = [60, 60]
  while not rospy.is_shutdown():
    try:
      robot.path_planning(goal_location)
      # robot.drive()
    except rospy.ROSInterruptException:
      break