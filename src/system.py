#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray
from pi_controller import PIController
from obstacle_detect import obstacle_detect
import time

class System():
  def __init__(self, goal_locations):
    self.Kp_turn = 0.5
    self.Ki_turn = 0.1
    self.Kt_turn = 1.5
    self.pi_controller = PIController(self.Kp_turn, self.Ki_turn, self.Kt_turn, 1, -1)
    
    self.goal_locations = goal_locations
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
    self.waypoint_reached = False
    
    self.Kp = 0.35
    self.dist_threshold = 3
    self.angle_threshold = np.pi/90
    
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

  def drive(self, goal):
    # self.set_left_motor_speed_pub.publish(self.Kp * dist_error)
    # self.set_right_motor_speed_pub.publish(-self.Kp * dist_error)
    distance_to_drive = self.distance_from_goal(goal)
    while distance_to_drive >= self.dist_threshold:
      # self.set_left_motor_speed_pub.publish(0.2)
      # self.set_right_motor_speed_pub.publish(-0.2)
      self.set_left_motor_speed_pub.publish(self.Kp * distance_to_drive)
      self.set_right_motor_speed_pub.publish(-self.Kp * distance_to_drive)
      distance_to_drive = self.distance_from_goal(goal)
    
    
  def turn(self, goal_angle):
    self.is_turning = True
    self.turning_pub.publish(self.is_turning)
    # angle_error = self.angle_from_goal(point)
    # print('angle error: ', angle_error)
    # goal_angle = angle_error + self.th
    # print("goal_angle: ", goal_angle)
    # print('goal angle: ', goal_angle)
    angle_error = abs(goal_angle - self.th)
    # print('diff: ', diff)
    # while diff > self.angle_threshold:
      # motor_speed_control_signal = self.Kp * diff
    curr_time = time.time()
    while angle_error > self.angle_threshold:
      motor_speed_control_signal = 0.1
      # motor_speed_control_signal = self.Kp * angle_error
      # motor_speed_control_signal = self.pi_controller.update_angle(goal_angle, self.th, curr_time)
      if goal_angle > self.th:
        self.set_left_motor_speed_pub.publish(-motor_speed_control_signal)
        self.set_right_motor_speed_pub.publish(-motor_speed_control_signal)
        # print('motor control signal: ', motor_speed_control_signal)
        
      elif goal_angle < self.th:
        self.set_left_motor_speed_pub.publish(motor_speed_control_signal)
        self.set_right_motor_speed_pub.publish(motor_speed_control_signal)
        # print('motor control signal: ', motor_speed_control_signal)
      curr_time = time.time()
      # print("time: ", curr_time)
      # angle_error = self.angle_from_goal(point)
      angle_error = abs(goal_angle - self.th)
      print("angle error: ", angle_error, "    goal_angle: ", goal_angle, "    th: ", self.th, "   left motor: ", self.left_motor_speed, '    right motor: ', self.right_motor_speed)
    # rospy.sleep(0.07)
      # print('angle: ', self.th)
      # print('angle error: ', angle_error)
      # print('diff: ', diff)
      # rospy.sleep(0.05)
        
    self.set_left_motor_speed_pub.publish(0)
    self.set_right_motor_speed_pub.publish(0)
    self.is_turning = False
    self.turning_pub.publish(self.is_turning)
    # rospy.sleep(3)

  def distance_from_goal(self, goal_location):
    x_diff = abs(float(goal_location[0]) - self.x)
    y_diff = abs(float(goal_location[1]) - self.y)
    distance_to_goal = np.hypot(x_diff, y_diff)
    return distance_to_goal
  
  # gives relative goal angle
  def angle_to_turn(self, goal_location): 
    x_diff = float(goal_location[0]) - self.x
    y_diff = float(goal_location[1]) - self.y
    angle = self.clamp_angle(np.arctan2(y_diff, x_diff) - self.th)
    return angle
  
  def clamp_angle(self, rad_angle, min_value=-np.pi, max_value=np.pi):
    if min_value > 0:
      min_value *= -1
    angle = (rad_angle + max_value) % (2 * np.pi) + min_value
    return angle
  
  def path_planning(self):
    for waypoint in self.goal_locations:
      # determines which relative direction to turn
      # do turn 
      goal_angle = self.angle_to_turn(waypoint) + self.th # Relative goal angle + global current angle = global goal angle
      print("goal angle: ", goal_angle)
      self.turn(goal_angle)
      
      print("Turning stopped")
      
      # drive straight
      self.drive(waypoint)
      
      # Stop when waypoint is reached
      self.set_left_motor_speed_pub.publish(0)
      self.set_right_motor_speed_pub.publish(0)
      print("Goal reached!")
      rospy.sleep(10)
      
  
    
if __name__ == "__main__":
  rospy.init_node('system')
  goal_locations = [[30, 20], [90, 80], [30, 80]]
  robot = System(goal_locations)
  rospy.sleep(1)
  robot.path_planning()