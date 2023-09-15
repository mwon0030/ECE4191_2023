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
    self.obstacle_detected = False
    self.not_waypoint_1 = False
    
    self.Kp = 0.1
    self.dist_threshold = 3
    self.angle_threshold = np.pi/50

    self.max_arena_size = 120
    
    self.front_left_sensor = rospy.Subscriber('/distance_sensor_0', Float32, self.front_left_sensor_cb)
    self.front_right_sensor = rospy.Subscriber('/distance_sensor_1', Float32, self.front_right_sensor_cb)
    self.left_sensor = rospy.Subscriber('/distance_sensor_2', Float32, self.left_sensor_cb)
    self.right_sensor = rospy.Subscriber('/distance_sensor_3', Float32, self.right_sensor_cb)
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    self.obstacle_detect_sub = rospy.Subscriber('/obstacle_detect', Bool, self.obstacle_detect_cb)
    
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

  def drive(self, goal):
    # self.set_left_motor_speed_pub.publish(self.Kp * dist_error)
    # self.set_right_motor_speed_pub.publish(-self.Kp * dist_error)
    distance_to_drive = self.distance_from_goal(goal)
    
    while distance_to_drive >= self.dist_threshold:
      self.set_left_motor_speed_pub.publish(0.3)
      self.set_right_motor_speed_pub.publish(-0.3)
      # motor_signal = 1 if self.Kp * distance_to_drive > 1 else self.Kp * distance_to_drive
      # self.set_left_motor_speed_pub.publish(motor_signal)
      # self.set_right_motor_speed_pub.publish(-motor_signa)

      # obstacle avoidance
      if self.obstacle_detected and self.not_waypoint_1:
        print('obstacle detected')
        self.obstacle_avoidance(goal)

      distance_to_drive = self.distance_from_goal(goal)
      print("dist error: ", distance_to_drive)

    
    while self.left_motor_speed != 0.0 and self.right_motor_speed != 0.0:
      self.set_left_motor_speed_pub.publish(0)
      self.set_right_motor_speed_pub.publish(0)

  def drive_straight(self, motor_signal = 0.4):
      self.set_left_motor_speed_pub.publish(motor_signal)
      self.set_right_motor_speed_pub.publish(-motor_signal)
    
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
      motor_speed_control_signal = 0.20
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
      # print("angle error: ", angle_error, "    goal_angle: ", goal_angle, "    th: ", self.th, "   left motor: ", self.left_motor_speed, '    right motor: ', self.right_motor_speed)
      self.turning_pub.publish(self.is_turning)
    
    while self.left_motor_speed != 0.0 and self.right_motor_speed != 0.0:
      self.set_left_motor_speed_pub.publish(0)
      self.set_right_motor_speed_pub.publish(0)
    
    
    print("Turn Complete")
    rospy.sleep(0.5)

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
    # waypoints = [[90,50], [30,50]]

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
      
      self.not_waypoint_1 = True
  
  def obstacle_avoidance(self, goal):
    # Stop when waypoint is reached
    # need to make this dependent on theta
    dist_threshold = 3
    print('obstacle detected')

    self.set_left_motor_speed_pub.publish(0)
    self.set_right_motor_speed_pub.publish(0) 

    rospy.sleep(5) 

    new_waypoint = [self.max_arena_size - 30 if self.x + 10  >= self.max_arena_size - 30 else self.x + 10, self.y]

    print('new waypoint', new_waypoint)
    rospy.sleep(0.5)

    dist_error = self.distance_from_goal(new_waypoint)

    while dist_error > dist_threshold: 
      self.drive_straight(motor_signal= -0.3)
      dist_error = self.distance_from_goal(new_waypoint)

    # self.turn(3*np.pi/2) 

    new_waypoint_2 = [self.x - 5, self.y - 35]
    print('new waypoint', new_waypoint_2)
    goal_angle = self.angle_to_turn(new_waypoint_2) + self.th
    
    self.turn(goal_angle)
    rospy.sleep(0.5)
    
    
    dist_error = self.distance_from_goal(new_waypoint_2)

    while dist_error > dist_threshold: 
      self.drive_straight(motor_signal= 0.3)
      dist_error = self.distance_from_goal(new_waypoint_2)

    new_waypoint_3 = [self.x - 60, self.y-5]
    print('new waypoint', new_waypoint_3)
    
    
    goal_angle = self.angle_to_turn(new_waypoint_3) + self.th - np.pi/60
    
    self.turn(goal_angle)

    dist_error = self.distance_from_goal(new_waypoint_3)

    rospy.sleep(0.5)

    while dist_error > 9: 
      self.drive_straight(motor_signal= 0.3)
      dist_error = self.distance_from_goal(new_waypoint_3)
      print("dist error: ", dist_error) 

    self.set_left_motor_speed_pub.publish(0)
    self.set_right_motor_speed_pub.publish(0) 

    rospy.sleep(0.5)
    
    print('Turning to goal')
    goal_angle = self.angle_to_turn(goal) + self.th + np.pi/20
    print('goal_angle', goal_angle)
    
    self.turn(goal_angle)

    rospy.sleep(0.5)

    self.dist_threshold = 9
    # if self.left_sensor_dist > self.right_sensor_dist: 
    #     # turn left
    #   if (self.th >= (5/6) * np.pi): # When theta is pi 
    #     self.turn(-np.pi/2) 
    #   else: 
    #     self.turn(self.th + np.pi/2)

    #   # look in the right direction and travel forward until it becomes clear 
    #   while self.right_sensor_dist + self.width/2 < self.x: 
    #     self.drive_forward()
      
    #   # when clear turn back perform right turn 
    #   if (self.th <= (-5/6) * np.pi): # When theta is -pi
    #     self.turn(-np.pi/2)
    #   else:
    #     self.turn(self.th - np.pi/2)

    #   # self.th left_angle right_angle 
    #   # 90 180 0 
    #   # 0 90 -90
    #   # 180 -90 90 
    #   # -180 -0 90 
    #   # -90 0 -180

    # else: 
    #   # turn right 
    #   if (self.th <= (-5/6) * np.pi): # When theta is -pi
    #     self.turn(-np.pi/2)
    #   else:
    #     self.turn(self.th - np.pi/2)

    #   # look in the right direction and travel forward until it becomes clear 
    #   while self.left_sensor_dist < self.x: 
    #     self.drive_forward()
      
    #   # when clear turn back perform left turn 
    #   if (self.th >= (5/6) * np.pi): # When theta is pi 
    #     self.turn(-np.pi/2) 
    #   else: 
    #     self.turn(self.th + np.pi/2)
      
  
    
if __name__ == "__main__":
  rospy.init_node('system')
  goal_locations = [[90, 80], [25, 80]] 
  # goal_locations = [[90, 80]]
  robot = System(goal_locations)
  rospy.sleep(1)
  # robot.turn(-2*np.pi)
  # robot.turn(-np.pi/4)
  # robot.turn(-(3/4) * np.pi)
  # robot.turn(-(5/4) * np.pi)
  # robot.turn(-(7/4) * np.pi)
  # robot.turn(-(9/4) * np.pi)
  
  # robot.turn(0)
  # robot.turn(-np.pi/2)
  # robot.turn(-(3/2)*np.pi)
  # robot.turn(-3/2 * np.pi)
  # robot.turn(-2*np.pi)
  # robot.path_planning()
  # robot.turn(np.pi)
  # robot.drive([30,90])
  
  
  robot.path_planning()