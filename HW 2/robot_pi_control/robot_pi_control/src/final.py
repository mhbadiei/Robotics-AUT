#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
import math 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt

kp_distance = 1
ki_distance = 0.01
kp_angle = 1

robot_x_vector = list()
robot_y_vector = list()
x_vector = list()
y_vector = list()
error = list()

class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        
        self.robot_x_vector = list()
        self.robot_y_vector = list()

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()


        last_rotation = 0
        linear_speed = 1   
        angular_speed = 1  

        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
 
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        total_distance = 0

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            
            self.robot_x_vector.append(np.float32(x_start))
            self.robot_y_vector.append(np.float32(y_start))
            
            #path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            print("l:",last_rotation,rotation)
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            elif rotation > 0 and goal_x < 0 and goal_y < 0:
                rotation = -2*pi + rotation
            print("l:",last_rotation,rotation) 

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + ki_distance*total_distance 

            control_signal_angle = kp_angle*(path_angle - rotation)

            move_cmd.angular.z = (control_signal_angle) 
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current positin and rotation are: ", (position, rotation))

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("following this point is completed")

        self.cmd_vel.publish(Twist())
        return

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def getRobotPath(self):
        return self.robot_x_vector, self.robot_y_vector

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


center_of_rotation_x = 0
center_of_rotation_y = 0
angle = math.radians(0) 
omega = 0.05

print("Please enter motion algprithm number for the following path (choose 1 or 2)")
print(" 1. Oval ")
print(" 2. Archimedean spirals ")
algprithm_number = input()

if algprithm_number == '1':
    print("Please enter x - axis point of the oval")
    x_radius = np.float32(input())
    print("Please enter y - axis point of the oval")
    y_radius = np.float32(input())
elif algprithm_number == '2':
    print("Please enter growth factor of the archimedean spirals")
    growth_factor = np.float32(input())
    print("Please enter starting angle of the archimedean spirals in degrees")
    angle = math.radians(np.float32(input()))
    x_radius = 0
    y_radius = 0

print("Please enter x position of Turtlebot for the starting position")
x_pos = input()

print("Please enter y position of Turtlebot for the starting position")
y_pos = input()

coord = [np.float32(x_pos),np.float32(y_pos)]
print('Initial X coordinate: ', coord[0])
print('Initial Y coordinate: ', coord[1])

print("Please enter the angle for the starting position of Turtlebot in degrees")
robot_starting_angle = [math.radians(np.float32(input()))]

print('Initial starting angle Theta wrt +X axis: ', robot_starting_angle[0])   

initial_position = np.concatenate((coord,robot_starting_angle))

print('Initial pose is:-')
print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

x_final = center_of_rotation_x + x_radius * cos(angle) #Starting position x
y_final = center_of_rotation_y - y_radius * sin(angle) #Starting position y
angle_final = 0

final = [x_final, y_final, angle_final]
print(final)
final_position = np.array(final)

x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]

q = quaternion_from_euler(0, 0, initial_position[2])
state_msg = ModelState()
state_msg.model_name = 'turtlebot3_burger'
state_msg.pose.position.x = initial_position[0]
state_msg.pose.position.y = initial_position[1]
state_msg.pose.position.z = 0

state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]

set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(5)

while angle < 2 * pi: #not rospy.is_shutdown():

    x_vector.append(x_final)
    y_vector.append(y_final)
        
    model = GotoPoint()
        
    angle = angle + omega # New angle, we add angular velocity
    r_x_vector, r_y_vector = model.getRobotPath()
    robot_x_vector = robot_x_vector + r_x_vector
    robot_y_vector = robot_y_vector + r_y_vector
    
    for i in range(len(r_x_vector)):
        error.append(sqrt((x_final - r_x_vector[i])**2+(y_final - r_y_vector[i])**2))
    
    print(len(r_x_vector), len(r_y_vector),len(robot_x_vector),len(robot_y_vector))
        
    if algprithm_number == '2':
        x_radius = angle * growth_factor
        y_radius = angle * growth_factor
    
    x_final = x_final + x_radius * omega * cos(angle + pi / 2) # New x
    y_final = y_final - y_radius * omega * sin(angle + pi / 2) # New y
    final = [x_final, y_final, angle_final]
    print(final)
    final_position = np.array(final)
    
    x_input = final_position[0]
    y_input = final_position[1]
    z_input = final_position[2]
    

plt.figure()
if algprithm_number == '1':
    plt.plot(x_vector, y_vector,linewidth = '3.2',color='red', label='oval path')
elif algprithm_number == '2':
    plt.plot(x_vector, y_vector, linewidth = '3.2' ,color='red', label='archimedean spirals path')
plt.plot(robot_x_vector, robot_y_vector,linewidth = '1.4',color='blue', label='robot path')
plt.xlabel("x - axis")
plt.ylabel("y - axis")
plt.xlim((-4,4))
plt.ylim((-4,4))
plt.legend()
plt.title('Mobile Robot Trajectory')

plt.figure()
plt.plot(range(len(error)), error)
plt.xlabel('iteration')
plt.ylabel('error')
plt.legend()
plt.title('Following Path Error In 2pi Radian Rotation')
plt.show()


