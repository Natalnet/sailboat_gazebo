#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Float32

current_position = (0,0)
desired_position = (0,0)
distance_to_target = 0
current_twist = Twist()
desired_twist = Twist()
rate = 10
Iacc = 0

rospy.set_param('rudderCtrl/Kp_x', 2)
rospy.set_param('rudderCtrl/Ki_x', 0.5)
rospy.set_param('rudderCtrl/cp_x', 1)

rospy.set_param('rudderCtrl/Kp_y', 2)
rospy.set_param('rudderCtrl/Ki_y', 0.5)
rospy.set_param('rudderCtrl/cp_y', 1)

rospy.set_param('rudderCtrl/Kp_psi', 2)
rospy.set_param('rudderCtrl/Ki_psi', 0.5)
rospy.set_param('rudderCtrl/cp_psi', 1)

#publisher and subscriber
def talker_ctrl():
    # publishes to thruster and rudder topics
    rospy.init_node('velocity_ctrl', anonymous=True)
    rate_node = rospy.Rate(rate) # 10h
    
    # subscribe to state and targer point topics
    rospy.Subscriber("/odometry/filtered", Odometry, get_current_state) 
    #rospy.Subscriber("/gps/fix_velocity", Vector3Stamped, get_current_velocity_xy) 
    #rospy.Subscriber("/imu/data", Imu, get_current_velocity_psi) 
    rospy.Subscriber("/motion_ctrl/set_point_velocity", Twist, get_desired_velocity) 

    pub_motor_left = rospy.Publisher('/left_thrust_cmd', Float32, queue_size=10)
    pub_motor_right = rospy.Publisher('/right_thrust_cmd', Float32, queue_size=10)
    pub_motor_lateral = rospy.Publisher('/lateral_thrust_cmd', Float32, queue_size=10)

    while not rospy.is_shutdown():
        commands = actuator_ctrl()
        
        pub_motor_left.publish(np.float32(commands[0]))
        pub_motor_right.publish(np.float32(commands[1]))
        pub_motor_lateral.publish(np.float32(commands[2]))
        
        rate_node.sleep()

#create message to motor actuator
def act_left_msg():
    msg = actuator_ctrl()
    return msg

def act_right_msg():
    msg = actuator_ctrl()
    return msg

def act_lateral_msg():
    msg = actuator_ctrl()
    return msg

#motor control 'law' constant for the moment
def actuator_ctrl():
    global current_twist
    global desired_twist

    #get gains
    Kp_x = rospy.get_param('rudderCtrl/Kp_x')
    Ki_x = rospy.get_param('rudderCtrl/Ki_x')
    cp_x = rospy.get_param('rudderCtrl/cp_x')

    Kp_y = rospy.get_param('rudderCtrl/Kp_y')
    Ki_y = rospy.get_param('rudderCtrl/Ki_y')
    cp_y = rospy.get_param('rudderCtrl/cp_y')

    Kp_psi = rospy.get_param('rudderCtrl/Kp_psi')
    Ki_psi = rospy.get_param('rudderCtrl/Ki_psi')
    cp_psi = rospy.get_param('rudderCtrl/cp_psi')
    
    vel_x_error = desired_twist.linear.x - current_twist.linear.x
    motor_cmd = P(vel_x_error, Kp_x) + I(vel_x_error, Ki_x)

    #a velocidade do gps nao esta no referencial do barco

    if desired_twist.angular.z == 0:
        power_left = motor_cmd
        power_right = motor_cmd
    else:
        vel_psi_error = desired_twist.angular.z - current_twist.angular.z
        motor_cmd_ang = P(vel_psi_error, Kp_psi) + I(vel_psi_error, Ki_psi)
        power_left = motor_cmd - motor_cmd_ang
        power_right = motor_cmd + motor_cmd_ang
        log_msg = "desired_twist: {0}; current_twist: {1}; error: {2}" .format(desired_twist.linear.z, current_twist.linear.z, vel_psi_error)
        rospy.loginfo(log_msg)

    vel_y_error = desired_twist.linear.y - current_twist.linear.y
    motor_lat_cmd = P(vel_y_error, Kp_y) + I(vel_y_error, Ki_y)

    commands = (power_left, power_right, motor_lat_cmd)

    log_msg = "commands: {0};" .format(commands)
    rospy.loginfo(log_msg)

    #satura ambos motores
    
    #if self.target_vel.angular.z != self.target_vel_ant.angular.z:
    #    self.I_ant_ang = 0

    return commands

def P(error, gain):
    return gain * error

def I(error, gain):
    Ki = gain
    global Iacc
    global rate
    if (Iacc > 0 and error < 0) or (Iacc < 0 and error > 0):
        Iacc = Iacc + Ki * error * 50 * (1./rate)
    else:
        Iacc = Iacc + Ki * error * (1./rate)
    return Iacc

def get_current_state(current_state):
    global current_twist
    current_twist = current_state.twist.twist
    #log_msg = "tst: {0}" .format(current_twist)
    #rospy.loginfo(log_msg)

def get_current_velocity_xy(current_twist_tmp):
    global current_twist
    current_twist.linear = current_twist_tmp.vector
    #log_msg = "tst: {0}" .format(current_twist)
    #rospy.loginfo(log_msg)

def get_current_velocity_psi(current_twist_tmp):
    global current_twist
    current_twist.linear = current_twist_tmp.angular_velocity
    #log_msg = "distance_to_target: {0}" .format(current_twist)
    #rospy.loginfo(log_msg)

def get_desired_velocity(desired_twist_tmp):
    global desired_twist
    desired_twist = desired_twist_tmp

if __name__ == '__main__':
    try:
        talker_ctrl()
    except rospy.ROSInterruptException:
        pass