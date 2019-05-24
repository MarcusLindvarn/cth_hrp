#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16
from cth_hrp_cobot.msg import ButtonPressed

# Author: Marcus Lindvarn
# This ROS Node converts Joystick inputs from the joy node
# into commands for HRP
# Small changes has been made from the original file to make 
# it usable for this purpose


# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    buttonpress = ButtonPressed()
    # vertical left stick axis = linear rate
    twist.linear.x = 0.4*data.axes[1]
    # horizontal left stick axis = turn rate
    twist.angular.z = 0.7*data.axes[0]
    
    
    buttonpress.xpress = False
    buttonpress.bpress = False
    buttonpress.apress = False  # TESTING
    buttonpress.ypress = False  # TESTING
    # A Pressed
    if (data.buttons[0]== 1):
        buttonpress.apress = True
        buttonpub.publish(buttonpress)
    # B Pressed
    if (data.buttons[1]== 1):
        buttonpress.bpress = True
        buttonpub.publish(buttonpress)
    # X Pressed
    if (data.buttons[2]== 1):
        buttonpress.xpress = True
        buttonpub.publish(buttonpress)
    # Y Pressed
    if (data.buttons[3]== 1):
        buttonpress.ypress = True
        buttonpub.publish(buttonpress)
    # RB Pressed
    if (data.buttons[4]== 1):
        twist.linear.x = 0*data.axes[1]
        twist.angular.z = 0*data.axes[0]
    # LB Pressed
    if (data.buttons[5]== 1):
        twist.linear.x = 0*data.axes[1]
        twist.angular.z = 0*data.axes[0]
  
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "/cmd_vel" to control AGV
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)# test diff values for queue_size
    global buttonpub
    buttonpub = rospy.Publisher('/button_state', ButtonPressed, queue_size=1) # test diff values for queue_size
    #global pub_mode
    #pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2HRP')
    rospy.spin()


if __name__ == '__main__':
    start()

