#!/usr/bin/python3
#coding=utf-8

# Author: christoph.roesmann@tu-dortmund.de
import rospy,math
import ocs2_msgs
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
def set_mini_4wd_velocity(data):
    wheel_control = rospy.Publisher('/zrobo/cmd_vel', Twist, queue_size=1)
    legged_control = rospy.Publisher('/cmd_vel2', Twist, queue_size=1)
    msg_wheel = Twist()
    msg_legged = Twist()
    
    vx = data.linear.x
    w = data.angular.z

    msg_wheel.linear.x=vx
    msg_wheel.angular.z=0
    
    msg_legged.linear.x=0
    msg_legged.angular.z=w
    
    
    wheel_control.publish(msg_wheel)
    legged_control.publish(msg_legged)
    

def Sub_cmd_vel_mini_4wd():

    rospy.init_node('Sub_cmd_vel', anonymous=True)

    

    rospy.Subscriber("/cmd_vel", Twist, set_mini_4wd_velocity, queue_size=1,buff_size=52428800)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            Sub_cmd_vel_mini_4wd()
    except rospy.ROSInterruptException:
        pass
