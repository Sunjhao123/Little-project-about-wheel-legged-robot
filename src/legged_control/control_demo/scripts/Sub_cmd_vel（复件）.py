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
    pub_vel_mini_4wd_left_front_wheel = rospy.Publisher('/controllers/LF_foot_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_right_front_wheel = rospy.Publisher('/controllers/RF_foot_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_left_rear_wheel = rospy.Publisher('/controllers/LH_foot_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_right_rear_wheel = rospy.Publisher('/controllers/RH_foot_controller/command', Float64, queue_size=1)
    
    a  = 0.32    #for mini_4wd
    b  = 0.12    #for mini_4wd
    Vlf = (data.linear.x-data.angular.z*(a+b))/0.112
    Vrf = (data.linear.x+data.angular.z*(a+b))/0.112
    Vrr = (data.linear.x+data.angular.z*(a+b))/0.112
    Vlr = (data.linear.x-data.angular.z*(a+b))/0.112
    pub_vel_mini_4wd_left_front_wheel.publish(Vlf)
    pub_vel_mini_4wd_right_front_wheel.publish(Vrf)
    pub_vel_mini_4wd_left_rear_wheel.publish(Vlr)
    pub_vel_mini_4wd_right_rear_wheel.publish(Vrr)

def Sub_cmd_vel_mini_4wd():

    rospy.init_node('Sub_cmd_vel', anonymous=True)

    

    rospy.Subscriber("/cmd_vel1", Twist, set_mini_4wd_velocity, queue_size=1,buff_size=52428800)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            Sub_cmd_vel_mini_4wd()
    except rospy.ROSInterruptException:
        pass
