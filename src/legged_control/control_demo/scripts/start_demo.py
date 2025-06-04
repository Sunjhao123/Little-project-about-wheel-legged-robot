#!/usr/bin/python3
#coding=utf-8

import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

def publish_cmd_vel_once():
    # 初始化节点
    rospy.init_node('publish_cmd_vel_once', anonymous=True)
    
    # 创建一个Twist消息
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0  # 设置线速度
    cmd_vel_msg.angular.z = 0  # 设置角速度

    
    # 创建一个发布者
    pub2 = rospy.Publisher('/cmd_vel2', Twist, queue_size=10)
    pub1 = rospy.Publisher('/zrobo/cmd_vel', Twist, queue_size=10)
    # 等待发布者准备好
    rospy.sleep(1.0)
    
    # 发布一次cmd_vel消息
    pub2.publish(cmd_vel_msg)
    pub1.publish(cmd_vel_msg)
    pub_com_h = rospy.Publisher('/comHeight', Float64MultiArray, queue_size=1)
    
    pub_leg_h1 = rospy.Publisher("/leg_height_topic", Float64MultiArray, queue_size=1)


    #pub_com_h.publish(msg)

    rospy.loginfo("Published cmd_vel message once.")
    
    # 等待一小段时间，确保消息被发送
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        publish_cmd_vel_once()
        while not rospy.is_shutdown():
            pass

    except rospy.ROSInterruptException:
        pass