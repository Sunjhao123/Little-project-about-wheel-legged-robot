#!/usr/bin/python3
#coding=utf-8
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties

class WheelController:
    def __init__(self):
        # 初始化节点
        rospy.init_node('wheel_controller', anonymous=True)
        
        # 车轮关节名称
        self.joint_names = rospy.get_param('controllers/wheel_velocity_controller/joints', ['LF_foot', 'RF_foot', 'LH_foot', 'RH_foot'])
        
        # 订阅速度指令话题
        self.speed_cmd_sub = rospy.Subscriber('/wheel_speed_cmd', Float64MultiArray, self.speed_cmd_callback)
        
        # Gazebo服务客户端
        self.apply_joint_effort_client = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.get_joint_properties_client = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        
        # 速度指令
        self.speed_cmds = np.zeros(len(self.joint_names))

    def speed_cmd_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            rospy.logerr("Received speed command with incorrect number of values")
            return
        self.speed_cmds = np.array(msg.data)

    def apply_joint_speeds(self):
        for joint_name, speed in zip(self.joint_names, self.speed_cmds):
            effort = ApplyJointEffort()
            effort.joint_name = joint_name
            effort.effort = speed
            effort.start_time = rospy.Time(0)
            effort.duration = rospy.Duration(0.1)
            try:
                self.apply_joint_effort_client(effort)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to apply joint effort for joint {joint_name}: {e}")

    def spin(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            self.apply_joint_speeds()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = WheelController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass