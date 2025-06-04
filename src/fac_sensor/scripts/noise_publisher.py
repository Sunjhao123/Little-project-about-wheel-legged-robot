#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
import math
from fac_msgs.msg import Noise 


class NoisePublisher:
    def __init__(self):
        rospy.init_node('noise_publisher')
        self.pub = rospy.Publisher('/noise', Noise, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz发布频率
        
        # 模拟参数
        self.base_db = 20.0    # 基础噪音分贝
        self.noise_range = 125.0  # 噪音波动范围
        self.device_factor = 0.0  # device对噪音的影响因子

    def generate_noise(self, device):
        """生成带动态特性的噪音数据"""
        msg = Noise()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        
        # 基础噪音 + 动态变化
        dynamic_noise = math.sin(rospy.get_time()) * 5.0  # 模拟周期变化
        device_effect = device * self.device_factor
        
        msg.decibel = self.base_db + random.uniform(0, self.noise_range) \
                      + dynamic_noise + device_effect
        
        # 模拟主频率变化 (50-5000Hz)
        msg.frequency = 50.0 + (msg.decibel - self.base_db) * 100.0
        return msg

    def run(self):
        prev_pose = None
        while not rospy.is_shutdown():
            current_device = 1.0  # 实际应用中应从odom获取
            self.pub.publish(self.generate_noise(current_device))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = NoisePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass