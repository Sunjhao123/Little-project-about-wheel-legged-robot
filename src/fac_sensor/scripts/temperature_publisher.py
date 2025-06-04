#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from sensor_msgs.msg import Temperature

class TemperaturePublisher:
    def __init__(self):
        rospy.init_node('temperature_publisher')
        self.pub = rospy.Publisher('/temperature', Temperature, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz发布频率
        self.base_temp = 25.0       # 基础温度
        self.temp_range = 30.0      # 温度波动范围

    def generate_temperature(self):
        msg = Temperature()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        msg.temperature = self.base_temp + random.uniform(-10, self.temp_range)
        return msg

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.generate_temperature())
            self.rate.sleep()




if __name__ == '__main__':
    try:
        node = TemperaturePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass