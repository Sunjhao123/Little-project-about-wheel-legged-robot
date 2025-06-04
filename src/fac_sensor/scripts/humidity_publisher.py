#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
import math
from sensor_msgs.msg import RelativeHumidity

class HumidityPublisher:
    def __init__(self):
        rospy.init_node('humidity_publisher')
        self.pub = rospy.Publisher('/humidity', RelativeHumidity, queue_size=10)
        self.rate = rospy.Rate(1)  # 1Hz发布频率
        
        # 模拟参数
        self.base_humidity = 50.0    # 基础湿度(%)
        self.humidity_range = 40.0   # 湿度波动范围
        self.water_sources = [       # 模拟水源位置(x,y,影响半径)
            (10.0, 10.0, 5.0),
            (-5.0, 8.0, 3.0)
        ]

    def calculate_environment_humidity(self, x, y):
        """计算环境湿度基准值"""
        base = self.base_humidity
        for (wx, wy, radius) in self.water_sources:
            distance = math.hypot(x - wx, y - wy)
            if distance < radius:
                base += (1 - distance/radius) * 30.0  # 水源附近增加湿度
        return base

    def generate_humidity(self):
        msg = RelativeHumidity()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
           
        # 生成湿度数据
        msg.relative_humidity = self.base_humidity + random.uniform(-self.humidity_range/2, self.humidity_range/2)
        msg.relative_humidity = max(0.0, min(100.0, msg.relative_humidity))
        msg.variance = 0.1  # 模拟传感器误差
        return msg

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.generate_humidity())
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = HumidityPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass