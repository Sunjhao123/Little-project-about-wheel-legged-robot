#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Temperature
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point


class TemperatureMapper:
    def __init__(self):
        rospy.init_node('temperature_mapper')
        
        # 地图参数
        self.map_size = 50.0        # 地图边长 (米)
        self.resolution = 0.2       # 分辨率 (米/栅格)
        self.grid_dim = int(self.map_size / self.resolution)
        
        # 数据存储结构
        self.temp_sum = np.zeros((self.grid_dim, self.grid_dim), dtype=np.float32)
        self.measure_count = np.zeros((self.grid_dim, self.grid_dim), dtype=np.int32)
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.initial_pose = None
        
        # 地图消息
        self.map_msg = OccupancyGrid()
        self._init_map_msg()
        self.map_pub = rospy.Publisher('/temperature_map', OccupancyGrid, queue_size=1)
        
        rospy.Subscriber('/temperature', Temperature, self.temp_callback)

    def _init_map_msg(self):
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.grid_dim
        self.map_msg.info.height = self.grid_dim
        self.map_msg.info.origin.position = Point(-self.map_size/2, -self.map_size/2, 0)
        self.map_msg.info.origin.orientation.w = 1.0

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base", rospy.Time(0))
            return transform.transform.translation
        except Exception as e:
            rospy.logwarn(f"获取位置失败: {str(e)}")
            return None

    def temp_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = self.get_robot_pose()
            if self.initial_pose is None:
                return
            rospy.loginfo(f"地图中心初始化在: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f})")
            return
        
        current_pose = self.get_robot_pose()
        if current_pose is None:
            return
        
        # 计算相对于初始位置的坐标
        rel_x = current_pose.x - self.initial_pose.x
        rel_y = current_pose.y - self.initial_pose.y
        
        # 转换为栅格坐标
        grid_col = int((rel_x + self.map_size/2) / self.resolution)
        grid_row = int((rel_y + self.map_size/2) / self.resolution)
        
        # 更新3x3区域
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                col = grid_col + dx
                row = grid_row + dy
                if 0 <= col < self.grid_dim and 0 <= row < self.grid_dim:
                    self.temp_sum[row, col] += msg.temperature
                    self.measure_count[row, col] += 1
        
        self.publish_map()

    def calculate_temperature(self):
        temp_grid = np.divide(self.temp_sum, self.measure_count, 
                            where=self.measure_count!=0)
        return np.nan_to_num(temp_grid)

    def publish_map(self):
        temp_data = self.calculate_temperature()
        
        # 将温度映射到0-100范围
        min_temp = 0.0
        max_temp = 55.0
        scaled_data = ((temp_data - min_temp) / (max_temp - min_temp) * 127).astype(np.int8)
        scaled_data = np.clip(scaled_data, 0, 127)
        scaled_data[self.measure_count == 0] = -1
        
        self.map_msg.header.stamp = rospy.Time.now()
        self.map_msg.data = scaled_data.flatten(order='C').tolist()
        self.map_pub.publish(self.map_msg)




if __name__ == '__main__':
    mapper = TemperatureMapper()
    rospy.spin()