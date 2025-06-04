#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point
from sensor_msgs.msg import RelativeHumidity

class HumidityMapper:
    def __init__(self):
        rospy.init_node('humidity_mapper')
        
        # 地图参数
        self.map_size = 50.0        # 地图边长 (米)
        self.resolution = 0.2       # 分辨率 (米/栅格)
        self.grid_dim = int(self.map_size / self.resolution)
        
        # 湿度参数
        self.diffusion_rate = 0.1   # 湿度扩散系数
        self.evaporation_rate = 0.0 # 蒸发速率(%/秒)
        
        # 数据存储结构
        self.humidity_grid = np.full((self.grid_dim, self.grid_dim), -1.0, dtype=np.float32)
        self.confidence_grid = np.zeros((self.grid_dim, self.grid_dim), dtype=np.float32)
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.initial_pose = None
        
        # ROS接口
        self.map_msg = OccupancyGrid()
        self._init_map_msg()
        self.map_pub = rospy.Publisher('/humidity_map', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/humidity', RelativeHumidity, self.humidity_callback)
        
        # 定时处理
        #rospy.Timer(rospy.Duration(1.0), self.apply_humidity_diffusion)
        #rospy.Timer(rospy.Duration(1.0), self.apply_evaporation)

    def _init_map_msg(self):
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.grid_dim
        self.map_msg.info.height = self.grid_dim
        self.map_msg.info.origin.position = Point(-self.map_size/2, -self.map_size/2, 0)
        self.map_msg.info.origin.orientation.w = 1.0

    def apply_evaporation(self, event):
        """应用湿度蒸发"""
        self.humidity_grid = np.where(
            self.humidity_grid >= 0,
            np.maximum(0, self.humidity_grid),
            self.humidity_grid
        )
        self.publish_map()

    def apply_humidity_diffusion(self, event):
        """应用湿度扩散（相邻网格平均）"""
        new_grid = self.humidity_grid.copy()
        for i in range(1, self.grid_dim-1):
            for j in range(1, self.grid_dim-1):
                if self.humidity_grid[i,j] < 0:
                    continue
                neighbors = self.humidity_grid[i-1:i+2, j-1:j+2]
                valid_neighbors = neighbors[neighbors >= 0]
                if len(valid_neighbors) > 0:
                    new_grid[i,j] = (self.humidity_grid[i,j] )
        self.humidity_grid = new_grid

    def humidity_callback(self, msg):
        if self.initial_pose is None:
            self._init_robot_pose()
            return
            
        try:
            transform = self.tf_buffer.lookup_transform("map", "base", rospy.Time(0))
            current_pose = transform.transform.translation
            
            # 计算相对坐标
            rel_x = current_pose.x - self.initial_pose.x
            rel_y = current_pose.y - self.initial_pose.y
            
            # 转换为栅格坐标
            grid_col = int((rel_x + self.map_size/2) / self.resolution)
            grid_row = int((rel_y + self.map_size/2) / self.resolution)
                        
            self.publish_map()
            # 更新传感器区域
            sensor_radius = 3  # 传感器影响半径（栅格数）
            for dx in range(-sensor_radius, sensor_radius+1):
                for dy in range(-sensor_radius, sensor_radius+1):
                    distance = math.hypot(dx, dy)
                    if distance > sensor_radius:
                        continue
                        
                    col = grid_col + dx
                    row = grid_row + dy
                    if 0 <= col < self.grid_dim and 0 <= row < self.grid_dim:
                        self.humidity_grid[row, col] = msg.relative_humidity
                        # 置信度加权更新
                       # weight = 0.5 * (1 - distance/sensor_radius)
                       # new_value = msg.relative_humidity * weight + \
                        #          self.humidity_grid[row, col] * (1 - weight)
                       # self.humidity_grid[row, col] = max(new_value, self.humidity_grid[row, col])
                       
            
            self.publish_map()

        except Exception as e:
            rospy.logwarn(f"湿度更新失败: {str(e)}")

    def _init_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base", rospy.Time(0))
            self.initial_pose = transform.transform.translation
            rospy.loginfo(f"地图中心初始化在: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f})")
        except Exception as e:
            rospy.logwarn(f"初始化失败: {str(e)}")

    def publish_map(self):
        """发布湿度地图"""
        # 转换为0-100范围（直接使用湿度百分比）
        scaled_data = np.clip(self.humidity_grid, 0, 127).astype(np.int8)
        scaled_data[self.humidity_grid < 0] = -1  # 未测量区域
        
        self.map_msg.header.stamp = rospy.Time.now()
        self.map_msg.data = scaled_data.flatten(order='C').tolist()
        self.map_pub.publish(self.map_msg)

if __name__ == '__main__':
    mapper = HumidityMapper()
    rospy.spin()