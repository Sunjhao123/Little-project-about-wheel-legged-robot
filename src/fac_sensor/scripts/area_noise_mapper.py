#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point
from fac_msgs.msg import Noise 

class NoiseMapper:
    def __init__(self):
        rospy.init_node('noise_mapper')
        
        # 地图参数
        self.map_size = 50.0        # 地图边长 (米)
        self.resolution = 0.2       # 分辨率 (米/栅格)
        self.grid_dim = int(self.map_size / self.resolution)
        
        # 噪音参数
        self.min_db = 0.0          # 最小显示分贝
        self.max_db = 150.0         # 最大显示分贝
        self.decay_rate = 1.0      # 噪音衰减率
        
        # 数据存储结构
        self.noise_grid = np.full((self.grid_dim, self.grid_dim), -1.0, dtype=np.float32)
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.initial_pose = None
        
        # ROS接口
        self.map_msg = OccupancyGrid()
        self._init_map_msg()
        self.map_pub = rospy.Publisher('/noise_map', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/noise', Noise, self.noise_callback)
        
        # 定时衰减
        # rospy.Timer(rospy.Duration(1.0), self.apply_noise_decay)

    def _init_map_msg(self):
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.grid_dim
        self.map_msg.info.height = self.grid_dim
        self.map_msg.info.origin.position = Point(-self.map_size/2, -self.map_size/2, 0)
        self.map_msg.info.origin.orientation.w = 1.0

    def apply_noise_decay(self, event):
        """定期应用噪音衰减"""
        self.noise_grid = np.where(
            self.noise_grid > self.min_db, 
            self.noise_grid * self.decay_rate, 
            self.noise_grid
        )
        self.publish_map()

    def noise_callback(self, msg):
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
            
            # 更新区域 (带衰减梯度)
            effect_radius = 3  # 影响半径（栅格数）
            for dx in range(-effect_radius, effect_radius+1):
                for dy in range(-effect_radius, effect_radius+1):
                    distance = math.hypot(dx, dy)
                    decay = max(0.0, 0.0 + distance/effect_radius)
                    value = msg.decibel * decay
                    
                    col = grid_col + dx
                    row = grid_row + dy
                    if 0 <= col < self.grid_dim and 0 <= row < self.grid_dim:
                        self.noise_grid[row, col] = min(value, self.noise_grid[row, col])
            
            self.publish_map()

        except Exception as e:
            rospy.logwarn(f"噪音更新失败: {str(e)}")

    def _init_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base", rospy.Time(0))
            self.initial_pose = transform.transform.translation
            rospy.loginfo(f"地图中心初始化在: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f})")
        except Exception as e:
            rospy.logwarn(f"初始化失败: {str(e)}")

    def publish_map(self):
        """发布噪音地图"""
        # 归一化到0-100范围
        scaled_data = np.clip(
            (self.noise_grid - self.min_db) / (self.max_db - self.min_db) * 100,
            0, 100
        ).astype(np.int8)
        
        # 未测量区域设为-1
        scaled_data[self.noise_grid < self.min_db] = -1
        
        self.map_msg.header.stamp = rospy.Time.now()
        self.map_msg.data = scaled_data.flatten(order='C').tolist()
        self.map_pub.publish(self.map_msg)

if __name__ == '__main__':
    mapper = NoiseMapper()
    rospy.spin()