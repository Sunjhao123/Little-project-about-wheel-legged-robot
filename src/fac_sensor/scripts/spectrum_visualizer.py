#!/usr/bin/env python3
import rospy
import numpy as np
from fac_msgs.msg import Noise
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
import tf2_ros

class SpectrumVisualizer:
    def __init__(self):
        rospy.init_node('spectrum_visualizer')
        self.pub = rospy.Publisher('/spectrum', Marker, queue_size=1)
        rospy.Subscriber('/noise', Noise, self.noise_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 历史数据存储
        self.freq_bins = np.linspace(100, 5000, 20)  # 20个频段
        self.db_values = np.zeros_like(self.freq_bins)
        
        # 初始化标记
        self.marker = Marker()
        self.marker.header.frame_id = "spectrum_frame"
        self.marker.type = Marker.LINE_STRIP
        self.marker.scale.x = 0.1
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0

    def publish_tf(self):
        # 创建坐标系变换消息
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"          # 父坐标系（根据实际情况调整）
        t.child_frame_id = "spectrum_frame" # 子坐标系
        
        # 设置坐标系位置（示例：原点）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # 设置坐标系方向（四元数，此处无旋转）
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)

    def noise_callback(self, msg):
        # 每次更新前清空历史数据
        self.marker.points = []
        self.marker.colors = []  # 关键：清空颜色列表
        
        # 确保频段数量与颜色数量严格一致
        for i, (freq, db) in enumerate(zip(self.freq_bins, self.db_values)):
            # 生成点
            x = freq / 1000.0  # 频率(kHz)
            y = db / 100.0     # 分贝归一化
            self.marker.points.append(Point(x, y, 0))
            
            # 生成颜色（确保每个点对应一个颜色）
            ratio = min(max(db / 100.0, 0.0), 1.0)  # 限制在[0,1]
            r = min(2 * (1 - ratio), 1.0)
            g = min(2 * ratio, 1.0)
            b = max(1 - 2 * ratio, 0.0)
            self.marker.colors.append(ColorRGBA(r, g, b, 1.0))
        
        # 验证长度一致性
        if len(self.marker.points) != len(self.marker.colors):
            rospy.logerr("Points and colors count mismatch!")
            return
        
        self.pub.publish(self.marker)
        self.db_values *= 0.95  # 衰减效果

if __name__ == '__main__':
    vis = SpectrumVisualizer()
    rospy.spin()