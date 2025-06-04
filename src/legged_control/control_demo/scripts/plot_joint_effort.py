#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from threading import Event
import os
import numpy as np

class TripleEffortPlotter:
    def __init__(self):
        self.start_time = None
        self.time_data = []
        self.effort_data = [[], [], []]  # 存储三个关节的数据
        self.stop_event = Event()
        self.data_received = False
        self.joint_names = []  # 存储关节名称

    def joint_state_callback(self, msg):
        if self.start_time is None:
            self.start_time = rospy.get_time()
            rospy.loginfo("Received first message, starting collection...")
            self.joint_names = msg.name  # 记录关节名称

        current_time = rospy.get_time()
        rel_time = current_time - self.start_time

        # 严格数据校验
        if len(msg.effort) < 3:
            rospy.logerr_throttle(5, "Effort数据不足3个！当前长度：%d", len(msg.effort))
            return

        if rel_time <= 10.0:
            self.time_data.append(rel_time)
            for i in range(3):
                self.effort_data[i].append(msg.effort[i])
            self.data_received = True
        else:
            self.stop_event.set()

    def plot_and_save(self):
        if not self.data_received:
            rospy.logerr("数据收集失败！可能原因：")
            rospy.logerr("1. /joint_states话题未发布")
            rospy.logerr("2. Effort数据不足3个")
            rospy.logerr("3. 消息频率过低（建议>10Hz）")
            return

        # 创建专业图表
        plt.figure(figsize=(14, 8))
        
        # 自定义颜色和样式
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
        line_styles = ['-', '--', '-.']
        markers = ['o', 's', '^']
        
        # 自动生成图例标签
        labels = [f'Joint {i} ({name})' for i, name in enumerate(self.joint_names[:3])]

        for i in range(3):
            plt.plot(self.time_data, self.effort_data[i],
                    color=colors[i],
                    linestyle=line_styles[i],
                    linewidth=1.5,
                    marker=markers[i],
                    markersize=5,
                    markevery=int(len(self.time_data)/20),  # 自动标记密度
                    label=labels[i])

        # 高级图表配置
        plt.xlabel('时间 (秒)', fontsize=14, fontfamily='SimHei')
        plt.ylabel('力矩 (牛米)', fontsize=14, fontfamily='SimHei')
        plt.title('三关节力矩时序图 (10秒)', fontsize=16, fontfamily='SimHei', pad=20)
        
        plt.legend(loc='upper center', 
                 bbox_to_anchor=(0.5, -0.15),
                 ncol=3,
                 fontsize=12)
        
        plt.grid(True, alpha=0.3)
        plt.xlim(0, 10)
        plt.xticks(np.arange(0, 10.5, 0.5))
        
        # 自动调整边距
        plt.tight_layout(rect=[0, 0.1, 1, 0.95])

        # 保存文件
        save_dir = os.path.expanduser('~/ROS_Plots')
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, 'triple_joint_effort.png')
        
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        rospy.loginfo(f"图表已保存至：{save_path}")
        plt.show()

def main():
    rospy.init_node('triple_joint_plotter')
    plotter = TripleEffortPlotter()
    
    # 等待话题可用
    try:
        rospy.loginfo("等待/joint_states话题...")
        rospy.wait_for_message('/joint_states', JointState, timeout=20)
    except rospy.ROSException:
        rospy.logerr("等待话题超时！请检查：")
        rospy.logerr("1. Gazebo是否正常启动")
        rospy.logerr("2. 是否正确加载机器人模型")
        return
    
    sub = rospy.Subscriber('/joint_states', JointState, plotter.joint_state_callback)
    
    # 主循环带进度显示
    rospy.loginfo("数据采集中... (剩余时间)")
    start_time = rospy.get_time()
    while (rospy.get_time() - start_time) < 12:
        if plotter.stop_event.is_set():
            break
        elapsed = rospy.get_time() - start_time
        rospy.loginfo_throttle(1, f"进度：{min(elapsed,10):.1f}/10 秒")
        rospy.sleep(0.05)
    
    sub.unregister()
    plotter.plot_and_save()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        plt.close('all')