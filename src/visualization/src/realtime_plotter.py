#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import signal
import sys
import csv
import os
from datetime import datetime
from haique_msgs.msg import statepub_msg, eso_msg, controlpub_msg

class RealtimePlotter:
    def __init__(self):
        rospy.init_node('realtime_plotter')
        
        # 数据缓存
        self.time_data = deque(maxlen=200)
        self.state_data = {'roll': deque(maxlen=200), 
                          'pitch': deque(maxlen=200), 
                          'yaw': deque(maxlen=200)}
        self.eso_data = {'torque1': deque(maxlen=200),
                        'torque2': deque(maxlen=200),
                        'torque3': deque(maxlen=200)}
        self.control_data = {'thrust1': deque(maxlen=200),
                            'thrust2': deque(maxlen=200),
                            'thrust3': deque(maxlen=200),
                            'thrust4': deque(maxlen=200)}
        
        self.start_time = rospy.Time.now().to_sec()
        
        # 完整数据存储（不限制长度，用于保存）
        self.full_time_data = []
        self.full_state_data = {'roll': [], 'pitch': [], 'yaw': []}
        self.full_eso_data = {'torque1': [], 'torque2': [], 'torque3': []}
        self.full_control_data = {'thrust1': [], 'thrust2': [], 'thrust3': [], 'thrust4': []}
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 订阅话题
        rospy.Subscriber('/statePub', statepub_msg, self.state_callback)
        rospy.Subscriber('/eso_pub', eso_msg, self.eso_callback)
        rospy.Subscriber('/mpc_ctl', controlpub_msg, self.control_callback)
        
        # 设置绘图
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8))
        self.setup_plots()
        
        print("Data logging started. Press Ctrl+C to save data and exit.")
        
    def signal_handler(self, sig, frame):
        print('\nReceived Ctrl+C signal. Saving data...')
        self.save_data()
        print('Data saved successfully!')
        plt.close('all')
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)
        
    def save_data(self):
        """保存数据到CSV文件"""
        try:
            # 创建保存目录
            save_dir = os.path.expanduser('~/flight_data')
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(save_dir, f'flight_data_{timestamp}.csv')
            
            # 确保所有数据列表有相同的长度
            max_len = len(self.full_time_data)
            
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['time', 'roll', 'pitch', 'yaw', 
                            'eso_torque1', 'eso_torque2', 'eso_torque3',
                            'thrust1', 'thrust2', 'thrust3', 'thrust4']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for i in range(max_len):
                    row = {
                        'time': self.full_time_data[i] if i < len(self.full_time_data) else '',
                        'roll': self.full_state_data['roll'][i] if i < len(self.full_state_data['roll']) else '',
                        'pitch': self.full_state_data['pitch'][i] if i < len(self.full_state_data['pitch']) else '',
                        'yaw': self.full_state_data['yaw'][i] if i < len(self.full_state_data['yaw']) else '',
                        'eso_torque1': self.full_eso_data['torque1'][i] if i < len(self.full_eso_data['torque1']) else '',
                        'eso_torque2': self.full_eso_data['torque2'][i] if i < len(self.full_eso_data['torque2']) else '',
                        'eso_torque3': self.full_eso_data['torque3'][i] if i < len(self.full_eso_data['torque3']) else '',
                        'thrust1': self.full_control_data['thrust1'][i] if i < len(self.full_control_data['thrust1']) else '',
                        'thrust2': self.full_control_data['thrust2'][i] if i < len(self.full_control_data['thrust2']) else '',
                        'thrust3': self.full_control_data['thrust3'][i] if i < len(self.full_control_data['thrust3']) else '',
                        'thrust4': self.full_control_data['thrust4'][i] if i < len(self.full_control_data['thrust4']) else '',
                    }
                    writer.writerow(row)
            
            print(f"Data saved to: {filename}")
            print(f"Total data points: {max_len}")
            
            # 同时保存为numpy格式（可选）
            np_filename = os.path.join(save_dir, f'flight_data_{timestamp}.npz')
            np.savez(np_filename,
                    time=np.array(self.full_time_data),
                    roll=np.array(self.full_state_data['roll']),
                    pitch=np.array(self.full_state_data['pitch']),
                    yaw=np.array(self.full_state_data['yaw']),
                    eso_torque1=np.array(self.full_eso_data['torque1']),
                    eso_torque2=np.array(self.full_eso_data['torque2']),
                    eso_torque3=np.array(self.full_eso_data['torque3']),
                    thrust1=np.array(self.full_control_data['thrust1']),
                    thrust2=np.array(self.full_control_data['thrust2']),
                    thrust3=np.array(self.full_control_data['thrust3']),
                    thrust4=np.array(self.full_control_data['thrust4']))
            print(f"NumPy data saved to: {np_filename}")
            
        except Exception as e:
            print(f"Error saving data: {e}")
    
    def state_callback(self, msg):
        current_time = rospy.Time.now().to_sec() - self.start_time
        
        # 添加到显示缓存
        self.time_data.append(current_time)
        self.state_data['roll'].append(msg.roll)
        self.state_data['pitch'].append(msg.pitch)
        self.state_data['yaw'].append(msg.yaw)
        
        # 添加到完整数据存储
        self.full_time_data.append(current_time)
        self.full_state_data['roll'].append(msg.roll)
        self.full_state_data['pitch'].append(msg.pitch)
        self.full_state_data['yaw'].append(msg.yaw)
    
    def eso_callback(self, msg):
        if len(self.time_data) > 0:
            # 添加到显示缓存
            self.eso_data['torque1'].append(msg.torque1)
            self.eso_data['torque2'].append(msg.torque2)
            self.eso_data['torque3'].append(msg.torque3)
            
            # 添加到完整数据存储
            self.full_eso_data['torque1'].append(msg.torque1)
            self.full_eso_data['torque2'].append(msg.torque2)
            self.full_eso_data['torque3'].append(msg.torque3)
    
    def control_callback(self, msg):
        if len(self.time_data) > 0:
            # 添加到显示缓存
            self.control_data['thrust1'].append(msg.thrust1)
            self.control_data['thrust2'].append(msg.thrust2)
            self.control_data['thrust3'].append(msg.thrust3)
            self.control_data['thrust4'].append(msg.thrust4)
            
            # 添加到完整数据存储
            self.full_control_data['thrust1'].append(msg.thrust1)
            self.full_control_data['thrust2'].append(msg.thrust2)
            self.full_control_data['thrust3'].append(msg.thrust3)
            self.full_control_data['thrust4'].append(msg.thrust4)
    
    def setup_plots(self):
        self.axes[0].set_title('Attitude States')
        self.axes[0].set_ylabel('Angle (rad)')
        self.axes[0].legend(['Roll', 'Pitch', 'Yaw'])
        self.axes[0].grid(True)
        
        self.axes[1].set_title('ESO Torque Estimates')
        self.axes[1].set_ylabel('Torque (Nm)')
        self.axes[1].legend(['Torque X', 'Torque Y', 'Torque Z'])
        self.axes[1].grid(True)
        
        self.axes[2].set_title('Control Outputs')
        self.axes[2].set_ylabel('Thrust (N)')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].legend(['Thrust 1', 'Thrust 2', 'Thrust 3', 'Thrust 4'])
        self.axes[2].grid(True)
    
    def animate(self, frame):
        if len(self.time_data) < 2:
            return
            
        time_array = np.array(self.time_data)
        
        # 清除之前的绘图
        for ax in self.axes:
            ax.clear()
        
        self.setup_plots()

        # 确保所有数据数组与时间数组长度一致
        data_length = len(time_array)
        
        # 绘制状态数据
        if len(self.state_data['roll']) > 0:
            min_len = min(data_length, len(self.state_data['roll']))
            time_slice = time_array[-min_len:]
            
            self.axes[0].plot(time_slice, list(self.state_data['roll'])[-min_len:], 'r-', label='Roll')
            self.axes[0].plot(time_slice, list(self.state_data['pitch'])[-min_len:], 'g-', label='Pitch')
            self.axes[0].plot(time_slice, list(self.state_data['yaw'])[-min_len:], 'b-', label='Yaw')
        
        # 绘制 ESO 数据
        if len(self.eso_data['torque1']) > 0:
            eso_len = min(data_length, len(self.eso_data['torque1']))
            time_eso = time_array[-eso_len:]
            
            self.axes[1].plot(time_eso, list(self.eso_data['torque1'])[-eso_len:], 'r-', label='Torque X')
            self.axes[1].plot(time_eso, list(self.eso_data['torque2'])[-eso_len:], 'g-', label='Torque Y')
            self.axes[1].plot(time_eso, list(self.eso_data['torque3'])[-eso_len:], 'b-', label='Torque Z')
        
        # 绘制控制数据
        if len(self.control_data['thrust1']) > 0:
            control_len = min(data_length, len(self.control_data['thrust1']))
            time_control = time_array[-control_len:]
            
            self.axes[2].plot(time_control, list(self.control_data['thrust1'])[-control_len:], 'r-', label='Thrust 1')
            self.axes[2].plot(time_control, list(self.control_data['thrust2'])[-control_len:], 'g-', label='Thrust 2')
            self.axes[2].plot(time_control, list(self.control_data['thrust3'])[-control_len:], 'b-', label='Thrust 3')
            self.axes[2].plot(time_control, list(self.control_data['thrust4'])[-control_len:], 'c-', label='Thrust 4')
    
    def run(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100)
        plt.tight_layout()
        
        try:
            plt.show()
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)

if __name__ == '__main__':
    try:
        plotter = RealtimePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('\nShutdown requested by user')