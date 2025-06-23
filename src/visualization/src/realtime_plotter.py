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
                          'yaw': deque(maxlen=200),
                          'x': deque(maxlen=200),
                          'y': deque(maxlen=200), 
                          'z': deque(maxlen=200),
                          'vx': deque(maxlen=200),
                          'vy': deque(maxlen=200),
                          'vz': deque(maxlen=200),
                          'ax': deque(maxlen=200),
                          'ay': deque(maxlen=200),
                          'az': deque(maxlen=200)}
        self.eso_data = {'torque1': deque(maxlen=200),
                        'torque2': deque(maxlen=200),
                        'torque3': deque(maxlen=200),
                        'force1': deque(maxlen=200),
                        'force2': deque(maxlen=200),
                        'force3': deque(maxlen=200)}
        # 修改为8个电机推力和2个舵机转角
        self.control_data = {'thrust1': deque(maxlen=200),
                            'thrust2': deque(maxlen=200),
                            'thrust3': deque(maxlen=200),
                            'thrust4': deque(maxlen=200),
                            'thrust5': deque(maxlen=200),
                            'thrust6': deque(maxlen=200),
                            'thrust7': deque(maxlen=200),
                            'thrust8': deque(maxlen=200),
                            'servo1': deque(maxlen=200),
                            'servo2': deque(maxlen=200)}
        self.target_data = {'target_x': deque(maxlen=200),
                           'target_y': deque(maxlen=200),
                           'target_z': deque(maxlen=200)}
        
        self.start_time = rospy.Time.now().to_sec()
        
        # 完整数据存储（不限制长度，用于保存）
        self.full_time_data = []
        self.full_state_data = {'roll': [], 'pitch': [], 'yaw': [],
                               'x': [], 'y': [], 'z': [],
                               'vx': [], 'vy': [], 'vz': [],
                               'ax': [], 'ay': [], 'az': []}
        self.full_eso_data = {'torque1': [], 'torque2': [], 'torque3': [],
                             'force1': [], 'force2': [], 'force3': []}
        # 修改完整数据存储
        self.full_control_data = {'thrust1': [], 'thrust2': [], 'thrust3': [], 'thrust4': [],
                                 'thrust5': [], 'thrust6': [], 'thrust7': [], 'thrust8': [],
                                 'servo1': [], 'servo2': []}
        self.full_target_data = {'target_x': [], 'target_y': [], 'target_z': []}
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 订阅话题
        rospy.Subscriber('/statePub', statepub_msg, self.state_callback)
        rospy.Subscriber('/eso_pub', eso_msg, self.eso_callback)
        rospy.Subscriber('/mpc_ctl', controlpub_msg, self.control_callback)
        # 假设目标位置话题，请根据实际情况修改
        # rospy.Subscriber('/target_position', target_msg, self.target_callback)
        
        # 设置绘图 - 增加一个子图用于舵机
        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 10))
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
            save_dir = os.path.join(os.path.expanduser('~'), 'Haique_Webots', 'flight_data')
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(save_dir, f'flight_data_{timestamp}.csv')
            
            # 确保所有数据列表有相同的长度
            max_len = len(self.full_time_data)
            
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = [
                    # 时间
                    'time',
                    # 姿态信息
                    'roll', 'pitch', 'yaw',
                    # 位置信息
                    'position_x', 'position_y', 'position_z',
                    # 速度信息
                    'velocity_x', 'velocity_y', 'velocity_z',
                    # 加速度信息
                    'acceleration_x', 'acceleration_y', 'acceleration_z',
                    # ESO估计的力矩
                    'eso_torque_x', 'eso_torque_y', 'eso_torque_z',
                    # ESO估计的力
                    'eso_force_x', 'eso_force_y', 'eso_force_z',
                    # 8个电机推力
                    'motor_thrust_1', 'motor_thrust_2', 'motor_thrust_3', 'motor_thrust_4',
                    'motor_thrust_5', 'motor_thrust_6', 'motor_thrust_7', 'motor_thrust_8',
                    # 2个舵机转角
                    'servo_angle_1', 'servo_angle_2',
                    # 目标位置
                    'target_position_x', 'target_position_y', 'target_position_z'
                ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for i in range(max_len):
                    row = {
                        'time': self.full_time_data[i] if i < len(self.full_time_data) else '',
                        # 姿态
                        'roll': self.full_state_data['roll'][i] if i < len(self.full_state_data['roll']) else '',
                        'pitch': self.full_state_data['pitch'][i] if i < len(self.full_state_data['pitch']) else '',
                        'yaw': self.full_state_data['yaw'][i] if i < len(self.full_state_data['yaw']) else '',
                        # 位置
                        'position_x': self.full_state_data['x'][i] if i < len(self.full_state_data['x']) else '',
                        'position_y': self.full_state_data['y'][i] if i < len(self.full_state_data['y']) else '',
                        'position_z': self.full_state_data['z'][i] if i < len(self.full_state_data['z']) else '',
                        # 速度
                        'velocity_x': self.full_state_data['vx'][i] if i < len(self.full_state_data['vx']) else '',
                        'velocity_y': self.full_state_data['vy'][i] if i < len(self.full_state_data['vy']) else '',
                        'velocity_z': self.full_state_data['vz'][i] if i < len(self.full_state_data['vz']) else '',
                        # 加速度
                        'acceleration_x': self.full_state_data['ax'][i] if i < len(self.full_state_data['ax']) else '',
                        'acceleration_y': self.full_state_data['ay'][i] if i < len(self.full_state_data['ay']) else '',
                        'acceleration_z': self.full_state_data['az'][i] if i < len(self.full_state_data['az']) else '',
                        # ESO力矩
                        'eso_torque_x': self.full_eso_data['torque1'][i] if i < len(self.full_eso_data['torque1']) else '',
                        'eso_torque_y': self.full_eso_data['torque2'][i] if i < len(self.full_eso_data['torque2']) else '',
                        'eso_torque_z': self.full_eso_data['torque3'][i] if i < len(self.full_eso_data['torque3']) else '',
                        # ESO力
                        'eso_force_x': self.full_eso_data['force1'][i] if i < len(self.full_eso_data['force1']) else '',
                        'eso_force_y': self.full_eso_data['force2'][i] if i < len(self.full_eso_data['force2']) else '',
                        'eso_force_z': self.full_eso_data['force3'][i] if i < len(self.full_eso_data['force3']) else '',
                        # 8个电机推力
                        'motor_thrust_1': self.full_control_data['thrust1'][i] if i < len(self.full_control_data['thrust1']) else '',
                        'motor_thrust_2': self.full_control_data['thrust2'][i] if i < len(self.full_control_data['thrust2']) else '',
                        'motor_thrust_3': self.full_control_data['thrust3'][i] if i < len(self.full_control_data['thrust3']) else '',
                        'motor_thrust_4': self.full_control_data['thrust4'][i] if i < len(self.full_control_data['thrust4']) else '',
                        'motor_thrust_5': self.full_control_data['thrust5'][i] if i < len(self.full_control_data['thrust5']) else '',
                        'motor_thrust_6': self.full_control_data['thrust6'][i] if i < len(self.full_control_data['thrust6']) else '',
                        'motor_thrust_7': self.full_control_data['thrust7'][i] if i < len(self.full_control_data['thrust7']) else '',
                        'motor_thrust_8': self.full_control_data['thrust8'][i] if i < len(self.full_control_data['thrust8']) else '',
                        # 2个舵机转角
                        'servo_angle_1': self.full_control_data['servo1'][i] if i < len(self.full_control_data['servo1']) else '',
                        'servo_angle_2': self.full_control_data['servo2'][i] if i < len(self.full_control_data['servo2']) else '',
                        # 目标位置
                        'target_position_x': self.full_target_data['target_x'][i] if i < len(self.full_target_data['target_x']) else '',
                        'target_position_y': self.full_target_data['target_y'][i] if i < len(self.full_target_data['target_y']) else '',
                        'target_position_z': self.full_target_data['target_z'][i] if i < len(self.full_target_data['target_z']) else '',
                    }
                    writer.writerow(row)
            
            print(f"Data saved to: {filename}")
            print(f"Total data points: {max_len}")
            
        except Exception as e:
            print(f"Error saving data: {e}")
    
    def state_callback(self, msg):
        current_time = rospy.Time.now().to_sec() - self.start_time
        
        # 添加到显示缓存
        self.time_data.append(current_time)
        self.state_data['roll'].append(msg.roll)
        self.state_data['pitch'].append(msg.pitch)
        self.state_data['yaw'].append(msg.yaw)
        
        # 添加位置、速度、加速度数据（请根据实际msg字段名修改）
        self.state_data['x'].append(getattr(msg, 'global_position_x', 0))
        self.state_data['y'].append(getattr(msg, 'global_position_y', 0))
        self.state_data['z'].append(getattr(msg, 'global_position_z', 0))
        self.state_data['vx'].append(getattr(msg, 'linear_velocity_x', 0))
        self.state_data['vy'].append(getattr(msg, 'linear_velocity_y', 0))
        self.state_data['vz'].append(getattr(msg, 'linear_velocity_z', 0))
        self.state_data['ax'].append(getattr(msg, 'angular_velocity_x', 0))
        self.state_data['ay'].append(getattr(msg, 'angular_velocity_y', 0))
        self.state_data['az'].append(getattr(msg, 'angular_velocity_z', 0))
        
        # 添加到完整数据存储
        self.full_time_data.append(current_time)
        self.full_state_data['roll'].append(msg.roll)
        self.full_state_data['pitch'].append(msg.pitch)
        self.full_state_data['yaw'].append(msg.yaw)
        self.full_state_data['x'].append(getattr(msg, 'global_position_x', 0))
        self.full_state_data['y'].append(getattr(msg, 'global_position_y', 0))
        self.full_state_data['z'].append(getattr(msg, 'global_position_z', 0))
        self.full_state_data['vx'].append(getattr(msg, 'linear_velocity_x', 0))
        self.full_state_data['vy'].append(getattr(msg, 'linear_velocity_y', 0))
        self.full_state_data['vz'].append(getattr(msg, 'linear_velocity_z', 0))
        self.full_state_data['ax'].append(getattr(msg, 'angular_velocity_x', 0))
        self.full_state_data['ay'].append(getattr(msg, 'angular_velocity_y', 0))
        self.full_state_data['az'].append(getattr(msg, 'angular_velocity_z', 0))
    
    def eso_callback(self, msg):
        if len(self.time_data) > 0:
            # 添加到显示缓存
            self.eso_data['torque1'].append(msg.torque1)
            self.eso_data['torque2'].append(msg.torque2)
            self.eso_data['torque3'].append(msg.torque3)
            # 添加ESO估计的力（请根据实际msg字段名修改）
            self.eso_data['force1'].append(getattr(msg, 'force1', 0))
            self.eso_data['force2'].append(getattr(msg, 'force2', 0))
            self.eso_data['force3'].append(getattr(msg, 'force3', 0))
            
            # 添加到完整数据存储
            self.full_eso_data['torque1'].append(msg.torque1)
            self.full_eso_data['torque2'].append(msg.torque2)
            self.full_eso_data['torque3'].append(msg.torque3)
            self.full_eso_data['force1'].append(getattr(msg, 'force1', 0))
            self.full_eso_data['force2'].append(getattr(msg, 'force2', 0))
            self.full_eso_data['force3'].append(getattr(msg, 'force3', 0))
    
    def control_callback(self, msg):
        if len(self.time_data) > 0:
            # 假设msg包含一个数组或者分别的字段，前8个是推力，后2个是舵机角度
            # 请根据实际的controlpub_msg结构修改字段名
            # 添加到显示缓存
            self.control_data['thrust1'].append(getattr(msg, 'thrust1', 0))
            self.control_data['thrust2'].append(getattr(msg, 'thrust2', 0))
            self.control_data['thrust3'].append(getattr(msg, 'thrust3', 0))
            self.control_data['thrust4'].append(getattr(msg, 'thrust4', 0))
            self.control_data['thrust5'].append(getattr(msg, 'thrust5', 0))
            self.control_data['thrust6'].append(getattr(msg, 'thrust6', 0))
            self.control_data['thrust7'].append(getattr(msg, 'thrust7', 0))
            self.control_data['thrust8'].append(getattr(msg, 'thrust8', 0))
            self.control_data['servo1'].append(getattr(msg, 'alpha', 0))
            self.control_data['servo2'].append(getattr(msg, 'beta', 0))
            
            # 添加到完整数据存储
            self.full_control_data['thrust1'].append(getattr(msg, 'thrust1', 0))
            self.full_control_data['thrust2'].append(getattr(msg, 'thrust2', 0))
            self.full_control_data['thrust3'].append(getattr(msg, 'thrust3', 0))
            self.full_control_data['thrust4'].append(getattr(msg, 'thrust4', 0))
            self.full_control_data['thrust5'].append(getattr(msg, 'thrust5', 0))
            self.full_control_data['thrust6'].append(getattr(msg, 'thrust6', 0))
            self.full_control_data['thrust7'].append(getattr(msg, 'thrust7', 0))
            self.full_control_data['thrust8'].append(getattr(msg, 'thrust8', 0))
            self.full_control_data['servo1'].append(getattr(msg, 'alpha', 0))
            self.full_control_data['servo2'].append(getattr(msg, 'beta', 0))
    
    def target_callback(self, msg):
        """目标位置回调函数，请根据实际的目标位置消息类型修改"""
        if len(self.time_data) > 0:
            # 添加到显示缓存（请根据实际msg字段名修改）
            self.target_data['target_x'].append(getattr(msg, 'x', 0))
            self.target_data['target_y'].append(getattr(msg, 'y', 0))
            self.target_data['target_z'].append(getattr(msg, 'z', 0))
            
            # 添加到完整数据存储
            self.full_target_data['target_x'].append(getattr(msg, 'x', 0))
            self.full_target_data['target_y'].append(getattr(msg, 'y', 0))
            self.full_target_data['target_z'].append(getattr(msg, 'z', 0))
    
    def setup_plots(self):
        self.axes[0].set_title('Position States')
        self.axes[0].set_ylabel('Position (m)')
        self.axes[0].legend(['X', 'Y', 'Z'])
        self.axes[0].grid(True)
        
        self.axes[1].set_title('ESO Torque Estimates')
        self.axes[1].set_ylabel('Torque (Nm)')
        self.axes[1].legend(['Torque X', 'Torque Y', 'Torque Z'])
        self.axes[1].grid(True)
        
        self.axes[2].set_title('Motor Thrust Outputs')
        self.axes[2].set_ylabel('Thrust (N)')
        self.axes[2].legend(['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6', 'Motor 7', 'Motor 8'])
        self.axes[2].grid(True)
        
        self.axes[3].set_title('Servo Angles')
        self.axes[3].set_ylabel('Angle (rad)')
        self.axes[3].set_xlabel('Time (s)')
        self.axes[3].legend(['Servo 1', 'Servo 2'])
        self.axes[3].grid(True)
    
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
        
        # 绘制电机推力数据
        if len(self.control_data['thrust1']) > 0:
            control_len = min(data_length, len(self.control_data['thrust1']))
            time_control = time_array[-control_len:]
            
            colors = ['r-', 'g-', 'b-', 'c-', 'm-', 'y-', 'k-', 'orange']
            thrusts = ['thrust1', 'thrust2', 'thrust3', 'thrust4', 'thrust5', 'thrust6', 'thrust7', 'thrust8']
            labels = ['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6', 'Motor 7', 'Motor 8']
            
            for i, (thrust, color, label) in enumerate(zip(thrusts, colors, labels)):
                self.axes[2].plot(time_control, list(self.control_data[thrust])[-control_len:], color, label=label)
        
        # 绘制舵机转角数据
        if len(self.control_data['servo1']) > 0:
            servo_len = min(data_length, len(self.control_data['servo1']))
            time_servo = time_array[-servo_len:]
            
            self.axes[3].plot(time_servo, list(self.control_data['servo1'])[-servo_len:], 'r-', label='Servo 1')
            self.axes[3].plot(time_servo, list(self.control_data['servo2'])[-servo_len:], 'g-', label='Servo 2')
    
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