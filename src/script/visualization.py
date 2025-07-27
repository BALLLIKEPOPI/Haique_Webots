import pandas as pd
import numpy as np
import os
import sys
import matplotlib.pyplot as plt

def plot_csv_data(csv_file_path):
    """
    读取CSV文件并根据表头自动分类绘制数据图表
    """
    # 读取CSV文件
    df = pd.read_csv(csv_file_path)
    
    # 获取所有列名
    columns = df.columns.tolist()
    print(f"CSV文件包含的列: {columns}")
    
    # 根据表头自动分类数据
    time_col = columns[0]  # 假设第一列是时间
    time = df[time_col]
    
    # 定义数据分类规则
    data_categories = {
        'Position': [],
        'Velocity': [],
        'Angles': [],
        'Acceleration': [],
        'Motor/Thrust': [],
        'ESO Torque': [],
        'ESO Force': [],
        'Servo': [],
        'Others': []
    }
    
    # 自动分类列
    for col in columns[1:]:  # 跳过时间列
        col_lower = col.lower()
        
        if any(keyword in col_lower for keyword in ['eso_torque', 'torque_eso', 'moment_eso']):
            data_categories['ESO Torque'].append(col)
        elif any(keyword in col_lower for keyword in ['eso_thrust', 'thrust_eso', 'eso_force']):
            data_categories['ESO Force'].append(col)
        elif any(keyword in col_lower for keyword in ['position', 'pos']) and not any(exclude in col_lower for exclude in ['velocity', 'vel', 'angular']):
            data_categories['Position'].append(col)
        elif any(keyword in col_lower for keyword in ['velocity', 'vel']) and not any(exclude in col_lower for exclude in ['angular']):
            data_categories['Velocity'].append(col)
        elif any(keyword in col_lower for keyword in ['angle', 'attitude', 'euler', 'roll', 'pitch', 'yaw', 'rotation']) and not any(exclude in col_lower for exclude in ['servo']):
            data_categories['Angles'].append(col)
        elif any(keyword in col_lower for keyword in ['acceleration', 'accel', 'angular_vel', 'omega']):
            data_categories['Acceleration'].append(col)
        elif any(keyword in col_lower for keyword in ['motor', 'thrust', 'force', 'propeller']) and not any(exclude in col_lower for exclude in ['eso']):
            data_categories['Motor/Thrust'].append(col)
        elif any(keyword in col_lower for keyword in ['servo', 'actuator']):
            data_categories['Servo'].append(col)
        else:
            data_categories['Others'].append(col)
    
    # 打印分类结果
    # for category, cols in data_categories.items():
    #     if cols:
    #         print(f"{category}: {cols}")
    
    # 创建子图
    fig, axes = plt.subplots(4, 2, figsize=(15, 20))
    fig.suptitle(f'Data Visualization - {os.path.basename(csv_file_path)}', fontsize=16)
    
    # 绘制各类数据
    plot_configs = [
        ('Position', axes[0, 0], 'Position (m)'),
        ('Velocity', axes[0, 1], 'Velocity (m/s)'),
        ('Angles', axes[1, 0], 'Angle (rad/deg)'),
        ('Acceleration', axes[1, 1], 'Acceleration (m/s²) / Angular Velocity (rad/s)'),
        ('Motor/Thrust', axes[2, 0], 'Thrust (N)'),
        ('ESO Torque', axes[2, 1], 'Torque (N·m)'),
        ('ESO Force', axes[3, 0], 'Force (N)'),
        ('Servo', axes[3, 1], 'Servo Position (rad)'),
    ]
    
    for category, ax, ylabel in plot_configs:
        cols = data_categories[category]
        if cols:
            for col in cols:
                ax.plot(time.values, df[col].values, label=col)
            ax.legend()
        
        ax.set_title(category)
        ax.set_xlabel('Time')
        ax.set_ylabel(ylabel)
        ax.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 使用示例
    save_dir = os.path.join(os.path.expanduser('~'), 'Haique_Webots', 'flight_data')
    # 获取所有CSV文件并找到最新的文件
    csv_files = [f for f in os.listdir(save_dir) if f.startswith('flight_data_') and f.endswith('.csv')]
    if csv_files:
        # 按文件名排序（时间戳格式确保字典序等于时间序）
        csv_files.sort(reverse=True)
        csv_file = csv_files[0]
        # csv_file = "flight_data_20250623_214457.csv"  # For testing purposes, replace with the latest file found
        print(f"找到最新的CSV文件: {csv_file}")
    else:
        print("未找到任何CSV文件")
        exit(1)
    plot_csv_data(save_dir + os.sep + csv_file)
