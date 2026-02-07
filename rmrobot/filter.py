import pandas as pd
import numpy as np
from scipy import signal
import os

def filter_and_process_optimized(input_file, output_file):
    # 1. 读取数据
    print(f"Reading {input_file}...")
    df = pd.read_csv(input_file)
    
    # 提取时间戳并计算 dt
    t = df['timestamp'].values
    dt_steps = np.diff(t)
    dt_mean = np.mean(dt_steps) 
    print(f"Estimated sample time (dt): {dt_mean:.6f} s (Freq: {1/dt_mean:.1f} Hz)")

    # 2. 定义滤波器 (用于速度、位置、电流的常规滤波)
    # Wn=0.1 对应 Nyquist 的 10%。如果数据还是噪，可以降到 0.05
    sos_general = signal.butter(N=5, Wn=0.1, btype='low', output='sos')
    sos_curr = signal.butter(N=5, Wn=0.20, btype='low', output='sos')
    
    # 3. 数据处理容器
    dof = 6
    gains = np.array([3.94, 3.94, 2.09, 8.28, 8.28, 8.28])
    out_data = {'timestamp': t}
    
    # --- 核心循环 ---
    for i in range(dof):
        idx = i + 1
        
        # 读取原始数据
        raw_pos = df[f'pos_{i}'].values
        raw_vel = df[f'vel_{i}'].values # 使用原始速度
        raw_curr = df[f'cur_{i}'].values
        
        # A. 位置 (Position)
        # 简单滤波即可
        out_data[f'pos{idx}'] = signal.sosfiltfilt(sos_general, raw_pos)
        
        # B. 速度 (Velocity) - 你的要求：使用原始速度滤波
        # 使用 sosfiltfilt 零相位滤波
        vel_filtered = signal.sosfiltfilt(sos_general, raw_vel)
        out_data[f'vel{idx}'] = vel_filtered
        
        # C. 加速度 (Acceleration) - 核心修改：使用 Savitzky-Golay 滤波器
        # 原理：在滤波后的速度上，取一个窗口（如51个点），拟合曲线并求导
        # window_length: 窗口长度，必须是奇数。越大越平滑，越小细节越多。
        # polyorder: 拟合多项式阶数，通常3。
        # deriv: 1，表示对速度求1阶导数 (=加速度)
        # delta: 采样时间，用于缩放数值
        
        # 调整建议：如果你觉得还抖，把 window_length 改大 (例如 81, 101)
        window_len = 51 
        if window_len > len(vel_filtered): window_len = len(vel_filtered) // 2 * 2 + 1 # 保护措施
            
        acc_savgol = signal.savgol_filter(vel_filtered, 
                                          window_length=window_len, 
                                          polyorder=3, 
                                          deriv=1, 
                                          delta=dt_mean)
        
        out_data[f'acc{idx}'] = acc_savgol
        
        # D. 力矩 (Torque)
        curr_filtered = signal.sosfiltfilt(sos_curr, raw_curr)
        out_data[f'torque{idx}'] = (curr_filtered * gains[i]) / 1000.0

    # 4. 保存
    df_out = pd.DataFrame(out_data)
    
    # 调整列顺序
    cols = ['timestamp']
    for tag in ['pos', 'vel', 'acc', 'torque']:
        for i in range(1, 7): cols.append(f'{tag}{i}')
            
    df_out = df_out[cols]
    df_out.to_csv(output_file, index=False)
    print(f"Processed data saved to {output_file}")
    print(f"Method: Velocity from raw(filtered); Acceleration via Savitzky-Golay (window={window_len}) on Velocity.")

if __name__ == "__main__":
    input_csv = 'rmrobot/q_ga_5_out.csv'
    output_csv = 'rmrobot/q_ga_5_processed_savgol.csv'
    
    if os.path.exists(input_csv):
        filter_and_process_optimized(input_csv, output_csv)
    else:
        print(f"Error: {input_csv} not found.")