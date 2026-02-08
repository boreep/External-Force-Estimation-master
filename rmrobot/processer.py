import pandas as pd
import numpy as np
import os

def calculate_filtered_acceleration(df, dof=6, window_size=5):
    """
    通过速度差分计算加速度，并应用滑动平均滤波
    """
    dt = df['timestamp'].diff().fillna(0.005)
    # 防止 dt 为 0
    dt[dt <= 0] = 0.005
    
    acc_data = {}
    for i in range(dof):
        # 1. 差分计算原始加速度: a = dv / dt
        vel_col = f'vel_{i}'
        acc_raw = df[vel_col].diff() / dt
        acc_raw.fillna(0.0, inplace=True)
        
        # 2. 简单的滑动平均滤波
        # window_size 越大越平滑，但延迟越高
        acc_filtered = acc_raw.rolling(window=window_size, center=True, min_periods=1).mean()
        acc_data[i] = acc_filtered.values
        
    return acc_data

def format_raw_with_torque_conversion(input_file, output_file):
    # 1. 读取数据
    print(f"Reading raw data from {input_file}...")
    df = pd.read_csv(input_file)
    
    # 2. 准备参数
    dof = 6
    # 关节电流转力矩系数 (根据你的电机型号)
    gains = np.array([3.94, 3.94, 2.09, 8.28, 8.28, 8.28])
    
    # --- 新增：计算滤波后的加速度 ---
    acc_calculated = calculate_filtered_acceleration(df, dof, window_size=10)
    
    out_data = {'timestamp': df['timestamp']}
    
    # --- 核心循环 ---
    for i in range(dof):
        idx = i + 1
        
        # A. 位置 (Position)
        out_data[f'pos{idx}'] = df[f'pos_{i}']
        
        # B. 速度 (Velocity)
        out_data[f'vel{idx}'] = df[f'vel_{i}']
        
        # C. 加速度 (Acceleration) - 使用计算出的滤波值
        out_data[f'acc{idx}'] = acc_calculated[i]
        
        # D. 力矩 (Torque) - 单位转换
        raw_curr = df[f'cur_{i}'].values
        out_data[f'torque{idx}'] = (raw_curr * gains[i]) / 1000.0

    # 3. 组织 DataFrame
    df_out = pd.DataFrame(out_data)
    
    # 4. 排序
    cols = ['timestamp']
    for tag in ['pos', 'vel', 'acc', 'torque']:
        for i in range(1, 7): 
            cols.append(f'{tag}{i}')
            
    df_out = df_out[cols]
    
    # 5. 保存
    print(f"Saving formatted data to {output_file}...")
    df_out.to_csv(output_file, index=False)
    print("Done.")

if __name__ == "__main__":
    # 使用示例
    input_csv = "rmrobot/trajdata/joint_data_movej_1.csv"  # 请修改为你的原始文件名
    output_csv = "rmrobot/joint_data_movej_1_filterd.csv"
    
    if os.path.exists(input_csv):
        format_raw_with_torque_conversion(input_csv, output_csv)
    else:
        print(f"Input file {input_csv} not found.")