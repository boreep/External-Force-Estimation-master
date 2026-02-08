import pandas as pd
import numpy as np
import os

def format_raw_with_torque_conversion(input_file, output_file):
    # 1. 读取数据
    print(f"Reading raw data from {input_file}...")
    df = pd.read_csv(input_file)
    
    # 2. 准备参数
    dof = 6
    # 关节电流转力矩系数
    gains = np.array([3.94, 3.94, 2.09, 8.28, 8.28, 8.28])
    
    out_data = {'timestamp': df['timestamp']}
    
    # --- 核心循环 ---
    for i in range(dof):
        idx = i + 1
        
        # A. 位置 (Position) - 直接拷贝原始值
        out_data[f'pos{idx}'] = df[f'pos_{i}']
        
        # B. 速度 (Velocity) - 直接拷贝原始值
        out_data[f'vel{idx}'] = df[f'vel_{i}']
        
        # C. 加速度 (Acceleration) - 填空 (填 0.0)
        # 注意：C++ 的 stod 函数无法处理空字符串，所以这里填 0.0 是最安全的
        out_data[f'acc{idx}'] = 0.0
        
        # D. 力矩 (Torque) - 仅做单位转换，不滤波
        # 公式：(原始电流 * gain) / 1000
        raw_curr = df[f'cur_{i}'].values
        out_data[f'torque{idx}'] = (raw_curr * gains[i]) / 1000.0

    # 3. 组织 DataFrame
    df_out = pd.DataFrame(out_data)
    
    # 4. 严格按照 C++ 程序要求的列顺序排列
    # 顺序: Time, Pos(1-6), Vel(1-6), Acc(1-6), Torque(1-6)
    cols = ['timestamp']
    for tag in ['pos', 'vel', 'acc', 'torque']:
        for i in range(1, 7): 
            cols.append(f'{tag}{i}')
            
    df_out = df_out[cols]
    
    # 5. 保存
    df_out.to_csv(output_file, index=False)
    print(f"Processed data saved to {output_file}")
    print(f"Summary: No filtering applied. Acceleration set to 0. Torque calculated from raw current.")

if __name__ == "__main__":
    # 输入文件路径
    input_csv = 'rmrobot/joint_data_movej_1_2.csv'
    # 输出文件路径 (例如命名为 raw_formatted.csv)
    output_csv = 'rmrobot/joint_data_movej_1_2_formatted.csv'
    
    if os.path.exists(input_csv):
        format_raw_with_torque_conversion(input_csv, output_csv)
    else:
        print(f"Error: {input_csv} not found.")