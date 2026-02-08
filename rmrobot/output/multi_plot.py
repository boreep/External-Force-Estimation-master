import pandas as pd
import matplotlib.pyplot as plt
import os

# ================= 配置区域 =================
# 在这里填入你的 CSV 文件路径列表
file_list = [
    "rmrobot/output/0模拟力矩/estimated_force_result_2.csv",
    "rmrobot/output/0模拟力矩/estimated_force_result_3.csv",
    "rmrobot/output/0模拟力矩/estimated_force_result_5.csv" 
    # 你可以继续添加更多文件...
]


# file_list = [
#     "rmrobot/output/5模拟力矩/estimated_force_result_2.csv",
#     "rmrobot/output/5模拟力矩/estimated_force_result_3.csv",
#     "rmrobot/output/5模拟力矩/estimated_force_result_5.csv" 
#     # 你可以继续添加更多文件...
# ]

# 关节数量
JOINT_COUNT = 6
# ===========================================

def plot_comparison():
    # 1. 读取所有数据
    data_frames = []
    valid_files = []
    
    print("正在读取文件...")
    for file_path in file_list:
        if os.path.exists(file_path):
            try:
                df = pd.read_csv(file_path)
                data_frames.append(df)
                valid_files.append(os.path.basename(file_path)) # 仅保留文件名用于逻辑判断
                print(f"成功加载: {file_path}")
            except Exception as e:
                print(f"读取错误 {file_path}: {e}")
        else:
            print(f"文件不存在 (跳过): {file_path}")

    if not data_frames:
        print("没有有效的数据文件，程序终止。")
        return

    # 基准数据 (取第一个文件)
    base_df = data_frames[0]
    time = base_df['time']

    # 定义数字到算法名称的映射关系
    observer_map = {
        '0': 'Momentum',
        '1': 'Nonlinear',
        '2': 'SlidingMode',
        '3': 'Momentum',
        '4': 'Kalman',
        '5': 'ZOH Kalman'
    }

    # 2. 创建画布
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    axes = axes.flatten()

    # 3. 循环绘制每个关节
    for j in range(JOINT_COUNT):
        ax = axes[j]
        joint_idx = j + 1 
        
        # --- A. 绘制 Measured Tau (只画一次, 半透明) ---
        meas_col = f'meas_tau{joint_idx}'
        if meas_col in base_df.columns:
            ax.plot(time, base_df[meas_col], 
                    label='Measured Torque', 
                    color='gray', 
                    alpha=0.5, 
                    linewidth=2,
                    zorder=1)

        # --- B. 绘制 Ground Truth Tau (只画一次, 虚线) ---
        gt_col = f'gt_tau{joint_idx}'
        if gt_col in base_df.columns:
            ax.plot(time, base_df[gt_col], 
                    label='Sim Ext Torque', 
                    color='black', 
                    linestyle='--', 
                    linewidth=1.5,
                    zorder=2)

        # --- C. 绘制 Estimated Tau (每个文件画一条) ---
        est_col = f'est_tau{joint_idx}'
        for i, df in enumerate(data_frames):
            if est_col in df.columns:
                current_filename = valid_files[i]
                
                # --- 核心修改：根据文件名数字确定图例名称 ---
                label_name = f'Est ({current_filename})' # 默认名称（如果没有匹配到数字）
                
                # 遍历字典查找匹配的数字
                for digit, name in observer_map.items():
                    if digit in current_filename:
                        label_name = name
                        break # 找到后立即停止，防止被后续数字覆盖
                # ----------------------------------------

                ax.plot(df['time'], df[est_col], 
                        label=label_name, 
                        linewidth=1.5,
                        zorder=3)

        # 设置子图样式
        ax.set_title(f'Joint {joint_idx}', fontsize=12, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.grid(True, linestyle=':', alpha=0.6)
        
        # 图例设置
        if j == 0:
            ax.legend(loc='upper right', fontsize='small', framealpha=0.9)
        else:
            ax.legend(loc='best', fontsize='x-small')

    fig.suptitle(f'Torque Estimation Comparison ({len(valid_files)} Methods)', fontsize=16)
    plt.show()

if __name__ == "__main__":
    plot_comparison()