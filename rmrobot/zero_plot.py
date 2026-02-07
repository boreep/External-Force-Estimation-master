import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ================= 配置 =================
CSV_FILE = 'rmrobot/q_ga_5_raw_formatted.csv'  # 请修改为你的实际文件名
# =======================================

def plot_raw_data():
    # 1. 读取数据
    if not os.path.exists(CSV_FILE):
        print(f"错误: 找不到文件 {CSV_FILE}")
        return

    try:
        df = pd.read_csv(CSV_FILE)
        print(f"成功读取 {len(df)} 行数据。")
    except Exception as e:
        print(f"读取 CSV 失败: {e}")
        return

    # 2. 处理时间轴 (归一化到从 0 开始)
    if 'timestamp' in df.columns:
        t = df['timestamp'].values - df['timestamp'].values[0]
    else:
        print("警告: 未找到 'timestamp' 列，使用行号作为 x 轴")
        t = df.index.values

    # 3. 自动根据列名前缀分组
    # 定义我们想要可视化的组及其对应的列名前缀
    # 格式: '显示标题': '列名前缀'
    group_definitions = {
        'Position (pos_*)': 'pos',
        'Velocity (vel_*)': 'vel',
        'acclerrate (cur_*)': 'acc',
        'Force/Torque (force_*)': 'torque'
    }

    # 筛选出当前 CSV 中实际存在的组
    valid_groups = {}
    for title, prefix in group_definitions.items():
        # 查找所有以 prefix 开头的列名 (例如 'pos_0', 'pos_1'...)
        cols = [c for c in df.columns if c.startswith(prefix)]
        if cols:
            valid_groups[title] = cols

    if not valid_groups:
        print("错误: CSV 中未找到符合 pos_, vel_, cur_, force_ 前缀的数据列。")
        return

    # 4. 绘图
    n_plots = len(valid_groups)
    # 动态调整高度：每张子图分配约 3.5 英寸高度
    fig, axs = plt.subplots(n_plots, 1, figsize=(12, 3.5 * n_plots), sharex=True)

    # 如果只有一张图，axs 不是列表，将其转换为列表以便统一处理
    if n_plots == 1:
        axs = [axs]

    # 遍历每个组进行绘制
    for i, (title, cols) in enumerate(valid_groups.items()):
        ax = axs[i]
        
        # 绘制该组下的所有列
        for col_name in cols:
            ax.plot(t, df[col_name], label=col_name, linewidth=1.2, alpha=0.9)
        
        ax.set_title(title, fontweight='bold', pad=10)
        ax.set_ylabel('Raw Value')
        ax.grid(True, linestyle='--', alpha=0.5)
        
        # 图例放旁边或者上方，避免遮挡数据
        ax.legend(loc='upper right', fontsize='small', ncol=6)

    # 设置最底部的 X 轴标签
    axs[-1].set_xlabel('Time (s)', fontsize=12)

    # 调整布局防止重叠
    plt.tight_layout()
    
    print(f"绘制完成，共生成 {n_plots} 张子图。")
    plt.show()

if __name__ == "__main__":
    plot_raw_data()