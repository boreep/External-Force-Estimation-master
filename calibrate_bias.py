import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
import sys

# ================= 配置区域 =================
# 输入文件：请确保这个 CSV 是机器人在【无外力】状态下运行生成的！
# 必须包含: q1...q6, qd1...qd6, est_tau1...est_tau6
INPUT_CSV = "rmrobot/output/my_calibration_data.csv" 
JOINT_NUM = 6
# ===========================================

def sign_smooth(x, slope=10.0):
    return np.tanh(slope * x)

def train_correction_model(csv_path):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    print("="*60)
    print(f"Loading data from: {csv_path}")
    print(f"Data points: {len(df)}")
    print("Training Bias Compensation Model...")
    print("="*60)
    
    # 存储生成的 C++ 代码
    cpp_code_lines = []
    
    # 对每个关节单独训练
    for i in range(JOINT_NUM):
        # 1. 准备数据
        # 列名需要根据你实际 CSV 的 header 调整，这里假设是标准格式
        # 如果你的 CSV header 是 est_0, est_1... 请相应修改
        try:
            # 尝试匹配常见的列名格式
            if f'est_tau{i+1}' in df.columns: # est_tau1
                y = df[f'est_tau{i+1}'].values
            elif f'est_{i}' in df.columns:    # est_0
                y = df[f'est_{i}'].values
            else:
                # 尝试按列索引读取 (假设顺序: time, est0-5, meas0-5...)
                # 这是一个兜底策略，最好还是检查列名
                y = df.iloc[:, 1+i].values 

            # 获取状态量 q 和 qd
            # 这里假设 CSV 里可能没有 q 和 qd，只有 est。
            # 如果 output.csv 里没有 q/qd，你需要修改 C++ 让它把 q/qd 也存进去！
            # 现在的 RM65_Test.cpp 输出里好像没存 q 和 qd？
            #  修正 : 你的 RM65_Test.cpp 输出确实只有 time, est, meas, gt
            # 你需要修改 C++ 先把 q 和 qd 存下来，或者直接读取原始的 Traj3_out.csv 配合时间戳？
            # 为了简便，建议你修改 C++ 的 output，把 q 和 qd 也打进去。
            # 这里我假设你已经把 q 和 qd 加到 CSV 里了。
            
            # 临时解决方案：如果 output.csv 没存 q/qd，我们无法做状态相关补偿，只能做常数补偿(Offset)。
            # 但既然是"二次校准"，强烈建议你把 q 和 qd 加到输出里。
            
            # 假设列名如下 (需修改 C++ 对应输出):
            q = df[f'q{i+1}'].values if f'q{i+1}' in df.columns else np.zeros_like(y)
            qd = df[f'qd{i+1}'].values if f'qd{i+1}' in df.columns else np.zeros_like(y)
            
        except KeyError as e:
            print(f"Key Error: {e}. 请检查 CSV 列名。")
            continue

        # 2. 构建特征矩阵 X
        # Model: y = k_v*qd + k_c*tanh(10*qd) + k_sin*sin(q) + k_cos*cos(q) + offset
        X = np.column_stack([
            qd,
            np.tanh(10.0 * qd),
            np.sin(q),
            np.cos(q)
        ])
        
        # 3. 拟合
        model = LinearRegression(fit_intercept=True)
        model.fit(X, y)
        
        coef = model.coef_ # [k_v, k_c, k_sin, k_cos]
        intercept = model.intercept_ # offset
        
        score = model.score(X, y)
        print(f"Joint {i+1}: R^2 Score = {score:.4f} (拟合度)")

        # 4. 生成 C++ 代码格式
        # 格式: {k_v, k_c, k_sin, k_cos, offset}
        line = f"    params[{i}] = {{{coef[0]:.6f}, {coef[1]:.6f}, {coef[2]:.6f}, {coef[3]:.6f}, {intercept:.6f}}};"
        cpp_code_lines.append(line)

    print("\n" + "="*20 + " C++ COPY PASTE BLOCK " + "="*20)
    print("// 请将以下代码复制到 ResidualCompensator 类的构造函数中")
    for line in cpp_code_lines:
        print(line)
    print("="*60)

if __name__ == "__main__":
    train_correction_model(INPUT_CSV)