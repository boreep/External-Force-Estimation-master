import pandas as pd
import numpy as np
from scipy.optimize import linprog

def optimize_threshold_params_full_dynamics(
    csv_file,
    lambda_slack=100.0,      # 松弛惩罚：越小越允许偶尔的“刺破”
    max_slack_per_sample=5.0,
    bounds_z=(0.0, 5.0),     # [修改] 稍微放宽 z 的上限，允许动态项系数更大
    max_k=1.5,               # [新增] 关键！强制 K 的最大值 (Nm)。根据你的传感器底噪设置，越小越动态。
    k_penalty_weight=2.0,    # [新增] 关键！K 的惩罚权重。>1.0 表示让 K 变得比动态项“更贵”
):
    """
    全动力学动态阈值辨识 (增强动态版):
      Th = zM*|M_t| + zC*|C_t| + zG*|G_t| + zF+*F+ + zF-*F- + K
    """
    print(f"Loading data from {csv_file}...")
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("Error: File not found!")
        return

    num_joints = 6
    params_list = []
    
    # 打印配置信息
    print(f"Configuration: max_k={max_k}, k_penalty={k_penalty_weight}, slack_penalty={lambda_slack}")

    print("\n" + "="*135)
    print(f"{'J':<3} | {'zM':<8} | {'zC':<8} | {'zG':<8} | {'zF+':<8} | {'zF-':<8} | {'K':<8} | {'mean_s':<8} | {'p95_s':<8} | {'Dyn%':<5}")
    print("-" * 135)

    for i in range(num_joints):
        # 读取特征
        R = np.abs(df[f"res_{i}"].to_numpy())  # |residual|
        M = np.abs(df[f"m_{i}"].to_numpy())    # |Inertia|
        C = np.abs(df[f"c_{i}"].to_numpy())    # |Coriolis|
        G = np.abs(df[f"g_{i}"].to_numpy())    # |Gravity|
        F = df[f"f_{i}"].to_numpy()

        Fplus  = np.maximum(F, 0.0)
        Fminus = np.maximum(-F, 0.0)

        N = len(R)

        # 变量: [zM, zC, zG, zF+, zF-, K, s_0...s_N-1]
        n_basic_vars = 6 
        n_vars = n_basic_vars + N

        # --- 目标函数构建 (关键修改) ---
        # min sum(Threshold) + lambda * sum(slack)
        # 也就是: min sum(zM*M + ... + K)
        # 系数对应: zM -> sum(M), K -> sum(1)*N
        
        c_obj = np.zeros(n_vars)
        c_obj[0] = np.sum(M)
        c_obj[1] = np.sum(C)
        c_obj[2] = np.sum(G)
        c_obj[3] = np.sum(Fplus)
        c_obj[4] = np.sum(Fminus)
        
        # [修改点] K 的代价系数
        # 原理：原本 K 的系数是 N (样本数)。如果不惩罚，优化器觉得用 1nm 的 K 和用 1nm 的动态项 (平均值) 代价一样。
        # 这里我们将 K 的代价乘以 k_penalty_weight，让它变得“昂贵”。
        c_obj[5] = N * k_penalty_weight 
        
        c_obj[n_basic_vars:] = lambda_slack

        # 约束: -Threshold - s <= -|R|
        A_ub = np.zeros((N, n_vars))
        A_ub[:, 0] = -M
        A_ub[:, 1] = -C
        A_ub[:, 2] = -G
        A_ub[:, 3] = -Fplus
        A_ub[:, 4] = -Fminus
        A_ub[:, 5] = -1.0
        A_ub[np.arange(N), n_basic_vars + np.arange(N)] = -1.0

        b_ub = -R

        # --- 边界设置 (关键修改) ---
        # K 的边界被 max_k 限制住，逼迫算法去寻找动态参数
        bounds_k = (0.05, max_k)
        
        bounds = [bounds_z] * 5 + [bounds_k] + [(0.0, max_slack_per_sample)] * N

        res = linprog(c_obj, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method="highs")

        if not res.success:
            print(f"{i+1} | Failed")
            params_list.append([0]*6)
            continue

        x = res.x
        zM, zC, zG, zFp, zFm, K = x[:6]
        slack = x[6:]
        
        # 计算动态项贡献占比 (调试用)
        # 看一下在所有样本中，动态部分平均贡献了多少阈值
        avg_thresh_dyn = zM*np.mean(M) + zC*np.mean(C) + zG*np.mean(G) + zFp*np.mean(Fplus) + zFm*np.mean(Fminus)
        avg_thresh_total = avg_thresh_dyn + K
        dyn_ratio = (avg_thresh_dyn / avg_thresh_total) * 100 if avg_thresh_total > 1e-6 else 0

        params_list.append([zM, zC, zG, zFp, zFm, K])
        print(f"{i+1:<3} | {zM:<8.4f} | {zC:<8.4f} | {zG:<8.4f} | {zFp:<8.4f} | {zFm:<8.4f} | {K:<8.4f} | {np.mean(slack):<8.4f} | {np.percentile(slack, 95):<8.4f} | {dyn_ratio:>4.1f}%")

    print("="*135)
    print("\n// Update DynamicThresholdDetector.h with:")
    print("// Order: {zM, zC, zG, zF_plus, zF_minus, K}")
    print("double params[6][6] = {")
    for j, p in enumerate(params_list):
        comma = "," if j < 5 else ""
        print(f"    {{{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}, {p[3]:.4f}, {p[4]:.4f}, {p[5]:.4f}}}{comma} // J{j+1}")
    print("};")

if __name__ == "__main__":
    # 使用建议：
    # 1. 如果你的底噪(静态时残差)大约是 0.5 Nm，那么把 max_k 设为 0.6~0.8 左右。
    # 2. k_penalty_weight 设为 2.0~5.0 可以有效逼出 zM 和 zC。
    optimize_threshold_params_full_dynamics(
        "threshold_data.csv", 
        max_k=0.8,              # 尝试调小这个值！例如 0.8 或 1.0
        k_penalty_weight=3.0,   # 惩罚 K，让算法更爱用动态项
        bounds_z=(0.0, 10.0)    # 放宽动态系数上限
    )