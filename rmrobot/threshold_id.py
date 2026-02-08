import pandas as pd
import numpy as np
from scipy.optimize import linprog

def optimize_threshold_params_slack_fric_split(
    csv_file,
    lambda_slack=100.0,     # slack 惩罚系数：越大越“逼近硬约束”，越小越允许离群点
    max_slack_per_sample=5.0, # 每个样本 slack 上限（Nm），防止数值发散
    bounds_z=(0.0, 2.0),
    bounds_k=(0.05, 10.0),
):
    """
    动态阈值辨识（带 slack + 摩擦正/负拆分）:
      Threshold_t = zC*|C_t| + zG*|G_t| + zF+*Fplus_t + zF-*Fminus_t + K
      约束：Threshold_t + s_t >= |R_t|,  s_t >= 0
      目标：min sum(Threshold_t) + lambda_slack * sum(s_t)

    说明：
      - 这样不会被少量毛刺/离群点把整体阈值抬爆（slack 吸收）
      - 摩擦拆正负可拟合方向不对称摩擦残差
    """
    print(f"Loading data from {csv_file}...")
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("Error: File not found! Please run the C++ generator first.")
        return

    num_joints = 6
    params_list = []

    print("\n" + "="*110)
    print("辨识结果（slack + 摩擦正负拆分）:")
    print(f"{'Joint':<6} | {'zC':<10} | {'zG':<10} | {'zF+':<10} | {'zF-':<10} | {'K':<10} | {'mean_slack':<12} | {'p95_slack':<12}")
    print("-"*110)

    for i in range(num_joints):
        # 数据
        R = np.abs(df[f"res_{i}"].to_numpy())  # |residual|
        C = np.abs(df[f"c_{i}"].to_numpy())    # |coriolis|
        G = np.abs(df[f"g_{i}"].to_numpy())    # |gravity|
        F = df[f"f_{i}"].to_numpy()            # friction (signed)

        # 摩擦正/负拆分特征
        Fplus  = np.maximum(F, 0.0)
        Fminus = np.maximum(-F, 0.0)

        N = len(R)

        # 变量顺序： [zC, zG, zFplus, zFminus, K, s_0 ... s_{N-1}]
        n_vars = 5 + N

        # 目标函数：min sum_t (zC*C + zG*G + zF+*F+ + zF-*F- + K) + lambda*sum(s)
        c_obj = np.zeros(n_vars)
        c_obj[0] = np.sum(C)
        c_obj[1] = np.sum(G)
        c_obj[2] = np.sum(Fplus)
        c_obj[3] = np.sum(Fminus)
        c_obj[4] = N
        c_obj[5:] = lambda_slack  # slack 权重

        # 约束： zC*C_t + zG*G_t + zF+*Fplus_t + zF-*Fminus_t + K + s_t >= R_t
        # 转为 A_ub x <= b_ub：
        # -(...) - s_t <= -R_t
        A_ub = np.zeros((N, n_vars))
        A_ub[:, 0] = -C
        A_ub[:, 1] = -G
        A_ub[:, 2] = -Fplus
        A_ub[:, 3] = -Fminus
        A_ub[:, 4] = -1.0
        # slack 的系数：每行对应自己的 s_t
        A_ub[np.arange(N), 5 + np.arange(N)] = -1.0

        b_ub = -R

        # bounds
        bounds = [
            bounds_z,      # zC
            bounds_z,      # zG
            bounds_z,      # zFplus
            bounds_z,      # zFminus
            bounds_k,      # K
        ] + [(0.0, max_slack_per_sample)] * N  # s_t

        res = linprog(c_obj, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method="highs")

        if not res.success:
            print(f"J{i+1:<5} | Optimization Failed: {res.message}")
            params_list.append([0.1, 0.1, 0.1, 0.1, 0.5])
            continue

        x = res.x
        zC, zG, zFp, zFm, K = x[:5]
        slack = x[5:]

        params_list.append([zC, zG, zFp, zFm, K])

        print(f"J{i+1:<5} | {zC:<10.4f} | {zG:<10.4f} | {zFp:<10.4f} | {zFm:<10.4f} | {K:<10.4f} | {np.mean(slack):<12.4f} | {np.percentile(slack, 95):<12.4f}")

    print("="*110)

    print("\n// 请将以下代码复制到 DynamicThresholdDetector.h 的 params 数组中（每关节 5 个参数）")
    print("// 顺序: {zC, zG, zF_plus, zF_minus, K}")
    print("double params[6][5] = {")
    for j, p in enumerate(params_list):
        comma = "," if j < 5 else ""
        print(f"    {{{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}, {p[3]:.4f}, {p[4]:.4f}}}{comma} // Joint {j+1}")
    print("};")

    print("\n提示：")
    print(" - 如果阈值仍偏大：把 lambda_slack 调小一点（如 50）或把 max_slack_per_sample 调小（如 2）")
    print(" - 如果阈值仍被毛刺抬高：把 lambda_slack 调大一点（如 150~300）")
    print(" - 看 mean_slack / p95_slack：如果很大，说明残差里离群点/模型误差很多，建议先检查 τ_meas 偏置或速度滤波")


if __name__ == "__main__":
    optimize_threshold_params_slack_fric_split("rmrobot/threshold_data.csv")
