#include <iostream>
#include <iomanip>
#include <cmath>
// 【新增】引入 Windows 头文件
#ifdef _WIN32
#include <windows.h>
#endif


// 引入你的封装类头文件
#include "RM65SimulationModel.h"

using namespace std;

int main() {
    // 【新增】如果是 Windows，强制设置控制台输出为 UTF-8
    #ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    #endif
    // =============================================================
    // 1. 实例化模型
    // =============================================================
    std::cout << "正在初始化 RM65 动力学模型..." << std::endl;
    RM65 robot; 
    std::cout << "模型初始化完成！" << std::endl;

    // =============================================================
    // 2. 准备测试数据
    // =============================================================
    
    // 使用 comma initializer (<<) 是 Eigen 最标准、兼容性最好的写法
    Vector q_in(6);
    q_in << 0.100000, -0.500000, 1.200000, -0.800000, 1.500000, 0.000000;
    
    Vector qd_in(6);
    qd_in << 0.200000, 0.000000, -0.300000, 0.100000, 0.000000, 0.100000;
    
    Vector q2d_in(6);
    q2d_in << 0.000000, 0.100000, 0.000000, -0.100000, 0.200000, 0.000000;

    // 填入 MATLAB 打印的期望值
    Vector tau_expected_matlab(6);
    tau_expected_matlab << 6.668621, 4.207260, -2.531207, 4.790006, -1.943114, 2.547556;

    // =============================================================
    // 3. 执行 C++ 计算
    // =============================================================
    
    // 【修正】这里必须使用上面定义的变量名 q_in, qd_in, q2d_in
    Vector tau_actual_cpp = robot.inverseDynamics(q_in, qd_in, q2d_in);
    
    // =============================================================
    // 4. 结果对比
    // =============================================================
    std::cout << "\n============================================================" << std::endl;
    std::cout << " 关节 |   MATLAB期望值 (Nm)  |    C++计算值 (Nm)    |    误差    " << std::endl;
    std::cout << "============================================================" << std::endl;

    std::cout << std::fixed << std::setprecision(6); 

    double max_error = 0.0;
    for (int i = 0; i < 6; i++) {
        double diff = std::abs(tau_expected_matlab(i) - tau_actual_cpp(i));
        if (diff > max_error) max_error = diff;

        std::cout << "  J" << i + 1 << "  | " 
                  << std::setw(18) << tau_expected_matlab(i) << " | " 
                  << std::setw(18) << tau_actual_cpp(i) << " | " 
                  << std::setw(12) << std::scientific << diff << std::fixed << std::endl;
    }
    std::cout << "============================================================" << std::endl;

    if (max_error < 1e-5) {
        std::cout << "✅ 验证通过！C++ 模型与 MATLAB 结果完全一致。" << std::endl;
    } else {
        std::cout << "❌ 验证失败！最大误差: " << max_error << std::endl;
        std::cout << "建议检查：\n"
                  << "1. rm65_config_data.h 中的 permutationMatrix 是否全为0？\n"
                  << "2. verify_one_point.m 使用的参数是否和 config 文件一致？" << std::endl;
    }

    return 0;
}