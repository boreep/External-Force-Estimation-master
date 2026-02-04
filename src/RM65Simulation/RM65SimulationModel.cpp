#include "RM65SimulationModel.h"
#include "rm65_config_data.h"
#include <iostream>
#include <cstring> // for memset, memcpy

// 引入 MATLAB Coder 生成的函数声明
#include "getregressorMatrix.h"
#include "getFrictionTorque.h"
#include "RM65_rena.h"
#include "RM65_rena_initialize.h"
#include "RM65_rena_terminate.h"

using namespace RM65Lib; // 使用生成的命名空间

RM65::RM65() {
    // 初始化库
    RM65_rena_initialize();
    tau.setZero();
    fric.setZero();
    tau_with_fric.setZero();
    baseParmSet1.setZero();
    baseParmSet2.setZero();

    // =============================================================
    // 一键从 rm65_config_data.h 加载所有数据
    // =============================================================
    
    // 1. 加载 baseQR 数据
    baseQR_data.numberOfBaseParameters = RM65Config::numberOfBaseParameters;
    baseQR_data.motorDynamicsIncluded  = RM65Config::motorDynamicsIncluded;
    
    // 拷贝大矩阵 (vector -> C数组)
    // 假设 vector 里的数据量和 struct 里的数组大小一致，直接内存拷贝最快
    copyVectorToArray(RM65Config::permutationMatrix, baseQR_data.permutationMatrix, 4356);
    copyVectorToArray(RM65Config::beta,              baseQR_data.beta,              1008);

    // 2. 加载 sol 数据
    copyVectorToArray(RM65Config::pi_b,  sol_data.pi_b,  42);
    baseParmSet1 = Eigen::Map<Eigen::VectorXd>(sol_data.pi_b, 42);
    copyVectorToArray(RM65Config::pi_fr, sol_data.pi_fr, 18);
    baseParmSet2 = Eigen::Map<Eigen::VectorXd>(sol_data.pi_fr, 18);
    copyVectorToArray(RM65Config::pi_s,  sol_data.pi_s,  66);

    std::cout << "[RM65] 所有动力学参数与配置已自动加载完成。" << std::endl;
}

// 辅助函数：安全的 vector 到 array 拷贝
// (建议作为类的私有成员函数或者静态辅助函数)
void RM65::copyVectorToArray(const std::vector<double>& src, double* dest, size_t expectedSize) {
    if (src.size() < expectedSize) {
        std::cerr << "[Warning] Config vector size mismatch! Filling with 0." << std::endl;
        std::memset(dest, 0, expectedSize * sizeof(double));
        // 如果数据不够，只拷贝有的部分
        std::memcpy(dest, src.data(), src.size() * sizeof(double));
    } else {
        std::memcpy(dest, src.data(), expectedSize * sizeof(double));
    }
}

// =============================================================
// 核心封装 1: 回归矩阵
// =============================================================
Matrix RM65::regressorMatrix(Vector &q, Vector &qd,Vector &q2d, double g) {
    // 准备输出缓存 (栈上分配，速度快)
    double phi_raw[252]; // 6 * 42

    // 调用生成函数
    // 注意：q.data() 返回的是 const double*，直接传给 C 函数
    getregressorMatrix(q.data(), qd.data(), q2d.data(), &baseQR_data, phi_raw);

    // 映射为 Eigen 矩阵并返回
    // MATLAB 输出默认是列优先 (ColMajor)，Eigen 默认也是，直接映射即可
    return Eigen::Map<Matrix>(phi_raw, 6, 42);
}

Vector RM65::rnea(Vector &q, Vector &qd, Vector &q2d, double g) {

    Matrix phi = regressorMatrix(q, qd, q2d, g);

    tau = phi * baseParmSet1;

    return tau;
}

// =============================================================
// 核心封装 2: 摩擦力矩
// =============================================================
Vector RM65::getFriction(Vector &qd) {

    double tau_f_raw[6];
    // getFriction 接受 pi_fr 数组指针，我们从 sol_data 结构体里取
    getFrictionTorque(qd.data(), sol_data.pi_fr, tau_f_raw);
    fric = Eigen::Map<Vector>(tau_f_raw, 6);

    return fric;
}

// =============================================================
// 核心封装 3: 逆动力学 (RNEA)
// =============================================================

Vector RM65::inverseDynamics(Vector &q, Vector &qd, Vector &q2d) {
    double full_tau[6];

    // 调用 RM65_rena，传入 baseQR 和 sol 的指针
    RM65_rena(q.data(), qd.data(), q2d.data(), &baseQR_data, &sol_data, full_tau);
    tau_with_fric = Eigen::Map<Vector>(full_tau, 6);

    return tau_with_fric;
}

int RM65::jointNo() {
    return RM65DOF;
}

// 在 RM65SimulationModel.cpp 文件末尾或合适位置添加

// 1. 末端执行器速度雅可比矩阵
// 注意：这里的 Matrix 需要替换为你代码中实际使用的类型（如 Eigen::MatrixXd）
Matrix RM65::getVelocityJacobianEndEffector(const Vector& q) { 
    // TODO: 实现具体的雅可比计算逻辑
    return Matrix::Zero(6, 6); // 假设返回一个 6x6 的零矩阵占位
}

// 2. 基座速度雅可比矩阵
Matrix RM65::getVelocityJacobianBase(const Vector& q) { 
    // TODO: 实现具体的雅可比计算逻辑
    return Matrix::Zero(6, 6); // 占位
}

// 3. 力变换矩阵
Matrix RM65::getForceTransformMatrix() { 
    // TODO: 实现具体的变换矩阵逻辑
    return Matrix::Identity(6, 6); // 占位
}