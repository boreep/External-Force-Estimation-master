#ifndef RM65SIMULATIONMODEL_H
#define RM65SIMULATIONMODEL_H

#include "ExtObserverLib_v3/external_observer.h"
#include <Eigen/Dense>

// 引入 MATLAB Coder 生成的类型定义
#include "RM65_rena_types.h" 

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

const int RM65DOF = 6;
const int baseParamSet1Size = 42;
const int baseParamSet2Size = 18;


class RM65 : public RobotDynamicsRnea {
 public:
  /**
   * @brief 默认构造函数
   */
  RM65();
  /**
   * @brief RNEA实现，继承RobotDynamicsRnea
   */
  Vector rnea(Vector &q, Vector &qd, Vector &q2d, double g = 0) override;
  /**
   * @brief 逆动力学实现，继承RobotDynamicsRnea
   */
  Vector inverseDynamics(Vector &q, Vector &qd, Vector &q2d,double g);
  /**
   * @brief friction，继承RobotDynamicsBase
   */
  Vector getFriction(Vector &qd) override;
  /**
   * @brief joint degree，继承RobotDynamicsBase
   */
  int jointNo() override;
  /**
   * @brief Jacobian matrix:TCP相对基坐标系的雅可比矩阵在TCP坐标系中的表示
   */

  Matrix getVelocityJacobianEndEffector(const Vector &q) override;
  /**
   * @brief Jacobian matrix:TCP相对基坐标系的雅可比矩阵在基坐标系中的表示
   */
  Matrix getVelocityJacobianBase(const Vector &q) override;
  /**
   * @brief 力变换矩阵:将力从传感器坐标系变换到TCP坐标系
   */
  Matrix getForceTransformMatrix() override;

 private:
  /**
   * @brief 读取基本惯性参数，通过辨识得到，，前42个是与惯性参数相关，后18个与摩擦力相关
   */
  // 辅助函数
  void copyVectorToArray(const std::vector<double>& src, double* dest, size_t expectedSize);

  /**
   * @brief 回归矩阵计算
   * @param q 关节位置
   * @param qd 关节速度
   * @param qdd 关节加速度
   * @param g 重力常量
   */
  Matrix regressorMatrix(Vector &q, Vector &qd, Vector &q2d, double g);

  Vector tau, fric, tau_with_fric, baseParmSet1, baseParmSet2;
  Matrix jacobian, forceTransformMatrix;
  RM65Lib::struct0_T baseQR_data;
  RM65Lib::struct1_T    sol_data;
};

#endif