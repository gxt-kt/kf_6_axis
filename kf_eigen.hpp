#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

// @input:
//   time: time stamp (s)
//   acc_*: accelerate (m/s^2) actually the unit is not important
//   gyro_*: gyroscope (rad/s)
// @Output:
//   Eigen::Vector3<T> : roll pitch yaw (rad/s)
template <typename Time, typename T>
inline Eigen::Vector3<T> KF_6_Axis(Time time, T acc_x, T acc_y, T acc_z,
                                   T gyro_x, T gyro_y, T gyro_z) {
  static Eigen::Matrix<T, 3, 3> Q = decltype(Q)::Identity();
  Q << 0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002;
  static Eigen::Matrix<T, 3, 3> R = decltype(R)::Identity();
  R << 0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2;

  static Eigen::Matrix<T, 3, 3> A = decltype(A)::Identity();
  static Eigen::Matrix<T, 3, 3> B = decltype(B)::Identity();
  static Eigen::Matrix<T, 3, 3> H = decltype(H)::Identity();
  H << 1, 0, 0, 0, 1, 0, 0, 0, 0;

  static Eigen::Vector3<T> X_bar_k_1 = decltype(X_bar_k_1)::Zero();
  static Eigen::Vector3<T> u_k = decltype(u_k)::Zero();
  static Eigen::Vector3<T> X_bar_k__ = decltype(X_bar_k__)::Zero();
  static Eigen::Vector3<T> X_bar_k = decltype(X_bar_k)::Zero();
  static Eigen::Vector3<T> Z_k = decltype(Z_k)::Zero();
  static Eigen::Matrix<T, 3, 3> P_k_1 = decltype(P_k_1)::Zero();
  static Eigen::Matrix<T, 3, 3> P_k__ = decltype(P_k__)::Zero();
  static Eigen::Matrix<T, 3, 3> P_k = decltype(P_k)::Identity();
  static Eigen::Matrix<T, 3, 3> K_k = decltype(K_k)::Zero();
  static Eigen::Matrix<T, 3, 3> I = decltype(I)::Identity();

  /*计算微分时间*/
  static Time last_time;           // 上一次采样时间(s)
  double dt = (time - last_time);  // 微分时间(s)
  last_time = time;

  B = decltype(B)::Identity() * dt;
  /*计算x,y轴上的角速度*/
  T droll_dt =
      gyro_x +
      ((sin(X_bar_k(1)) * sin(X_bar_k(0))) / cos(X_bar_k(1))) * gyro_y +
      ((sin(X_bar_k(1)) * cos(X_bar_k(0))) / cos(X_bar_k(1))) *
          gyro_z;  // roll轴的角速度
  T dpitch_dt =
      cos(X_bar_k(0)) * gyro_y - sin(X_bar_k(0)) * gyro_z;  // pitch轴的角速度
  T dyaw_dt = sin(X_bar_k(0)) / cos(X_bar_k(1)) * gyro_y +
              cos(X_bar_k(0)) / cos(X_bar_k(1)) * gyro_z;

  u_k(0) = droll_dt;
  u_k(1) = dpitch_dt;
  u_k(2) = dyaw_dt;

  // 第一步计算 状态外推方程
  X_bar_k__ = A * X_bar_k_1 + B * u_k;
  // 第二步计算 协方差外推方程
  P_k__ = A * P_k_1 * A.transpose() + Q;
  // 第三步计算 卡尔曼增益
  K_k = P_k__ * H.transpose() * (H * P_k__ * H.transpose() + R).inverse();
  // 第四步计算 更新协方差矩阵
  P_k = (I - K_k * H) * P_k__;

  // 下面计算观测量
  // roll角度
  T acc_roll = atan(acc_y / acc_z);
  // pitch角度
  T acc_pitch = -1 * atan(acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2)));
  // 对观测矩阵赋值
  Z_k(0) = acc_roll;
  Z_k(1) = acc_pitch;
  Z_k(2) = 0;

  // 第五步计算 更新状态量
  X_bar_k = X_bar_k__ + K_k * (Z_k - H * X_bar_k__);

  // 更改历史值为下次循环做准备
  P_k_1 = P_k;
  X_bar_k_1 = X_bar_k;

  return {X_bar_k(0), X_bar_k(1), X_bar_k(2)};
}
