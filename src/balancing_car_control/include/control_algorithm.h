#ifndef _CONTROL_ALGORITHM_H
#define _CONTROL_ALGORITHM_H

namespace control_algorithm {
namespace pid {
class PID {
 public:
  // 构造函数
  PID(double Kp, double Ki, double Kd, double maxOutput, double minOutput)
      : Kp_(Kp),
        Ki_(Ki),
        Kd_(Kd),
        maxOutput_(maxOutput),
        minOutput_(minOutput),
        integral_(0.0),
        prev_error_(0.0) {}

  // 更新PID控制器
  inline double update(double setValue, double feedBack) {
    double error = setValue - feedBack;       // 计算误差
    integral_ += error;                       // 积分项
    double derivative = error - prev_error_;  // 微分项

    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

    // 限制输出
    output = output > maxOutput_ ? maxOutput_ : output;
    output = output < minOutput_ ? minOutput_ : output;

    // 更新上一次的误差
    prev_error_ = error;

    return output;
  }

 private:
  double Kp_;          // 比例增益
  double Ki_;          // 积分增益
  double Kd_;          // 微分增益
  double maxOutput_;   // 最大输出
  double minOutput_;   // 最小输出
  double integral_;    // 积分项
  double prev_error_;  // 上一次的误差
};
}  // namespace pid
}  // namespace control_algorithm

#endif