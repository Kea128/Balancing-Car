#include "balancing_car_control/control_param.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"  // 包含关节状态消息类型的头文件
#include "std_msgs/Float64MultiArray.h"  // 包含标准消息类型 Float64MultiArray 的头文件，用于发布轮子扭矩
#include "tf/tf.h"  // 包含 TF 变换的头文件，用于四元数和欧拉角的转换。

ros::ServiceClient client;  // 服务客户端，用于请求控制解决方案
ros::Publisher wheelsCommandPub;  // 发布者，用于发布轮子扭矩命令

double lastPitch,
    lastYaw;  // The previous Euler angle is used to obtain the angular velocity
double leftWheelVelocity, rightWheelVelocity;  // 左右轮的速度
double dst_vel;                                // 目标速度
double dst_yaw;                                // 目标偏航角

// Subscribe the speed of two rounds
// 接收和处理轮子速度信息
void WheelVelocityCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg->velocity.size() > 0) {  // 检查消息中是否有速度信息
    leftWheelVelocity = msg->velocity[0];   // 获取左轮速度
    rightWheelVelocity = msg->velocity[1];  // 获取右轮速度
  }
}

// Subscribe keyboard control information
// 回调函数，用于接收和处理键盘控制命令
void KeyboardCmd(geometry_msgs::Twist msg) {
  dst_vel = msg.linear.x;   //  获取目标速度
  dst_yaw = msg.angular.z;  // 获取目标偏航角
}

// Subscribe IMU data
// 用于接收和处理 IMU 数据。
void IMUCallback(sensor_msgs::Imu msg) {
  if (msg.orientation_covariance[0] < 0)
    return;  //检查 IMU 数据的有效性，若无效则返回

  // Convert quaternions in message packets to quaternions in tf
  tf::Quaternion quaternion(msg.orientation.x, msg.orientation.y,
                            msg.orientation.z, msg.orientation.w);

  // Calculate the posture at this time
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  pitch = pitch * 180 / M_PI;
  yaw = yaw * 180 / M_PI;
  // The angular velocities of pitch and yaw
  double pitchAngVelocity = pitch - lastPitch;
  double yawAngVelocity = yaw - lastYaw;

  // Load request message
  balancing_car_control::control_param cp;
  cp.request.dst_vel = dst_vel;
  cp.request.dst_yaw = dst_yaw;
  cp.request.leftWheelVelocity = leftWheelVelocity;
  cp.request.rightWheelVelocity = rightWheelVelocity;
  cp.request.pitch = pitch;
  cp.request.yaw = yaw;
  cp.request.yawAngVelocity = yawAngVelocity;
  cp.request.pitchAngVelocity = pitchAngVelocity;
  // Send request message
  bool flag = client.call(cp);

  // Receive the calculated torque value and publish it
  if (flag) {
    std_msgs::Float64MultiArray commandMsg;
    commandMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    commandMsg.layout.dim[0].label = "";
    commandMsg.layout.dim[0].size = 2;
    commandMsg.layout.dim[0].stride = 1;
    commandMsg.layout.data_offset = 0;
    commandMsg.data.push_back(
        cp.response.leftTorque);  // Left wheel, driving the car forward when
                                  // the value is positive
    commandMsg.data.push_back(
        cp.response.rightTorque);  // Right wheel, driving the car forward when
                                   // the value is positive

    wheelsCommandPub.publish(commandMsg);
  }

  lastPitch = pitch;
  lastYaw = yaw;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor");
  ros::NodeHandle nh;

  client =
      nh.serviceClient<balancing_car_control::control_param>("control_param");
  ros::service::waitForService("control_param");

  // Control the wheel output torque
  wheelsCommandPub = nh.advertise<std_msgs::Float64MultiArray>(
      "/wheels_controller/command", 1);

  ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, IMUCallback);

  // Subscription of speed information for two wheels for feedback control
  ros::Subscriber wheelVelocity_sub = nh.subscribe<sensor_msgs::JointState>(
      "/joint_states", 10, WheelVelocityCallback);

  ros::Subscriber keyboard_cmd_sub =
      nh.subscribe<geometry_msgs::Twist>("/my_cmd_vel", 10, KeyboardCmd);

  ros::spin();
  return 0;
}
