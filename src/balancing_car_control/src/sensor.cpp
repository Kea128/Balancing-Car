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

  // 将消息包中的四元数转换为 tf 中的四元数
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
    // 创建一个std_msgs::Float64MultiArray类型的消息，用于发布两个轮子的扭矩命令
    std_msgs::Float64MultiArray commandMsg;
    // 为消息添加一个维度，这里只添加一个维度，用于存储两个扭矩值
    commandMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // 这个维度的标签设置为空字符串，通常用于在消息中标识维度的名称
    commandMsg.layout.dim[0].label = "";
    // 设置维度的大小为2，因为我们有两个扭矩值要存储
    commandMsg.layout.dim[0].size = 2;
    // 步长设置为1，表示每个扭矩值之间占用一个数据项的空间
    commandMsg.layout.dim[0].stride = 1;
    // 数据偏移量设置为0，表示数据从数组的第一个位置开始存储
    commandMsg.layout.data_offset = 0;
    // 添加左轮的扭矩值到消息的数据数组中
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

  // 创建控制消息客户端对象
  client =
      nh.serviceClient<balancing_car_control::control_param>("control_param");
  ros::service::waitForService("control_param");

  // Control the wheel output torque
  // 创建关节力矩发布者对象
  wheelsCommandPub = nh.advertise<std_msgs::Float64MultiArray>(
      "/wheels_controller/command", 1);

  // 创建关IMU订阅者对象
  ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, IMUCallback);

  // Subscription of speed information for two wheels for feedback control
  // 创建关节状态订阅者对象
  ros::Subscriber wheelVelocity_sub = nh.subscribe<sensor_msgs::JointState>(
      "/joint_states", 10, WheelVelocityCallback);

  // 创建键盘控制订阅者对象
  ros::Subscriber keyboard_cmd_sub =
      nh.subscribe<geometry_msgs::Twist>("/my_cmd_vel", 10, KeyboardCmd);

  ros::spin();
  return 0;
}
