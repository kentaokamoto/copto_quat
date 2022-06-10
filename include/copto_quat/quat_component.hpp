#ifndef COPTO_QUAT__QUAT_COMPONENT_HPP_
#define COPTO_QUAT__QUAT_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COPTO_QUAT_QUAT_COMPONENT_EXPORT __attribute__((dllexport))
#define COPTO_QUAT_QUAT_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COPTO_QUAT_QUAT_COMPONENT_EXPORT __declspec(dllexport)
#define COPTO_QUAT_QUAT_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COPTO_QUAT_QUAT_COMPONENT_BUILDING_DLL
#define COPTO_QUAT_QUAT_COMPONENT_PUBLIC COPTO_QUAT__QUAT_COMPONENT_EXPORT
#else
#define COPTO_QUAT_QUAT_COMPONENT_PUBLIC COPTO_QUAT__QUAT_COMPONENT_IMPORT
#endif
#define COPTO_QUAT__QUAT_COMPONENT_PUBLIC_TYPE COPTO_QUAT__QUAT_COMPONENT_PUBLIC
#define COPTO_QUAT_QUAT_COMPONENT_LOCAL
#else
#define COPTO_QUAT_QUAT_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COPTO_QUAT_QUAT_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COPTO_QUAT_QUAT_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COPTO_QUAT_QUAT_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COPTO_QUAT_QUAT_COMPONENT_PUBLIC
#define COPTO_QUAT_QUAT_COMPONENT_LOCAL
#endif
#define COPTO_QUAT_QUAT_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sensor_msgs/msg/imu.hpp"

namespace copto_quat
{
class QUATComponent : public rclcpp::Node
{
public:
  COPTO_QUAT_QUAT_COMPONENT_PUBLIC
  explicit QUATComponent(const rclcpp::NodeOptions & options);

  bool initialized = false;
  Eigen::MatrixXd P;
  Eigen::VectorXd x;
  Eigen::VectorXd z;
  Eigen::VectorXd u;
  Eigen::VectorXd a;
  Eigen::VectorXd am;
  Eigen::VectorXd beta;
  
  Eigen::MatrixXd I;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  Eigen::MatrixXd E;
  Eigen::MatrixXd G;

  rclcpp::Time imutimestamp;

private:
  float k = 0.6;
  float eps = 0.1;
  float dt = 0.01;
  float g = 9.81;
  void IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void state_eq();
  void observation_eq();
  void prefilter();
  void LPF();
  void jacobi();
  bool init();
  void update();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace copto_quat

#endif  // COPTO_QUAT__QUAT_COMPONENT_HPP_