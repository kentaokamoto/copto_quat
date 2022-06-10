#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <copto_quat/quat_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace  std::chrono_literals;
namespace copto_quat
{
QUATComponent::QUATComponent(const rclcpp::NodeOptions & options) : Node("copto_quat_node", options)
{
  
  A = Eigen::MatrixXd::Zero(7, 7);
  B = Eigen::MatrixXd::Zero(7, 6);

  C = Eigen::MatrixXd::Zero(3, 7);
  R = Eigen::MatrixXd::Zero(3, 3);
  Q = Eigen::MatrixXd::Zero(6, 6);
  E = Eigen::MatrixXd::Zero(3, 3);

  K = Eigen::MatrixXd::Zero(7, 3);
  S = Eigen::MatrixXd::Zero(3, 3);
  P = Eigen::MatrixXd::Zero(7, 7);
  I = Eigen::MatrixXd::Identity(7, 7);

  a = Eigen::VectorXd::Zero(3);
  am = Eigen::VectorXd::Zero(3);
  z = Eigen::VectorXd::Zero(3);
  G = Eigen::VectorXd::Zero(3);
  beta = Eigen::VectorXd::Zero(3);
  x = Eigen::VectorXd::Zero(7);
  u = Eigen::VectorXd::Zero(6);

  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/copto/imu", 10, std::bind(&QUATComponent::IMUtopic_callback, this, std::placeholders::_1));

  Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/copto/pose", 1);

  timer_ = this->create_wall_timer(10ms, std::bind(&QUATComponent::update, this));
}

void QUATComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imutimestamp = msg->header.stamp;
  u(0) = msg->linear_acceleration.x;
  u(1) = msg->linear_acceleration.y;
  u(2) = msg->linear_acceleration.z;
  u(3) = msg->angular_velocity.x;
  u(4) = msg->angular_velocity.y;
  u(5) = msg->angular_velocity.z;

  am << u(0), u(1), u(2);
}

void QUATComponent::LPF()
{
  a = a + k * (am - a); 
}

void QUATComponent::prefilter()
{
  Eigen::VectorXd a_dr(3);
  a_dr = E.transpose() * a - G;
  a = a - E * a_dr;
  double norm_am = std::sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2));
    if(norm_am < g + eps)
    {
      a_dr << 0, 0, 0;
    }
  a = a + E * a_dr;
}

bool QUATComponent::init()
{
  x << 1, 0, 0, 0, 0.01, 0.01, 0.01;
  P << 10, 0, 0, 0, 0, 0, 0,
      0, 10, 0, 0, 0, 0, 0,
      0, 0, 10, 0, 0, 0, 0,
      0, 0, 0, 10, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 1;
  G << 0, 0, -g;
  beta << 0.1, 0.1, 0.1;
  return initialized = true;
}

void QUATComponent::state_eq()
{
  double q0, q1, q2, q3, dwx, dwy, dwz;
  q0 = x(0);
  q1 = x(1);
  q2 = x(2);
  q3 = x(3);
  dwx = x(4);
  dwy = x(5);
  dwz = x(6);

  x(0) = (-(u(3)-dwx) * q1 - (u(4)-dwy) * q2 - (u(5)-dwz) * q3) * 0.5 * dt + q0;
  x(1) = ((u(3)-dwx) * q0 + (u(5)-dwz) * q2 - (u(4)-dwy) * q3) * 0.5 * dt + q1;
  x(2) = ((u(4)-dwy) * q0 - (u(5)-dwz) * q1 + (u(3)-dwx) * q2) * 0.5 * dt + q2;
  x(3) = ((u(5)-dwz) * q0 + (u(4)-dwy) * q1 - (u(3)-dwx) * q2) * 0.5 * dt + q3;
  x(4) = - beta(0)*dwx*dt + dwx;
  x(5) = - beta(1)*dwy*dt + dwy;
  x(6) = - beta(2)*dwz*dt + dwz;
}

void QUATComponent::observation_eq()
{
  z = E*G;
}

void QUATComponent::jacobi()
{
  A << 1, -(u(3)-x(4))*0.5*dt, -(u(4)-x(5))*0.5*dt, -(u(5)-x(6))*0.5*dt, x(1)*0.5*dt, x(2)*0.5*dt, x(3)*0.5*dt,
      (u(3)-x(4))*0.5*dt, 1, (u(5)-x(6))*0.5*dt, -(u(4)-x(5))*0.5*dt, -x(0)*0.5*dt, x(3)*0.5*dt, -x(2)*0.5*dt,
      (u(4)-x(5))*0.5*dt, -(u(5)-x(6))*0.5*dt, 1, (u(3)-x(4))*0.5*dt, -x(3)*0.5*dt, -x(0)*0.5*dt, x(1)*0.5*dt,
      (u(5)-x(6))*0.5*dt, (u(4)-x(5))*0.5*dt, -(u(3)-x(4))*0.5*dt, 1, x(2)*0.5*dt, -x(1)*0.5*dt, -x(0)*0.5*dt,
      0, 0, 0, 0, 1-beta(0)*dt, 0, 0,
      0, 0, 0, 0, 0, 1-beta(1)*dt, 0,
      0, 0, 0, 0, 0, 0, 1-beta(2)*dt;

  B << -dt, -dt, -dt, 0, 0, 0, 
        dt, -dt, dt, 0, 0, 0,
        dt, dt, -dt, 0, 0, 0,
        -dt, dt, dt, 0, 0, 0,
        0, 0, 0, dt, 0, 0,
        0, 0, 0, 0, dt, 0,
        0, 0, 0, 0, 0, dt;

  C <<  2*-x(2)*(-g), 2*x(3)*(-g), 2*-x(0)*(-g), 2*x(1)*(-g), 0, 0, 0,
        2*x(1)*(-g), 2*x(0)*(-g), 2*-x(3)*(-g), 2*x(2)*(-g), 0, 0, 0,
        2*x(0)*(-g), 2*-x(1)*(-g), 2*-x(2)*(-g), 2*x(3)*(-g), 0, 0, 0;

  /*
  C <<  2*(-x(2)*g), 2*(x(3)*g), 2*(-x(0)*g), 2*(x(1)*g), 0, 0, 0,
        2*(x(1)*g), 2*(x(0)*g), 2*(-x(3)*g), 2*(x(2)*g), 0, 0, 0,
        2*(x(0)*g), 2*(-x(1)*g), 2*(-x(2)*g), 2*(x(3)*g), 0, 0, 0;
  */

  R << 100, 0, 0,
       0, 100, 0,
       0, 0, 100;

  Q << 10, 0, 0, 0, 0, 0,
       0, 10, 0, 0, 0, 0,
       0, 0, 10, 0, 0, 0,
       0, 0, 0, 10, 0, 0,
       0, 0, 0, 0, 10, 0,
       0, 0, 0, 0, 0, 10;

  // frame_base -> Inertial_base
  E << (x(0) * x(0) + x(1) * x(1) - x(2) * x(2) - x(3) * x(3)),
    2 * (x(1) * x(2) - x(0) * x(3)), 2 * (x(1) * x(3) + x(0) * x(2)), 
    2 * (x(1) * x(2) + x(0) * x(3)), (x(0) * x(0) - x(1) * x(1) + x(2) * x(2) - x(3) * x(3)),
    2 * (x(2) * x(3) - x(0) * x(1)), 2 * (x(1) * x(3) - x(0) * x(2)),
    2 * (x(2) * x(3) + x(0) * x(1)), (x(0) * x(0) - x(1) * x(1) - x(2) * x(2) + x(3) * x(3));
}

void QUATComponent::update()
{
    if(!initialized){init();}
    if (!initialized) {
        std::cout << "NOT Initialized" << std::endl;
    }

    // 予測ステップ
    LPF();
    prefilter();
    state_eq();
    jacobi();
    // filtering step 1
    P = A * P * A.transpose() + B * Q * B.transpose();
    S = C * P * C.transpose() + R;
    K = P * C.transpose() * S.inverse();

    observation_eq();
    x = x + K * (a - z);
    P = (I - K * C) * P;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = imutimestamp;
  
    pose_msg.pose.pose.orientation.w = x(0);
    pose_msg.pose.pose.orientation.x = x(1);
    pose_msg.pose.pose.orientation.y = x(2);
    pose_msg.pose.pose.orientation.z = x(3);

    Posepublisher_->publish(pose_msg);

}
}  // namespace copto_quat

RCLCPP_COMPONENTS_REGISTER_NODE(copto_quat::QUATComponent)