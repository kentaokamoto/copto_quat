#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <copto_quat/quat_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<copto_quat::QUATComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}