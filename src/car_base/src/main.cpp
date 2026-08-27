#include "car_base/base_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<car_base::CarBaseNode>());
  rclcpp::shutdown();
  return 0;
}
