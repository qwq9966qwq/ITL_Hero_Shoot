#include "rclcpp/rclcpp.hpp"
#include "lob_shot_manager/lob_shot_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lob_shot_manager::LobShotManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
