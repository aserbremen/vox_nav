#include <vox_nav_map_server/interactive_map_manager.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vox_nav_map_server::InteractiveMapManager>());
  rclcpp::shutdown();
  return 0;
}