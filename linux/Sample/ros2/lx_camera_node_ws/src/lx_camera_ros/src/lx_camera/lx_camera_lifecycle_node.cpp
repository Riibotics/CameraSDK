#include "lx_camera/lx_camera.h"

int main(int argc, char **argv) {
  DcLib lib;
  if (!DynamicLink(&lib)) {
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<LxCamera>(&lib);
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  executor.remove_node(node->get_node_base_interface());
  node.reset();
  rclcpp::shutdown();
  DisDynamicLink(&lib);
  return 0;
}
