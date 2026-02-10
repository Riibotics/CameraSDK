#include "lx_camera/lx_camera.h"
#include "lifecycle_msgs/msg/transition.hpp"

int main(int argc, char **argv) {
  DcLib lib;
  if (!DynamicLink(&lib)) {
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<LxCamera>(&lib);
  executor.add_node(node->get_node_base_interface());

  auto cb_ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                           cb_ret);
  if (cb_ret !=
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Lifecycle configure failed");
    executor.remove_node(node->get_node_base_interface());
    node.reset();
    rclcpp::shutdown();
    DisDynamicLink(&lib);
    return 1;
  }
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                           cb_ret);
  if (cb_ret !=
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Lifecycle activate failed");
    executor.remove_node(node->get_node_base_interface());
    node.reset();
    rclcpp::shutdown();
    DisDynamicLink(&lib);
    return 1;
  }

  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  node.reset();
  rclcpp::shutdown();
  DisDynamicLink(&lib);
  return 0;
}
