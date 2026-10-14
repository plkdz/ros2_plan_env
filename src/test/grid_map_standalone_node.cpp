#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

// 独立 GridMap 节点仅作为 plan_env 的测试、调试和可视化入口。
// 当前 planner 高频碰撞查询仍应优先使用同进程 GridMap 接口，避免跨进程复制整块局部栅格。
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("grid_map_standalone_node");

  GridMap grid_map;
  grid_map.initMap(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
