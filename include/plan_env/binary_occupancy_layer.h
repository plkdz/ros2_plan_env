#ifndef BINARY_OCCUPANCY_LAYER_H_
#define BINARY_OCCUPANCY_LAYER_H_

#include <Eigen/Eigen>
#include <plan_env/voxel_grid_geometry.h>
#include <vector>

/**
 * @brief 原始未膨胀的二值占据层，主要保存激光雷达等确定性障碍观测。
 */
class BinaryOccupancyLayer {
public:
  BinaryOccupancyLayer() = default;
  ~BinaryOccupancyLayer() = default;

  /**
   * @brief 绑定体素几何并初始化原始二值占据数据。
   * @param grid_geometry 共享体素几何对象，调用方必须保证其生命周期长于本层。
   */
  void initialize(const VoxelGridGeometry* grid_geometry);

  /**
   * @brief 清空指定世界坐标区域内的原始占据标记。
   * @param min_position 输入区域最小角点，世界坐标。
   * @param max_position 输入区域最大角点，世界坐标。
   * @param padding 输入区域外扩距离，单位为米。
   */
  void clearRegion(const Eigen::Vector3d& min_position,
                   const Eigen::Vector3d& max_position,
                   double padding);

  /**
   * @brief 将世界坐标对应体素标记为原始占据。
   * @param position 输入世界坐标，单位为米。
   */
  void setRawOccupied(const Eigen::Vector3d& position);

  /**
   * @brief 设置一维地址对应体素的原始二值占据值。
   * @param buffer_index 输入一维 buffer 地址。
   * @param occupancy_value 输入占据值，0 表示空闲，1 表示占据。
   */
  void setRawOccupancy(int buffer_index, int occupancy_value);

  /**
   * @brief 查询一维地址对应体素的原始二值占据值。
   * @param buffer_index 输入一维 buffer 地址。
   * @return 0 表示空闲，1 表示占据，-1 表示越界。
   */
  int getRawOccupancy(int buffer_index) const;

  /**
   * @brief 判断体素索引是否为原始占据。
   * @param index 输入体素索引。
   * @return 占据返回 true，否则返回 false。
   */
  bool isRawOccupied(const Eigen::Vector3i& index) const;

  /**
   * @brief 查询世界坐标对应体素的原始二值占据状态。
   * @param position 输入世界坐标，单位为米。
   * @return 占据返回 1，空闲返回 0，越界返回 -1。
   */
  int getRawOccupancy(const Eigen::Vector3d& position) const;

private:
  const VoxelGridGeometry* grid_geometry_{nullptr};

  /// 原始未膨胀的二值障碍数据，0 表示空闲，1 表示原始障碍观测。
  std::vector<char> raw_binary_obstacles_;
};

#endif  // BINARY_OCCUPANCY_LAYER_H_
