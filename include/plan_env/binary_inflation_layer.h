#ifndef BINARY_INFLATION_LAYER_H_
#define BINARY_INFLATION_LAYER_H_

#include <Eigen/Eigen>
#include <plan_env/voxel_grid_geometry.h>
#include <vector>

/**
 * @brief 二值膨胀占据层，作为 planner 查询的最终障碍层。
 */
class BinaryInflationLayer {
public:
  BinaryInflationLayer() = default;
  ~BinaryInflationLayer() = default;

  /**
   * @brief 绑定体素几何并初始化二值膨胀 buffer。
   * @param grid_geometry 共享体素几何对象，调用方必须保证其生命周期长于本层。
   */
  void initialize(const VoxelGridGeometry* grid_geometry);

  /**
   * @brief 清空指定世界坐标区域内的膨胀占据标记。
   * @param min_position 输入区域最小角点，世界坐标。
   * @param max_position 输入区域最大角点，世界坐标。
   * @param padding 输入区域外扩距离，单位为米。
   */
  void clearRegion(const Eigen::Vector3d& min_position,
                   const Eigen::Vector3d& max_position,
                   double padding);

  /**
   * @brief 将世界坐标对应体素标记为膨胀占据。
   * @param position 输入世界坐标，单位为米。
   */
  void setInflatedOccupied(const Eigen::Vector3d& position);

  /**
   * @brief 设置一维地址对应体素的二值膨胀占据值。
   * @param buffer_index 输入一维 buffer 地址。
   * @param occupancy_value 输入占据值，0 表示空闲，1 表示占据。
   */
  void setInflatedOccupancy(int buffer_index, int occupancy_value);

  /**
   * @brief 查询一维地址对应体素的二值膨胀占据值。
   * @param buffer_index 输入一维 buffer 地址。
   * @return 0 表示空闲，1 表示占据，-1 表示越界。
   */
  int getInflatedOccupancy(int buffer_index) const;

  /**
   * @brief 判断体素索引是否为膨胀占据。
   * @param index 输入体素索引。
   * @return 占据返回 true，否则返回 false。
   */
  bool isInflatedOccupied(const Eigen::Vector3i& index) const;

  /**
   * @brief 查询世界坐标对应体素的二值膨胀占据状态。
   * @param position 输入世界坐标，单位为米。
   * @return 占据返回 1，空闲返回 0，越界返回 -1。
   */
  int getInflatedOccupancy(const Eigen::Vector3d& position) const;

  /**
   * @brief 生成指定体素周围的立方体膨胀索引集合。
   * @param center_index 输入中心体素索引。
   * @param inflation_step 输入每个方向膨胀的体素步数。
   * @param inflated_indices 输出膨胀后的体素索引集合，调用方负责预分配足够空间。
   */
  void inflatePoint(const Eigen::Vector3i& center_index,
                    int inflation_step,
                    std::vector<Eigen::Vector3i>& inflated_indices) const;

  /**
   * @brief 生成指定体素周围 xy/z 分离步数的长方体膨胀索引集合。
   * @param center_index 输入中心体素索引。
   * @param xy_inflation_step 输入 x/y 方向膨胀的体素步数。
   * @param z_inflation_step 输入 z 方向膨胀的体素步数。
   * @param inflated_indices 输出膨胀后的体素索引集合，调用方负责预分配足够空间。
   */
  void inflatePoint(const Eigen::Vector3i& center_index,
                    int xy_inflation_step,
                    int z_inflation_step,
                    std::vector<Eigen::Vector3i>& inflated_indices) const;

private:
  const VoxelGridGeometry* grid_geometry_{nullptr};

  /// 膨胀后的二值障碍数据，0 表示空闲，1 表示障碍或膨胀障碍。
  std::vector<char> binary_inflated_obstacles_;
};

#endif  // BINARY_INFLATION_LAYER_H_
