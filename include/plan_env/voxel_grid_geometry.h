#ifndef VOXEL_GRID_GEOMETRY_H_
#define VOXEL_GRID_GEOMETRY_H_

#include <Eigen/Eigen>

/**
 * @brief 不依赖 ROS 的体素地图几何信息和坐标转换工具。
 */
class VoxelGridGeometry {
public:
  VoxelGridGeometry() = default;
  ~VoxelGridGeometry() = default;

  /**
   * @brief 按物理尺寸初始化地图几何参数；通常只在建图启动时调用一次。
   * @param map_min_boundary 地图最小边界角点，世界坐标，对应体素索引 (0, 0, 0) 的最小角点。
   * @param map_size 地图在 x/y/z 三个方向上的物理尺寸，单位为米。
   * @param resolution 单个体素边长，单位为米。
   */
  void initializeFromMapSize(const Eigen::Vector3d& map_min_boundary,
                             const Eigen::Vector3d& map_size,
                             double resolution);

  /**
   * @brief 按体素数量初始化地图几何参数，避免由物理尺寸反推体素数量时产生浮点误差。
   * @param map_min_boundary 地图最小边界角点，世界坐标，对应体素索引 (0, 0, 0) 的最小角点。
   * @param voxel_counts 地图在 x/y/z 三个方向上的体素数量，每个方向必须大于 0。
   * @param resolution 单个体素边长，单位为米。
   */
  void initializeFromVoxelCounts(const Eigen::Vector3d& map_min_boundary,
                                 const Eigen::Vector3i& voxel_counts,
                                 double resolution);

  /**
   * @brief 按新的地图最小角点重设地图边界，最大边界由 map_min_boundary + map_size_ 得到。
   * @param map_min_boundary 输入新的地图最小边界角点，世界坐标。
   */
  void setMapBoundariesFromMinBoundary(const Eigen::Vector3d& map_min_boundary);

  /**
   * @brief 将世界坐标转换为体素索引，不检查输入坐标是否在地图内。
   * @param position 输入世界坐标，单位为米。
   * @param index 输出体素索引，可能位于地图有效索引范围外。
   */
  void positionToIndexUnchecked(const Eigen::Vector3d& position, Eigen::Vector3i& index) const;

  /**
   * @brief 将体素索引转换为体素中心的世界坐标，不检查输入索引是否在地图内。
   * @param index 输入体素索引，可以位于地图有效索引范围外。
   * @param position 输出体素中心世界坐标，单位为米。
   */
  void indexToPositionUnchecked(const Eigen::Vector3i& index, Eigen::Vector3d& position) const;

  /**
   * @brief 将三维体素索引转换为一维 buffer 地址，不检查输入索引是否在地图内。
   * @param index 输入体素索引，调用方必须保证其位于地图有效索引范围内。
   * @return 一维 buffer 地址。
   */
  int addressUnchecked(const Eigen::Vector3i& index) const;

  /**
   * @brief 将三维体素索引分量转换为一维 buffer 地址，不检查输入索引是否在地图内。
   * @param x 输入 x 方向体素索引，调用方必须保证其位于有效范围内。
   * @param y 输入 y 方向体素索引，调用方必须保证其位于有效范围内。
   * @param z 输入 z 方向体素索引，调用方必须保证其位于有效范围内。
   * @return 一维 buffer 地址。
   */
  int addressUnchecked(int x, int y, int z) const;

  /**
   * @brief 当世界坐标位于地图内时转换为体素索引。
   * @param position 输入世界坐标，单位为米。
   * @param index 输出体素索引，仅在返回 true 时有效。
   * @return position 位于地图有效范围内时返回 true，否则返回 false。
   */
  bool positionToIndexIfInMap(const Eigen::Vector3d& position, Eigen::Vector3i& index) const;

  /**
   * @brief 当体素索引位于地图内时转换为体素中心的世界坐标。
   * @param index 输入体素索引。
   * @param position 输出体素中心世界坐标，仅在返回 true 时有效。
   * @return index 位于地图有效索引范围内时返回 true，否则返回 false。
   */
  bool indexToPositionIfInMap(const Eigen::Vector3i& index, Eigen::Vector3d& position) const;

  /**
   * @brief 当体素索引位于地图内时转换为一维 buffer 地址。
   * @param index 输入体素索引。
   * @param buffer_address 输出一维 buffer 地址，仅在返回 true 时有效。
   * @return index 位于地图有效索引范围内时返回 true，否则返回 false。
   */
  bool addressIfInMap(const Eigen::Vector3i& index, int& buffer_address) const;

  /**
   * @brief 将体素索引裁剪到地图有效索引范围内。
   * @param index 输入体素索引。
   * @return 裁剪后的体素索引。
   */
  Eigen::Vector3i clampedIndex(const Eigen::Vector3i& index) const;

  /**
   * @brief 判断世界坐标是否位于地图有效范围内。
   * @param position 输入世界坐标，单位为米。
   * @return 在地图内返回 true，否则返回 false。
   */
  bool isInMap(const Eigen::Vector3d& position) const;

  /**
   * @brief 判断体素索引是否位于地图有效索引范围内。
   * @param index 输入体素索引。
   * @return 在地图内返回 true，否则返回 false。
   */
  bool isInMap(const Eigen::Vector3i& index) const;

  /**
   * @brief 获取地图物理尺寸。
   * @return x/y/z 三个方向的地图尺寸，单位为米。
   */
  const Eigen::Vector3d& getMapSize() const;

  /**
   * @brief 获取地图最小边界。
   * @return 最小边界世界坐标。
   */
  const Eigen::Vector3d& getMinBoundary() const;

  /**
   * @brief 获取地图最大边界。
   * @return 最大边界世界坐标。
   */
  const Eigen::Vector3d& getMaxBoundary() const;

  /**
   * @brief 获取地图体素数量。
   * @return x/y/z 三个方向的体素数量。
   */
  const Eigen::Vector3i& getVoxelNum() const;

  /**
   * @brief 获取体素分辨率。
   * @return 单个体素边长，单位为米。
   */
  double getResolution() const;

  /**
   * @brief 获取体素分辨率倒数。
   * @return 1 / resolution。
   */
  double getResolutionInv() const;

  /**
   * @brief 获取地图总的体素数量。
   * @return 一维 buffer 所需长度。
   */
  int getVoxelCount() const;

private:
  // -----------------主动维护的成员变量-----------------
  /// 地图在 x/y/z 三个方向上的物理尺寸，单位为米。
  Eigen::Vector3d map_size_{Eigen::Vector3d::Zero()};
  /// 单个体素的边长，单位为米。
  double resolution_{0.0};

  /// 地图有效范围的最小世界坐标，对应体素索引 (0, 0, 0) 的最小角点，单位为米。
  Eigen::Vector3d min_boundary_{Eigen::Vector3d::Zero()};
  /// 地图有效范围的最大世界坐标，等于 min_boundary_ + map_size_。
  Eigen::Vector3d max_boundary_{Eigen::Vector3d::Zero()};
  /// 地图在 x/y/z 三个方向上的体素数量，由初始化接口写入或由 map_size_ 和 resolution_ 计算得到。
  Eigen::Vector3i voxel_num_{Eigen::Vector3i::Zero()};
  /// 分辨率倒数，用于把世界坐标快速换算为体素索引。
  double resolution_inv_{0.0};
};

#endif  // VOXEL_GRID_GEOMETRY_H_
