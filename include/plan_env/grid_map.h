#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>

#include <plan_env/occupancy_grid.h>

struct MapGeometryParameters
{
  Eigen::Vector3d local_update_half_range_meters_; // 局部更新范围的半边长，单位为米。
  double rolling_map_min_z_meters_; // 滚动体素地图 z 方向最小边界，单位为米。
  std::string frame_id_; // 地图点云发布和输入点云期望使用的坐标系名称。
};

struct InflationParameters
{
  double xy_obstacle_inflation_meters_; // 水平 x/y 方向障碍膨胀尺寸，单位为米。
  double z_obstacle_inflation_meters_; // 垂直 z 方向障碍膨胀尺寸，单位为米。
};

struct OccupancyMapPublishingParameters
{
  double max_published_z_meters_; // 发布膨胀占据点云时允许输出的最大 z 坐标，单位为米。
};

struct MappingParameters
{
  MapGeometryParameters map_; // 地图几何和局部更新半范围配置。
  InflationParameters inflation_; // 障碍物膨胀配置。
  OccupancyMapPublishingParameters publishing_; // 膨胀占据点云发布配置。
};

struct SensorPoseState
{
  Eigen::Vector3d sensor_position_; // 当前点云传感器在地图坐标系下的位置，单位为米。
};

struct MappingStatus
{
  bool has_updated_local_bounds_; // 是否已经有可用于发布的本轮局部地图边界。
  bool has_odometry_; // 是否已经收到至少一帧里程计。
  bool has_point_cloud_; // 是否已经收到至少一帧点云。
};

struct LocalUpdateBounds
{
  Eigen::Vector3i min_voxel_index_; // 本轮更新区域的最小体素索引。
  Eigen::Vector3i max_voxel_index_; // 本轮更新区域的最大体素索引。
};

struct PointCloudRuntimeState
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> recent_point_clouds_; // 最近若干帧点云缓存。
};

struct MappingData
{
  SensorPoseState sensor_pose_; // 当前传感器位姿相关运行时状态。
  MappingStatus mapping_status_; // 建图输入和局部边界更新状态。
  LocalUpdateBounds local_bounds_; // 最近一次有效更新覆盖的局部体素边界。
  PointCloudRuntimeState point_cloud_state_; // 点云缓存和合并相关运行时状态。
};

class GridMap
{
public:
  /**
   * @brief 构造空 GridMap 对象。
   * @note 不初始化 ROS 接口、地图几何或占据层；必须先调用 initMap()。
   */
  GridMap() = default;

  /**
   * @brief 析构 GridMap 对象。
   * @note ROS 订阅器、发布器、定时器和智能指针成员按默认析构流程释放。
   */
  ~GridMap() = default;

  /**
   * @brief 从 ROS 参数读取点云建图参数并初始化地图、二值占据层、订阅器、发布器和定时器。
   * @param node 输入 ROS2 节点；读取 grid_map 命名空间参数，并创建点云、里程计、可视化 topic 接口。
   * @note 写入 mapping_parameters_、runtime_state_、voxel_grid_geometry_、raw_binary_occupancy_layer_
   * 和 binary_inflation_layer_。
   */
  void initMap(const rclcpp::Node::SharedPtr &node);

  /**
   * @brief 查询世界坐标对应体素的膨胀占据状态。
   * @param position 输入查询点的世界坐标，单位为米。
   * @return 1 表示膨胀占据，0 表示空闲，-1 表示越界。
   * @note 读取 binary_inflation_layer_ 和 voxel_grid_geometry_ 中的地图状态。
   */
  int getInflatedOccupancy(const Eigen::Vector3d &position) const;

  /**
   * @brief 查询世界坐标所在体素的中心世界坐标。
   * @param position 输入查询点的世界坐标，单位为米。
   * @param voxel_center 输出该查询点所在体素的中心世界坐标，仅在返回 true 时有效。
   * @return position 位于当前滚动体素地图内时返回 true，否则返回 false。
   * @note 读取 voxel_grid_geometry_ 中的坐标和索引转换状态。
   */
  bool positionToVoxelCenterIfInMap(const Eigen::Vector3d &position,
                                    Eigen::Vector3d &voxel_center) const;

  /**
   * @brief 获取体素地图分辨率。
   * @return 单个体素边长，单位为米。
   * @note 读取 voxel_grid_geometry_ 中的 resolution。
   */
  double getResolution() const;

  /**
   * @brief 获取滚动体素地图在 x/y/z 三个方向上的体素数量。
   * @return 地图体素数量，只读引用。
   * @note 读取 voxel_grid_geometry_ 中的 voxel_num。
   */
  const Eigen::Vector3i& getVoxelNum() const;

  /**
   * @brief 获取滚动体素地图在 x/y/z 三个方向上的物理尺寸。
   * @return 地图物理尺寸，只读引用，单位为米。
   * @note 读取 voxel_grid_geometry_ 中的 map_size。
   */
  const Eigen::Vector3d& getRollingMapSize() const;

  /**
   * @brief 获取当前滚动体素地图的最小世界坐标边界。
   * @return 当前地图最小边界，只读引用，单位为米。
   * @note 读取 voxel_grid_geometry_ 中的 min_boundary。
   */
  const Eigen::Vector3d& getRollingMapMinBoundary() const;

  /**
   * @brief 获取当前滚动体素地图的最大世界坐标边界。
   * @return 当前地图最大边界，只读引用，单位为米。
   * @note 读取 voxel_grid_geometry_ 中的 max_boundary。
   */
  const Eigen::Vector3d& getRollingMapMaxBoundary() const;

  /**
   * @brief 判断当前局部膨胀占据地图是否已经完成过一次有效更新。
   * @return 已收到里程计、点云，并完成过局部地图边界更新时返回 true。
   * @note 读取 runtime_state_.mapping_status_。
   */
  bool hasUpdatedLocalMap() const;

  /**
   * @brief 判断世界坐标点是否位于当前局部更新半范围方框内。
   * @param point 输入世界坐标点，单位为米。
   * @return 已收到里程计且点相对当前传感器位置位于 local_update_half_range 内时返回 true。
   * @note 读取 runtime_state_.sensor_pose_ 和 mapping_parameters_.map_。
   */
  bool isPointWithinCurrentLocalUpdateRange(const Eigen::Vector3d &point) const;

  typedef std::shared_ptr<GridMap> Ptr;

  // 保证包含固定大小 Eigen 成员的 GridMap 在堆上分配时满足 Eigen 内存对齐要求。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /**
   * @brief 根据当前传感器位置重设滚动地图边界。
   * @note 输入来自 runtime_state_.sensor_pose_.sensor_position_ 和 mapping_parameters_.map_。
   * @note 修改 voxel_grid_geometry_ 的 min_boundary 和 max_boundary。
   */
  void updateRollingMapBoundaries();

  /**
   * @brief 清空指定世界坐标区域内的二值占据缓存。
   * @param min_position 输入清空区域的世界坐标最小角点，单位为米。
   * @param max_position 输入清空区域的世界坐标最大角点，单位为米。
   * @note 读取 voxel_grid_geometry_ 的当前地图几何。
   * @note 修改 raw_binary_occupancy_layer_ 和 binary_inflation_layer_ 对应区域。
   */
  void clearBinaryOccupancyRegion(const Eigen::Vector3d &min_position,
                                  const Eigen::Vector3d &max_position);

  /**
   * @brief 接收里程计输入并更新当前传感器位置状态。
   * @param odometry 输入里程计消息，pose.position 被用作当前传感器位置。
   * @note 修改 runtime_state_.sensor_pose_.sensor_position_ 和 runtime_state_.mapping_status_.has_odometry_。
   */
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odometry);

  /**
   * @brief 将当前点云加入最近帧缓存，并输出最近帧合并点云。
   * @param current_point_cloud_message 输入当前点云消息。
   * @param merged_point_cloud 输出最近帧缓存合并后的点云。
   * @note 修改 runtime_state_.point_cloud_state_.recent_point_clouds_。
  */
  void mergeRecentPointClouds(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &current_point_cloud_message,
                              pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud);

  /**
   * @brief 判断点云路线当前是否具备更新二值占据地图的条件。
   * @param merged_point_cloud 输入最近帧缓存合并后的点云。
   * @return 可以更新返回 true，否则返回 false。
   * @note 读取 runtime_state_.mapping_status_ 和 runtime_state_.sensor_pose_。
   */
  bool isPointCloudUpdateReady(const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud) const;

  /**
   * @brief 根据当前位姿准备点云路线的滚动地图边界和局部二值清空区域。
   * @note 读取 runtime_state_.sensor_pose_ 和 mapping_parameters_.map_。
   * @note 修改 voxel_grid_geometry_、raw_binary_occupancy_layer_ 和 binary_inflation_layer_。
   */
  void preparePointCloudLocalMap();

  /**
   * @brief 判断点云点坐标是否为有限数值。
   * @param point 输入 PCL 点。
   * @return xyz 坐标均为有限数值返回 true，否则返回 false。
   */
  bool isFinitePointCloudPoint(const pcl::PointXYZ &point) const;

  /**
   * @brief 判断点云点相对传感器的位置是否位于局部更新半范围内。范围外的点云哪怕是累帧的历史点云也会被删除。
   * @param point_offset 输入点相对当前传感器位置的偏移量，单位为米。
   * @return 在局部更新范围内返回 true，否则返回 false。
   * @note 读取 mapping_parameters_.map_.local_update_half_range_meters_。
   */
  bool isPointWithinLocalUpdateHalfRange(const Eigen::Vector3d &point_offset) const;

  /**
   * @brief 从合并点云中收集去重后的原始占据体素，并写入原始二值占据层。
   * @param merged_point_cloud 输入最近帧缓存合并后的点云。
   * @param raw_occupied_voxel_indices 输出去重后的原始占据体素索引集合。
   * @note 读取 runtime_state_.sensor_pose_、mapping_parameters_.map_ 和 voxel_grid_geometry_。
   * @note 修改 raw_binary_occupancy_layer_。
   */
  void collectUniqueRawOccupiedVoxelsFromPointCloud(
      const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud,
      std::vector<Eigen::Vector3i> &raw_occupied_voxel_indices);

  /**
   * @brief 对去重后的原始占据体素进行膨胀，并写入膨胀二值占据层。
   * @param raw_occupied_voxel_indices 输入去重后的原始占据体素索引集合。
   * @param min_position 输出本轮膨胀体素覆盖区域的最小世界坐标。
   * @param max_position 输出本轮膨胀体素覆盖区域的最大世界坐标。
   * @note 读取 mapping_parameters_.inflation_、mapping_parameters_.map_ 和 voxel_grid_geometry_。
   * @note 修改 binary_inflation_layer_。
   */
  void inflateRawOccupiedVoxels(const std::vector<Eigen::Vector3i> &raw_occupied_voxel_indices,
                                Eigen::Vector3d &min_position,
                                Eigen::Vector3d &max_position);

  /**
   * @brief 使用合并后的点云更新原始二值占据层和膨胀二值占据层。
   * @param merged_point_cloud 输入最近帧缓存合并后的点云。
   * @param min_position 输出本轮膨胀点云覆盖区域的最小世界坐标。
   * @param max_position 输出本轮膨胀点云覆盖区域的最大世界坐标。
   * @note 读取 runtime_state_.sensor_pose_、mapping_parameters_ 和 voxel_grid_geometry_。
   * @note 修改 raw_binary_occupancy_layer_ 和 binary_inflation_layer_。
   */
  void updateBinaryOccupancyFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud,
                                           Eigen::Vector3d &min_position,
                                           Eigen::Vector3d &max_position);

  /**
   * @brief 根据本轮点云覆盖区域更新局部体素边界。
   * @param min_position 输入本轮覆盖区域的最小世界坐标。
   * @param max_position 输入本轮覆盖区域的最大世界坐标。
   * @note 读取 runtime_state_.sensor_pose_、mapping_parameters_.map_ 和 voxel_grid_geometry_。
   * @note 修改 runtime_state_.local_bounds_。
   */
  void updatePointCloudLocalBounds(const Eigen::Vector3d &min_position,
                                   const Eigen::Vector3d &max_position);

  /**
   * @brief 在当前局部边界范围内向膨胀层添加虚拟天花板。
   * @note 读取 runtime_state_.local_bounds_ 和 voxel_grid_geometry_。
   * @note 修改 binary_inflation_layer_。
   */
  void addVirtualCeilingToInflatedOccupancy();

  /**
   * @brief 接收点云输入并直接更新原始二值占据层和膨胀占据层。
   * @param current_point_cloud_message 输入点云消息，点坐标应与地图 frame 一致。
   * @note 读取 mapping_parameters_.map_、mapping_parameters_.inflation_、runtime_state_.sensor_pose_、
   * runtime_state_.mapping_status_ 和 voxel_grid_geometry_。
   * @note 修改 runtime_state_.mapping_status_、runtime_state_.local_bounds_、
   * raw_binary_occupancy_layer_、binary_inflation_layer_。
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &current_point_cloud_message);

  /**
   * @brief 发布膨胀后的二值占据地图点云。
   * @note 输入来自 runtime_state_.local_bounds_、mapping_parameters_.map_、mapping_parameters_.publishing_、
   * voxel_grid_geometry_ 和 binary_inflation_layer_。
   * @note 输出到 inflated_occupancy_map_publisher_ 对应 ROS topic；无订阅者时直接返回。
   */
  void publishInflatedOccupancyMap();

  /**
   * @brief 发布滚动地图边界和局部更新范围边界。
   * @note 输入来自 runtime_state_.sensor_pose_、mapping_parameters_.map_ 和 voxel_grid_geometry_。
   * @note 输出到 map_boundary_marker_publisher_。
   */
  void publishMapBoundaryMarkers();

  /**
   * @brief 定时发布膨胀占据地图和地图边界可视化。
   * @note 读取 runtime_state_、mapping_parameters_、voxel_grid_geometry_ 和 binary_inflation_layer_。
   * @note 输出到 inflated_occupancy_map_publisher_ 和 map_boundary_marker_publisher_。
   */
  void visualizationCallback();

  MappingParameters mapping_parameters_; // GridMap 初始化后固定使用的建图参数集合。
  MappingData runtime_state_; // GridMap 回调运行期间持续更新的建图状态集合。
  VoxelGridGeometry voxel_grid_geometry_; // 滚动体素地图的几何、边界、索引和地址转换对象。
  BinaryOccupancyLayer raw_binary_occupancy_layer_; // 点云直接写入的原始未膨胀二值占据层。
  BinaryInflationLayer binary_inflation_layer_; // 由原始占据层膨胀得到、供规划查询的二值占据层。

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr independent_cloud_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr independent_odometry_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_occupancy_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_boundary_marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

#endif // GRID_MAP_H_
