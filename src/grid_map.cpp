#include "plan_env/grid_map.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
using namespace std::chrono_literals;

template <typename ParameterType>
void getOrDeclareParameter(const rclcpp::Node::SharedPtr& node,
                           const std::string& name,
                           ParameterType& value,
                           const ParameterType& default_value)
{
  if (node->has_parameter(name))
  {
    node->get_parameter(name, value);
    return;
  }

  value = node->declare_parameter<ParameterType>(name, default_value);
}

geometry_msgs::msg::Point eigenVectorToPoint(const Eigen::Vector3d& vector)
{
  geometry_msgs::msg::Point point;
  point.x = vector(0);
  point.y = vector(1);
  point.z = vector(2);
  return point;
}

void appendBoundingBoxEdge(visualization_msgs::msg::Marker& marker,
                           const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end)
{
  marker.points.push_back(eigenVectorToPoint(start));
  marker.points.push_back(eigenVectorToPoint(end));
}

visualization_msgs::msg::Marker makeBoundingBoxMarker(const std::string& frame_id,
                                                      const rclcpp::Time& stamp,
                                                      int marker_id,
                                                      const std::string& marker_namespace,
                                                      const Eigen::Vector3d& min_position,
                                                      const Eigen::Vector3d& max_position,
                                                      double line_width,
                                                      double red,
                                                      double green,
                                                      double blue)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0;

  const Eigen::Vector3d corner_000(min_position(0), min_position(1), min_position(2));
  const Eigen::Vector3d corner_100(max_position(0), min_position(1), min_position(2));
  const Eigen::Vector3d corner_010(min_position(0), max_position(1), min_position(2));
  const Eigen::Vector3d corner_110(max_position(0), max_position(1), min_position(2));
  const Eigen::Vector3d corner_001(min_position(0), min_position(1), max_position(2));
  const Eigen::Vector3d corner_101(max_position(0), min_position(1), max_position(2));
  const Eigen::Vector3d corner_011(min_position(0), max_position(1), max_position(2));
  const Eigen::Vector3d corner_111(max_position(0), max_position(1), max_position(2));

  appendBoundingBoxEdge(marker, corner_000, corner_100);
  appendBoundingBoxEdge(marker, corner_100, corner_110);
  appendBoundingBoxEdge(marker, corner_110, corner_010);
  appendBoundingBoxEdge(marker, corner_010, corner_000);
  appendBoundingBoxEdge(marker, corner_001, corner_101);
  appendBoundingBoxEdge(marker, corner_101, corner_111);
  appendBoundingBoxEdge(marker, corner_111, corner_011);
  appendBoundingBoxEdge(marker, corner_011, corner_001);
  appendBoundingBoxEdge(marker, corner_000, corner_001);
  appendBoundingBoxEdge(marker, corner_100, corner_101);
  appendBoundingBoxEdge(marker, corner_110, corner_111);
  appendBoundingBoxEdge(marker, corner_010, corner_011);

  return marker;
}
}  // namespace

int GridMap::getInflatedOccupancy(const Eigen::Vector3d& position) const
{
  return binary_inflation_layer_.getInflatedOccupancy(position);
}

bool GridMap::positionToVoxelCenterIfInMap(const Eigen::Vector3d& position,
                                           Eigen::Vector3d& voxel_center) const
{
  Eigen::Vector3i voxel_index;
  if (!voxel_grid_geometry_.positionToIndexIfInMap(position, voxel_index))
  {
    return false;
  }

  return voxel_grid_geometry_.indexToPositionIfInMap(voxel_index, voxel_center);
}

double GridMap::getResolution() const
{
  return voxel_grid_geometry_.getResolution();
}

const Eigen::Vector3i& GridMap::getVoxelNum() const
{
  return voxel_grid_geometry_.getVoxelNum();
}

const Eigen::Vector3d& GridMap::getRollingMapSize() const
{
  return voxel_grid_geometry_.getMapSize();
}

const Eigen::Vector3d& GridMap::getRollingMapMinBoundary() const
{
  return voxel_grid_geometry_.getMinBoundary();
}

const Eigen::Vector3d& GridMap::getRollingMapMaxBoundary() const
{
  return voxel_grid_geometry_.getMaxBoundary();
}

bool GridMap::hasUpdatedLocalMap() const
{
  return runtime_state_.mapping_status_.has_odometry_ &&
         runtime_state_.mapping_status_.has_point_cloud_ &&
         runtime_state_.mapping_status_.has_updated_local_bounds_;
}

bool GridMap::isPointWithinCurrentLocalUpdateRange(const Eigen::Vector3d &point) const
{
  if (!runtime_state_.mapping_status_.has_odometry_)
  {
    return false;
  }

  return isPointWithinLocalUpdateHalfRange(point - runtime_state_.sensor_pose_.sensor_position_);
}

void GridMap::initMap(const rclcpp::Node::SharedPtr &node)
{
  node_ = node;

  double rolling_map_size_x = -1.0;
  double rolling_map_size_y = -1.0;
  double rolling_map_size_z = -1.0;
  double voxel_resolution = -1.0;

  getOrDeclareParameter(node_, "grid_map/resolution", voxel_resolution, -1.0);
  getOrDeclareParameter(node_, "grid_map/rolling_map_size_x", rolling_map_size_x, -1.0);
  getOrDeclareParameter(node_, "grid_map/rolling_map_size_y", rolling_map_size_y, -1.0);
  getOrDeclareParameter(node_, "grid_map/rolling_map_size_z", rolling_map_size_z, -1.0);
  getOrDeclareParameter(node_, "grid_map/local_update_half_range_x", mapping_parameters_.map_.local_update_half_range_meters_(0), -1.0);
  getOrDeclareParameter(node_, "grid_map/local_update_half_range_y", mapping_parameters_.map_.local_update_half_range_meters_(1), -1.0);
  getOrDeclareParameter(node_, "grid_map/local_update_half_range_z", mapping_parameters_.map_.local_update_half_range_meters_(2), -1.0);
  getOrDeclareParameter(node_, "grid_map/xy_obstacle_inflation_meters", mapping_parameters_.inflation_.xy_obstacle_inflation_meters_, 0.35);
  getOrDeclareParameter(node_, "grid_map/z_obstacle_inflation_meters", mapping_parameters_.inflation_.z_obstacle_inflation_meters_, 0.35);
  getOrDeclareParameter(node_, "grid_map/max_published_z_meters", mapping_parameters_.publishing_.max_published_z_meters_, -0.1);
  getOrDeclareParameter(node_, "grid_map/frame_id", mapping_parameters_.map_.frame_id_, std::string("world"));
  getOrDeclareParameter(node_, "grid_map/rolling_map_min_z_meters", mapping_parameters_.map_.rolling_map_min_z_meters_, 1.0);

  const Eigen::Vector3d rolling_map_size_meters(rolling_map_size_x, rolling_map_size_y, rolling_map_size_z);
  const Eigen::Vector3d initial_map_min_boundary(-rolling_map_size_x / 2.0,
                                                 -rolling_map_size_y / 2.0,
                                                 mapping_parameters_.map_.rolling_map_min_z_meters_);

  voxel_grid_geometry_.initializeFromMapSize(initial_map_min_boundary,
                                             rolling_map_size_meters,
                                             voxel_resolution);
  raw_binary_occupancy_layer_.initialize(&voxel_grid_geometry_);
  binary_inflation_layer_.initialize(&voxel_grid_geometry_);

  const auto default_qos = rclcpp::SystemDefaultsQoS();
  const auto sensor_qos = rclcpp::SensorDataQoS();

  independent_cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", sensor_qos, std::bind(&GridMap::pointCloudCallback, this, std::placeholders::_1));
  independent_odometry_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", default_qos, std::bind(&GridMap::odometryCallback, this, std::placeholders::_1));

  visualization_timer_ = node_->create_wall_timer(110ms, std::bind(&GridMap::visualizationCallback, this));
  inflated_occupancy_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);
  map_boundary_marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/boundary_markers", 10);

  runtime_state_.mapping_status_.has_updated_local_bounds_ = false;
  runtime_state_.mapping_status_.has_odometry_ = false;
  runtime_state_.mapping_status_.has_point_cloud_ = false;
}

void GridMap::updateRollingMapBoundaries()
{
  const Eigen::Vector3d& rolling_map_size_meters = voxel_grid_geometry_.getMapSize();
  const Eigen::Vector3d rolling_map_min_boundary(
      runtime_state_.sensor_pose_.sensor_position_(0) - rolling_map_size_meters(0) / 2.0,
      runtime_state_.sensor_pose_.sensor_position_(1) - rolling_map_size_meters(1) / 2.0,
      mapping_parameters_.map_.rolling_map_min_z_meters_);
  voxel_grid_geometry_.setMapBoundariesFromMinBoundary(rolling_map_min_boundary);
}

void GridMap::clearBinaryOccupancyRegion(const Eigen::Vector3d& min_position,
                                         const Eigen::Vector3d& max_position)
{
  raw_binary_occupancy_layer_.clearRegion(min_position, max_position, 2.5);
  binary_inflation_layer_.clearRegion(min_position, max_position, 2.5);
}

void GridMap::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odometry)
{
  runtime_state_.sensor_pose_.sensor_position_(0) = odometry->pose.pose.position.x;
  runtime_state_.sensor_pose_.sensor_position_(1) = odometry->pose.pose.position.y;
  runtime_state_.sensor_pose_.sensor_position_(2) = odometry->pose.pose.position.z;

  runtime_state_.mapping_status_.has_odometry_ = true;
}

void GridMap::mergeRecentPointClouds(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &current_point_cloud_message,
                                     pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> current_point_cloud;
  pcl::fromROSMsg(*current_point_cloud_message, current_point_cloud);
  runtime_state_.point_cloud_state_.recent_point_clouds_.push_back(current_point_cloud);
  if (runtime_state_.point_cloud_state_.recent_point_clouds_.size() > 20)
  {
    runtime_state_.point_cloud_state_.recent_point_clouds_.erase(runtime_state_.point_cloud_state_.recent_point_clouds_.begin());
  }

  for (size_t i = 0; i < runtime_state_.point_cloud_state_.recent_point_clouds_.size(); ++i)
  {
    merged_point_cloud += runtime_state_.point_cloud_state_.recent_point_clouds_[i];
  }
}

bool GridMap::isPointCloudUpdateReady(const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud) const
{
  if (!runtime_state_.mapping_status_.has_odometry_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "No odometry received for point cloud occupancy update.");
    return false;
  }

  if (merged_point_cloud.points.empty())
  {
    return false;
  }

  return std::isfinite(runtime_state_.sensor_pose_.sensor_position_(0)) &&
         std::isfinite(runtime_state_.sensor_pose_.sensor_position_(1)) &&
         std::isfinite(runtime_state_.sensor_pose_.sensor_position_(2));
}

void GridMap::preparePointCloudLocalMap()
{
  updateRollingMapBoundaries();
  clearBinaryOccupancyRegion(runtime_state_.sensor_pose_.sensor_position_ - mapping_parameters_.map_.local_update_half_range_meters_,
                             runtime_state_.sensor_pose_.sensor_position_ + mapping_parameters_.map_.local_update_half_range_meters_);
}

bool GridMap::isFinitePointCloudPoint(const pcl::PointXYZ &point) const
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool GridMap::isPointWithinLocalUpdateHalfRange(const Eigen::Vector3d &point_offset) const
{
  return fabs(point_offset(0)) < mapping_parameters_.map_.local_update_half_range_meters_(0) &&
         fabs(point_offset(1)) < mapping_parameters_.map_.local_update_half_range_meters_(1) &&
         fabs(point_offset(2)) < mapping_parameters_.map_.local_update_half_range_meters_(2);
}

void GridMap::collectUniqueRawOccupiedVoxelsFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud,
    std::vector<Eigen::Vector3i> &raw_occupied_voxel_indices)
{
  std::vector<char> is_voxel_collected(voxel_grid_geometry_.getVoxelCount(), 0);
  Eigen::Vector3d point_position;
  Eigen::Vector3i point_index;

  for (size_t i = 0; i < merged_point_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &point = merged_point_cloud.points[i];
    if (!isFinitePointCloudPoint(point))
    {
      continue;
    }

    point_position(0) = point.x;
    point_position(1) = point.y;
    point_position(2) = point.z;

    const Eigen::Vector3d point_offset = point_position - runtime_state_.sensor_pose_.sensor_position_;
    if (point_offset.norm() <= 0.3)
    {
      continue;
    }

    // 合并点云中的历史点只在当前局部更新半范围内继续参与建图，范围外会随本轮局部清空被丢弃。
    if (!isPointWithinLocalUpdateHalfRange(point_offset))
    {
      continue;
    }

    voxel_grid_geometry_.positionToIndexUnchecked(point_position, point_index);
    if (!voxel_grid_geometry_.isInMap(point_index))
    {
      continue;
    }

    const int raw_buffer_index = voxel_grid_geometry_.addressUnchecked(point_index);
    if (is_voxel_collected[raw_buffer_index] != 0)
    {
      continue;
    }

    is_voxel_collected[raw_buffer_index] = 1;
    raw_binary_occupancy_layer_.setRawOccupancy(raw_buffer_index, 1);
    raw_occupied_voxel_indices.push_back(point_index);
  }
}

void GridMap::inflateRawOccupiedVoxels(const std::vector<Eigen::Vector3i> &raw_occupied_voxel_indices,
                                       Eigen::Vector3d &min_position,
                                       Eigen::Vector3d &max_position)
{
  const int xy_inflation_step = std::ceil(mapping_parameters_.inflation_.xy_obstacle_inflation_meters_ /
                                          voxel_grid_geometry_.getResolution());
  const int z_inflation_step = std::ceil(mapping_parameters_.inflation_.z_obstacle_inflation_meters_ /
                                         voxel_grid_geometry_.getResolution());
  std::vector<Eigen::Vector3i> inflated_indices((2 * xy_inflation_step + 1) *
                                                (2 * xy_inflation_step + 1) *
                                                (2 * z_inflation_step + 1));

  for (size_t i = 0; i < raw_occupied_voxel_indices.size(); ++i)
  {
    binary_inflation_layer_.inflatePoint(raw_occupied_voxel_indices[i], xy_inflation_step,
                                         z_inflation_step, inflated_indices);

    for (size_t k = 0; k < inflated_indices.size(); ++k)
    {
      const Eigen::Vector3i &inflated_index = inflated_indices[k];
      if (!voxel_grid_geometry_.isInMap(inflated_index))
      {
        continue;
      }

      Eigen::Vector3d inflated_position;
      voxel_grid_geometry_.indexToPositionUnchecked(inflated_index, inflated_position);

      min_position(0) = std::min(min_position(0), inflated_position(0));
      min_position(1) = std::min(min_position(1), inflated_position(1));
      min_position(2) = std::min(min_position(2), inflated_position(2));

      max_position(0) = std::max(max_position(0), inflated_position(0));
      max_position(1) = std::max(max_position(1), inflated_position(1));
      max_position(2) = std::max(max_position(2), inflated_position(2));

      binary_inflation_layer_.setInflatedOccupancy(voxel_grid_geometry_.addressUnchecked(inflated_index), 1);
    }
  }
}

void GridMap::updateBinaryOccupancyFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> &merged_point_cloud,
                                                  Eigen::Vector3d &min_position,
                                                  Eigen::Vector3d &max_position)
{
  min_position = voxel_grid_geometry_.getMaxBoundary();
  max_position = voxel_grid_geometry_.getMinBoundary();

  std::vector<Eigen::Vector3i> raw_occupied_voxel_indices;
  collectUniqueRawOccupiedVoxelsFromPointCloud(merged_point_cloud, raw_occupied_voxel_indices);
  inflateRawOccupiedVoxels(raw_occupied_voxel_indices, min_position, max_position);
}

void GridMap::updatePointCloudLocalBounds(const Eigen::Vector3d &min_position,
                                          const Eigen::Vector3d &max_position)
{
  double min_x = min_position(0);
  double min_y = min_position(1);
  double min_z = min_position(2);
  double max_x = max_position(0);
  double max_y = max_position(1);
  double max_z = max_position(2);

  min_x = std::min(min_x, runtime_state_.sensor_pose_.sensor_position_(0));
  min_y = std::min(min_y, runtime_state_.sensor_pose_.sensor_position_(1));
  min_z = std::min(min_z, runtime_state_.sensor_pose_.sensor_position_(2));

  max_x = std::max(max_x, runtime_state_.sensor_pose_.sensor_position_(0));
  max_y = std::max(max_y, runtime_state_.sensor_pose_.sensor_position_(1));
  max_z = std::max(max_z, runtime_state_.sensor_pose_.sensor_position_(2));

  max_z = std::max(max_z, mapping_parameters_.map_.rolling_map_min_z_meters_);

  voxel_grid_geometry_.positionToIndexUnchecked(Eigen::Vector3d(max_x, max_y, max_z), runtime_state_.local_bounds_.max_voxel_index_);
  voxel_grid_geometry_.positionToIndexUnchecked(Eigen::Vector3d(min_x, min_y, min_z), runtime_state_.local_bounds_.min_voxel_index_);

  runtime_state_.local_bounds_.min_voxel_index_ = voxel_grid_geometry_.clampedIndex(runtime_state_.local_bounds_.min_voxel_index_);
  runtime_state_.local_bounds_.max_voxel_index_ = voxel_grid_geometry_.clampedIndex(runtime_state_.local_bounds_.max_voxel_index_);
  runtime_state_.mapping_status_.has_updated_local_bounds_ = true;
}

void GridMap::addVirtualCeilingToInflatedOccupancy()
{
  if (!runtime_state_.mapping_status_.has_updated_local_bounds_)
  {
    return;
  }

  const int ceiling_index_z = voxel_grid_geometry_.getVoxelNum()(2) - 1;
  if (ceiling_index_z < 0)
  {
    return;
  }

  for (int x = runtime_state_.local_bounds_.min_voxel_index_(0); x <= runtime_state_.local_bounds_.max_voxel_index_(0); ++x)
  {
    for (int y = runtime_state_.local_bounds_.min_voxel_index_(1); y <= runtime_state_.local_bounds_.max_voxel_index_(1); ++y)
    {
      binary_inflation_layer_.setInflatedOccupancy(voxel_grid_geometry_.addressUnchecked(x, y, ceiling_index_z), 1);
    }
  }
}

void GridMap::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &current_point_cloud_message)
{
  pcl::PointCloud<pcl::PointXYZ> merged_point_cloud;
  mergeRecentPointClouds(current_point_cloud_message, merged_point_cloud);

  runtime_state_.mapping_status_.has_point_cloud_ = true;

  if (!isPointCloudUpdateReady(merged_point_cloud))
  {
    return;
  }

  preparePointCloudLocalMap();

  Eigen::Vector3d min_position;
  Eigen::Vector3d max_position;
  updateBinaryOccupancyFromPointCloud(merged_point_cloud, min_position, max_position);
  updatePointCloudLocalBounds(min_position, max_position);
  addVirtualCeilingToInflatedOccupancy();
}

void GridMap::publishInflatedOccupancyMap()
{
  if (inflated_occupancy_map_publisher_->get_subscription_count() <= 0 ||
      !runtime_state_.mapping_status_.has_updated_local_bounds_)
  {
    return;
  }

  pcl::PointXYZ point;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = runtime_state_.local_bounds_.min_voxel_index_;
  Eigen::Vector3i max_cut = runtime_state_.local_bounds_.max_voxel_index_;

  min_cut = voxel_grid_geometry_.clampedIndex(min_cut);
  max_cut = voxel_grid_geometry_.clampedIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
  {
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (binary_inflation_layer_.getInflatedOccupancy(voxel_grid_geometry_.addressUnchecked(x, y, z)) == 0)
        {
          continue;
        }

        Eigen::Vector3d position;
        voxel_grid_geometry_.indexToPositionUnchecked(Eigen::Vector3i(x, y, z), position);
        if (position(2) > mapping_parameters_.publishing_.max_published_z_meters_)
        {
          continue;
        }

        point.x = position(0);
        point.y = position(1);
        point.z = position(2);
        cloud.push_back(point);
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mapping_parameters_.map_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  inflated_occupancy_map_publisher_->publish(cloud_msg);
}

void GridMap::publishMapBoundaryMarkers()
{
  if (map_boundary_marker_publisher_->get_subscription_count() <= 0 ||
      !runtime_state_.mapping_status_.has_odometry_)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time stamp = node_->now();
  marker_array.markers.push_back(makeBoundingBoxMarker(mapping_parameters_.map_.frame_id_,
                                                       stamp,
                                                       0,
                                                       "rolling_map_size",
                                                       voxel_grid_geometry_.getMinBoundary(),
                                                       voxel_grid_geometry_.getMaxBoundary(),
                                                       0.06,
                                                       0.0,
                                                       0.85,
                                                       1.0));

  const Eigen::Vector3d local_update_half_range_min_position =
      runtime_state_.sensor_pose_.sensor_position_ - mapping_parameters_.map_.local_update_half_range_meters_;
  const Eigen::Vector3d local_update_half_range_max_position =
      runtime_state_.sensor_pose_.sensor_position_ + mapping_parameters_.map_.local_update_half_range_meters_;
  marker_array.markers.push_back(makeBoundingBoxMarker(mapping_parameters_.map_.frame_id_,
                                                       stamp,
                                                       1,
                                                       "local_update_half_range",
                                                       local_update_half_range_min_position,
                                                       local_update_half_range_max_position,
                                                       0.04,
                                                       1.0,
                                                       0.7,
                                                       0.0));

  map_boundary_marker_publisher_->publish(marker_array);
}

void GridMap::visualizationCallback()
{
  publishInflatedOccupancyMap();
  publishMapBoundaryMarkers();
}
