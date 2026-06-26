#include <plan_env/binary_occupancy_layer.h>

void BinaryOccupancyLayer::initialize(const VoxelGridGeometry* grid_geometry)
{
  grid_geometry_ = grid_geometry;
  raw_binary_obstacles_.assign(grid_geometry_->getVoxelCount(), 0);
}

void BinaryOccupancyLayer::clearRegion(const Eigen::Vector3d& min_position,
                                       const Eigen::Vector3d& max_position,
                                       double padding)
{
  Eigen::Vector3d padded_min = min_position - Eigen::Vector3d(padding, padding, padding);
  Eigen::Vector3d padded_max = max_position + Eigen::Vector3d(padding, padding, padding);

  Eigen::Vector3i min_index; // 输出padded_min对应的体素索引index
  Eigen::Vector3i max_index; // 输出padded_max对应的体素索引index
  grid_geometry_->positionToIndexUnchecked(padded_min, min_index);
  grid_geometry_->positionToIndexUnchecked(padded_max, max_index);

  //做一层clampping，确保索引在地图范围内
  min_index = grid_geometry_->clampedIndex(min_index);
  max_index = grid_geometry_->clampedIndex(max_index);

  for (int x = min_index(0); x <= max_index(0); ++x)
    for (int y = min_index(1); y <= max_index(1); ++y)
      for (int z = min_index(2); z <= max_index(2); ++z)
      {
        raw_binary_obstacles_[grid_geometry_->addressUnchecked(x, y, z)] = 0;
      }
}

void BinaryOccupancyLayer::setRawOccupied(const Eigen::Vector3d& position)
{
  if (!grid_geometry_->isInMap(position))
  {
    //这种情况是运行合理的，因为地图只维护一小部分，而传感器传回的点云数据可能在当前维护的地图范围外，直接忽略即可
    return;
  }

  Eigen::Vector3i index;
  grid_geometry_->positionToIndexUnchecked(position, index);
  raw_binary_obstacles_[grid_geometry_->addressUnchecked(index)] = 1;
}

void BinaryOccupancyLayer::setRawOccupancy(int buffer_index, int occupancy_value)
{
  if (occupancy_value != 0 && occupancy_value != 1)
  {
    return;
  }

  if (buffer_index < 0 || buffer_index >= static_cast<int>(raw_binary_obstacles_.size()))
  {
    return;
  }

  raw_binary_obstacles_[buffer_index] = static_cast<char>(occupancy_value);
}

int BinaryOccupancyLayer::getRawOccupancy(int buffer_index) const
{
  if (buffer_index < 0 || buffer_index >= static_cast<int>(raw_binary_obstacles_.size()))
  {
    return -1;
  }

  return static_cast<int>(raw_binary_obstacles_[buffer_index]);
}

bool BinaryOccupancyLayer::isRawOccupied(const Eigen::Vector3i& index) const
{
  if (!grid_geometry_->isInMap(index))
  {
    return false;
  }

  return raw_binary_obstacles_[grid_geometry_->addressUnchecked(index)] == 1;
}

int BinaryOccupancyLayer::getRawOccupancy(const Eigen::Vector3d& position) const
{
  if (!grid_geometry_->isInMap(position))
  {
    return -1;
  }

  Eigen::Vector3i index;
  grid_geometry_->positionToIndexUnchecked(position, index);
  return static_cast<int>(raw_binary_obstacles_[grid_geometry_->addressUnchecked(index)]);
}
