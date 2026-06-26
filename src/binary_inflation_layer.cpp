#include <plan_env/binary_inflation_layer.h>

void BinaryInflationLayer::initialize(const VoxelGridGeometry* grid_geometry)
{
  grid_geometry_ = grid_geometry;
  binary_inflated_obstacles_.assign(grid_geometry_->getVoxelCount(), 0);
}

void BinaryInflationLayer::clearRegion(const Eigen::Vector3d& min_position,
                                       const Eigen::Vector3d& max_position,
                                       double padding)
{
  Eigen::Vector3d padded_min = min_position - Eigen::Vector3d(padding, padding, padding);
  Eigen::Vector3d padded_max = max_position + Eigen::Vector3d(padding, padding, padding);

  Eigen::Vector3i min_index;
  Eigen::Vector3i max_index;
  grid_geometry_->positionToIndexUnchecked(padded_min, min_index);
  grid_geometry_->positionToIndexUnchecked(padded_max, max_index);

  min_index = grid_geometry_->clampedIndex(min_index);
  max_index = grid_geometry_->clampedIndex(max_index);

  for (int x = min_index(0); x <= max_index(0); ++x)
    for (int y = min_index(1); y <= max_index(1); ++y)
      for (int z = min_index(2); z <= max_index(2); ++z)
      {
        binary_inflated_obstacles_[grid_geometry_->addressUnchecked(x, y, z)] = 0;
      }
}

void BinaryInflationLayer::setInflatedOccupied(const Eigen::Vector3d& position)
{
  if (!grid_geometry_->isInMap(position))
  {
    return;
  }

  Eigen::Vector3i index;
  grid_geometry_->positionToIndexUnchecked(position, index);
  binary_inflated_obstacles_[grid_geometry_->addressUnchecked(index)] = 1;
}

void BinaryInflationLayer::setInflatedOccupancy(int buffer_index, int occupancy_value)
{
  if (occupancy_value != 0 && occupancy_value != 1)
  {
    return;
  }

  if (buffer_index < 0 || buffer_index >= static_cast<int>(binary_inflated_obstacles_.size()))
  {
    return;
  }

  binary_inflated_obstacles_[buffer_index] = static_cast<char>(occupancy_value);
}

int BinaryInflationLayer::getInflatedOccupancy(int buffer_index) const
{
  if (buffer_index < 0 || buffer_index >= static_cast<int>(binary_inflated_obstacles_.size()))
  {
    return -1;
  }

  return static_cast<int>(binary_inflated_obstacles_[buffer_index]);
}

bool BinaryInflationLayer::isInflatedOccupied(const Eigen::Vector3i& index) const
{
  if (!grid_geometry_->isInMap(index))
  {
    return false;
  }

  return binary_inflated_obstacles_[grid_geometry_->addressUnchecked(index)] == 1;
}

int BinaryInflationLayer::getInflatedOccupancy(const Eigen::Vector3d& position) const
{
  if (!grid_geometry_->isInMap(position))
  {
    return -1;
  }

  Eigen::Vector3i index;
  grid_geometry_->positionToIndexUnchecked(position, index);
  return static_cast<int>(binary_inflated_obstacles_[grid_geometry_->addressUnchecked(index)]);
}

void BinaryInflationLayer::inflatePoint(const Eigen::Vector3i& center_index,
                                        int inflation_step,
                                        std::vector<Eigen::Vector3i>& inflated_indices) const
{
  inflatePoint(center_index, inflation_step, inflation_step, inflated_indices);
}

void BinaryInflationLayer::inflatePoint(const Eigen::Vector3i& center_index,
                                        int xy_inflation_step,
                                        int z_inflation_step,
                                        std::vector<Eigen::Vector3i>& inflated_indices) const
{
  // 当膨胀步数为 0 时，仍然会将中心体素加入输出集合。
  int num = 0;
  for (int x = -xy_inflation_step; x <= xy_inflation_step; ++x)
    for (int y = -xy_inflation_step; y <= xy_inflation_step; ++y)
      for (int z = -z_inflation_step; z <= z_inflation_step; ++z)
      {
        inflated_indices[num++] = Eigen::Vector3i(center_index(0) + x, center_index(1) + y, center_index(2) + z);
      }
}
