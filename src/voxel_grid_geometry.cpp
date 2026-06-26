#include <plan_env/voxel_grid_geometry.h>

#include <algorithm>
#include <cmath>

void VoxelGridGeometry::initializeFromMapSize(const Eigen::Vector3d& map_min_boundary,
                                              const Eigen::Vector3d& map_size,
                                              double resolution)
{
  min_boundary_ = map_min_boundary;
  map_size_ = map_size;
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution_;

  for (int i = 0; i < 3; ++i)
  {
    voxel_num_(i) = std::ceil(map_size_(i) / resolution_);
  }

  max_boundary_ = min_boundary_ + map_size_;
}

void VoxelGridGeometry::initializeFromVoxelCounts(const Eigen::Vector3d& map_min_boundary,
                                                  const Eigen::Vector3i& voxel_counts,
                                                  double resolution)
{
  min_boundary_ = map_min_boundary;
  voxel_num_ = voxel_counts;
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution_;
  map_size_ = voxel_num_.cast<double>() * resolution_;
  max_boundary_ = min_boundary_ + map_size_;
}

void VoxelGridGeometry::setMapBoundariesFromMinBoundary(const Eigen::Vector3d& map_min_boundary)
{
  min_boundary_ = map_min_boundary;
  max_boundary_ = min_boundary_ + map_size_;
}

void VoxelGridGeometry::positionToIndexUnchecked(const Eigen::Vector3d& position, Eigen::Vector3i& index) const
{
  for (int i = 0; i < 3; ++i)
  {
    index(i) = std::floor((position(i) - min_boundary_(i)) * resolution_inv_);
  }
}

void VoxelGridGeometry::indexToPositionUnchecked(const Eigen::Vector3i& index, Eigen::Vector3d& position) const
{
  for (int i = 0; i < 3; ++i)
  {
    position(i) = (index(i) + 0.5) * resolution_ + min_boundary_(i);
  }
}

int VoxelGridGeometry::addressUnchecked(const Eigen::Vector3i& index) const
{
  return addressUnchecked(index(0), index(1), index(2));
}

int VoxelGridGeometry::addressUnchecked(int x, int y, int z) const
{
  return x * voxel_num_(1) * voxel_num_(2) + y * voxel_num_(2) + z;
}

bool VoxelGridGeometry::positionToIndexIfInMap(const Eigen::Vector3d& position, Eigen::Vector3i& index) const
{
  if (!isInMap(position))
  {
    return false;
  }

  positionToIndexUnchecked(position, index);
  return true;
}

bool VoxelGridGeometry::indexToPositionIfInMap(const Eigen::Vector3i& index, Eigen::Vector3d& position) const
{
  if (!isInMap(index))
  {
    return false;
  }

  indexToPositionUnchecked(index, position);
  return true;
}

bool VoxelGridGeometry::addressIfInMap(const Eigen::Vector3i& index, int& buffer_address) const
{
  if (!isInMap(index))
  {
    return false;
  }

  buffer_address = addressUnchecked(index);
  return true;
}

Eigen::Vector3i VoxelGridGeometry::clampedIndex(const Eigen::Vector3i& index) const
{
  Eigen::Vector3i clamped_index;
  clamped_index(0) = std::max(std::min(index(0), voxel_num_(0) - 1), 0);
  clamped_index(1) = std::max(std::min(index(1), voxel_num_(1) - 1), 0);
  clamped_index(2) = std::max(std::min(index(2), voxel_num_(2) - 1), 0);
  return clamped_index;
}

bool VoxelGridGeometry::isInMap(const Eigen::Vector3d& position) const
{
  if (!position.allFinite())
  {
    return false;
  }

  if (position(0) < min_boundary_(0) + 1e-4 || position(1) < min_boundary_(1) + 1e-4 ||
      position(2) < min_boundary_(2) + 1e-4)
  {
    return false;
  }
  if (position(0) > max_boundary_(0) - 1e-4 || position(1) > max_boundary_(1) - 1e-4 ||
      position(2) > max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

bool VoxelGridGeometry::isInMap(const Eigen::Vector3i& index) const
{
  if (index(0) < 0 || index(1) < 0 || index(2) < 0)
  {
    return false;
  }
  if (index(0) > voxel_num_(0) - 1 || index(1) > voxel_num_(1) - 1 ||
      index(2) > voxel_num_(2) - 1)
  {
    return false;
  }
  return true;
}

const Eigen::Vector3d& VoxelGridGeometry::getMapSize() const { return map_size_; }

const Eigen::Vector3d& VoxelGridGeometry::getMinBoundary() const { return min_boundary_; }

const Eigen::Vector3d& VoxelGridGeometry::getMaxBoundary() const { return max_boundary_; }

const Eigen::Vector3i& VoxelGridGeometry::getVoxelNum() const { return voxel_num_; }

double VoxelGridGeometry::getResolution() const { return resolution_; }

double VoxelGridGeometry::getResolutionInv() const { return resolution_inv_; }

int VoxelGridGeometry::getVoxelCount() const
{
  return voxel_num_(0) * voxel_num_(1) * voxel_num_(2);
}
