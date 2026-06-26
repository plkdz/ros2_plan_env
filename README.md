# plan_env

`plan_env` 提供 planner 使用的局部滚动占据地图。当前实现已经删除深度相机、OpenCV、message_filters、log-odds 和 raycast 路线，只保留点云/激光雷达输入路线。

## 文件结构

```text
include/plan_env/
  voxel_grid_geometry.h        体素地图几何、边界、索引和地址转换
  binary_occupancy_layer.h     原始未膨胀二值占据层
  binary_inflation_layer.h     膨胀后二值占据层
  occupancy_grid.h             占据地图基础组件聚合头文件
  grid_map.h                   ROS 建图入口和 planner 查询接口

src/
  voxel_grid_geometry.cpp
  binary_occupancy_layer.cpp
  binary_inflation_layer.cpp
  grid_map.cpp
```

## 核心组件

### VoxelGridGeometry

`VoxelGridGeometry` 保存地图尺寸、分辨率、最小边界、最大边界和体素数量。它只处理几何关系，不保存占据数据。

主要接口：

- `initializeFromMapSize(map_min_boundary, map_size, resolution)`：按物理尺寸初始化地图几何，适合 `GridMap` 这类由配置参数给出地图尺寸的场景。
- `initializeFromVoxelCounts(map_min_boundary, voxel_counts, resolution)`：按体素数量初始化地图几何，适合 A* 局部搜索窗口这类节点池尺寸已确定的场景。
- `setMapBoundariesFromMinBoundary(map_min_boundary)`：按新的最小边界移动滚动地图窗口，同时更新最大边界。
- `positionToIndexUnchecked(position, index)`：世界坐标转体素索引，不检查坐标是否在地图内。
- `indexToPositionUnchecked(index, position)`：体素索引转体素中心世界坐标，不检查索引是否在地图内。
- `addressUnchecked(index)` / `addressUnchecked(x, y, z)`：三维索引转一维 buffer 地址，不检查索引是否在地图内。
- `positionToIndexIfInMap(position, index)`：仅当世界坐标在地图内时转换为体素索引。
- `indexToPositionIfInMap(index, position)`：仅当体素索引在地图内时转换为体素中心世界坐标。
- `addressIfInMap(index, buffer_address)`：仅当体素索引在地图内时转换为一维 buffer 地址。
- `clampedIndex(index)`：把索引裁剪到地图范围内。
- `isInMap(position/index)`：判断世界坐标或索引是否在地图内；世界坐标版本会把 NaN/inf 这类非有限坐标直接判为地图外。

### BinaryOccupancyLayer

`BinaryOccupancyLayer` 保存原始、未膨胀的二值障碍物。点云输入会先写入这个层，再生成膨胀层。

主要接口：

- `initialize(grid_geometry)`：绑定共享地图几何，并初始化 buffer。
- `clearRegion(min_position, max_position, padding)`：清空指定世界坐标区域。
- `setRawOccupied(position)`：把世界坐标所在体素设为原始占据。
- `setRawOccupancy(buffer_index, occupancy_value)`：按一维地址设置原始占据值。
- `getRawOccupancy(buffer_index/position)`：查询原始二值占据，越界返回 `-1`。
- `isRawOccupied(index)`：按体素索引判断是否为原始占据。

### BinaryInflationLayer

`BinaryInflationLayer` 保存最终膨胀后的二值障碍物，是外部 planner 主要查询的地图层。

主要接口：

- `initialize(grid_geometry)`：绑定共享地图几何，并初始化膨胀 buffer。
- `clearRegion(min_position, max_position, padding)`：清空指定世界坐标区域的膨胀占据。
- `setInflatedOccupied(position)`：把世界坐标所在体素设为膨胀占据。
- `setInflatedOccupancy(buffer_index, occupancy_value)`：按一维地址设置膨胀占据值。
- `getInflatedOccupancy(buffer_index/position)`：查询膨胀占据，越界返回 `-1`。
- `isInflatedOccupied(index)`：按体素索引判断是否膨胀占据。
- `inflatePoint(center_index, inflation_step, inflated_indices)`：生成统一步数的立方体膨胀索引。
- `inflatePoint(center_index, xy_inflation_step, z_inflation_step, inflated_indices)`：生成 xy/z 分离步数的长方体膨胀索引。

## GridMap

`GridMap` 是 ROS 侧入口。它持有配置参数、运行时状态、地图几何和两个二值占据层：

```cpp
MappingParameters mapping_parameters_;
MappingData runtime_state_;
VoxelGridGeometry voxel_grid_geometry_;
BinaryOccupancyLayer raw_binary_occupancy_layer_;
BinaryInflationLayer binary_inflation_layer_;
```

外部主要使用的 C++ 接口：

- `initMap(node)`：读取 ROS 参数，初始化地图、占据层、订阅器、发布器和定时器。
- `getInflatedOccupancy(position)`：查询膨胀占据层，返回 `1/0/-1`。
- `positionToVoxelCenterIfInMap(position, voxel_center)`：当世界坐标位于当前滚动地图内时，返回该点所在体素的中心世界坐标。
- `getResolution()`：获取体素分辨率。
- `getVoxelNum()`：获取滚动地图 buffer 在 x/y/z 方向上的体素数量。
- `getRollingMapSize()`：获取滚动地图 buffer 在 x/y/z 方向上的物理尺寸。
- `getRollingMapMinBoundary()`：获取当前滚动地图最小世界坐标边界。
- `getRollingMapMaxBoundary()`：获取当前滚动地图最大世界坐标边界。
- `hasUpdatedLocalMap()`：判断是否已经收到里程计和点云，并完成过一次有效局部膨胀占据地图更新。测试节点或规划节点需要等地图真正可查询时，应使用该接口，而不是只等原始点云 topic 到达。
- `isPointWithinCurrentLocalUpdateRange(point)`：判断世界坐标点是否位于当前传感器位置加减 `grid_map/local_update_half_range_*` 构成的局部更新方框内。未收到里程计时返回 `false`。

`grid_map.h` 的类声明顺序按对外阅读优先级维护：`public` 查询和初始化接口在前，`private` 回调/更新/发布辅助函数在中间，最后集中放置 `private` 成员变量。这样阅读头文件时可以先看到外部契约，再看到内部流程，最后确认对象持有的状态。

## 点云建图路线

输入是已经在地图坐标系下的点云和里程计：

1. `odometryCallback()` 更新当前传感器位置。
2. `pointCloudCallback()` 接收 `grid_map/cloud` 点云。
3. `mergeRecentPointClouds()` 缓存并合并最近若干帧点云。
4. `preparePointCloudLocalMap()` 根据当前位姿更新滚动地图边界，并清空局部二值占据区域。
5. `collectUniqueRawOccupiedVoxelsFromPointCloud()` 从点云生成去重后的原始占据体素；合并点云中超出当前局部更新半范围的历史点不会继续写回地图。
6. `inflateRawOccupiedVoxels()` 对原始占据体素执行 xy/z 分离膨胀。
7. `updatePointCloudLocalBounds()` 更新本轮局部发布边界。
8. `addVirtualCeilingToInflatedOccupancy()` 添加虚拟天花板。
9. `publishInflatedOccupancyMap()` 发布最终膨胀占据点云。
10. `publishMapBoundaryMarkers()` 发布滚动地图边界和局部更新半范围边界。

点云路线会过滤非有限点，并跳过距离传感器过近的点。当前近距离自过滤阈值仍是代码中的固定值 `0.3 m`。

## ROS 接口

### 订阅

- `grid_map/cloud`：点云输入，点坐标应与 `grid_map/frame_id` 一致。
- `grid_map/odom`：里程计输入，`pose.position` 被用作当前传感器位置。

### 发布

- `grid_map/occupancy_inflate`：膨胀后二值占据点云，是 planner 应关注的输出。
- `grid_map/boundary_markers`：`visualization_msgs::msg::MarkerArray`，包含两个 `LINE_LIST` 边框：
  - `rolling_map_size`：当前滚动地图 buffer 的真实边界，来自 `VoxelGridGeometry::getMinBoundary()/getMaxBoundary()`。
  - `local_update_half_range`：当前传感器位置加减 `grid_map/local_update_half_range_*` 得到的局部清空和点云接收范围。

当前不再订阅 `grid_map/depth`、`grid_map/pose` 或 `/vins_estimator/extrinsic`，也不再发布未膨胀概率占据 topic `grid_map/occupancy`。

## 关键参数

地图几何：

- `grid_map/resolution`
- `grid_map/rolling_map_size_x`：滚动地图 x 方向全边长，单位为米。
- `grid_map/rolling_map_size_y`：滚动地图 y 方向全边长，单位为米。
- `grid_map/rolling_map_size_z`：滚动地图 z 方向全边长，单位为米。
- `grid_map/local_update_half_range_x`：以当前传感器位置为中心的局部更新 x 方向半边长，单位为米。
- `grid_map/local_update_half_range_y`：以当前传感器位置为中心的局部更新 y 方向半边长，单位为米。
- `grid_map/local_update_half_range_z`：以当前传感器位置为中心的局部更新 z 方向半边长，单位为米。
- `grid_map/rolling_map_min_z_meters`
- `grid_map/frame_id`

障碍膨胀：

- `grid_map/xy_obstacle_inflation_meters`：x/y 方向膨胀尺寸，单位为米。
- `grid_map/z_obstacle_inflation_meters`：z 方向膨胀尺寸，单位为米。

膨胀占据点云发布：

- `grid_map/max_published_z_meters`

## 设计注意事项

- `VoxelGridGeometry` 是共享几何对象，两个占据层内部保存的是指向它的指针。调用方必须保证几何对象生命周期长于占据层。
- `VoxelGridGeometry` 也可以作为非地图占据层的局部几何结构使用，例如 `AStar` 用它表达自己的局部搜索窗口；这种用法只复用几何转换，不代表 A* 绑定到 `GridMap` 的完整地图边界。
- `AStar` 初始化时直接从 `GridMap::getVoxelNum()` 读取节点池尺寸，搜索时使用 `GridMap::getResolution()`，使 A* 搜索窗口的物理边长和 `GridMap` 滚动地图 buffer 一致。
- 物理边长一致不代表覆盖范围一致：A* 搜索窗口通常按搜索起终点中点居中，而 `GridMap` 滚动地图窗口按当前传感器位置移动。
- 滚动地图移动的是 `min_boundary/max_boundary`，不是地图中心变量。地图中心由当前传感器位置和地图尺寸推导。
- `getRollingMapMinBoundary()` / `getRollingMapMaxBoundary()` 返回的就是当前 `VoxelGridGeometry` 中已经同步的世界坐标边界，可用于规划侧检查点是否位于当前 rolling map buffer 内。
- 滚动地图的 x/y 边界随当前传感器位置居中移动；z 方向不是以里程计上下对称，而是固定 `grid_map/rolling_map_min_z_meters` 作为下边界，再加 `grid_map/rolling_map_size_z` 得到上边界。
- `grid_map/local_update_half_range_*` 是半边长，不是全边长。实际局部更新边框尺寸是该参数的两倍，中心为当前传感器位置。
- `isPointWithinCurrentLocalUpdateRange(point)` 是规划侧判断目标点是否仍在当前局部更新方框内的只读接口。它和 `getRollingMapMinBoundary()` / `getRollingMapMaxBoundary()` 的 rolling map buffer 判断不同：前者以当前传感器位置和 `local_update_half_range_*` 为边界，后者以滚动地图物理 buffer 为边界。
- `hasUpdatedLocalMap()` 是规划侧或手动测试节点的 readiness 判断接口。它要求 `GridMap` 已经收到里程计、收到点云，并完成过一次局部地图边界更新；只收到原始点云不代表膨胀占据层已经可用。
- 局部更新半范围同时是本轮点云接收范围。即使 `mergeRecentPointClouds()` 合并了最近若干帧点云，范围外的历史点云也会被过滤，不会在局部清空后重新写回占据层。
- `getInflatedOccupancy()` 是规划侧最直接的查询入口，语义是膨胀障碍查询，不是原始障碍查询；返回值 `-1` 表示查询点超出 `GridMap` 当前覆盖范围。非有限查询坐标会在 `VoxelGridGeometry::isInMap(position)` 这一层被统一视为地图外，因此同样向上表现为 `-1`。A* 按 EGO-Planner 原始局部规划语义显式把 `-1` 当作不可通行，而不是未知可通行。
- 因此局部目标可以超出当前传感器观测/更新范围，但不应超出 `GridMap` 滚动地图 buffer。若 A* 搜索段落在 buffer 外，查询会返回 `-1`，该区域会被局部 A* 当作占据区域。
- `AStar::searchPath()` 会显式检查起点和终点是否位于当前 rolling map 边界内，并检查二者是否落在 A* 搜索窗口内部层；即使端点位于 A* 搜索窗口内，只要超出 `GridMap` 当前 rolling map 边界，也会直接返回失败并输出日志。
- `path_searching_online_test_node` 内部会创建自己的 `GridMap`，直接订阅点云和里程计并发布测试专用的膨胀点云/边界 marker；运行该在线测试 launch 时不需要额外启动 `grid_map_standalone_node`。
- 在线测试里的随机 goal 会检查 `getInflatedOccupancy(goal) == 0`；当前飞机位置来自 odom，若起点处在膨胀障碍、rolling map 边界或 A* 搜索窗口边界附近，A* 按端点前置条件返回失败是合理行为。
- 当前没有显式 mutex 或双 buffer。默认单线程 ROS spin 下回调顺序执行；如果未来使用多线程回调，需要重新审查地图清空、写入和查询之间的一致性。

## 作者与致谢

当前 ROS2 版本的 `plan_env` 由 plkdz 重新设计和重写，主要实现包括 `GridMap`、`VoxelGridGeometry`、`BinaryOccupancyLayer` 和 `BinaryInflationLayer`。

本包最早参考 EGO-Planner 中 `plan_env` 模块的规划侧地图接口语义，用于提供局部体素地图、膨胀占据查询和 ESDF-free 局部规划所需的地图访问能力。感谢 EGO-Planner 原始作者 Xin Zhou、Zhepei Wang、Hongkai Ye、Chao Xu、Fei Gao 对该类局部规划框架和开源实现的贡献。
