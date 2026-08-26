#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

constexpr double kZeroZThreshold = 1.0e-6;

std::string defaultInputPcd()
{
#ifdef OCTO_PLANNER3D_SOURCE_DIR
  return std::string(OCTO_PLANNER3D_SOURCE_DIR) + "/octomap/pcd_files/building2_9.pcd";
#else
  return "../octomap/pcd_files/building2_9.pcd";
#endif
}

// 场景缓存默认目录: 与 PCD 同源(包内 octomap/cache), 绝对路径, 跨 cwd 稳定。
std::string defaultSceneCacheDir()
{
#ifdef OCTO_PLANNER3D_SOURCE_DIR
  return std::string(OCTO_PLANNER3D_SOURCE_DIR) + "/octomap/cache";
#else
  return "../octomap/cache";
#endif
}

// 取文件大小+最后修改时间(秒, since epoch)用作缓存失效判据。Linux only。
bool fileIdentity(const std::string & path, std::uint64_t & size, std::uint64_t & mtime)
{
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return false;
  }
  size = static_cast<std::uint64_t>(st.st_size);
  mtime = static_cast<std::uint64_t>(st.st_mtime);
  return true;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

global_planner::PointPose toPlannerPoint(
  const geometry_msgs::msg::Pose & pose,
  double fallback_z)
{
  global_planner::PointPose point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = std::abs(pose.position.z) > kZeroZThreshold ? pose.position.z : fallback_z;
  return point;
}

global_planner::PointPose toPlannerPoint(const geometry_msgs::msg::Point & point_msg)
{
  global_planner::PointPose point;
  point.x = point_msg.x;
  point.y = point_msg.y;
  point.z = point_msg.z;
  return point;
}

// 打印一组耗时样本的统计: 全部均值 / 丢掉第1次(冷启动)的均值 / 中位数 / 最小 / 最大
void printBenchStats(
  const rclcpp::Logger & logger,
  const std::string & tag,
  const std::vector<double> & samples)
{
  const std::size_t n = samples.size();
  if (n == 0) {
    return;
  }
  double sum_all = 0.0;
  for (const double v : samples) {
    sum_all += v;
  }
  const double mean_all = sum_all / static_cast<double>(n);

  double mean_discard = mean_all;
  if (n > 1) {
    double sum_disc = 0.0;
    for (std::size_t i = 1; i < n; ++i) {  // 跳过第1次(冷启动)
      sum_disc += samples[i];
    }
    mean_discard = sum_disc / static_cast<double>(n - 1);
  }

  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const double mn = sorted.front();
  const double mx = sorted.back();
  const double median = (n % 2 == 1)
    ? sorted[n / 2]
    : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);

  RCLCPP_INFO(
    logger,
    "%s summary (N=%zu): mean(all)=%.3f | mean(丢第1次)=%.3f | median=%.3f | "
    "min=%.3f | max=%.3f ms",
    tag.c_str(), n, mean_all, mean_discard, median, mn, mx);
}

}  // namespace

class OctoPlannerRvizNode : public rclcpp::Node
{
public:
  OctoPlannerRvizNode()
  : Node("octo_planner_rviz_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    start_z_ = declare_parameter<double>("start_z", 0.3);
    goal_z_ = declare_parameter<double>("goal_z", 0.3);
    map_alpha_ = declare_parameter<double>("map_alpha", 1.0);
    map_color_mode_ = declare_parameter<std::string>("map_color_mode", "height");
    const std::string clicked_point_topic =
      declare_parameter<std::string>("clicked_point_topic", "clicked_point");
    const std::string input_pcd = declare_parameter<std::string>("input_pcd", defaultInputPcd());
    const std::string output_bt = declare_parameter<std::string>("output_bt", "result_cleaned.bt");
    const double map_publish_period =
      declare_parameter<double>("map_publish_period", 2.0);
    const int benchmark_runs = declare_parameter<int>("benchmark_runs", 0);

    const bool strict_direct_ground_support =
      declare_parameter<bool>("strict_direct_ground_support", false);

    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd);
    converter_->setOutputBtFile(output_bt);
    planner_ = std::make_shared<global_planner::GlobalPlanner>();
    planner_->setStrictDirectGroundSupport(strict_direct_ground_support);

    // ===== 地图构建开关 =====
    // rebuild_map=true(ON): PCD->OctoMap(Tp) + 场景评估(Te) + 落盘 .bt/.scenecache
    // rebuild_map=false(OFF): 直接读 .bt + .scenecache, 跳过 Tp/Te; 读失败/过期自动回退 ON
    const bool rebuild_map = declare_parameter<bool>("rebuild_map", true);
    const std::string scene_cache_dir =
      declare_parameter<std::string>("scene_cache_dir", defaultSceneCacheDir());

    // 缓存路径按 PCD basename 派生; manifest 用 PCD 绝对路径+大小+mtime 做失效校验
    const std::filesystem::path abs_pcd = std::filesystem::absolute(input_pcd);
    const std::string stem = abs_pcd.stem().string();
    const std::string cache_bt = scene_cache_dir + "/" + stem + ".bt";
    const std::string cache_scene = scene_cache_dir + "/" + stem + ".scenecache";

    std::uint64_t pcd_size = 0;
    std::uint64_t pcd_mtime = 0;
    if (!fileIdentity(abs_pcd.string(), pcd_size, pcd_mtime)) {
      RCLCPP_WARN(
        get_logger(),
        "Cannot stat input PCD '%s'; scene cache will not self-invalidate on PCD change.",
        abs_pcd.string().c_str());
    }
    global_planner::SceneCacheMeta meta;
    meta.format_version = global_planner::kSceneCacheFormatVersion;
    meta.pcd_abs_path = abs_pcd.string();
    meta.pcd_size = pcd_size;
    meta.pcd_mtime = pcd_mtime;
    meta.strict_direct_ground_support = strict_direct_ground_support;

    // ON 路径: Tp + Te + 落盘。失败返回 false(调用方据此 return, 留住节点供诊断)。
    auto build_and_save = [&]() -> bool {
      RCLCPP_INFO(get_logger(), "Building OctoMap from configured PCD file...");
      std::vector<double> tp_samples;
      const int tp_runs = benchmark_runs > 0 ? benchmark_runs : 1;
      for (int i = 0; i < tp_runs; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        if (!converter_->convert()) {
          RCLCPP_ERROR(get_logger(), "Failed to build OctoMap. Node will stay alive for diagnostics.");
          return false;
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (benchmark_runs > 0) {
          RCLCPP_INFO(get_logger(), "[BENCH] Tp run %d/%d = %.3f ms", i + 1, tp_runs, ms);
          tp_samples.push_back(ms);
        } else {
          RCLCPP_INFO(get_logger(), "[BENCH] Tp (预处理: PCD->OctoMap) = %.3f ms", ms);
        }
      }
      if (benchmark_runs > 0) {
        printBenchStats(get_logger(), "[BENCH] Tp (预处理)", tp_samples);
      }

      octree_ = converter_->getOctomap();
      // setOctomap 内部会触发 [BENCH] Te (由规划器打印)
      planner_->setOctomap(octree_);

      if (benchmark_runs > 0) {
        RCLCPP_INFO(get_logger(), "Running Te benchmark x%d ...", benchmark_runs);
        planner_->benchmarkTe(benchmark_runs);
      }

      // 落盘供下次 OFF 模式直接读取(失败只 WARN, 不影响本次规划)
      std::error_code dirc;
      std::filesystem::create_directories(scene_cache_dir, dirc);
      if (dirc) {
        RCLCPP_WARN(
          get_logger(), "Cannot create cache dir '%s': %s. Scene cache will not be saved.",
          scene_cache_dir.c_str(), dirc.message().c_str());
      } else if (!octree_->writeBinary(cache_bt)) {
        RCLCPP_WARN(get_logger(), "Scene cache: failed to write %s", cache_bt.c_str());
      } else if (!planner_->saveSceneCache(cache_scene, meta)) {
        RCLCPP_WARN(get_logger(), "Scene cache: failed to write %s", cache_scene.c_str());
      }
      return true;
    };

    bool ready = false;
    if (rebuild_map) {
      ready = build_and_save();
    } else {
      // OFF 路径: 直接读 .bt + .scenecache, 跳过 Tp/Te
      RCLCPP_INFO(get_logger(), "rebuild_map=false: loading cached map + scene evaluation...");
      const auto t_load0 = std::chrono::steady_clock::now();
      const bool bt_ok = converter_->loadFromBtFile(cache_bt);
      bool scene_ok = false;
      if (bt_ok) {
        octree_ = converter_->getOctomap();
        scene_ok = planner_->loadSceneCache(cache_scene, meta, octree_);
      }
      const auto t_load1 = std::chrono::steady_clock::now();
      if (bt_ok && scene_ok) {
        ready = true;
        const double ms = std::chrono::duration<double, std::milli>(t_load1 - t_load0).count();
        RCLCPP_INFO(get_logger(), "[BENCH] Load (read .bt + .scenecache) = %.3f ms", ms);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Scene cache load failed (bt_ok=%d scene_ok=%d); falling back to rebuild.",
          bt_ok, scene_ok);
        ready = build_and_save();
      }
    }

    if (!ready) {
      return;  // build_and_save 已报错; 留住节点供诊断(与旧行为一致)
    }

    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("occupied_map", transient_qos);
    traversable_map_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("traversable_map", transient_qos);
    stair_map_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("stair_map", transient_qos);
    stair_endpoints_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("stair_endpoints", transient_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", transient_qos);
    // 平滑路径话题: 与 /planned_path 相同的 nav_msgs/Path 类型, 内容直接取 planner
    // 当前选中的平滑方法(getPlannerResults)的输出, 不引入新的平滑逻辑。
    smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>("smoothed_path", transient_qos);
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", transient_qos);
    smoothed_path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("smoothed_path_marker", transient_qos);
    start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("start_marker", transient_qos);
    goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("goal_marker", transient_qos);

    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onStartPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onGoalPose, this, std::placeholders::_1));
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onClickedPoint, this, std::placeholders::_1));

    publishAllMaps();
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, map_publish_period))),
      std::bind(&OctoPlannerRvizNode::publishAllMaps, this));

    RCLCPP_INFO(
      get_logger(),
      "Ready. Use RViz2 Publish Point on %s: first click sets start, second click sets goal.",
      clicked_point_topic.c_str());
  }

private:
  void onStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = toPlannerPoint(msg->pose.pose, start_z_);
    has_start_ = true;
    publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Start set to [%.3f, %.3f, %.3f]",
      start_.x,
      start_.y,
      start_.z);
    planIfReady();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = toPlannerPoint(msg->pose, goal_z_);
    has_goal_ = true;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal set to [%.3f, %.3f, %.3f]",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady();
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (next_clicked_point_is_start_) {
      start_ = toPlannerPoint(msg->point);
      has_start_ = true;
      has_goal_ = false;
      next_clicked_point_is_start_ = false;
      publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
      RCLCPP_INFO(
        get_logger(),
        "Start point set to [%.3f, %.3f, %.3f]. Publish the next point as goal.",
        start_.x,
        start_.y,
        start_.z);
      return;
    }

    goal_ = toPlannerPoint(msg->point);
    has_goal_ = true;
    next_clicked_point_is_start_ = true;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal point set to [%.3f, %.3f, %.3f]. Planning with clicked start and goal.",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady();
  }

  void planIfReady()
  {
    if (!planner_ || !octree_ || !has_start_ || !has_goal_) {
      return;
    }

    planner_->makePlan(start_, goal_);

    // 用吸附后的实际位置重发 start/goal marker(机器人真正所在的自由格, 在占据表面上方, 不再嵌进体素)
    global_planner::PointPose snapped_start, snapped_goal;
    if (planner_->getSnappedStart(snapped_start)) {
      publishPoseMarker(
        snapped_start, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
    }
    if (planner_->getSnappedGoal(snapped_goal)) {
      publishPoseMarker(
        snapped_goal, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    }

    // path = 平滑后路径(planner 当前选中方法); raw_path = 平滑前的原始A*路径
    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    std::vector<global_planner::PointPose> raw_path;
    planner_->getRawPlannerResults(raw_path);

    // /planned_path = 原始A*路径; /smoothed_path = 平滑后路径(两者均为 nav_msgs/Path)
    publishPathMsg(raw_path, path_pub_);
    publishPathMsg(path, smoothed_path_pub_);

    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "Planner returned an empty (smoothed) path.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Published paths: /planned_path (raw A*) = %zu poses, /smoothed_path = %zu poses.",
        raw_path.size(),
        path.size());
    }

    // 可视化 marker: 与 Path 话题一一对应——
    //   /planned_path_marker      = 原始A*(橙色细线, 对应 /planned_path)
    //   /smoothed_path_marker     = 平滑后(深紫色粗线, 对应 /smoothed_path)
    publishPathMarker(
      raw_path, "planned_path_marker", 0.06F, makeColor(0.95F, 0.55F, 0.10F, 0.8F),
      path_marker_pub_);
    publishPathMarker(
      path, "smoothed_path_marker", 0.18F, makeColor(0.32F, 0.16F, 0.62F, 1.0F),
      smoothed_path_marker_pub_);
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_) {
      return;
    }

    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const double z_range = std::max(1.0e-6, max_z - min_z);
    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));

    std::unordered_map<double, visualization_msgs::msg::Marker> markers_by_size;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }

      const double size = it.getSize();
      auto marker_it = markers_by_size.find(size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        // 全部体素统一暗色
        // marker.color = makeColor(0.35F, 0.18F, 0.06F, alpha);
        marker.color = makeColor(0.45F, 0.22F, 0.06F, alpha);
        marker_it = markers_by_size.emplace(size, std::move(marker)).first;
      }

      marker_it->second.points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
    }

    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (auto & entry : markers_by_size) {
      auto & marker = entry.second;
      marker.header.stamp = now();
      marker.id = id++;
      array.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.insert(array.markers.begin(), cleanup);

    map_pub_->publish(array);
  }

  void publishTraversableMap()
  {
    if (!traversable_map_pub_ || !planner_) {
      return;
    }

    std::vector<global_planner::PointPose> cells;
    planner_->getTraversableCells(cells);

    visualization_msgs::msg::MarkerArray array;

    // 先清理上一帧的可通行体素, 再发布当前帧(与 occupied_map 相同的清理方式)
    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "traversable_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    if (!cells.empty()) {
      const double resolution = octree_ ? octree_->getResolution() : 0.1;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "traversable_voxels";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = resolution;
      marker.scale.y = resolution;
      marker.scale.z = resolution;
      // 完全不透明(alpha = 1.0); 绿色表示可通行区域
      marker.color = makeColor(0.20F, 0.95F, 0.55F, 1.0F);
      marker.points.reserve(cells.size());
      for (const auto & c : cells) {
        marker.points.push_back(makePoint(c.x, c.y, c.z));
      }
      array.markers.push_back(marker);
    }

    traversable_map_pub_->publish(array);
  }

  // 段调色板: 按段 id 循环取色, 让相邻段颜色差异明显。
  // 用于观察"同一段长楼梯是否被误判成多段"——同段应同色, 若一条长楼梯出现多种颜色,
  // 说明连通域把它拆成了多段(可能需要放宽连通条件)。
  std_msgs::msg::ColorRGBA segmentColor(std::size_t index) const
  {
    static const float palette[10][3] = {
      {1.00F, 0.55F, 0.00F},   // 橙
      {0.20F, 0.80F, 0.95F},   // 青
      {0.95F, 0.30F, 0.75F},   // 品红
      {0.95F, 0.90F, 0.20F},   // 黄
      {0.40F, 0.85F, 0.40F},   // 绿
      {0.60F, 0.40F, 0.95F},   // 紫
      {0.95F, 0.45F, 0.30F},   // 橘红
      {0.30F, 0.65F, 0.95F},   // 蓝
      {0.95F, 0.60F, 0.85F},   // 粉
      {0.70F, 0.95F, 0.40F},   // 黄绿
    };
    const float * c = palette[index % 10];
    return makeColor(c[0], c[1], c[2], 0.85F);
  }

  void publishStairMap()
  {
    if (!stair_map_pub_ || !planner_) {
      return;
    }

    // 按段分组取格, 每段单独一个 marker 着不同颜色
    std::vector<std::vector<global_planner::PointPose>> segs_cells;
    planner_->getStairCellsBySegment(segs_cells);
    std::vector<global_planner::StairSegment> segs;
    planner_->getStairSegments(segs);

    visualization_msgs::msg::MarkerArray array;

    // 先清理上一帧的楼梯 marker
    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "stair_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    const double resolution = octree_ ? octree_->getResolution() : 0.2;

    // 每段楼梯一个 CUBE_LIST marker, 用不同颜色
    for (std::size_t i = 0; i < segs_cells.size(); ++i) {
      if (segs_cells[i].empty()) { continue; }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "stair_cells";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = resolution;
      marker.scale.y = resolution;
      marker.scale.z = resolution;
      marker.color = segmentColor(i);
      marker.points.reserve(segs_cells[i].size());
      for (const auto & c : segs_cells[i]) {
        marker.points.push_back(makePoint(c.x, c.y, c.z));
      }
      array.markers.push_back(marker);
    }

    // 走向箭头: 每段一个, 颜色与该段踏面块一致(便于对应), alpha=1.0 更醒目
    const double arrow_len = 1.5;
    for (std::size_t i = 0; i < segs.size(); ++i) {
      const auto & s = segs[i];
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = now();
      arrow.ns = "stair_dir";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      arrow.scale.x = 0.08;  // 杆直径
      arrow.scale.y = 0.18;  // 箭头头直径
      arrow.color = segmentColor(i);
      arrow.color.a = 1.0F;
      arrow.points.push_back(makePoint(s.center.x, s.center.y, s.center.z));
      arrow.points.push_back(makePoint(
        s.center.x + s.dir_x * arrow_len,
        s.center.y + s.dir_y * arrow_len,
        s.center.z));
      array.markers.push_back(arrow);
    }

    stair_map_pub_->publish(array);
  }

  // 楼梯端点/过渡点(找的点)单独发到 /stair_endpoints, 与楼梯格显示分开, 方便观察。
  // 颜色区分: 绿=整段真实顶(最高端), 红=整段真实底(最低端), 蓝=子段之间的中间过渡点。
  void publishStairEndpoints()
  {
    if (!stair_endpoints_pub_ || !planner_) {
      return;
    }
    std::vector<global_planner::StairSegment> segs;
    planner_->getStairSegments(segs);

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "stair_endpoints_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    if (segs.empty()) {
      stair_endpoints_pub_->publish(array);
      return;
    }

    const double resolution = octree_ ? octree_->getResolution() : 0.2;
    const double end_size = resolution * 2.0;

    // 分类用"邻近"而非精确同格: 一条物理楼梯可能被检测成多个段(弱桥切断/走向切分),
    // 各段独立算中线, 相邻段衔接处的 top/bottom 不是同一格、但彼此相邻(≤~2格)。
    // 故: 一个 top 附近(≤D)若有 bottom -> 不是真实顶, 是中间过渡点; bottom 同理。
    // 真实顶/底 = 附近找不到互补端点(没有继续向上/向下的邻居)。D 取 2.5 格, 介于
    // "衔接处相邻格(≤√3≈1.73)" 与 "同段子段边界间距(stair_segment_layers_=4)" 之间。
    const double D = 2.5 * resolution;
    const double Dsq = D * D;
    std::vector<global_planner::PointPose> tops, bots;
    tops.reserve(segs.size());
    bots.reserve(segs.size());
    for (const auto & s : segs) {
      tops.push_back(s.top_center);
      bots.push_back(s.bottom_center);
    }
    auto has_near = [&](const global_planner::PointPose & p,
                        const std::vector<global_planner::PointPose> & lst) -> bool {
      for (const auto & q : lst) {
        const double dx = p.x - q.x;
        const double dy = p.y - q.y;
        const double dz = p.z - q.z;
        if (dx * dx + dy * dy + dz * dz <= Dsq) {
          return true;
        }
      }
      return false;
    };

    visualization_msgs::msg::Marker top_m;
    top_m.header.frame_id = frame_id_;
    top_m.header.stamp = now();
    top_m.ns = "stair_endpoints_top";
    top_m.id = 0;
    top_m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    top_m.action = visualization_msgs::msg::Marker::ADD;
    top_m.pose.orientation.w = 1.0;
    top_m.scale.x = end_size;
    top_m.scale.y = end_size;
    top_m.scale.z = end_size;
    top_m.color = makeColor(0.10F, 0.95F, 0.20F, 1.0F);   // 绿 = 真实顶(最高端)

    visualization_msgs::msg::Marker bot_m = top_m;
    bot_m.ns = "stair_endpoints_bottom";
    bot_m.color = makeColor(0.95F, 0.15F, 0.15F, 1.0F);   // 红 = 真实底(最低端)

    visualization_msgs::msg::Marker mid_m = top_m;
    mid_m.ns = "stair_endpoints_mid";
    mid_m.color = makeColor(0.10F, 0.45F, 0.95F, 1.0F);   // 蓝 = 子段间中间过渡点

    // 中间点按邻近去重: 衔接处的 top 与 bottom 是两个相邻格, 合并成一个蓝点
    auto add_mid = [&](const global_planner::PointPose & p) {
      for (const auto & q : mid_m.points) {
        const double dx = p.x - q.x;
        const double dy = p.y - q.y;
        const double dz = p.z - q.z;
        if (dx * dx + dy * dy + dz * dz <= Dsq) {
          return;
        }
      }
      mid_m.points.push_back(makePoint(p.x, p.y, p.z));
    };
    for (const auto & s : segs) {
      // top_center: 附近有 bottom -> 中间过渡点; 否则 -> 真实顶
      if (has_near(s.top_center, bots)) {
        add_mid(s.top_center);
      } else {
        top_m.points.push_back(makePoint(s.top_center.x, s.top_center.y, s.top_center.z));
      }
      // bottom_center: 附近有 top -> 中间过渡点; 否则 -> 真实底
      if (has_near(s.bottom_center, tops)) {
        add_mid(s.bottom_center);
      } else {
        bot_m.points.push_back(makePoint(s.bottom_center.x, s.bottom_center.y, s.bottom_center.z));
      }
    }

    array.markers.push_back(top_m);
    array.markers.push_back(bot_m);
    array.markers.push_back(mid_m);

    // 进近走廊末端(橙点) = 每条走廊的平台侧末端(与楼梯端点同样大小), 用于调 stair_endpoint_extend_dist_
    {
      std::vector<global_planner::PointPose> cor;
      planner_->getStairApproachCorridor(cor);
      if (!cor.empty()) {
        visualization_msgs::msg::Marker cor_m = top_m;   // 沿用 top_m 的 scale(=end_size), 与楼梯端点同样大小
        cor_m.ns = "stair_approach_corridor";
        cor_m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        cor_m.color = makeColor(0.95F, 0.60F, 0.10F, 0.85F);   // 橙点 = 进近走廊末端
        cor_m.points.reserve(cor.size());
        for (const auto & p : cor) {
          cor_m.points.push_back(makePoint(p.x, p.y, p.z));
        }
        array.markers.push_back(cor_m);
      }
    }

    stair_endpoints_pub_->publish(array);
  }

  void publishAllMaps()
  {
    publishMap();
    publishTraversableMap();
    publishStairMap();
    publishStairEndpoints();
  }

  std_msgs::msg::ColorRGBA heightColor(double t, float alpha) const
  {
    if (t < 0.33) {
      const float k = static_cast<float>(t / 0.33);
      return makeColor(0.10F, 0.45F + 0.35F * k, 0.95F - 0.25F * k, alpha);
    }
    if (t < 0.66) {
      const float k = static_cast<float>((t - 0.33) / 0.33);
      return makeColor(0.10F + 0.85F * k, 0.80F + 0.10F * k, 0.70F - 0.55F * k, alpha);
    }
    const float k = static_cast<float>((t - 0.66) / 0.34);
    return makeColor(0.95F, 0.90F - 0.45F * k, 0.15F + 0.05F * k, alpha);
  }

  // 把一条路径(世界坐标点序列)打包成 nav_msgs/Path 并发布到指定 publisher。
  void publishPathMsg(
    const std::vector<global_planner::PointPose> & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher)
  {
    if (!publisher) {
      return;
    }

    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(point.x, point.y, point.z);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    publisher->publish(msg);
  }

  void publishPathMarker(
    const std::vector<global_planner::PointPose> & path,
    const std::string & ns,
    float width,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    if (!publisher) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = path.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;   // 线宽
    marker.color = color;

    marker.points.reserve(path.size());

    for (const auto & point : path) {
      marker.points.push_back(makePoint(point.x, point.y, point.z));
    }

    publisher->publish(marker);
  }

  void publishPoseMarker(
    const global_planner::PointPose & pose,
    const std::string & ns,
    int id,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    if (!publisher) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(pose.x, pose.y, pose.z);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color = color;
    publisher->publish(marker);
  }

  std::string frame_id_;
  double start_z_ = 0.3;
  double goal_z_ = 0.3;
  double map_alpha_ = 1.0;
  std::string map_color_mode_ = "height";
  bool has_start_ = false;
  bool has_goal_ = false;
  bool next_clicked_point_is_start_ = true;

  global_planner::PointPose start_;
  global_planner::PointPose goal_;
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traversable_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stair_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stair_endpoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr smoothed_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerRvizNode>());
  rclcpp::shutdown();
  return 0;
}
