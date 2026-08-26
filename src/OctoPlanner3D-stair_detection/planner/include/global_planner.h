/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>

#include "octomap/OcTree.h"

namespace global_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct PointPose
{
    double x;
    double y;
    double z;
};

// 楼梯段信息: 用于可视化与诊断
struct StairSegment
{
    PointPose center;        // 段中心(世界坐标)
    double dir_x = 0.0;      // 走向单位向量(水平分量)
    double dir_y = 0.0;
    int layer_count = 0;     // 台阶级数
    PointPose top_center;    // 楼梯顶端(高z端)踏面中点, 接上平台
    PointPose bottom_center; // 楼梯底端(低z端)踏面中点, 接下平台
};

// 场景评估(Te)缓存的元信息: 写入 .scenecache 头部, 加载时与当前期望逐字段比对,
// 任一不匹配即视为过期(回退到重新评估)。改任何 Te 相关代码/参数时,
// 必须同步 bump 下面的 kSceneCacheFormatVersion。
inline constexpr std::uint32_t kSceneCacheFormatVersion = 4;

struct SceneCacheMeta
{
    std::uint32_t format_version = 0;  // 与 kSceneCacheFormatVersion 对应
    std::string pcd_abs_path;          // 源 PCD 绝对路径
    std::uint64_t pcd_size = 0;        // 源 PCD 文件大小(字节)
    std::uint64_t pcd_mtime = 0;       // 源 PCD 修改时间(秒, since epoch)
    bool strict_direct_ground_support = false;  // 唯一影响 Te 的运行期开关
};

// 路径平滑方法
enum class PathSmoothingMethod
{
  MovingAverage = 0,       // 局部移动平均 + 逐点碰撞回退(默认: 稳定, 不鼓包/不回环)
  ClampedCubicBspline = 1  // 捷径剪枝 + 钳位三次B样条 + 碰撞回退(更丝滑, 但尖角处可能过冲/偏离贴地路径)
};

class GlobalPlanner
{
public:
  GlobalPlanner();
  
  ~GlobalPlanner();

  void setOctomap(std::shared_ptr<octomap::OcTree> map);

  // 严格直接地面支撑: true=只看正下方一格; false=下方 support_xy_radius×depth 范围内有占据即可
  // 必须在 setOctomap() 之前调用, 否则场景评估(Te)不会使用新值。
  void setStrictDirectGroundSupport(bool v) { strict_direct_ground_support_ = v; }

  // 路径平滑总开关: true=A*规划后做平滑后处理; false=输出原始A*路径。
  void setEnablePathSmoothing(bool v) { enable_path_smoothing_ = v; }

  // 路径平滑方法选择: MovingAverage(默认, 稳定) 或 ClampedCubicBspline(更丝滑但尖角处可能过冲)。
  void setPathSmoothingMethod(PathSmoothingMethod m) { path_smoothing_method_ = m; }

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  // 获取平滑前的原始A*路径(世界坐标)。若关闭平滑或未规划, 内容与 getPlannerResults 相同。
  void getRawPlannerResults(std::vector<PointPose>& out) const;

  // 获取场景评估得到的全部可通行格(世界坐标)。setOctomap 后可用,未构建返回空。
  void getTraversableCells(std::vector<PointPose>& out) const;

  // 获取检测到的楼梯踏面格(世界坐标)。用于 RViz 可视化。
  void getStairCells(std::vector<PointPose>& out) const;

  // 获取检测到的楼梯段(中心+走向向量+级数)。用于 RViz 可视化与诊断。
  void getStairSegments(std::vector<StairSegment>& out) const;

  // 获取进近走廊末端(世界坐标): 每条走廊的平台侧末端(tip), 走向约束在其所在走廊全程生效。用于 RViz 可视化。
  void getStairApproachCorridor(std::vector<PointPose>& out) const;

  // 获取按段分组的楼梯踏面格: out[段id] = 该段所有格(世界坐标)。用于按段着色可视化。
  void getStairCellsBySegment(std::vector<std::vector<PointPose>>& out) const;

  // 基准测试: 重复运行场景评估(通行度评估+代价地图) N 次, 打印每次耗时与统计
  void benchmarkTe(int runs);

  // 获取最近一次规划中, 起点/终点吸附到的实际可通行格(世界坐标)。未规划过返回false。
  bool getSnappedStart(PointPose & out) const;
  bool getSnappedGoal(PointPose & out) const;

  // ===== 场景评估(Te)缓存: 跳过 Tp/Te, 直接读盘 =====
  // ON 模式跑完 setOctomap(Te) 后调用: 把 8 个派生容器 + meta 头写入 path(.scenecache)。
  bool saveSceneCache(const std::string & path, const SceneCacheMeta & meta) const;
  // OFF 模式调用: 校验 meta 头与 expected 一致后, 填充 8 个容器, 置 octree_/map_ready_,
  // 不调用三个 rebuild。校验失败/文件缺失/反序列化失败一律返回 false(由调用方回退)。
  bool loadSceneCache(
    const std::string & path,
    const SceneCacheMeta & expected,
    std::shared_ptr<octomap::OcTree> map);

private:

  void fillBounds(PointPose & min_bound,PointPose & max_bound) const;

  void onGoalPose(const PointPose goal);

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool isOccupiedCell(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void rebuildPreblockedCells();

//   void onExternalPreblockedMarker(const visualization_msgs::msg::Marker::SharedPtr msg);

  void rebuildPreblockedCostmap();

  double getPreblockedCost(const GridIndex & idx) const;

//   void publishCellSetMarker(
//     const std::unordered_set<GridIndex, GridIndexHash> & cells,
//     const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
//     const std::string & ns,
//     float r_color,
//     float g_color,
//     float b_color,
//     float a_color) const;

  void publishPreblockedCellsMarker();

  void publishRiskCostCloud() const;

  void rebuildDerivedLayers();

  // 楼梯检测: 标记楼梯踏面格 + 连通域分组 + 平面拟合估走向(任意角度)。
  void rebuildStairSegments();

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    GridIndex & out) const;

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  // ====== 路径平滑 ======
  // 对原始A*世界坐标路径做后处理, 返回平滑后的路径。要求 octree_ 已设置。
  // 平滑调度: 按 path_smoothing_method_ 选择下方两种方法之一。
  // 不强制地面支撑、不保护楼梯段(可横向平滑/穿过楼梯); 安全性由碰撞回退保证。
  std::vector<PointPose> smoothPath(const std::vector<PointPose> & raw) const;

  // [方法1] 局部移动平均 + 逐点碰撞回退(默认, 稳定: 不鼓包/不回环/不走两遍)。
  std::vector<PointPose> smoothPathMovingAverage(const std::vector<PointPose> & raw) const;

  // [方法2] 捷径剪枝 + 钳位三次B样条 + 碰撞回退(曲线更丝滑, 但尖角处可能过冲/偏离贴地路径)。
  std::vector<PointPose> smoothPathBspline(const std::vector<PointPose> & raw) const;
  std::vector<PointPose> shortcutPath(const std::vector<PointPose> & raw) const;
  std::vector<PointPose> sampleClampedCubicBspline(const std::vector<PointPose> & pts) const;
  std::vector<PointPose> assembleCollisionAware(
    const std::vector<PointPose> & smooth,
    const std::vector<PointPose> & pruned) const;

  // 线段 a->b 是否无碰撞: 以体素分辨率为步长逐点 isCellTraversable(不要求地面支撑)。
  // extra_radius: 额外膨胀半径, B样条管线用它给曲线留圆角余量(>0 时路径离墙更远)。
  bool isLineCollisionFree(const PointPose & a, const PointPose & b, double extra_radius = 0.0) const;

  bool startPlan();

  void publishPath(
    const std::vector<GridIndex> & cells,
    const std::string & frame_id);

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }


private:
 
  std::string source_world_file_;

  double robot_radius_ = 0.25;                  // 机器人半径
  int max_iterations_ = 800000;                 // 最大迭代次数
  int snap_search_radius_cells_ = 12;           //_SNAP搜索半径（栅格数）
  bool require_ground_support_ = true;          // 是否需要地面支撑
  bool strict_direct_ground_support_ = false;   // 严格直接地面支撑
  int ground_support_xy_radius_cells_ = 1;      // 地面支撑XY半径（栅格数）
  int ground_support_depth_cells_ = 1;          // 地面支撑深度（栅格数）
  bool enable_preblocked_costmap_ = true;       // 是否启用预禁行代价地图
  int preblocked_costmap_radius_cells_ = 3;     // 预禁行代价地图半径（栅格数）
  double preblocked_costmap_weight_ = 2.5;      // 预禁行代价地图权重
  bool lowest_traversable_only_ = false;        // 是否只考虑最低可通行路径

  // ====== 路径平滑参数 ======
  bool enable_path_smoothing_ = true;           // A*规划后是否做平滑后处理
  PathSmoothingMethod path_smoothing_method_ = PathSmoothingMethod::MovingAverage;  // 平滑方法选择 ClampedCubicBspline MovingAverage
  int smoothing_window_ = 2;                    // [移动平均] 单侧邻居数(窗口=2w+1); 越大越平滑、越偏离原始
  int smoothing_iterations_ = 3;                // [移动平均] 迭代轮数; 越大越平滑
  int bspline_samples_per_segment_ = 5;         // [B样条] 每段(相邻控制点间)采样数; 越大曲线越密
  double bspline_clearance_ = 0.1;              // [B样条] 碰撞检测额外膨胀半径; 给曲线留圆角余量, 避免弯道处被碰撞回退拉直成折线; 0=不膨胀
  int bspline_shortcut_max_jump_ = 8;           // [B样条] 捷径剪枝单次最大跳跃点数; 0=不限制(最远直达, 可能过度抽稀致控制点<4退化为直线); >0 限制跳跃保留更多控制点

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool has_goal_pose_ = false;
  bool planning_in_progress_ = false;

  std::uint64_t plan_seq_ = 0;
  std::uint64_t last_success_seq_ = 0;
  std::uint64_t last_octomap_hash_ = 0;

  PointPose start_point_;
  PointPose goal_point_;
  PointPose goal_pose_;

  // 最近一次规划中吸附到的实际起点/终点(世界坐标)
  PointPose snapped_start_;
  PointPose snapped_goal_;
  bool has_snapped_ = false;

  std::vector<PointPose> planner_results_;

  // 平滑前的原始A*路径(世界坐标), 供 RViz 对比显示。
  std::vector<PointPose> raw_planner_results_;

  std::shared_ptr<octomap::OcTree> octree_;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;

  // 楼梯检测结果: stair_segments_ 为检测到的各段(走向/中心/级数),
  // stair_cell_seg_ 把每个楼梯踏面格映射到所属段 id (A* 邻居扩展时 O(1) 查走向)。
  std::vector<StairSegment> stair_segments_;
  std::unordered_map<GridIndex, int, GridIndexHash> stair_cell_seg_;
  bool enable_stair_diagonal_block_ = true;  // 走向约束开关: 楼梯格 + 进近走廊格上, 只允许沿楼梯走向移动(禁止斜切/横移); 默认开
  bool enable_stair_endpoint_traverse_ = true; // 新: 楼梯巡线开关——楼梯只能从端点(顶/底中点)进/出
  int stair_min_layers_ = 5;                   // 至少这么多级离散高度才算楼梯
  double stair_slope_min_ = 0.1;               // 平面拟合坡度下限(栅格空间), 过滤平地
  double stair_perp_tolerance_ = 0.7;          // 归一化横向分量阈值(=sin夹角); 需>sin22.5°≈0.383
  std::unordered_set<GridIndex, GridIndexHash> stair_endpoints_;       // 各段顶/底端点区域(精确点 + 扩展邻居), A*巡线过渡检查用
  std::unordered_set<GridIndex, GridIndexHash> stair_exact_endpoints_; // 各段顶/底精确端点(仅中线点, 扩展前快照)
  std::unordered_map<GridIndex, double, GridIndexHash> stair_endpoint_deviation_cost_; // 扩展端点(非精确)的偏离代价, 距离精确端点越现代价越高
  // 进近走廊末端: 真实端点(顶/底)沿 ±dir 向外延伸的平台格——全程写入 stair_cell_seg_(当楼梯格
  // 一样规划, A* 走向约束自动生效), 末端并入 stair_endpoints_(过渡门放行); 这里只存每条走廊的
  // 平台侧末端(tip)供可视化(橙色点)。迫使路径沿走向直入楼梯口, 防止侧面斜切(内切)。
  std::unordered_set<GridIndex, GridIndexHash> stair_corridor_cells_;
  bool enable_stair_endpoint_deviation_ = true;  // 端点偏离代价开关: 给非中线端点加代价, 引导A*优选中线入口
  double stair_endpoint_deviation_weight_ = 0; // 端点偏离代价权重; 越大越倾向中线, 0=无偏好; 建议1~20
  double stair_endpoint_reward_weight_ = 5;    // 经过精确端点(中线点)的奖励(减分); 让"经P"比"绕过P"便宜2*reward, 克服P偏离最短路的距离差; 越大越倾向走中线端点, 过大会强行绕远去经P
  int stair_segment_layers_ = 3;               // 长楼梯按 z 切子段, 每子段最多这么多级(端点约束逐子段生效, 避免长距离斜切)
  double stair_endpoint_extend_dist_ = 0.6;    // 进近走廊长度(米): 真实端点沿 ±dir 向外延伸距离, 把端点外平台格纳入走向约束, 阻止侧向斜切入楼梯(内切); 0=关闭走廊; 默认0.2(ON), 内部按 round(距离/分辨率) 转格数
  int stair_bridge_min_support_ = 1;           // 弱桥切断: 26连通分组时相邻候选格的"共同候选邻居数"下限; 上一段顶端↔下一段底端的端对端弱桥(无平台)共同邻居=0→切断, 防止两段反向楼梯被焊成一段; 正常楼梯(踏面宽>=2)走向/宽度连接都有侧翼格提供共同邻居不受影响; 0=关闭切断
  int stair_bridge_degree_pardon_ = 2;         // 弱桥切断的度数兜底: 共同邻居不足时, 若相邻两端候选邻居数min<=此值则豁免(保护踏面宽=1的单格楼梯不被误切); 弱桥两端在宽>=2段内度数通常>=3>pardon仍被切断; 0=关闭兜底
  bool enable_stair_direction_split_ = true;   // 走向一致性切分(第2.5步): 按局部上升方向 up(c) 投影符号, 把经平台/侧翼(common>=1, K盲区)相连的两段反向楼梯拆开; 正常单段楼梯所有格 up 同向→不切; false=关闭(退回仅弱桥切断)
  bool enable_stair_merge_ = true;             // 合并同向相邻段(第2.6步): 把"格子26相邻 + 方向同向 + z上下相接"的段并成一组, 还原被弱桥/走向切分误切的一条直楼梯; 之后第3步按组统一拟合 dir/line_w, 使整条楼梯的过渡点共线; false=各段各算(接缝处过渡点可能不共线)
  double stair_merge_dir_threshold_ = 0.9;     // 同向判定阈值: 两段 dir 点积 > 此值才合并(0.9≈夹角<25°); 越小越易合并, 过小会并掉略有转折的楼梯
  bool enable_stair_path_stabilize_ = false;   // 楼梯巡线: 楼梯段以最小偏差(贴合走向)为目标, 距离退为次要
  double stair_stabilize_weight_ = 10.0;       // 楼梯段偏离走向的代价权重; 越大越贴合走向(最小偏差), 0=纯最短; 建议1~100, 过大(如1e8)会淹没距离tie-break, perp相同时路径反而不稳定

  // [BENCH] 仅用于累计 isCellTraversable 各段耗时(场景评估时)
  mutable double bounds_check_ms_ = 0.0;       // ① isInsideMetricBounds
  mutable double ground_support_ms_ = 0.0;     // ② hasGroundSupport
  mutable double column_check_ms_ = 0.0;       // ③ 下方柱体 preblocked
  mutable double footprint_check_ms_ = 0.0;    // ④ 足迹碰撞三重循环
};

}  // namespace global_planner
