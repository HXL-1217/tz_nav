#include "global_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <unordered_set>

namespace
{
// RAII 计时器: 构造开始计时, 析构时打印耗时(ms)。用于 Te/Ts 分段计时
// (即使 A* 在循环内 return true 也能正确打印)
struct ScopedBenchTimer
{
    const char * tag;
    std::chrono::steady_clock::time_point start;
    explicit ScopedBenchTimer(const char * t)
    : tag(t), start(std::chrono::steady_clock::now()) {}
    ~ScopedBenchTimer()
    {
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start)
                              .count();
        printf("%s = %.3f ms\n", tag, ms);
    }
};

// [BENCH] RAII 累加计时器: 析构时把耗时累加到外部 double (用于累计足迹循环耗时)
// (即使循环内 return false 也能正确累加)
struct FootprintTimerAccum
{
    double & accum;
    std::chrono::steady_clock::time_point start;
    explicit FootprintTimerAccum(double & a)
    : accum(a), start(std::chrono::steady_clock::now()) {}
    ~FootprintTimerAccum()
    {
        accum += std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - start)
                     .count();
    }
};

// 打印一组耗时样本的统计: 全部均值 / 丢掉第1次(冷启动)的均值 / 中位数 / 最小 / 最大
void printBenchStats(const char * tag, const std::vector<double> & samples)
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

    printf("%s summary (N=%zu): mean(all)=%.3f | mean(丢第1次)=%.3f | median=%.3f | "
           "min=%.3f | max=%.3f ms\n",
           tag, n, mean_all, mean_discard, median, mn, mx);
}

// ===== 场景评估(Te)缓存: .scenecache 二进制读写工具 =====
// 小端二进制, 仅本机/本构建使用。改任何 Te 相关代码/参数时 bump kSceneCacheFormatVersion,
// 否则 OFF 模式会读到过期结果。
constexpr char kSceneCacheMagic[8] = {'O', 'P', '3', 'D', 'S', 'C', '1', '\0'};

inline bool bwU8(std::ofstream & os, std::uint8_t v)
{
    os.write(reinterpret_cast<const char *>(&v), 1);
    return static_cast<bool>(os);
}
inline bool bwU32(std::ofstream & os, std::uint32_t v)
{
    os.write(reinterpret_cast<const char *>(&v), 4);
    return static_cast<bool>(os);
}
inline bool bwU64(std::ofstream & os, std::uint64_t v)
{
    os.write(reinterpret_cast<const char *>(&v), 8);
    return static_cast<bool>(os);
}
inline bool bwI32(std::ofstream & os, std::int32_t v)
{
    os.write(reinterpret_cast<const char *>(&v), 4);
    return static_cast<bool>(os);
}
inline bool bwF64(std::ofstream & os, double v)
{
    os.write(reinterpret_cast<const char *>(&v), 8);
    return static_cast<bool>(os);
}
inline bool bwStr(std::ofstream & os, const std::string & s)
{
    if (!bwU32(os, static_cast<std::uint32_t>(s.size()))) {
        return false;
    }
    os.write(s.data(), static_cast<std::streamsize>(s.size()));
    return static_cast<bool>(os);
}
inline bool bwGrid(std::ofstream & os, int x, int y, int z)
{
    return bwI32(os, x) && bwI32(os, y) && bwI32(os, z);
}
inline bool bwPoint(std::ofstream & os, double x, double y, double z)
{
    return bwF64(os, x) && bwF64(os, y) && bwF64(os, z);
}

inline bool brU8(std::ifstream & is, std::uint8_t & v)
{
    is.read(reinterpret_cast<char *>(&v), 1);
    return static_cast<std::size_t>(is.gcount()) == 1;
}
inline bool brU32(std::ifstream & is, std::uint32_t & v)
{
    is.read(reinterpret_cast<char *>(&v), 4);
    return static_cast<std::size_t>(is.gcount()) == 4;
}
inline bool brU64(std::ifstream & is, std::uint64_t & v)
{
    is.read(reinterpret_cast<char *>(&v), 8);
    return static_cast<std::size_t>(is.gcount()) == 8;
}
inline bool brI32(std::ifstream & is, std::int32_t & v)
{
    is.read(reinterpret_cast<char *>(&v), 4);
    return static_cast<std::size_t>(is.gcount()) == 4;
}
inline bool brF64(std::ifstream & is, double & v)
{
    is.read(reinterpret_cast<char *>(&v), 8);
    return static_cast<std::size_t>(is.gcount()) == 8;
}
inline bool brStr(std::ifstream & is, std::string & s)
{
    std::uint32_t len = 0;
    if (!brU32(is, len)) {
        return false;
    }
    s.assign(len, '\0');
    if (len > 0) {
        is.read(&s[0], static_cast<std::streamsize>(len));
        if (static_cast<std::size_t>(is.gcount()) != len) {
            return false;
        }
    }
    return true;
}
inline bool brGrid(std::ifstream & is, int & x, int & y, int & z)
{
    return brI32(is, x) && brI32(is, y) && brI32(is, z);
}
inline bool brPoint3(std::ifstream & is, double & x, double & y, double & z)
{
    return brF64(is, x) && brF64(is, y) && brF64(is, z);
}
}  // namespace

namespace global_planner
{

    GlobalPlanner::GlobalPlanner():map_ready_(false),has_start_(false),has_goal_(false),planning_in_progress_(false),plan_seq_(0),last_success_seq_(0)
    {
        printf("GlobalPlanner Constructure!!! \n");
    }

    GlobalPlanner::~GlobalPlanner()
    {
        printf("GlobalPlanner Destructure!!! \n");
    }

    void GlobalPlanner::setOctomap(std::shared_ptr<octomap::OcTree> map)
    {
        if (!map)
        {
            printf("Octomap is Null!!! return.\n");
            return;
        }

        if (octree_ == map)
        {
            printf("Octomap is No Update!!! return.\n");
            return;
        }

        octree_ = map;
        map_ready_ = true;

        // Te: 场景评估 = 被障碍侵占栅格预处理 + 通行度评估 + 代价地图。一次性构建, 后续查询复用。
        // 三个内层 ScopedBenchTimer 分别打印三步耗时; 外层 te_total 打印合计。
        ScopedBenchTimer te_total("[BENCH] Te  (场景评估 total)");
        {
            ScopedBenchTimer t("[BENCH] Te1 (rebuildPreblockedCells)");
            rebuildPreblockedCells();
        }
        {
            ScopedBenchTimer t("[BENCH] Te2 (rebuildDerivedLayers)");
            rebuildDerivedLayers();
        }
        {
            ScopedBenchTimer t("[BENCH] Te3 (rebuildPreblockedCostmap)");
            rebuildPreblockedCostmap();
        }
    }

    void GlobalPlanner::makePlan(const PointPose start,const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        printf("start = (%f,%f,%f),goal = (%f,%f,%f) \n",start_point_.x,start_point_.y,start_point_.z,goal_point_.x,goal_point_.y,goal_point_.z);
         
        tryPlan();

    }

    void GlobalPlanner::tryPlan()
    {
        printf("GlobalPlanner::tryPlan planning...\n");
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) 
        {
            printf("GlobalPlanner::tryPlan 异常退出规划器\n");
            return;
        }
        planning_in_progress_ = true;
        ++plan_seq_;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok) 
        {
            printf("GlobalPlanner::tryPlan() A* planning failed. \n");
        } 
        else 
        {
            last_success_seq_ = plan_seq_;
        }
    }

    void GlobalPlanner::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    void GlobalPlanner::getRawPlannerResults(std::vector<PointPose>& out) const
    {
        out = raw_planner_results_;
    }

    bool GlobalPlanner::getSnappedStart(PointPose & out) const
    {
        if (!has_snapped_) {
            return false;
        }
        out = snapped_start_;
        return true;
    }

    bool GlobalPlanner::getSnappedGoal(PointPose & out) const
    {
        if (!has_snapped_) {
            return false;
        }
        out = snapped_goal_;
        return true;
    }

    void GlobalPlanner::benchmarkTe(int runs)
    {
        if (runs <= 0 || !octree_) {
            printf("[BENCH] Te benchmark skipped (runs<=0 or no map).\n");
            return;
        }
        std::vector<double> s_pre, s_der, s_cost, s_total;
        s_pre.reserve(static_cast<std::size_t>(runs));
        s_der.reserve(static_cast<std::size_t>(runs));
        s_cost.reserve(static_cast<std::size_t>(runs));
        s_total.reserve(static_cast<std::size_t>(runs));
        for (int i = 0; i < runs; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            rebuildPreblockedCells();
            const auto t1 = std::chrono::steady_clock::now();
            rebuildDerivedLayers();
            const auto t2 = std::chrono::steady_clock::now();
            rebuildPreblockedCostmap();
            const auto t3 = std::chrono::steady_clock::now();
            const double ms_pre   = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const double ms_der   = std::chrono::duration<double, std::milli>(t2 - t1).count();
            const double ms_cost  = std::chrono::duration<double, std::milli>(t3 - t2).count();
            const double ms_total = std::chrono::duration<double, std::milli>(t3 - t0).count();
            printf(
                "[BENCH] Te run %d/%d: preblocked=%.3f derived=%.3f costmap=%.3f total=%.3f ms\n",
                i + 1, runs, ms_pre, ms_der, ms_cost, ms_total);
            s_pre.push_back(ms_pre);
            s_der.push_back(ms_der);
            s_cost.push_back(ms_cost);
            s_total.push_back(ms_total);
        }
        printBenchStats("[BENCH] Te1 (rebuildPreblockedCells)", s_pre);
        printBenchStats("[BENCH] Te2 (rebuildDerivedLayers)", s_der);
        printBenchStats("[BENCH] Te3 (rebuildPreblockedCostmap)", s_cost);
        printBenchStats("[BENCH] Te  (total)", s_total);
    }

    bool GlobalPlanner::startPlan()
    {
        // 规划器参数
        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const int snap_radius = snap_search_radius_cells_;
        const bool require_ground_support = require_ground_support_; 
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells =  ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;
        
        // 将起点和终点从世界坐标转换为栅格索引
        const GridIndex start_raw = worldToGrid(
        start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
        goal_point_.x, goal_point_.y, goal_point_.z);

        GridIndex start = start_raw;
        GridIndex goal = goal_raw;
        // 尝试在起点和终点附近找到可通行的栅格索引
        const bool start_ok = findNearestFreeCell(
        start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, start);
        const bool goal_ok = findNearestFreeCell(
        goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, goal);

        if (!start_ok) 
        {
            printf("GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!goal_ok)
        {
            printf("GlobalPlanner::startPlan() Goal is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        // 记录吸附后的实际起点/终点(机器人真正所在的自由格, 世界坐标), 供外部显示
        {
            const auto ps = gridToWorld(start);
            snapped_start_.x = ps.x();
            snapped_start_.y = ps.y();
            snapped_start_.z = ps.z();
            const auto pg = gridToWorld(goal);
            snapped_goal_.x = pg.x();
            snapped_goal_.y = pg.y();
            snapped_goal_.z = pg.z();
            has_snapped_ = true;
        }

        if (!(start == start_raw))
        {
            const auto p = gridToWorld(start);
            printf("GlobalPlanner::startPlan() Start snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }

        if (!(goal == goal_raw))
        {
            const auto p = gridToWorld(goal);
            printf("GlobalPlanner::startPlan() Goal snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }
        
        // 初始化 A* 算法的数据结构
        // open_set 是一个优先队列，用于存储待扩展的节点，按照 f 值（总代价）从小到大排序。
        // g_score 从起点到某个格子的实际代价。
        // came_from 路径回溯表，记录每个格子是从哪个父格子来的。
        // closed_set 已完成扩展的格子。
        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        // 代码通过 make26Directions() 生成三维邻接方向：
        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;

        // Ts: 路径搜索 = A* 主循环(含邻居扩展与碰撞/代价检查, 不含 start/goal 贴近搜索)
        ScopedBenchTimer ts_timer("[BENCH] Ts (路径搜索: A*)");
        // 循环退出有两种情况: open_set 为空：没有可扩展节点，说明找不到路。或达到最大迭代次数
        while (!open_set.empty() && iters < max_iterations)
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);
            // 如果当前格子是目标格子，则路径搜索成功，开始回溯路径
            if (current.idx == goal) {
                const auto cells = reconstructPath(came_from, current.idx);
                printf("GlobalPlanner::startPlan() A* path found in %d iterations. waypoints=%zu \n", iters, cells.size());
                planner_results_.clear();
                // 将栅格索引路径转换为世界坐标路径，并存储在 planner_results_ 中
                for (std::size_t i = 0; i < cells.size(); ++i)
                {
                    const auto & c = cells[i];
                    const auto p = gridToWorld(c);
                    PointPose temp;
                    temp.x = p.x();
                    temp.y = p.y();
                    temp.z = p.z();
                    planner_results_.push_back(temp);
                }

                // 保存平滑前的原始A*路径, 供外部(如 RViz)对比显示
                raw_planner_results_ = planner_results_;

                // To: 路径平滑。方法由 path_smoothing_method_ 决定:
                //   MovingAverage(默认) = 局部移动平均 + 逐点碰撞回退;
                //   ClampedCubicBspline = 捷径剪枝 + 钳位三次B样条 + 碰撞回退。
                // 不强制地面支撑、不保护楼梯段; 安全性完全由碰撞回退保证。
                {
                    ScopedBenchTimer to_timer("[BENCH] To (路径平滑)");
                    if (enable_path_smoothing_ && planner_results_.size() >= 2) {
                        const std::size_t raw_n = planner_results_.size();
                        planner_results_ = smoothPath(planner_results_);
                        printf(
                            "GlobalPlanner::startPlan() path smoothed. waypoints=%zu -> %zu \n",
                            raw_n, planner_results_.size());
                    }
                }

                return true;
            }

            for (const auto & d : directions) 
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                // 如果邻居格子已经在 closed_set 中，说明已经扩展过，跳过
                if (closed_set.find(nbr) != closed_set.end()) {
                continue;
                }
                // 不可通行的格子直接跳过
                if (!isCellTraversable(
                    nbr, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                continue;
                }
                // 楼梯走向约束: 楼梯格(含进近走廊格——它们已写入 stair_cell_seg_, 故一视同仁)
                // 上只允许沿楼梯走向移动(禁止任何横向/宽度方向偏移)——既禁止斜切下楼梯, 也禁止
                // 横向平移(机器人尺寸大于踏面尺度, 横移会导致部分底盘悬空/卡踏面边缘)。
                // 用连续走向向量点积分, 对任意角度楼梯成立。current 或 nbr 任一是楼梯格即约束。
                if (enable_stair_diagonal_block_) {
                    const auto sc_it = stair_cell_seg_.find(current.idx);
                    const auto sn_it = stair_cell_seg_.find(nbr);
                    if (sc_it != stair_cell_seg_.end() || sn_it != stair_cell_seg_.end()) {
                        const StairSegment & seg = (sc_it != stair_cell_seg_.end())
                            ? stair_segments_[sc_it->second]
                            : stair_segments_[sn_it->second];
                        // 归一化宽度分量: (m·w)/|m| = sin(运动方向与楼梯走向的夹角)。
                        // 归一化后轴向步(长1)与对角步(长√2)可比, 避免对角步因步长大而虚高被卡。
                        // 阈值需 > sin(22.5°)≈0.383; 垂直步(m_len=0)记 0 放行。
                        const double m_perp = -d.x * seg.dir_y + d.y * seg.dir_x;
                        const double m_len = std::sqrt(static_cast<double>(d.x * d.x + d.y * d.y));
                        const double m_perp_norm = (m_len > 1e-9) ? (m_perp / m_len) : 0.0;
                        if (std::abs(m_perp_norm) > stair_perp_tolerance_) {
                            continue;
                        }
                    }
                }
                // 楼梯巡线约束: 楼梯↔非楼梯的过渡只能发生在端点(顶/底中点)。
                // 强制全局路径从 top_center 进楼梯、从 bottom_center 出楼梯(或反向),
                // 楼梯↔非楼梯, 或 不同子段之间 的过渡都只能在端点发生。
                // 后者强制路径经过每个子段端点(每 stair_segment_layers_ 级一个锚点),
                // 避免长楼梯内部斜切; 楼梯同子段内部步不受此约束。
                if (enable_stair_endpoint_traverse_) {
                    const auto sc_it = stair_cell_seg_.find(current.idx);
                    const auto sn_it = stair_cell_seg_.find(nbr);
                    const bool cur_stair = sc_it != stair_cell_seg_.end();
                    const bool nbr_stair = sn_it != stair_cell_seg_.end();
                    const bool cross_seg = cur_stair && nbr_stair && (sc_it->second != sn_it->second);
                    if (cur_stair != nbr_stair || cross_seg) {
                        const bool cur_is_ep = stair_endpoints_.find(current.idx) != stair_endpoints_.end();
                        const bool nbr_is_ep = stair_endpoints_.find(nbr) != stair_endpoints_.end();
                        if (!cur_is_ep && !nbr_is_ep) {
                            continue;   // 过渡点不是端点 → 禁止
                        }
                    }
                }
                // 基础代价 = 当前格子到邻居格子的欧几里得距离
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                // 如果开启了预禁行代价地图 preblocked_costmap，则在基础代价上加上预阻塞代价
                if (enable_preblocked_costmap) {
                tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
                }
                // 楼梯段最小偏差巡线: 给偏离走向的步加代价, 权重(stair_stabilize_weight_)决定
                // 偏差相对距离的主导程度。权重大(>>步长)时, 楼梯段以最小化偏离走向为主、距离退为次要
                // (不再追求最短); 代价与终点无关故路径稳定。权重=0 则退回纯最短路径。
                // 注意: 权重过大(如1e8)会淹没距离代价, perp相同时距离无法tie-break, 路径反而不稳定。
                if (enable_stair_path_stabilize_) {
                    const auto sc_it = stair_cell_seg_.find(current.idx);
                    if (sc_it != stair_cell_seg_.end()) {
                        const StairSegment & seg = stair_segments_[sc_it->second];
                        const double m_perp = -d.x * seg.dir_y + d.y * seg.dir_x;
                        const double m_len = std::sqrt(static_cast<double>(d.x * d.x + d.y * d.y));
                        const double perp = (m_len > 1e-9) ? std::abs(m_perp / m_len) : 0.0;
                        tentative_g += stair_stabilize_weight_ * perp;
                    }
                }
                // 端点偏离代价: 只罚"两端都不是精确端点"的步。
                // 若 current 或 nbr 任一为精确中线端点, 说明路径已到达/正走向中线,
                // 不加罚——避免把"从扩展邻居跨到精确端点"这种正确的入口行为也罚掉。
                // 只有两端都是非精确端点时, 才按偏离距离加代价, 引导 A* 选中线。
                if (enable_stair_endpoint_deviation_) {
                    const bool cur_exact = stair_exact_endpoints_.find(current.idx) != stair_exact_endpoints_.end();
                    const bool nbr_exact = stair_exact_endpoints_.find(nbr) != stair_exact_endpoints_.end();
                    if (!cur_exact && !nbr_exact) {
                        // 两端都不是精确端点 → 按偏离距离加罚(扩展邻居越远越贵)
                        auto dev_it = stair_endpoint_deviation_cost_.find(current.idx);
                        if (dev_it != stair_endpoint_deviation_cost_.end()) {
                            tentative_g += stair_endpoint_deviation_weight_ * dev_it->second;
                        }
                        dev_it = stair_endpoint_deviation_cost_.find(nbr);
                        if (dev_it != stair_endpoint_deviation_cost_.end()) {
                            tentative_g += stair_endpoint_deviation_weight_ * dev_it->second;
                        }
                    } else {
                        // 经过精确端点(中线点) → 奖励(减分)。经P路径有2步含P(进P+出P), 绕过P没有,
                        // 所以经P比绕过便宜 2*reward, 克服"P偏离最短路"的距离差, 引导A*选中线端点。
                        tentative_g -= stair_endpoint_reward_weight_;
                    }
                }

                // 如果邻居格子还没有 g_score，或者新的 g 值更小，则更新 g_score 和 came_from，并将邻居格子加入 open_set
                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                came_from[nbr] = current.idx;
                g_score[nbr] = tentative_g;
                const double f = tentative_g + euclidean(nbr, goal);
                open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        return false;
    }

    std::vector<GridIndex> GlobalPlanner::reconstructPath(const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,GridIndex current) const
    {
        std::vector<GridIndex> path;
        path.push_back(current);
        while (came_from.find(current) != came_from.end()) 
        {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    // 先判断当前格子是否可通行, 如果不可通行, 则在其周围snap_search_radius_cells_半径内搜索可通行的格子, 返回第一个找到的可通行格子
    bool GlobalPlanner::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,GridIndex & out) const
    {
        if (isCellTraversable(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
        out = seed;
        return true;
        }

        for (int r = 1; r <= radius_cells; ++r) {
        for (int dz = 0; dz <= r; ++dz) {
            for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                continue;
                }

                GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
                if (isCellTraversable(
                    c1, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                out = c1;
                return true;
                }

                if (dz > 0) {
                GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
                if (isCellTraversable(
                    c2, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                    out = c2;
                    return true;
                }
                }
            }
            }
        }
        }
        return false;
    }

    double GlobalPlanner::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    
    std::vector<GridIndex> GlobalPlanner::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        // 26 个方向: 3D 邻居格子
        dirs.reserve(26);
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dz = -1; dz <= 1; ++dz) 
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        return dirs;
    }

    bool GlobalPlanner::isCellTraversable(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells) const
    {
        // ① 必须在地图范围内
        {
        FootprintTimerAccum t(bounds_check_ms_);
        if (!isInsideMetricBounds(idx)) {
            return false;
        }
        }
        // ② 如果要求地面支撑, 则必须有地面支撑
        // hasGroundSupport支持两种模式: 1) 严格模式: 直接下方必须有占据体素; 2) 非严格模式: 下方一定范围内有占据体素即可
        {
        FootprintTimerAccum t(ground_support_ms_);
        if (require_ground_support &&
            !hasGroundSupport(
                idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
            return false;
        }
        }
        // ③ 检查该格子正下方是否有被障碍侵占的格子, 如果有则不可通行
        {
        FootprintTimerAccum t(column_check_ms_);
        for (int z = idx.z - 1; z >= 0; --z) {
            const GridIndex below_idx{idx.x, idx.y, z};
            if (isOccupiedCell(below_idx)) {
                break;
            }
            if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
                return false;
            }
        }
        }

        const octomap::point3d center = gridToWorld(idx);
        //  根据机器人半径, 检查该格子周围是否有占据体素或被障碍侵占的格子, 如果有则不可通行
        const double r = octree_->getResolution(); // 栅格分辨率
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r))); // 机器人半径对应的栅格数
        const double radius_sq = robot_radius * robot_radius;

        // Collision check for vehicle body volume (same height and above),
        // while allowing occupied support cells below. Apply the same footprint
        // rule to preblocked cells so a cell is rejected if the vehicle radius
        // overlaps any preblocked voxel.
        FootprintTimerAccum fp_acc(footprint_check_ms_);   // [BENCH] 计时下面这个足迹三重循环

        for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
            for (int dz = 0; dz <= n; ++dz) {
            const double dist_x = static_cast<double>(dx) * r;
            const double dist_y = static_cast<double>(dy) * r;
            const double dist_z = static_cast<double>(dz) * r;
            const double dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
            if (dist_sq > radius_sq) {
                continue;
            }
            const octomap::point3d p(
                center.x() + static_cast<float>(dx * r),
                center.y() + static_cast<float>(dy * r),
                center.z() + static_cast<float>(dz * r));
            const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
            if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
                return false;
            }
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return false;
            }
            }
        }
        }
        return true;
    }

    bool GlobalPlanner::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,int support_depth_cells) const
    {
        if (strict_direct_ground_support) {
        GridIndex below{idx.x, idx.y, idx.z - 1};
        if (!isInsideMetricBounds(below)) {
            return false;
        }
        const auto p = gridToWorld(below);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
        }

        for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
        for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
            for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
            GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
            if (!isInsideMetricBounds(below)) {
                continue;
            }
            const auto p = gridToWorld(below);
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return true;
            }
            }
        }
        }
        return false;
    }

    // rebuildPreblockedCells() 的作用就是提前找出这些“虽然不是占据体素，但也应该被规划器排除”的格子，并存入：preblocked_cells_
    void GlobalPlanner::rebuildPreblockedCells()
    {
        preblocked_cells_.clear();
        if (!octree_) {
        return;
        }

        std::unordered_set<GridIndex, GridIndexHash> candidates;
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            continue;
        }
        const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
            }
        }
        }

        // 检查每个候选栅格是否满足被障碍侵占的条件, 并将符合条件的栅格加入 preblocked_cells_
        for (const auto & c : candidates) {
        if (!isInsideMetricBounds(c)) {
            continue;
        }
        if (isOccupiedCell(c)) {
            continue;
        }
        // 第一类预禁行：下方有占据支撑，但邻居上方也有障碍
        const GridIndex below0{c.x, c.y, c.z - 1};
        const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
        if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
            preblocked_cells_.insert(c);
            continue;
        }
        // 第二类预禁行：旁边有空格，上方没障碍，但下方悬空
        // 当前格同高度周围至少要有一个非占据邻居。
        // 当前格上方一层不能是占据格。
        // 当前格下方一层必须在地图范围内。
        // 当前格下方一层不是占据格。
        const GridIndex above1{c.x, c.y, c.z + 1};
        const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
        if (!hasNonOccupiedNeighborSameLevel(c)) {
            continue;
        }
        if (above1_occ) {
            continue;
        }
        const GridIndex below1{c.x, c.y, c.z - 1};
        if (!isInsideMetricBounds(below1)) {
            continue;
        }
        const bool below1_non_occupied = !isOccupiedCell(below1);
        if (below1_non_occupied) {
            preblocked_cells_.insert(c);
        }
        }
        
        // 将外部预处理的栅格加入 preblocked_cells_ (如果在地图范围内且不是占据栅格)
        for (const auto & c : external_preblocked_cells_) {
        if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
            preblocked_cells_.insert(c);
        }
        }
        printf("Preprocess mask rebuilt. preblocked_cells=%zu external=%zu \n",preblocked_cells_.size(), external_preblocked_cells_.size());
        // publishPreblockedCellsMarker();
    }

    bool GlobalPlanner::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            const GridIndex n_above1{n.x, n.y, n.z + 1};
            if (!isInsideMetricBounds(n_above1)) {
            continue;
            }
            if (isOccupiedCell(n_above1)) {
            return true;
            }
        }
        }
        return false;
    }

    bool GlobalPlanner::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            if (!isOccupiedCell(n)) {
            return true;
            }
        }
        }
        return false;
    }

    // rebuildDerivedLayers() 的作用就是提前找出这些“可通行的格子”，并存入：traversable_cells_
    // 它综合考虑了地图边界、占据体素、地面支撑、预禁行格、机器人半径和身体碰撞，是后续路径规划的基础。
    void GlobalPlanner::rebuildDerivedLayers()
    {
        traversable_cells_.clear();
        if (!octree_) {
        return;
        }

        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;  
        const int support_depth_cells = ground_support_depth_cells_;
        const double robot_radius = robot_radius_;
        const bool lowest_traversable_only = lowest_traversable_only_;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        // 获取八叉树的最小和最大坐标范围（真实的坐标边界）
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        // 转换成栅格索引范围（离散的格子边界）
        const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
        const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

        bounds_check_ms_ = 0.0;       // [BENCH] 复位各段累计计时
        ground_support_ms_ = 0.0;
        column_check_ms_ = 0.0;
        footprint_check_ms_ = 0.0;
        for (int x = min_idx.x; x <= max_idx.x; ++x) {
        for (int y = min_idx.y; y <= max_idx.y; ++y) {
            for (int z = min_idx.z; z <= max_idx.z; ++z) {
            const GridIndex idx{x, y, z};
            // 排除地图外和占据的格子
            if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
                continue;
            }
            // 检查该格子是否可通行（满足机器人半径、地面支撑等条件）
            if (isCellTraversable(
                idx, robot_radius, require_ground_support, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells))
            {
                traversable_cells_.insert(idx);
                if (lowest_traversable_only) {
                break;
                }
            }
            }
        }
        }

        printf("[BENCH] Te2 ① 边界检查累计 = %.3f ms\n", bounds_check_ms_);
        printf("[BENCH] Te2 ② 地面支撑累计 = %.3f ms\n", ground_support_ms_);
        printf("[BENCH] Te2 ③ 下方柱体累计 = %.3f ms\n", column_check_ms_);
        printf("[BENCH] Te2 ④ 足迹碰撞累计 = %.3f ms\n", footprint_check_ms_);

        // 楼梯检测: 复用已构建的 traversable_cells_, 输出走向约束所需的 stair_cell_seg_
        rebuildStairSegments();

        // publishCellSetMarker(
        // traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F,
        // 0.55F);
    }

    // 楼梯检测: 第1步标记候选格(水平8邻域内有z±1相邻踏面) → 第2步26连通分组 →
    // 第3步逐段平面拟合估走向(任意角度)。走向保持连续浮点向量, 不离散成X/Y。
    void GlobalPlanner::rebuildStairSegments()
    {
        stair_segments_.clear();
        stair_cell_seg_.clear();
        stair_endpoints_.clear();
        stair_corridor_cells_.clear();
        // 楼梯检测独立于约束开关: 即使关掉方向约束, 也要检测楼梯并算端点(供巡线用)
        if (traversable_cells_.empty()) {
            return;
        }

        // ---- 第1步: 标记楼梯候选格 ----
        // 水平8邻域(含对角)内存在 z±1 的相邻可通行踏面 → 该格在台阶上。
        // 必须用8邻域: 斜向楼梯的相邻踏面落在对角位置, 4邻域会整段漏检。
        std::unordered_set<GridIndex, GridIndexHash> candidate;
        candidate.reserve(traversable_cells_.size());
        for (const auto & c : traversable_cells_) {
            bool is_stair = false;
            for (int dx = -1; dx <= 1 && !is_stair; ++dx) {
                for (int dy = -1; dy <= 1 && !is_stair; ++dy) {
                    if (dx == 0 && dy == 0) { continue; }
                    if (traversable_cells_.count(GridIndex{c.x + dx, c.y + dy, c.z + 1}) != 0u ||
                        traversable_cells_.count(GridIndex{c.x + dx, c.y + dy, c.z - 1}) != 0u)
                    {
                        is_stair = true;
                    }
                }
            }
            if (is_stair) { candidate.insert(c); }
        }

        // ---- 第2步: 26连通 flood-fill 分组(带弱桥切断) ----
        // 不同段方向不同, 必须分段; 绝不全局一起拟合(会被占多数的地板主导)。
        // 弱桥切断: 纯26连通会把"上一段楼梯顶端格 ↔ 下一段楼梯底端格"这种端对端弱桥
        // (只有1~2对格子相邻, 中间无平台)焊成一个 raw_segment, 第3步平面拟合再把两段反向
        // 走向平均成错误值。解法: 相邻候选格 cur→nb 必须有 >=K 个"共同候选邻居"(同时26邻接
        // cur 和 nb 的候选格)才连通; 端对端弱桥没有第三个格子同时邻接两端→common=0→切断。
        // 正常楼梯(踏面宽>=2)的走向/宽度连接都有侧翼格(同级宽度方向相邻格)做共同邻居,
        // common>=1 不受影响。度数兜底(stair_bridge_degree_pardon_)保护踏面宽=1的单格楼梯。
        const int K = stair_bridge_min_support_;
        const int pardon = stair_bridge_degree_pardon_;
        // 预计算每个候选格的候选邻居数(度数), 供度数兜底 O(1) 查询
        std::unordered_map<GridIndex, int, GridIndexHash> cand_deg;
        if (K > 0) {
            cand_deg.reserve(candidate.size());
            for (const auto & c : candidate) {
                int d = 0;
                for (int ax = -1; ax <= 1; ++ax) {
                    for (int ay = -1; ay <= 1; ++ay) {
                        for (int az = -1; az <= 1; ++az) {
                            if (ax == 0 && ay == 0 && az == 0) { continue; }
                            if (candidate.count(GridIndex{c.x + ax, c.y + ay, c.z + az}) != 0u) { ++d; }
                        }
                    }
                }
                cand_deg[c] = d;
            }
        }
        // 共同候选邻居数: 同时26邻接 a 和 b、且在 candidate 中、且≠a≠b 的格子数; 达 need 即返回 true
        auto common_support = [&](const GridIndex & a, const GridIndex & b, int need) -> bool {
            int common = 0;
            for (int ax = -1; ax <= 1; ++ax) {
                for (int ay = -1; ay <= 1; ++ay) {
                    for (int az = -1; az <= 1; ++az) {
                        if (ax == 0 && ay == 0 && az == 0) { continue; }
                        const GridIndex w{a.x + ax, a.y + ay, a.z + az};
                        if (w == b) { continue; }
                        if (candidate.count(w) == 0u) { continue; }
                        if (std::abs(w.x - b.x) <= 1 && std::abs(w.y - b.y) <= 1 && std::abs(w.z - b.z) <= 1) {
                            if (++common >= need) { return true; }
                        }
                    }
                }
            }
            return false;
        };
        std::unordered_map<GridIndex, int, GridIndexHash> visited;
        std::vector<std::vector<GridIndex>> raw_segments;
        for (const auto & seed : candidate) {
            if (visited.count(seed) != 0u) { continue; }
            raw_segments.emplace_back();
            auto & cells = raw_segments.back();
            std::vector<GridIndex> stack{seed};
            visited[seed] = 1;
            while (!stack.empty()) {
                const GridIndex cur = stack.back();
                stack.pop_back();
                cells.push_back(cur);
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) { continue; }
                            const GridIndex nb{cur.x + dx, cur.y + dy, cur.z + dz};
                            if (candidate.count(nb) == 0u || visited.count(nb) != 0u) { continue; }
                            // 弱桥切断: 共同候选邻居不足 → 视为端对端弱桥
                            if (K > 0 && !common_support(cur, nb, K)) {
                                if (pardon <= 0) { continue; }   // 无兜底 → 切断
                                // 度数兜底: min(两端度数)<=pardon 豁免(单格楼梯边缘); 都>pardon 才切断
                                const auto itc = cand_deg.find(cur);
                                const auto itn = cand_deg.find(nb);
                                const int dc = (itc != cand_deg.end()) ? itc->second : 0;
                                const int dn = (itn != cand_deg.end()) ? itn->second : 0;
                                if (std::min(dc, dn) > pardon) { continue; }
                            }
                            visited[nb] = 1;
                            stack.push_back(nb);
                        }
                    }
                }
            }
        }

        // ---- 第2.5步: 走向一致性切分(把含反向子结构的段拆开) ----
        // 弱桥切断只切 common<K 的端对端弱桥; 两段反向楼梯若经平台/侧翼(common>=1)相连
        // 仍被合并, 第3步平面拟合会把反向走向平均成错误值(dot_lo*hi<0)。按"局部上升方向"拆分:
        // up(c)=c 的 z+1 候选邻居平均方向(指向上一级); 同段 up 同向, 折返处 up 反向→按投影符号切。
        // 正常单段楼梯所有格 up 同向→不切; 仅含反向子结构的段被拆开。
        if (enable_stair_direction_split_) {
            std::unordered_set<GridIndex, GridIndexHash> cells_set;
            std::vector<std::vector<GridIndex>> split_segments;
            for (auto & cells : raw_segments) {
                if (cells.size() < 2) { split_segments.push_back(std::move(cells)); continue; }
                cells_set.clear();
                for (const auto & c : cells) { cells_set.insert(c); }
                // up(c) = z+1 候选邻居的 (dx,dy) 之和(指向上一级踏面的方向)
                auto up_of = [&](const GridIndex & c) -> std::pair<double, double> {
                    double ux = 0.0, uy = 0.0;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) { continue; }
                            if (cells_set.count(GridIndex{c.x + dx, c.y + dy, c.z + 1}) != 0u) { ux += dx; uy += dy; }
                        }
                    }
                    return {ux, uy};
                };
                // down(c) = z-1 候选邻居的 (dx,dy) 之和(指向下一级踏面的方向)
                auto down_of = [&](const GridIndex & c) -> std::pair<double, double> {
                    double dxs = 0.0, dys = 0.0;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) { continue; }
                            if (cells_set.count(GridIndex{c.x + dx, c.y + dy, c.z - 1}) != 0u) { dxs += dx; dys += dy; }
                        }
                    }
                    return {dxs, dys};
                };
                // 主方向 d = up 模最大格的 up(归一化参考方向)
                double rx = 0.0, ry = 0.0, best_mag = 0.0;
                for (const auto & c : cells) {
                    const auto up = up_of(c);
                    const double m = up.first * up.first + up.second * up.second;
                    if (m > best_mag) { best_mag = m; rx = up.first; ry = up.second; }
                }
                if (best_mag < 1e-9) { split_segments.push_back(std::move(cells)); continue; }  // 整段无方向, 不切
                // label: 优先用下方连接 (-down, 更可靠反映格子归属: 踏面在下方)。
                // 下段顶部格 z+1 是平台/上段 → up 偏向上段(误归上段); 但 z-1 是本段踏面 → -down 正确归本段。
                // 仅当无 z-1 候选邻居(段底端)时才退回 up。+1 同主方向 / -1 反向 / 0 无方向。
                std::unordered_map<GridIndex, int, GridIndexHash> label;
                label.reserve(cells.size() * 2);
                for (const auto & c : cells) {
                    const auto dn = down_of(c);
                    const double md = dn.first * dn.first + dn.second * dn.second;
                    if (md > 1e-9) {
                        label[c] = ((-dn.first) * rx + (-dn.second) * ry > 0.0) ? 1 : -1;
                    } else {
                        const auto up = up_of(c);
                        const double m2 = up.first * up.first + up.second * up.second;
                        if (m2 < 1e-9) { label[c] = 0; }
                        else { label[c] = (up.first * rx + up.second * ry > 0.0) ? 1 : -1; }
                    }
                }
                // 按 label 连通分组: 同 label 有方向格连通, 无方向格(label0)被相邻分量吸收, 反向切断
                std::unordered_set<GridIndex, GridIndexHash> lvis;
                for (const auto & seed : cells) {
                    if (lvis.count(seed) != 0u || label[seed] == 0) { continue; }
                    const int sl = label[seed];
                    split_segments.emplace_back();
                    auto & sub = split_segments.back();
                    std::vector<GridIndex> stk{seed};
                    lvis.insert(seed);
                    while (!stk.empty()) {
                        const GridIndex cur = stk.back();
                        stk.pop_back();
                        sub.push_back(cur);
                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dz = -1; dz <= 1; ++dz) {
                                    if (dx == 0 && dy == 0 && dz == 0) { continue; }
                                    const GridIndex nb{cur.x + dx, cur.y + dy, cur.z + dz};
                                    if (cells_set.count(nb) == 0u || lvis.count(nb) != 0u) { continue; }
                                    const int lb = label[nb];
                                    if (lb == 0 || lb == sl) { lvis.insert(nb); stk.push_back(nb); }  // 无方向或同向→吸收; 反向→切断
                                }
                            }
                        }
                    }
                }
                // 残余未分组格(孤立无方向格): 各自单独成段(后续级数<5 会被滤, 影响小)
                for (const auto & c : cells) {
                    if (lvis.count(c) == 0u) {
                        split_segments.emplace_back();
                        split_segments.back().push_back(c);
                    }
                }
            }
            raw_segments = std::move(split_segments);
        }

        // ---- 第2.6步: 合并"相邻 + 同向 + z上下相接"的段, 还原被误切的一条直楼梯 ----
        // 弱桥切断/走向切分可能把一条直楼梯切成多段, 每段各自算 dir/line_w ->
        // 过渡点不在同一中线上(接缝处因 line_w 不同而侧移, 红点偏出绿-蓝线)。
        // 把满足三条件的段并成一组, 第3步按组统一拟合 dir/line_w -> 一条中线 -> 共线。
        // 三条件: ① 格子26相邻; ② 平面拟合方向同向(dot>thr, 排除反向折返楼梯);
        //         ③ z 范围上下相接(排除并排的两条平行楼梯)。
        if (enable_stair_merge_) {
            const int M = static_cast<int>(raw_segments.size());
            if (M > 1) {
                // 每段: 拟合方向(归一化) + z 范围
                std::vector<std::pair<double, double>> seg_dir(M, {0.0, 0.0});
                std::vector<int> seg_zlo(M, 0);
                std::vector<int> seg_zhi(M, 0);
                auto fit_dir2 = [](const std::vector<GridIndex> & cs, double & a, double & b) {
                    a = 0.0; b = 0.0;
                    double Sxx = 0, Sxy = 0, Syy = 0, Sx = 0, Sy = 0, Sxz = 0, Syz = 0, Sz = 0, N = 0;
                    for (const auto & c : cs) {
                        N += 1; Sx += c.x; Sy += c.y; Sz += c.z;
                        Sxx += c.x * c.x; Sxy += c.x * c.y; Syy += c.y * c.y;
                        Sxz += c.x * c.z; Syz += c.y * c.z;
                    }
                    const double det =
                        Sxx * (Syy * N - Sy * Sy) - Sxy * (Sxy * N - Sy * Sx) + Sx * (Sxy * Sy - Syy * Sx);
                    if (std::abs(det) > 1e-9) {
                        a = (Sxz * (Syy * N - Sy * Sy) - Sxy * (Syz * N - Sy * Sz) +
                             Sx * (Syz * Sy - Syy * Sz)) / det;
                        b = (Sxx * (Syz * N - Sy * Sz) - Sxz * (Sxy * N - Sy * Sx) +
                             Sx * (Sxy * Sz - Syz * Sx)) / det;
                    }
                };
                for (int i = 0; i < M; ++i) {
                    double a = 0, b = 0;
                    fit_dir2(raw_segments[static_cast<std::size_t>(i)], a, b);
                    const double mag = std::sqrt(a * a + b * b);
                    if (mag > 1e-9) {
                        seg_dir[static_cast<std::size_t>(i)] = {a / mag, b / mag};
                    }
                    int zlo = raw_segments[static_cast<std::size_t>(i)].front().z;
                    int zhi = zlo;
                    for (const auto & c : raw_segments[static_cast<std::size_t>(i)]) {
                        if (c.z < zlo) { zlo = c.z; }
                        if (c.z > zhi) { zhi = c.z; }
                    }
                    seg_zlo[static_cast<std::size_t>(i)] = zlo;
                    seg_zhi[static_cast<std::size_t>(i)] = zhi;
                }
                // 格 -> 段id
                std::unordered_map<GridIndex, int, GridIndexHash> cell_seg;
                for (int i = 0; i < M; ++i) {
                    for (const auto & c : raw_segments[static_cast<std::size_t>(i)]) { cell_seg[c] = i; }
                }
                // 并查集(路径折半)
                std::vector<int> uf(static_cast<std::size_t>(M));
                for (int i = 0; i < M; ++i) { uf[static_cast<std::size_t>(i)] = i; }
                auto find_fn = [&](int x) -> int {
                    while (uf[static_cast<std::size_t>(x)] != x) {
                        uf[static_cast<std::size_t>(x)] = uf[static_cast<std::size_t>(uf[static_cast<std::size_t>(x)])];
                        x = uf[static_cast<std::size_t>(x)];
                    }
                    return x;
                };
                // 邻接扫描: 相邻不同段, 满足三条件则合并
                for (int i = 0; i < M; ++i) {
                    for (const auto & c : raw_segments[static_cast<std::size_t>(i)]) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dz = -1; dz <= 1; ++dz) {
                                    if (dx == 0 && dy == 0 && dz == 0) { continue; }
                                    const auto it = cell_seg.find(GridIndex{c.x + dx, c.y + dy, c.z + dz});
                                    if (it == cell_seg.end()) { continue; }
                                    const int j = it->second;
                                    if (j == i) { continue; }
                                    const auto & di = seg_dir[static_cast<std::size_t>(i)];
                                    const auto & dj = seg_dir[static_cast<std::size_t>(j)];
                                    const double dot = di.first * dj.first + di.second * dj.second;
                                    if (dot <= stair_merge_dir_threshold_) { continue; }   // ② 非同向 -> 不并
                                    const int zloi = seg_zlo[static_cast<std::size_t>(i)];
                                    const int zhii = seg_zhi[static_cast<std::size_t>(i)];
                                    const int zloj = seg_zlo[static_cast<std::size_t>(j)];
                                    const int zhij = seg_zhi[static_cast<std::size_t>(j)];
                                    const bool stacked = (zhii <= zloj + 1) || (zhij <= zloi + 1);  // ③ z上下相接
                                    if (!stacked) { continue; }
                                    int ra = find_fn(i); int rb = find_fn(j);
                                    if (ra != rb) { uf[static_cast<std::size_t>(ra)] = rb; }
                                }
                            }
                        }
                    }
                }
                // 按 root 收集 -> 新的 raw_segments
                std::vector<std::vector<GridIndex>> grouped(static_cast<std::size_t>(M));
                for (int i = 0; i < M; ++i) {
                    const int r = find_fn(i);
                    auto & dst = grouped[static_cast<std::size_t>(r)];
                    const auto & src = raw_segments[static_cast<std::size_t>(i)];
                    dst.insert(dst.end(), src.begin(), src.end());
                }
                std::vector<std::vector<GridIndex>> merged;
                for (int i = 0; i < M; ++i) {
                    if (!grouped[static_cast<std::size_t>(i)].empty()) {
                        merged.push_back(std::move(grouped[static_cast<std::size_t>(i)]));
                    }
                }
                raw_segments = std::move(merged);
            }
        }

        // ---- 第3步: 逐段判定 + 平面拟合估走向 ----
        int stair_skip_layers = 0;   // 因级数不足被滤掉的段数
        int stair_skip_slope = 0;    // 因坡度不足被滤掉的段数
        int stair_max_layers = 0;    // 见过的最大段级数(诊断用)
        for (auto & cells : raw_segments) {
            // 级数 = 段内不同 z 层数。区分楼梯(多级密集)与两层楼板(仅2层)。
            std::unordered_set<int> zs;
            for (const auto & c : cells) { zs.insert(c.z); }
            stair_max_layers = std::max(stair_max_layers, static_cast<int>(zs.size()));
            if (static_cast<int>(zs.size()) < stair_min_layers_) { ++stair_skip_layers; continue; }

            // 最小二乘平面拟合 z = a*x + b*y + c (栅格坐标; 与世界坐标同尺度, 方向一致)。
            double Sxx = 0, Sxy = 0, Syy = 0, Sx = 0, Sy = 0, Sxz = 0, Syz = 0, Sz = 0;
            double N = 0;
            for (const auto & c : cells) {
                N += 1; Sx += c.x; Sy += c.y; Sz += c.z;
                Sxx += c.x * c.x; Sxy += c.x * c.y; Syy += c.y * c.y;
                Sxz += c.x * c.z; Syz += c.y * c.z;
            }
            double a = 0, b = 0;
            const double det =
                Sxx * (Syy * N - Sy * Sy) - Sxy * (Sxy * N - Sy * Sx) + Sx * (Sxy * Sy - Syy * Sx);
            if (std::abs(det) > 1e-9) {
                a = (Sxz * (Syy * N - Sy * Sy) - Sxy * (Syz * N - Sy * Sz) + Sx * (Syz * Sy - Syy * Sz)) / det;
                b = (Sxx * (Syz * N - Sy * Sz) - Sxz * (Sxy * N - Sy * Sx) + Sx * (Sxy * Sz - Syz * Sx)) / det;
            }
            const double mag = std::sqrt(a * a + b * b);
            if (mag < stair_slope_min_) { ++stair_skip_slope; continue; }   // 太平 → 不是楼梯

            // 通过过滤的大段: 走向 dir, 按 z 切成多个子段(每 stair_segment_layers_ 级),
            // 每子段独立登记(走向沿用整段走向, 端点=子段 z 范围的层中点)。
            // 端点约束自动作用到每个子段 → 长楼梯被每 N 级一个锚点钉在走向线上, 避免长距离斜切。
            const double dir_x = a / mag;
            const double dir_y = b / mag;

            // 大段 z 范围 + 段中心 xy + 各层格数
            int z_lo_all = cells.front().z;
            int z_hi_all = cells.front().z;
            double cx_all = 0.0, cy_all = 0.0;
            std::unordered_map<int, int> z_cnt;
            for (const auto & c : cells) {
                if (c.z < z_lo_all) { z_lo_all = c.z; }
                if (c.z > z_hi_all) { z_hi_all = c.z; }
                cx_all += c.x; cy_all += c.y; z_cnt[c.z] += 1;
            }
            cx_all /= static_cast<double>(cells.size());
            cy_all /= static_cast<double>(cells.size());
            // 端点选取: 连线必须严格平行楼梯走向 → 所有端点的 w(宽度方向)坐标对齐同一个 line_w。
            // line_w = 入口/出口(高z端 z_hi_all / 低z端 z_lo_all)中较窄一端的宽度中位数 →
            // 窄端端点落在踏面宽度中央(通行瓶颈必须走中间), 宽端端点 w 对齐 line_w → 连线沿走向。
            // w=(-dir_y,dir_x) 为宽度方向。
            auto layer_w_of = [&](const GridIndex & c) -> double {
                return -(c.x - cx_all) * dir_y + (c.y - cy_all) * dir_x;
            };
            auto layer_w_mid = [&](int z) -> double {
                std::vector<double> ws;
                for (const auto & c : cells) { if (c.z == z) { ws.push_back(layer_w_of(c)); } }
                if (ws.empty()) { return 0.0; }
                std::sort(ws.begin(), ws.end());
                return ws[ws.size() / 2];
            };
            auto layer_w_range = [&](int z) -> double {
                double wmin = 1e18, wmax = -1e18;
                for (const auto & c : cells) {
                    if (c.z != z) { continue; }
                    const double w = layer_w_of(c);
                    if (w < wmin) { wmin = w; }
                    if (w > wmax) { wmax = w; }
                }
                return wmax - wmin;
            };
            // 较窄端 = 宽度范围小的端; line_w 取该端的宽度中位数
            const int narrow_z = (layer_w_range(z_hi_all) <= layer_w_range(z_lo_all))
                                 ? z_hi_all : z_lo_all;
            const double line_w = layer_w_mid(narrow_z);
            // 端点 = 该层 w 最接近 line_w 的格(所有端点 w 对齐 → 连线严格平行走向);
            // w 并列时取 t 最接近走向线的(段中心处)。
            auto layer_mid = [&](int z) -> GridIndex {
                if (z_cnt.count(z) == 0u) { return GridIndex{0, 0, z}; }
                GridIndex best = cells.front();
                double best_wd = 1e18;
                double best_td = 1e18;
                for (const auto & c : cells) {
                    if (c.z != z) { continue; }
                    const double wd = std::abs(layer_w_of(c) - line_w);
                    const double td = std::abs((c.x - cx_all) * dir_x + (c.y - cy_all) * dir_y);
                    if (wd < best_wd - 1e-9 ||
                        (std::abs(wd - best_wd) <= 1e-9 && td < best_td)) {
                        best_wd = wd; best_td = td; best = c;
                    }
                }
                return best;
            };

            // 边界(过渡点) z 序列: z_lo_all, z_lo_all+L, ..., z_hi_all。
            // 端点 = 每个边界层中点(相邻子段共享同一过渡点), 不再每子段算 top+bottom 两个 → 无冗余。
            std::vector<int> bnds;
            for (int z = z_lo_all; z < z_hi_all; z += stair_segment_layers_) { bnds.push_back(z); }
            bnds.push_back(z_hi_all);
            for (int b : bnds) {
                if (z_cnt.count(b) != 0u) { stair_endpoints_.insert(layer_mid(b)); }
            }

            // 子段 = 相邻边界之间; 上边界 bnd_{i+1} 归下一子段, 最后子段含 z_hi_all
            int seg_id_first = -1, seg_id_last = -1;  // 首个/末个子段 id, 给进近走廊归属用
            for (std::size_t i = 0; i + 1 < bnds.size(); ++i) {
                const int z_lo = bnds[i];
                const bool is_last = (i + 2 == bnds.size());
                const int z_hi = is_last ? bnds[i + 1] : (bnds[i + 1] - 1);
                std::vector<GridIndex> sub_cells;
                for (const auto & c : cells) {
                    if (c.z >= z_lo && c.z <= z_hi) { sub_cells.push_back(c); }
                }
                if (sub_cells.empty()) { continue; }

                const int seg_id = static_cast<int>(stair_segments_.size());
                if (seg_id_first < 0) { seg_id_first = seg_id; }
                seg_id_last = seg_id;
                StairSegment seg;
                seg.dir_x = dir_x;
                seg.dir_y = dir_y;
                seg.layer_count = z_hi - z_lo + 1;
                double scx = 0.0, scy = 0.0;
                for (const auto & c : sub_cells) { scx += c.x; scy += c.y; }
                const int z_mid = (z_lo + z_hi) / 2;
                const auto sc = gridToWorld(GridIndex{static_cast<int>(std::round(scx / sub_cells.size())),
                                                      static_cast<int>(std::round(scy / sub_cells.size())), z_mid});
                seg.center = PointPose{sc.x(), sc.y(), sc.z()};
                // 子段两端中点 = 相邻边界层中点(过渡点, 相邻子段共享, 已存 stair_endpoints_)
                if (z_cnt.count(z_lo) != 0u) {
                    const auto bw = gridToWorld(layer_mid(z_lo));
                    seg.bottom_center = PointPose{bw.x(), bw.y(), bw.z()};
                } else { seg.bottom_center = seg.center; }
                if (z_cnt.count(bnds[i + 1]) != 0u) {
                    const auto tw = gridToWorld(layer_mid(bnds[i + 1]));
                    seg.top_center = PointPose{tw.x(), tw.y(), tw.z()};
                } else { seg.top_center = seg.center; }
                stair_segments_.push_back(seg);
                for (const auto & c : sub_cells) { stair_cell_seg_[c] = seg_id; }
            }

            // ---- 进近走廊: 从真实端点(z==z_lo_all 底端 / z==z_hi_all 顶端)沿 ±dir 向外延伸
            // N 格, 沿射线穿过"可通行"格(含被候选检测误判为楼梯的边界平台格——方案①), 只给其中
            // 非楼梯格(真平台)写入 stair_cell_seg_ 加走向约束(已是楼梯格的不覆盖); 末端(tip)在端点
            // 扩展前并入 stair_endpoints_, 与真实端点完全同逻辑(exact 快照→奖励 / ±2 扩展 / 偏离代价);
            // 另存一份 stair_corridor_cells_ 仅供橙色点可视化。只从真实端点延伸: 内部子段边界沿 dir 恒 z 投影会离开踏面碰到 riser/void。
            if (stair_endpoint_extend_dist_ > 1e-9) {
                const double r = octree_->getResolution();
                const int N = std::max(1, static_cast<int>(std::round(stair_endpoint_extend_dist_ / r)));
                // 底端向外 = -dir(dir 指向 z 增大/上行方向); 顶端向外 = +dir
                auto extend_corridor = [&](const GridIndex & ep, double sgn, int seg_id) {
                    GridIndex tip{0, 0, 0};   // 走廊末端(平台侧最后一个成功格); 当作完整端点(同真实端点逻辑)
                    bool has_tip = false;
                    int cur_z = ep.z;
                    for (int i = 1; i <= N; ++i) {
                        const int gx = ep.x + static_cast<int>(std::round(sgn * dir_x * static_cast<double>(i)));
                        const int gy = ep.y + static_cast<int>(std::round(sgn * dir_y * static_cast<double>(i)));
                        // 贴平台表面: 先当前 z, 再 z-1, 再 z+1(逐格延续, 下一格从本格 z 继续)
                        const int z_tries[3] = {cur_z, cur_z - 1, cur_z + 1};
                        bool found = false;
                        int found_z = cur_z;
                        for (int t = 0; t < 3 && !found; ++t) {
                            const GridIndex cand{gx, gy, z_tries[t]};
                            // 方案①: 只要可通行就接受(楼梯格也算), 越过被误判的边界平台格走到真平台。
                            if (traversable_cells_.count(cand) != 0u) {
                                found_z = z_tries[t];
                                found = true;
                            }
                        }
                        if (!found) {
                            // 诊断: 仅当 i=1 即失败(该端将无末端)时打印。方案①后楼梯格不再被拒,
                            // 故 !found 表示该投影格在三个 z 都不可通行(墙/越界/高度差>1)。
                            if (!has_tip) {
                                const char * end_name = (sgn < 0.0) ? "bottom" : "top";
                                char zstat[4] = {'.', '.', '.', '\0'};
                                for (int t = 0; t < 3; ++t) {
                                    const GridIndex cand{gx, gy, z_tries[t]};
                                    if (traversable_cells_.count(cand) != 0u) {
                                        zstat[t] = (stair_cell_seg_.find(cand) != stair_cell_seg_.end()) ? 'S' : 'T';
                                    }
                                }
                                printf("[CORRIDOR] NO-TIP %s: ep=(%d,%d,%d) proj(%d,%d) z=[%d,%d,%d] stat=%s"
                                       "  (方案①后仅 . =不可通行/越界 会触发)\n",
                                       end_name, ep.x, ep.y, ep.z, gx, gy,
                                       z_tries[0], z_tries[1], z_tries[2], zstat);
                            }
                            break;   // 三个 z 都不可通行 -> 走廊终止(须连续)
                        }
                        const GridIndex cc{gx, gy, found_z};
                        // 只给非楼梯格(真平台)加走向约束; 已是楼梯格的不覆盖(保留它本来的段/走向)。
                        if (stair_cell_seg_.find(cc) == stair_cell_seg_.end()) {
                            stair_cell_seg_[cc] = seg_id;
                        }
                        tip = cc;                        // 记最后一个成功格 = 走廊末端
                        has_tip = true;
                        cur_z = found_z;
                    }
                    if (has_tip) {
                        stair_corridor_cells_.insert(tip);   // 可视化(橙点)
                        stair_endpoints_.insert(tip);        // 和真实端点同逻辑: 在端点扩展前并入,
                                                             // 自动进 exact 快照(→奖励) / ±2 扩展 / 偏离代价
                    }
                };
                if (z_cnt.count(z_lo_all) != 0u && seg_id_first >= 0) { extend_corridor(layer_mid(z_lo_all), -1.0, seg_id_first); }
                if (z_cnt.count(z_hi_all) != 0u && seg_id_last >= 0) { extend_corridor(layer_mid(z_hi_all), +1.0, seg_id_last); }
            }
        }

        // 端点区域扩展: 端点 + 其可通行26邻居。放宽端点约束的一步可达性——
        // 端点本身虽在可行域, 但 A* 须一步到达它; 若端点孤立/邻居不可通行/相邻子段层xy差>1,
        // 会堵死。扩展为区域后, 经过端点或其可通行邻居都算满足约束, 路径仍钉在端点附近。
        {
            // 保存扩展前的精确端点(中线点), 供 A* 偏离代价查询
            stair_exact_endpoints_ = stair_endpoints_;
            stair_endpoint_deviation_cost_.clear();

            std::unordered_set<GridIndex, GridIndexHash> ep_region = stair_endpoints_;
            for (const auto & ep : stair_endpoints_) {
                for (int dx = -2; dx <= 2; ++dx) {
                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dz = 0; dz <= 1; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) { continue; }
                            const GridIndex nb{ep.x + dx, ep.y + dy, ep.z + dz};
                            if (traversable_cells_.count(nb) != 0u) {
                                ep_region.insert(nb);
                                // 扩展邻居不是精确端点 → 记录到精确端点的距离作为偏离代价
                                if (stair_exact_endpoints_.count(nb) == 0u) {
                                    const double dist = std::sqrt(static_cast<double>(
                                        dx * dx + dy * dy + dz * dz));
                                    auto it = stair_endpoint_deviation_cost_.find(nb);
                                    if (it == stair_endpoint_deviation_cost_.end() ||
                                        dist < it->second) {
                                        stair_endpoint_deviation_cost_[nb] = dist;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            stair_endpoints_ = std::move(ep_region);
        }

        printf("Stair segments rebuilt. segments=%zu stair_cells=%zu | "
               "candidate=%zu raw_segments=%zu max_layers=%d skip_layers=%d skip_slope=%d corridor=%zu\n",
               stair_segments_.size(), stair_cell_seg_.size(),
               candidate.size(), raw_segments.size(),
               stair_max_layers, stair_skip_layers, stair_skip_slope,
               stair_corridor_cells_.size());
    }

    void GlobalPlanner::getStairCells(std::vector<PointPose>& out) const
    {
        out.clear();
        for (const auto & kv : stair_cell_seg_) {
            const auto p = gridToWorld(kv.first);
            out.push_back(PointPose{p.x(), p.y(), p.z()});
        }
    }

    void GlobalPlanner::getStairSegments(std::vector<StairSegment>& out) const
    {
        out = stair_segments_;
    }

    void GlobalPlanner::getStairApproachCorridor(std::vector<PointPose>& out) const
    {
        out.clear();
        out.reserve(stair_corridor_cells_.size());
        for (const auto & c : stair_corridor_cells_) {
            const auto p = gridToWorld(c);
            out.push_back(PointPose{p.x(), p.y(), p.z()});
        }
    }

    // 按段分组返回楼梯踏面格: out[段id] = 该段所有格(世界坐标)。供 RViz 按段着色。
    void GlobalPlanner::getStairCellsBySegment(std::vector<std::vector<PointPose>>& out) const
    {
        out.assign(stair_segments_.size(), {});
        for (const auto & kv : stair_cell_seg_) {
            const auto p = gridToWorld(kv.first);
            out[kv.second].push_back(PointPose{p.x(), p.y(), p.z()});
        }
    }

    void GlobalPlanner::getTraversableCells(std::vector<PointPose>& out) const
    {
        out.clear();
        out.reserve(traversable_cells_.size());
        for (const auto & idx : traversable_cells_) {
            const auto p = gridToWorld(idx);
            out.push_back(PointPose{p.x(), p.y(), p.z()});
        }
    }

    bool GlobalPlanner::saveSceneCache(const std::string & path, const SceneCacheMeta & meta) const
    {
        if (!octree_) {
            printf("saveSceneCache: octree_ is null, skip.\n");
            return false;
        }
        std::ofstream os(path, std::ios::binary | std::ios::trunc);
        if (!os) {
            printf("saveSceneCache: cannot open %s for writing.\n", path.c_str());
            return false;
        }

        // ---- manifest 头 ----
        if (!(os.write(kSceneCacheMagic, 8) && bwU32(os, kSceneCacheFormatVersion) &&
              bwStr(os, meta.pcd_abs_path) && bwU64(os, meta.pcd_size) &&
              bwU64(os, meta.pcd_mtime) && bwU8(os, meta.strict_direct_ground_support ? 1 : 0))) {
            printf("saveSceneCache: failed writing manifest.\n");
            return false;
        }

        // ---- 9 个 section: 每个 section 先 u32 count 再逐条记录 ----
        // GridIndex=3×i32; PointPose=3×f64; StairSegment 逐字段写(避免结构体 padding)。
        auto writeGridSet = [&](const std::unordered_set<GridIndex, GridIndexHash> & s) -> bool {
            if (!bwU32(os, static_cast<std::uint32_t>(s.size()))) { return false; }
            for (const auto & g : s) {
                if (!bwGrid(os, g.x, g.y, g.z)) { return false; }
            }
            return true;
        };

        // 1) traversable_cells_  2) preblocked_cells_
        if (!writeGridSet(traversable_cells_)) { return false; }
        if (!writeGridSet(preblocked_cells_)) { return false; }

        // 3) preblocked_costmap_ (GridIndex -> double)
        if (!bwU32(os, static_cast<std::uint32_t>(preblocked_costmap_.size()))) { return false; }
        for (const auto & kv : preblocked_costmap_) {
            if (!(bwGrid(os, kv.first.x, kv.first.y, kv.first.z) && bwF64(os, kv.second))) { return false; }
        }

        // 4) stair_segments_
        if (!bwU32(os, static_cast<std::uint32_t>(stair_segments_.size()))) { return false; }
        for (const auto & seg : stair_segments_) {
            if (!(bwPoint(os, seg.center.x, seg.center.y, seg.center.z) &&
                  bwF64(os, seg.dir_x) && bwF64(os, seg.dir_y) &&
                  bwI32(os, static_cast<std::int32_t>(seg.layer_count)) &&
                  bwPoint(os, seg.top_center.x, seg.top_center.y, seg.top_center.z) &&
                  bwPoint(os, seg.bottom_center.x, seg.bottom_center.y, seg.bottom_center.z))) {
                return false;
            }
        }

        // 5) stair_cell_seg_ (GridIndex -> int)
        if (!bwU32(os, static_cast<std::uint32_t>(stair_cell_seg_.size()))) { return false; }
        for (const auto & kv : stair_cell_seg_) {
            if (!(bwGrid(os, kv.first.x, kv.first.y, kv.first.z) &&
                  bwI32(os, static_cast<std::int32_t>(kv.second)))) {
                return false;
            }
        }

        // 6) stair_endpoints_  7) stair_exact_endpoints_
        if (!writeGridSet(stair_endpoints_)) { return false; }
        if (!writeGridSet(stair_exact_endpoints_)) { return false; }

        // 8) stair_endpoint_deviation_cost_ (GridIndex -> double)
        if (!bwU32(os, static_cast<std::uint32_t>(stair_endpoint_deviation_cost_.size()))) { return false; }
        for (const auto & kv : stair_endpoint_deviation_cost_) {
            if (!(bwGrid(os, kv.first.x, kv.first.y, kv.first.z) && bwF64(os, kv.second))) { return false; }
        }

        // 9) stair_corridor_cells_ (GridIndex set, 仅供可视化)
        if (!writeGridSet(stair_corridor_cells_)) { return false; }

        printf("Scene cache saved: %s (traversable=%zu preblocked=%zu costmap=%zu segs=%zu "
               "cell_seg=%zu endpoints=%zu exact=%zu dev=%zu corridor=%zu)\n",
               path.c_str(), traversable_cells_.size(), preblocked_cells_.size(),
               preblocked_costmap_.size(), stair_segments_.size(), stair_cell_seg_.size(),
               stair_endpoints_.size(), stair_exact_endpoints_.size(),
               stair_endpoint_deviation_cost_.size(), stair_corridor_cells_.size());
        return true;
    }

    bool GlobalPlanner::loadSceneCache(
        const std::string & path,
        const SceneCacheMeta & expected,
        std::shared_ptr<octomap::OcTree> map)
    {
        if (!map) {
            printf("loadSceneCache: map is null.\n");
            return false;
        }
        std::ifstream is(path, std::ios::binary);
        if (!is) {
            printf("loadSceneCache: cannot open %s.\n", path.c_str());
            return false;
        }

        // ---- manifest 头: 逐字段校验, 任一不匹配即视为过期 ----
        char magic[8] = {0};
        is.read(magic, 8);
        if (static_cast<std::size_t>(is.gcount()) != 8 ||
            std::memcmp(magic, kSceneCacheMagic, 8) != 0) {
            printf("loadSceneCache: magic mismatch, cache stale.\n");
            return false;
        }
        std::uint32_t version = 0;
        std::string pcd_path;
        std::uint64_t psize = 0;
        std::uint64_t pmtime = 0;
        std::uint8_t strict = 0;
        if (!(brU32(is, version) && brStr(is, pcd_path) && brU64(is, psize) &&
              brU64(is, pmtime) && brU8(is, strict))) {
            printf("loadSceneCache: manifest read failed.\n");
            return false;
        }
        // 逐字段比对: 任一不一致即判过期。打印每个不一致项的"存的值 vs 期望值",
        // 方便直接看出重建原因。
        bool manifest_stale = false;
        // [TEST] 临时跳过 version / path 校验(代码注释保留, 不删除),
        // 用于测试能否加载旧版本/不同路径的缓存。注意: 下面的 9 个数据段仍按
        // 当前版本布局解析, 版本不匹配时可能反序列化失败或读到错乱数据。
        /*
        if (version != expected.format_version) {
            printf("[CACHE MISS] format_version: stored=%u, expected=%u\n",
                   version, expected.format_version);
            manifest_stale = true;
        }
        if (pcd_path != expected.pcd_abs_path) {
            printf("[CACHE MISS] pcd_abs_path:\n"
                   "    stored  = %s\n"
                   "    expected= %s\n",
                   pcd_path.c_str(), expected.pcd_abs_path.c_str());
            manifest_stale = true;
        }
        */
        if (psize != expected.pcd_size) {
            printf("[CACHE MISS] pcd_size: stored=%lu, expected=%lu\n",
                   static_cast<unsigned long>(psize),
                   static_cast<unsigned long>(expected.pcd_size));
            manifest_stale = true;
        }
        if (pmtime != expected.pcd_mtime) {
            printf("[CACHE MISS] pcd_mtime: stored=%lu, expected=%lu (epoch seconds)\n",
                   static_cast<unsigned long>(pmtime),
                   static_cast<unsigned long>(expected.pcd_mtime));
            manifest_stale = true;
        }
        if ((strict != 0) != expected.strict_direct_ground_support) {
            printf("[CACHE MISS] strict_direct_ground_support: stored=%d, expected=%d\n",
                   static_cast<int>(strict != 0),
                   static_cast<int>(expected.strict_direct_ground_support));
            manifest_stale = true;
        }
        if (manifest_stale) {
            printf("loadSceneCache: manifest mismatch -> cache stale, will rebuild.\n");
            return false;
        }

        // ---- 9 个 section ----
        auto readGridSet = [&](std::unordered_set<GridIndex, GridIndexHash> & dst) -> bool {
            std::uint32_t n = 0;
            if (!brU32(is, n)) { return false; }
            dst.clear();
            dst.reserve(n);
            for (std::uint32_t i = 0; i < n; ++i) {
                int x = 0, y = 0, z = 0;
                if (!brGrid(is, x, y, z)) { return false; }
                dst.insert(GridIndex{x, y, z});
            }
            return true;
        };

        // 1) traversable_cells_  2) preblocked_cells_
        if (!readGridSet(traversable_cells_)) {
            printf("loadSceneCache: traversable_cells_ read failed.\n");
            return false;
        }
        if (!readGridSet(preblocked_cells_)) {
            printf("loadSceneCache: preblocked_cells_ read failed.\n");
            return false;
        }

        // 3) preblocked_costmap_ (GridIndex -> double)
        {
            std::uint32_t n = 0;
            if (!brU32(is, n)) { return false; }
            preblocked_costmap_.clear();
            preblocked_costmap_.reserve(n);
            for (std::uint32_t i = 0; i < n; ++i) {
                int x = 0, y = 0, z = 0;
                double c = 0.0;
                if (!(brGrid(is, x, y, z) && brF64(is, c))) { return false; }
                preblocked_costmap_[GridIndex{x, y, z}] = c;
            }
        }

        // 4) stair_segments_
        {
            std::uint32_t n = 0;
            if (!brU32(is, n)) { return false; }
            stair_segments_.clear();
            stair_segments_.resize(n);
            for (auto & seg : stair_segments_) {
                double cx = 0, cy = 0, cz = 0, dx = 0, dy = 0;
                double tcx = 0, tcy = 0, tcz = 0, bcx = 0, bcy = 0, bcz = 0;
                std::int32_t layer = 0;
                if (!(brPoint3(is, cx, cy, cz) && brF64(is, dx) && brF64(is, dy) &&
                      brI32(is, layer) && brPoint3(is, tcx, tcy, tcz) &&
                      brPoint3(is, bcx, bcy, bcz))) {
                    return false;
                }
                seg.center = PointPose{cx, cy, cz};
                seg.dir_x = dx;
                seg.dir_y = dy;
                seg.layer_count = static_cast<int>(layer);
                seg.top_center = PointPose{tcx, tcy, tcz};
                seg.bottom_center = PointPose{bcx, bcy, bcz};
            }
        }

        // 5) stair_cell_seg_ (GridIndex -> int)
        {
            std::uint32_t n = 0;
            if (!brU32(is, n)) { return false; }
            stair_cell_seg_.clear();
            stair_cell_seg_.reserve(n);
            for (std::uint32_t i = 0; i < n; ++i) {
                int x = 0, y = 0, z = 0;
                std::int32_t sid = 0;
                if (!(brGrid(is, x, y, z) && brI32(is, sid))) { return false; }
                stair_cell_seg_[GridIndex{x, y, z}] = static_cast<int>(sid);
            }
        }

        // 6) stair_endpoints_  7) stair_exact_endpoints_
        if (!readGridSet(stair_endpoints_)) {
            printf("loadSceneCache: stair_endpoints_ read failed.\n");
            return false;
        }
        if (!readGridSet(stair_exact_endpoints_)) {
            printf("loadSceneCache: stair_exact_endpoints_ read failed.\n");
            return false;
        }

        // 8) stair_endpoint_deviation_cost_ (GridIndex -> double)
        {
            std::uint32_t n = 0;
            if (!brU32(is, n)) { return false; }
            stair_endpoint_deviation_cost_.clear();
            stair_endpoint_deviation_cost_.reserve(n);
            for (std::uint32_t i = 0; i < n; ++i) {
                int x = 0, y = 0, z = 0;
                double c = 0.0;
                if (!(brGrid(is, x, y, z) && brF64(is, c))) { return false; }
                stair_endpoint_deviation_cost_[GridIndex{x, y, z}] = c;
            }
        }

        // 9) stair_corridor_cells_ (GridIndex set, 仅供可视化)
        if (!readGridSet(stair_corridor_cells_)) {
            printf("loadSceneCache: stair_corridor_cells_ read failed.\n");
            return false;
        }

        // 布局一致性兜底: 9 个 section 读完后应恰好到 EOF; 仍有字节说明布局变了(版本号没 bump) -> 视为过期
        if (is.peek() != std::char_traits<char>::eof()) {
            printf("loadSceneCache: trailing bytes after 9 sections, cache layout mismatch.\n");
            return false;
        }

        octree_ = map;
        map_ready_ = true;
        printf("Scene cache loaded: %s (traversable=%zu preblocked=%zu costmap=%zu segs=%zu "
               "cell_seg=%zu endpoints=%zu exact=%zu dev=%zu corridor=%zu)\n",
               path.c_str(), traversable_cells_.size(), preblocked_cells_.size(),
               preblocked_costmap_.size(), stair_segments_.size(), stair_cell_seg_.size(),
               stair_endpoints_.size(), stair_exact_endpoints_.size(),
               stair_endpoint_deviation_cost_.size(), stair_corridor_cells_.size());
        return true;
    }

    bool GlobalPlanner::isInsideMetricBounds(const GridIndex & idx) const
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
            p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
            p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
    }

    bool GlobalPlanner::isOccupiedCell(const GridIndex & idx) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }
        const auto p = gridToWorld(idx);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
    }

    GridIndex GlobalPlanner::worldToGrid(double x, double y, double z) const
    {
        const double r = octree_->getResolution();
        return GridIndex{
        static_cast<int>(std::floor(x / r)),
        static_cast<int>(std::floor(y / r)),
        static_cast<int>(std::floor(z / r))};
    }

    octomap::point3d GlobalPlanner::gridToWorld(const GridIndex & idx) const
    {
        const double r = octree_->getResolution();
        return octomap::point3d(
        static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    // 构建的是风险缓冲区。它不会直接禁止通行，而是通过增加代价来引导路径规划器避开这些区域。距离越近，代价越高；距离超过指定半径，代价为零。
    void GlobalPlanner::rebuildPreblockedCostmap()
    {
        preblocked_costmap_.clear();
        if (!octree_) {
        return;
        }
        const bool enable = enable_preblocked_costmap_;
        if (!enable) {
        return;
        }
        // 计算代价地图的半径（以栅格为单位），确保至少为1
        const int radius_cells = std::max(
        1, static_cast<int>(preblocked_costmap_radius_cells_));
        const double denom = static_cast<double>(radius_cells) + 1.0;

        // 遍历所有预禁行格子，并在其周围一定半径内计算代价值
        for (const auto & c : preblocked_cells_) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                continue;
                }
                const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
                if (!isInsideMetricBounds(n)) {
                continue;
                }
                if (traversable_cells_.find(n) == traversable_cells_.end()) {
                continue;
                }
                if (preblocked_cells_.find(n) != preblocked_cells_.end()) {
                continue;
                }
                // 计算当前格子 n 与预禁行格子 c 的欧几里得距离 d
                const double d = std::sqrt(
                static_cast<double>(dx * dx + dy * dy + dz * dz));
                if (d > static_cast<double>(radius_cells)) {
                continue;
                }
                // 距离越近，代价越高。距离为0时代价为1，距离大于半径时代价为0
                const double cst = std::max(0.0, (denom - d) / denom);
                auto it = preblocked_costmap_.find(n);
                if (it == preblocked_costmap_.end() || cst > it->second) {
                preblocked_costmap_[n] = cst;
                }
            }
            }
        }
        }

        // RCLCPP_INFO(
        // get_logger(),
        // "Preblocked costmap rebuilt. cells=%zu radius=%d",
        // preblocked_costmap_.size(), radius_cells);
        printf("Preblocked costmap rebuilt. cells=%zu radius=%d \n",preblocked_costmap_.size(),radius_cells);
        // publishRiskCostCloud();
    }

    // ===========================================================================
    // 路径平滑: 局部移动平均 + 逐点碰撞回退 —— 纯实现
    // 设计见 global_planner.h 中相关声明。自包含, 不引入新依赖。
    // ===========================================================================

    // ---- 碰撞谓词(Step1/Step3 共用) ----
    // 线段 a->b 是否无碰撞: 以体素分辨率为步长逐点 isCellTraversable。
    // 注意 require_ground_support=false —— 平滑不限于地面支撑, 仅查机器人半径足迹碰撞。
    bool GlobalPlanner::isLineCollisionFree(const PointPose & a, const PointPose & b, double extra_radius) const
    {
        if (!octree_) {
            return false;
        }
        const double r = octree_->getResolution();
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 1e-9) {
            return true;
        }
        const double radius = robot_radius_ + extra_radius;
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / r)));
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(steps);
            const GridIndex idx = worldToGrid(a.x + dx * t, a.y + dy * t, a.z + dz * t);
            if (!isCellTraversable(
                    idx, radius, /*require_ground_support=*/false, strict_direct_ground_support_,
                    ground_support_xy_radius_cells_, ground_support_depth_cells_))
            {
                return false;
            }
        }
        return true;
    }

    // ---- 平滑调度: 按 path_smoothing_method_ 选择方法 ----
    std::vector<PointPose> GlobalPlanner::smoothPath(const std::vector<PointPose> & raw) const
    {
        if (path_smoothing_method_ == PathSmoothingMethod::ClampedCubicBspline) {
            return smoothPathBspline(raw);
        }
        return smoothPathMovingAverage(raw);
    }

    // ---- [方法1] 路径平滑: 局部移动平均 + 逐点碰撞回退 ----
    // 对每个内部点取前后 w 个邻居的(端点截断)平均作为平滑候选; 候选与上一个已接受点
    // 之间的线段无碰撞(isLineCollisionFree, 不要求地面支撑)则采用, 否则保留(原始/上一轮)点。
    // 迭代 smoothing_iterations_ 次。起点/终点始终保持。
    //
    // 关键特性:
    //   1) 移动平均是"收缩"运算 -> 始终落在局部凸包内, 不会在尖角处鼓包/回环/重影;
    //   2) 严格按点序逐个输出(每个原始点恰好对应一个输出点) -> 路径不会"走两遍";
    //   3) 逐点碰撞回退 -> 平滑后路径恒不碰撞(最坏退化为原始A*点)。
    std::vector<PointPose> GlobalPlanner::smoothPathMovingAverage(const std::vector<PointPose> & raw) const
    {
        if (!octree_ || raw.size() < 3) {
            return raw;
        }
        const std::size_t N = raw.size();
        const int w = std::max(1, smoothing_window_);
        const int iters = std::max(1, smoothing_iterations_);

        std::vector<PointPose> cur = raw;   // 当前迭代输入
        std::vector<PointPose> nxt;         // 当前迭代输出
        nxt.reserve(N);

        for (int it = 0; it < iters; ++it) {
            nxt.clear();
            nxt.push_back(cur.front());     // 起点保持
            for (std::size_t i = 1; i + 1 < N; ++i) {
                // 截断窗口邻居平均
                PointPose avg{0.0, 0.0, 0.0};
                int cnt = 0;
                const int lo = static_cast<int>(i) - w;
                const int hi = static_cast<int>(i) + w;
                for (int j = lo; j <= hi; ++j) {
                    const int jj = j < 0 ? 0 : (j >= static_cast<int>(N) ? static_cast<int>(N) - 1 : j);
                    avg.x += cur[static_cast<std::size_t>(jj)].x;
                    avg.y += cur[static_cast<std::size_t>(jj)].y;
                    avg.z += cur[static_cast<std::size_t>(jj)].z;
                    ++cnt;
                }
                avg.x /= static_cast<double>(cnt);
                avg.y /= static_cast<double>(cnt);
                avg.z /= static_cast<double>(cnt);
                // 逐点碰撞回退: 候选与上一点之间线段无碰撞才采用, 否则保留当前点
                if (isLineCollisionFree(nxt.back(), avg)) {
                    nxt.push_back(avg);
                } else {
                    nxt.push_back(cur[i]);
                }
            }
            nxt.push_back(cur.back());      // 终点保持
            cur.swap(nxt);
        }
        return cur;
    }

    // ===========================================================================
    // [方法2] 路径平滑: 捷径剪枝 + 钳位三次B样条 + 碰撞回退
    // 注: 三次B样条在尖角处会过冲/鼓包, 可能造成回环重影或偏离贴地路径,
    //     故默认用方法1(移动平均); 此方法作为可选, 由 path_smoothing_method_ 切换。
    // ===========================================================================
    namespace
    {
    // PointPose 线性插值: a + (b-a)*t
    inline PointPose lerpPose(const PointPose & a, const PointPose & b, double t)
    {
        return PointPose{
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t};
    }

    // 钳位(clamped)均匀节点向量: n=控制点最后下标, p=度数; 首/末各 p+1 个重复节点。
    std::vector<double> buildClampedKnots(int n, int p)
    {
        std::vector<double> U(static_cast<std::size_t>(n + p + 2), 0.0);
        const double umax = static_cast<double>(n - p + 1);
        for (int i = 0; i <= p; ++i) {
            U[static_cast<std::size_t>(i)] = 0.0;
            U[static_cast<std::size_t>(n + 1 + i)] = umax;
        }
        for (int j = p + 1; j <= n; ++j) {
            U[static_cast<std::size_t>(j)] = static_cast<double>(j - p);
        }
        return U;
    }

    // 找参数 u 所在 span 下标 k (U[k] <= u < U[k+1]); 钳位向量两端做边界处理。
    int findSpanIndex(int n, int p, double u, const std::vector<double> & U)
    {
        if (u >= U[static_cast<std::size_t>(n) + 1]) { return n; }
        if (u <= U[static_cast<std::size_t>(p)]) { return p; }
        int lo = p;
        int hi = n + 1;
        int mid = (lo + hi) / 2;
        while (u < U[static_cast<std::size_t>(mid)] || u >= U[static_cast<std::size_t>(mid) + 1]) {
            if (u < U[static_cast<std::size_t>(mid)]) { hi = mid; } else { lo = mid; }
            mid = (lo + hi) / 2;
        }
        return mid;
    }

    // de Boor 算法: 求 B 样条在参数 u(位于 span k) 处的点。
    PointPose deBoor(
        const std::vector<PointPose> & d, const std::vector<double> & U, int p, int k, double u)
    {
        std::vector<PointPose> work(static_cast<std::size_t>(p) + 1);
        for (int m = 0; m <= p; ++m) {
            work[static_cast<std::size_t>(m)] = d[static_cast<std::size_t>(k - p + m)];
        }
        for (int r = 1; r <= p; ++r) {
            for (int m = p; m >= r; --m) {
                const std::size_t ia = static_cast<std::size_t>(k - p + m);
                const double num = u - U[ia];
                const double den = U[static_cast<std::size_t>(k + m - r + 1)] - U[ia];
                const double alpha = (den > 1e-12) ? (num / den) : 0.0;
                work[static_cast<std::size_t>(m)] = lerpPose(
                    work[static_cast<std::size_t>(m) - 1], work[static_cast<std::size_t>(m)], alpha);
            }
        }
        return work[static_cast<std::size_t>(p)];
    }
    }  // namespace

    // ---- Step1: 捷径剪枝 ----
    // 贪心 line-of-sight: 从 i 起找最远可直线到达点 j, 保留 i 并跳到 j。
    // 结果为稀疏折线, 每条保留段都经碰撞验证 -> 整条折线保证无碰撞。
    // 用 bspline_clearance_ 膨胀碰撞检测 -> 剪枝后的折线离墙有余量, 给后续B样条圆角留空间。
    // bspline_shortcut_max_jump_>0 时限制单次跳跃点数 -> 避免过度抽稀(控制点<4会退化为直线)。
    std::vector<PointPose> GlobalPlanner::shortcutPath(const std::vector<PointPose> & raw) const
    {
        std::vector<PointPose> pruned;
        if (raw.empty()) { return pruned; }
        pruned.push_back(raw.front());
        std::size_t i = 0;
        while (i + 1 < raw.size()) {
            std::size_t next = i + 1;
            // 单次跳跃上界: 无限制或 i + max_jump
            std::size_t j_max = raw.size() - 1;
            if (bspline_shortcut_max_jump_ > 0) {
                const std::size_t cap = i + static_cast<std::size_t>(bspline_shortcut_max_jump_);
                j_max = (cap < j_max) ? cap : j_max;
            }
            for (std::size_t j = j_max; j > i; --j) {
                if (isLineCollisionFree(raw[i], raw[j], bspline_clearance_)) { next = j; break; }
            }
            pruned.push_back(raw[next]);
            i = next;
        }
        return pruned;
    }

    // ---- Step2: 钳位三次B样条密集采样 ----
    // 以 pts 为控制点(度数 p=3, 钳位节点向量) -> 精确经过首/末控制点; 控制点<4 退化为折线插值。
    std::vector<PointPose> GlobalPlanner::sampleClampedCubicBspline(
        const std::vector<PointPose> & pts) const
    {
        std::vector<PointPose> out;
        const std::size_t N = pts.size();
        if (N == 0) { return out; }
        if (N == 1) { out.push_back(pts.front()); return out; }

        const int M = std::max(1, bspline_samples_per_segment_);
        const int p = 3;

        if (static_cast<int>(N) < p + 1) {
            out.push_back(pts.front());
            for (std::size_t i = 0; i + 1 < N; ++i) {
                for (int s = 1; s <= M; ++s) {
                    const double t = static_cast<double>(s) / static_cast<double>(M);
                    out.push_back(lerpPose(pts[i], pts[i + 1], t));
                }
            }
            return out;
        }

        const int n = static_cast<int>(N) - 1;
        const std::vector<double> U = buildClampedKnots(n, p);
        const double umax = U[static_cast<std::size_t>(n + 1)];
        const int spans = n - p + 1;
        const int total = spans * M;

        out.reserve(static_cast<std::size_t>(total + 1));
        out.push_back(pts.front());
        for (int s = 1; s < total; ++s) {
            const double u = umax * static_cast<double>(s) / static_cast<double>(total);
            const int k = findSpanIndex(n, p, u, U);
            out.push_back(deBoor(pts, U, p, k, u));
        }
        out.push_back(pts.back());
        return out;
    }

    // ---- Step3: 细粒度碰撞回退拼接 ----
    // 沿 smooth 曲线逐点前进(相邻小弦保留曲线); 一旦某弦碰撞, 回退到 pruned 安全骨架桥接。
    std::vector<PointPose> GlobalPlanner::assembleCollisionAware(
        const std::vector<PointPose> & smooth, const std::vector<PointPose> & pruned) const
    {
        std::vector<PointPose> result;
        if (smooth.empty()) { return result; }
        const std::size_t K = smooth.size();
        const std::size_t P = pruned.size();

        result.push_back(smooth.front());
        std::size_t p_head = 0;
        std::size_t k = 1;
        while (k < K) {
            const PointPose & anchor = result.back();
            if (isLineCollisionFree(anchor, smooth[k])) {
                result.push_back(smooth[k]);
                ++k;
                continue;
            }
            bool spliced = false;
            for (std::size_t pi = p_head; pi < P; ++pi) {
                if (isLineCollisionFree(anchor, pruned[pi])) {
                    result.push_back(pruned[pi]);
                    p_head = pi + 1;
                    std::size_t best = k;
                    double best_d = std::numeric_limits<double>::infinity();
                    for (std::size_t m = k; m < K; ++m) {
                        const double ddx = smooth[m].x - pruned[pi].x;
                        const double ddy = smooth[m].y - pruned[pi].y;
                        const double ddz = smooth[m].z - pruned[pi].z;
                        const double d = ddx * ddx + ddy * ddy + ddz * ddz;
                        if (d < best_d) { best_d = d; best = m; }
                    }
                    k = best + 1;
                    spliced = true;
                    break;
                }
            }
            if (!spliced) { ++k; }
        }

        if (P > 0) {
            const PointPose & goal = pruned.back();
            const PointPose & tail = result.back();
            const bool at_goal = (tail.x == goal.x && tail.y == goal.y && tail.z == goal.z);
            if (!at_goal) {
                if (isLineCollisionFree(tail, goal)) {
                    result.push_back(goal);
                } else {
                    for (std::size_t pi = p_head; pi < P; ++pi) {
                        if (isLineCollisionFree(result.back(), pruned[pi])) {
                            result.push_back(pruned[pi]);
                        }
                    }
                }
            }
        }
        return result;
    }

    // ---- [方法2] 编排: 剪枝 -> B样条 -> 碰撞回退 ----
    std::vector<PointPose> GlobalPlanner::smoothPathBspline(const std::vector<PointPose> & raw) const
    {
        if (!octree_ || raw.size() < 2) { return raw; }
        const std::vector<PointPose> pruned = shortcutPath(raw);
        if (pruned.size() < 2) { return raw; }
        const std::vector<PointPose> smooth = sampleClampedCubicBspline(pruned);
        if (smooth.size() < 2) { return pruned; }
        return assembleCollisionAware(smooth, pruned);
    }








}