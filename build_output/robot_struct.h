/**
 * @file  robot_struct.h
 * @brief 外骨骼机器人全局类型定义（原 JointDef.h / RobotConfig.h 已整合至此）
 *
 * 本文件是 robot_manager 层与 robot_algorithm 层共享的唯一类型头文件，
 * 不引入任何控制算法实现，可被上层软件直接包含。
 *
 * 包含内容（由上到下）：
 *   ① JointID / JointLimit / ControlMode 枚举与结构体（原 JointDef.h）
 *   ② namespace RobotConfig  关节限位默认值与工具函数（原 RobotConfig.h）
 *   ③ ImuID / Quaternion     IMU 节段枚举与四元数结构体
 *   ④ RobotData / RobotStatusMachine / RobotStatus / JointEnableConfig
 *   ⑤ RobotOutput / JointLimitTable
 */

#pragma once

#include <cstdint>
#include <algorithm>   // std::clamp
#include <cmath>       // std::abs

/* =========================================================
 * ① 关节枚举、限位结构体、控制模式（原 JointDef.h）
 * ========================================================= */

/**
 * @brief 全身 16 关节枚举
 *
 * 关节布局：
 *   下肢（单腿 3 × 2）：髋屈伸 / 膝屈伸 / 踝背跖屈
 *   上肢（单臂 5 × 2）：肩屈伸 / 肩外展 / 肩旋转 / 肘屈伸 / 腕屈伸
 *
 * 符号约定：零点 = 解剖学中立位；
 *   下肢：髋 + = 屈髋，膝 − = 屈膝，踝 + = 背屈
 *   上肢：+ = 生理学意义的屈 / 外展 / 外旋方向
 */
enum class JointID : int
{
    // ── 左下肢 ──────────────────────────────────────────
    L_HIP           = 0,           ///< 髋屈伸：+ 屈髋（大腿前倾），− 伸髋
    L_KNEE          = 1,                ///< 膝屈伸：− 屈膝（小腿后收），0 = 伸直
    L_ANKLE         = 2,               ///< 踝背跖屈：+ 背屈（勾脚），− 跖屈（踮脚）

    // ── 右下肢 ──────────────────────────────────────────
    R_HIP           = 3,                 ///< 同左，镜像
    R_KNEE          = 4,
    R_ANKLE         = 5,

    // ── 左上肢 ──────────────────────────────────────────
    L_SHOULDER_FLEX = 6,       ///< 肩屈伸：+ 屈（前举），− 伸（后伸）
    L_SHOULDER_ABD  = 7,        ///< 肩外展：+ 外展，− 内收
    L_SHOULDER_ROT  = 8,        ///< 肩旋转：+ 外旋，− 内旋
    L_ELBOW         = 9,               ///< 肘屈伸：+ 屈，− 伸（解剖零位 = 伸直）
    L_WRIST         = 10,               ///< 腕屈伸：+ 掌屈，− 背伸

    // ── 右上肢 ──────────────────────────────────────────
    R_SHOULDER_FLEX = 11,       ///< 同左，镜像
    R_SHOULDER_ABD  = 12,
    R_SHOULDER_ROT  = 13,
    R_ELBOW         = 14,
    R_WRIST         = 15,

    JOINT_COUNT     = 16       ///< 关节总数，用于数组定义
};

/**
 * @brief 关节约束参数
 *
 *   tau_max    N·m   峰值力矩（主循环截断上限）
 *   tau_rated  N·m   额定连续力矩（过载告警阈值）
 *   dq_max     rad/s 最大角速度
 *   q_min      rad   关节角软限位下限
 *   q_max      rad   关节角软限位上限
 *
 * 软限位（q_min / q_max）应比硬限位（机械挡块）各留 5°~10° 安全余量。
 */
struct JointLimit
{
    double tau_max;    ///< N·m  峰值力矩（主循环截断上限）
    double tau_rated;  ///< N·m  额定连续力矩（过载告警阈值）
    double dq_max;     ///< rad/s 最大角速度
    double q_min;      ///< rad  关节角软限位下限
    double q_max;      ///< rad  关节角软限位上限
};

/** @brief 将 JointID 转为数组下标 */
constexpr int toIdx(JointID j) noexcept { return static_cast<int>(j); }

/** @brief 判断 j 是否属于下肢 */
constexpr bool isLegJoint(JointID j) noexcept
{
    const int i = toIdx(j);
    return i >= toIdx(JointID::L_HIP) && i <= toIdx(JointID::R_ANKLE);
}

/** @brief 判断 j 是否属于上肢 */
constexpr bool isArmJoint(JointID j) noexcept
{
    const int i = toIdx(j);
    return i >= toIdx(JointID::L_SHOULDER_FLEX) && i <= toIdx(JointID::R_WRIST);
}

/** @brief 判断 j 是否属于左侧 */
constexpr bool isLeft(JointID j) noexcept
{
    const int i = toIdx(j);
    return (i >= toIdx(JointID::L_HIP)           && i <= toIdx(JointID::L_ANKLE))
        || (i >= toIdx(JointID::L_SHOULDER_FLEX) && i <= toIdx(JointID::L_WRIST));
}

/**
 * @brief 关节控制模式
 *
 *   TRANSPARENCY  透明模式：仅执行摩擦补偿，患者自由活动
 *   ASSIST        助力模式：AAN 变阻抗控制律，每周期由 setTarget() 输入期望轨迹
 *   ZERO_FORCE    零力模式：扭矩传感器比例反馈，实时使接触力矩归零
 */
enum class ControlMode
{
    TRANSPARENCY,
    ASSIST,
    ZERO_FORCE,
};

/* =========================================================
 * ② 关节限位默认值与工具函数（原 RobotConfig.h / namespace RobotConfig）
 *
 * 下表所有数值为占位默认值，需根据实际电机规格书和人体 ROM 标定后替换。
 * 顺序须与 JointID 枚举一致（下标 = toIdx(JointID)）。
 *
 * 列：{ tau_max,  tau_rated,  dq_max,   q_min,   q_max  }
 *      N·m       N·m         rad/s      rad      rad
 * ========================================================= */

namespace RobotConfig
{

//                                          ── 活动度参考 ──────────────────────────────
//  关节              tau_max  tau_rated  dq_max   q_min         q_max
//                    N·m      N·m        rad/s    rad           rad
inline constexpr JointLimit kLimits[static_cast<int>(JointID::JOINT_COUNT)] =
{
    // ── 左下肢 ──────────────────────────────────────────────────────────────────────
    // L_HIP            髋屈伸   + 屈髋        参考 ROM：−20° ~ 80°
    { 120.0,  80.0,  4.0,  -0.35,  1.40 },
    // L_KNEE           膝屈伸   − 屈膝        参考 ROM：−120° ~ 0°
    { 120.0,  80.0,  4.0,  -2.09,  0.00 },
    // L_ANKLE          踝背跖屈 + 背屈        参考 ROM：−30° ~ 30°
    {  60.0,  40.0,  6.0,  -0.52,  0.52 },

    // ── 右下肢（与左侧相同） ────────────────────────────────────────────────────────
    { 120.0,  80.0,  4.0,  -0.35,  1.40 },  // R_HIP
    { 120.0,  80.0,  4.0,  -2.09,  0.00 },  // R_KNEE
    {  60.0,  40.0,  6.0,  -0.52,  0.52 },  // R_ANKLE

    // ── 左上肢 ──────────────────────────────────────────────────────────────────────
    // L_SHOULDER_FLEX  肩屈伸   + 屈（前举）  参考 ROM：−30° ~ 170°
    {  60.0,  40.0,  3.0,  -0.52,  2.97 },
    // L_SHOULDER_ABD   肩外展   + 外展        参考 ROM：−15° ~ 90°
    {  50.0,  35.0,  3.0,  -0.26,  1.57 },
    // L_SHOULDER_ROT   肩旋转   + 外旋        参考 ROM：−90° ~ 90°
    {  20.0,  15.0,  4.0,  -1.57,  1.57 },
    // L_ELBOW          肘屈伸   + 屈          参考 ROM：0° ~ 140°
    {  40.0,  25.0,  4.0,   0.00,  2.44 },
    // L_WRIST          腕屈伸   + 掌屈        参考 ROM：−70° ~ 70°
    {  15.0,  10.0,  5.0,  -1.22,  1.22 },

    // ── 右上肢（与左侧相同） ────────────────────────────────────────────────────────
    {  60.0,  40.0,  3.0,  -0.52,  2.97 },  // R_SHOULDER_FLEX
    {  50.0,  35.0,  3.0,  -0.26,  1.57 },  // R_SHOULDER_ABD
    {  20.0,  15.0,  4.0,  -1.57,  1.57 },  // R_SHOULDER_ROT
    {  40.0,  25.0,  4.0,   0.00,  2.44 },  // R_ELBOW
    {  15.0,  10.0,  5.0,  -1.22,  1.22 },  // R_WRIST
};

/** @brief 返回指定关节的约束参数（const 引用，零开销） */
inline const JointLimit& limit(JointID j) noexcept
{
    return kLimits[toIdx(j)];
}

/** @brief 力矩截断（双向对称限幅） */
inline double clampTau(JointID j, double tau) noexcept
{
    const double mx = kLimits[toIdx(j)].tau_max;
    return std::clamp(tau, -mx, mx);
}

/** @brief 速度截断（双向对称限幅） */
inline double clampDq(JointID j, double dq) noexcept
{
    const double mx = kLimits[toIdx(j)].dq_max;
    return std::clamp(dq, -mx, mx);
}

/** @brief 检查关节角是否在软限位内（true = 安全） */
inline bool inRange(JointID j, double q) noexcept
{
    const JointLimit& lim = kLimits[toIdx(j)];
    return q >= lim.q_min && q <= lim.q_max;
}

/** @brief 将关节角钳位到软限位范围内 */
inline double clampQ(JointID j, double q) noexcept
{
    const JointLimit& lim = kLimits[toIdx(j)];
    return std::clamp(q, lim.q_min, lim.q_max);
}

/** @brief 检查力矩是否超过额定值（true = 超额定，持续输出可能过热） */
inline bool isOverRated(JointID j, double tau) noexcept
{
    return std::abs(tau) > kLimits[toIdx(j)].tau_rated;
}

} // namespace RobotConfig

/* =========================================================
 * 全局常量
 * ========================================================= */

static constexpr int JOINT_NUM = static_cast<int>(JointID::JOINT_COUNT);  // 16

/* =========================================================
 * ③ IMU 节段枚举与四元数
 * ========================================================= */

/**
 * @brief IMU 安装节段标识
 *
 * 下标布局：
 *   下肢  [0] L_THIGH  [1] L_SHANK  [2] L_FOOT
 *         [3] R_THIGH  [4] R_SHANK  [5] R_FOOT
 *   上肢  [6] L_UPPER_ARM  [7] L_FOREARM
 *         [8] R_UPPER_ARM  [9] R_FOREARM
 */
enum class ImuID
{
    L_THIGH     = 0,
    L_SHANK     = 1,
    L_FOOT      = 2,
    R_THIGH     = 3,
    R_SHANK     = 4,
    R_FOOT      = 5,
    L_UPPER_ARM = 6,
    L_FOREARM   = 7,
    R_UPPER_ARM = 8,
    R_FOREARM   = 9,
    IMU_COUNT   = 10,
};

static constexpr int IMU_NUM = static_cast<int>(ImuID::IMU_COUNT);  // 10

inline int toImuIdx(ImuID id) { return static_cast<int>(id); }

struct Quaternion
{
    double w = 1.0;  ///< 实部（单位四元数默认值）
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/* =========================================================
 * ④ 机器人数据结构
 * ========================================================= */

/**
 * @brief 机器人固有参数（上电后由硬件配置层填写，运行期间只读）
 */
struct RobotData
{
    int  active_joint_num = 0;          // 本次上电实际激活的关节数量
    int  total_joint_num  = JOINT_NUM;  // 全身关节总数（固定为 16）
    double dt_s  = 0.004;              // 控制周期（s）
    int    dt_ms = 4;                  // 控制周期（ms）
    char robot_name[32] = "exo_default";
    bool is_inited = false;
};

/**
 * @brief 整机状态机
 */
enum class RobotStatusMachine
{
    UNINITIALIZED = -2, ///< 未初始化：init() 尚未调用
    ERROR         = -1, ///< 故障：通信失败超限，需调用 clearError() 后恢复
    STANDBY       =  0, ///< 就绪：init() 完成，等待 start()
    PAUSING       =  1, ///< 暂停中：正在减速至零力矩，过渡到 STANDBY 的中间态
    RUNNING       =  2, ///< 运行：updateAll() 正常执行中
};

/**
 * @brief 全身关节实时状态（由 Robot::updateAll() 每控制周期刷新，外部只读）
 *
 * 所有数组下标对应 toIdx(JointID)。
 */
struct RobotStatus
{
    static constexpr int JOINT_NUM = ::JOINT_NUM;
    static constexpr int IMU_NUM   = ::IMU_NUM;

    // ── 下标 → 关节映射 ──────────────────────────────────────────────────────
    //  左下肢  [0] L_HIP  [1] L_KNEE  [2] L_ANKLE
    //  右下肢  [3] R_HIP  [4] R_KNEE  [5] R_ANKLE
    //  左上肢  [6] L_SHOULDER_FLEX  [7] L_SHOULDER_ABD  [8] L_SHOULDER_ROT
    //          [9] L_ELBOW  [10] L_WRIST
    //  右上肢  [11] R_SHOULDER_FLEX [12] R_SHOULDER_ABD [13] R_SHOULDER_ROT
    //          [14] R_ELBOW  [15] R_WRIST
    // ─────────────────────────────────────────────────────────────────────────

    // ── 实际反馈（外圈编码器 / 力矩传感器）────────────────────────────────────
    double act_q      [JOINT_NUM]{};  // 关节实际角度（rad）
    double act_dq     [JOINT_NUM]{};  // 关节实际角速度（rad/s）
    double act_torque [JOINT_NUM]{};  // 关节实际力矩（N·m）

    // ── 实际反馈（内圈编码器）────────────────────────────────────────────────
    double act_motor_q  [JOINT_NUM]{};  // 电机输出轴角度（rad）
    double act_motor_dq [JOINT_NUM]{};  // 电机输出轴角速度（rad/s）
    double act_motor_i  [JOINT_NUM]{};  // 电机电流（A）

    // ── 期望指令（控制器下发值）────────────────────────────────────────────────
    double cmd_q      [JOINT_NUM]{};  // 期望位置（rad）
    double cmd_dq     [JOINT_NUM]{};  // 期望速度（rad/s）
    double cmd_ddq    [JOINT_NUM]{};  // 期望加速度（rad/s²）

    // ── 跟踪误差 ──────────────────────────────────────────────────────────────
    double q_err  [JOINT_NUM]{};  // 位置误差  act_q  − cmd_q  （rad）
    double dq_err [JOINT_NUM]{};  // 速度误差  act_dq − cmd_dq （rad/s）

    // ── IMU 四元数（下标 = toImuIdx(ImuID)）──────────────────────────────────
    //  下肢  [0] L_THIGH  [1] L_SHANK  [2] L_FOOT
    //        [3] R_THIGH  [4] R_SHANK  [5] R_FOOT
    //  上肢  [6] L_UPPER_ARM  [7] L_FOREARM
    //        [8] R_UPPER_ARM  [9] R_FOREARM
    Quaternion imu_quat[IMU_NUM]{};

    void clear() { *this = RobotStatus{}; }
};

/**
 * @brief 关节使能配置表（链式调用）
 *
 *   JointEnableConfig cfg;
 *   cfg.enableLegs(ControlMode::TRANSPARENCY)
 *      .enable(JointID::L_HIP, ControlMode::ASSIST);
 */
struct JointEnableConfig
{
    bool        enabled [JOINT_NUM]{};
    ControlMode mode    [JOINT_NUM]{};

    JointEnableConfig& enable(JointID jid, ControlMode m = ControlMode::TRANSPARENCY)
    {
        enabled[toIdx(jid)] = true;
        mode   [toIdx(jid)] = m;
        return *this;
    }

    JointEnableConfig& disable(JointID jid)
    {
        enabled[toIdx(jid)] = false;
        return *this;
    }

    JointEnableConfig& enableAll(ControlMode m = ControlMode::TRANSPARENCY)
    {
        for (int i = 0; i < JOINT_NUM; ++i) { enabled[i] = true; mode[i] = m; }
        return *this;
    }

    JointEnableConfig& enableLegs(ControlMode m = ControlMode::TRANSPARENCY)
    {
        for (int i = 0; i < JOINT_NUM; ++i)
            if (isLegJoint(static_cast<JointID>(i))) { enabled[i] = true; mode[i] = m; }
        return *this;
    }

    JointEnableConfig& enableArms(ControlMode m = ControlMode::TRANSPARENCY)
    {
        for (int i = 0; i < JOINT_NUM; ++i)
            if (isArmJoint(static_cast<JointID>(i))) { enabled[i] = true; mode[i] = m; }
        return *this;
    }

    JointEnableConfig& disableAll()
    {
        for (int i = 0; i < JOINT_NUM; ++i) enabled[i] = false;
        return *this;
    }
};

/* =========================================================
 * ⑤ 整机输出与关节限位表
 * ========================================================= */

/**
 * @brief 整机输出（每控制周期由 updateAll 填充，供上层软件读取）
 *
 * TRANSPARENCY 模式：cmd_q/dq = act_q/dq，cmd_ddq = 0
 * ASSIST       模式：cmd_q/dq/ddq = 本周期 setTarget() 输入值
 */
struct RobotOutput
{
    double cmd_q  [JOINT_NUM]{};
    double cmd_dq [JOINT_NUM]{};
    double cmd_ddq[JOINT_NUM]{};
    double cmd_tau[JOINT_NUM]{};

    void clear() { *this = RobotOutput{}; }
};

/**
 * @brief 全身关节限位表
 *
 * Robot 构造时从 RobotConfig::kLimits 自动填充默认值；
 * 上层软件可在 init() 前通过链式 .set() 覆盖特定关节的限位。
 *
 * 示例：
 *   g_robot.limit_table
 *       .set(JointID::L_HIP,   { 120.0, 80.0, 4.0, -0.35, 1.40 })
 *       .set(JointID::L_KNEE,  { 120.0, 80.0, 4.0, -2.09, 0.00 });
 */
struct JointLimitTable
{
    JointLimit limits[JOINT_NUM]{};

    JointLimitTable& set(JointID jid, const JointLimit& lim)
    {
        limits[toIdx(jid)] = lim;
        return *this;
    }

    const JointLimit& operator[](JointID jid) const { return limits[toIdx(jid)]; }
    JointLimit&       operator[](JointID jid)       { return limits[toIdx(jid)]; }
};
