/**
 * @file  robot.h
 * @brief 外骨骼整机管理类（纯软件数据层）
 *
 * 职责：
 *   - 根据 JointEnableConfig 管理激活关节集合
 *   - 接收上层每周期传入的传感器数据（setFeedback）
 *   - 维护全身状态（status）与期望输出（output）
 *   - 执行软限位 / 超额定力矩安全保护
 *   - 管理整机状态机（STANDBY / RUNNING / PAUSING / ERROR）
 *
 * 控制模式：
 *   TRANSPARENCY  透明模式：cmd 跟随 act，q_err 恒为零
 *   ZERO_FORCE    零力模式：导纳控制，扭矩传感器驱动
 *   ASSIST        助力模式：cmd 取每周期 setTarget() 输入值
 *
 * 线程安全：
 *   setFeedback / setTarget / setMode 与 updateAll 共享内部数据，
 *   均受 mtx_ 保护，可从不同线程调用。
 *
 * 全局单例：
 *   仅在一个 .cpp 中定义前置宏，其余文件得到 extern 声明：
 *     #define ROBOT_GLOBAL
 *     #include "robot.h"
 */

#pragma once

#include <mutex>
#include <cmath>
#include <memory>

#include "robot_struct.h"
#include "TextLogger.h"
#include "JogController.h"
#include "JointControlTable.h"

class Robot
{
public:

    Robot();
    ~Robot() { log_.save("robot_run.log"); }

    Robot(const Robot&)            = delete;
    Robot& operator=(const Robot&) = delete;

    // ── 公开数据（updateAll 后只读）──────────────────────────────────────────

    RobotData       data;         ///< 固有参数：dt_s、active_joint_num、robot_name
    RobotStatus     status;       ///< 全身关节实时状态（反馈 + 指令 + 误差）
    RobotOutput     output;       ///< 期望输出：cmd_q / cmd_dq / cmd_ddq / cmd_tau
    JointLimitTable limit_table;  ///< 关节限位表；init() 前可通过 .set() 覆盖默认值

    // ── 生命周期 ──────────────────────────────────────────────────────────────

    /** @brief 初始化整机；至少一个关节使能才返回 true */
    bool init(JointEnableConfig& cfg,
              double dt_s      = 0.004,
              const char* name = "exo_default");

    /** @brief STANDBY → RUNNING */
    bool start();

    /** @brief RUNNING → STANDBY（立即） */
    void stop();

    /** @brief RUNNING → PAUSING → STANDBY（锁定当前角度，400 ms 渐停） */
    void pause();

    /** @brief 任意状态 → ERROR（立即清零输出） */
    void emergencyStop();

    // ── 传感器输入（每周期 updateAll 前调用）─────────────────────────────────

    /** @brief 单关节输入（含 NaN/Inf 检查） */
    void setFeedback(JointID jid, double q, double dq, double tau = 0.0);

    /**
     * @brief 全身批量输入（线程安全）
     * @param motor_q  内圈编码器角度（rad）
     * @param q        外圈编码器角度（rad）
     * @param dq       关节角速度（rad/s）
     * @param tau      关节力矩（N·m），nullptr 则保持上一周期值
     */
    void setFeedback(const double motor_q[], const double q[],
                     const double dq[], const double tau[] = nullptr);

    // ── 期望轨迹输入（ASSIST 模式，每周期 updateAll 前调用）─────────────────

    /** @brief 设置单关节期望轨迹（位置 rad、速度 rad/s、加速度 rad/s²） */
    void setTarget(JointID jid, double q_d, double dq_d = 0.0, double ddq_d = 0.0);

    // ── 控制步进 ──────────────────────────────────────────────────────────────

    /**
     * @brief 刷新 status / output，执行安全检查；RUNNING 或 PAUSING 时返回 true
     *
     * 软限位或超额定力矩连续违规超过阈值后自动切换到 ERROR。
     */
    bool updateAll();

    // ── 控制模式（线程安全）──────────────────────────────────────────────────

    void setMode(JointID jid, ControlMode mode);
    void setModeAll(ControlMode mode);

    // ── 状态查询 ──────────────────────────────────────────────────────────────

    RobotStatusMachine getState()           const { return state_; }
    ControlMode        getMode(JointID jid) const { return runtime_[toIdx(jid)].mode; }
    bool               isActive(JointID jid)const { return active_[toIdx(jid)]; }
    int                getActiveCount()     const { return data.active_joint_num; }
    bool               isOverRated(JointID jid) const;

    /** @brief ERROR → STANDBY（重置所有控制器状态） */
    void clearError();

    // ── 阻抗控制参数 ──────────────────────────────────────────────────────────

    /** @brief 更新全身参数表；init() 前调用则在 init() 时生效，之后调用立即重建控制器 */
    void setControlTable(const JointControlTable& table);

    /** @brief 更新单关节参数；init() 后调用立即重建该关节控制器 */
    void setControlParams(JointID jid, const ImpedanceParams& p);

    // ── 点动 ─────────────────────────────────────────────────────────────────

    /** @brief 覆盖点动速度 / 加速度 / jerk（软限位取自 limit_table） */
    void jogConfig(JointID jid,
                   double v_max = 0.05,
                   double a_max = 0.20,
                   double j_max = 1.00);

    /**
     * @brief 触发点动：空闲时启动，运动中触发则减速停止
     *
     * 点动期间 updateAll 自动输出双 S 曲线轨迹，结束后恢复原 ControlMode。
     */
    void jogTrigger(JointID jid, JogController::Direction dir = JogController::Direction::POS);

    /** @brief 立即锁定当前角度，终止点动 */
    void jogEstop(JointID jid);

    /** @brief 点动是否处于空闲（未触发或已减速停止） */
    bool jogIsIdle(JointID jid) const;

private:

    static constexpr int N                   = JOINT_NUM;
    static constexpr int VIOLATION_THRESHOLD = 10;   // 连续违规 10 次触发 ERROR
    static constexpr int PAUSING_CYCLES      = 100;  // 渐停周期数（4 ms × 100 = 400 ms）

    bool active_[N]{};

    struct JointRuntime
    {
        ControlMode mode  = ControlMode::TRANSPARENCY;
        double      q_d   = 0.0;
        double      dq_d  = 0.0;
        double      ddq_d = 0.0;
    };
    JointRuntime runtime_[N];

    double q_locked_[N]{};         // pause() 时锁定的关节角
    int    violation_count_[N]{};  // 每关节连续违规计数
    int    pausing_tick_ = 0;      // PAUSING 已持续周期数

    RobotStatusMachine state_ = RobotStatusMachine::UNINITIALIZED;

    std::unique_ptr<JogController> joggers_[N];
    bool jog_active_[N]{};

    JointControlTable  ctrl_table_;
    std::unique_ptr<ImpedanceController> controllers_[N];

    mutable std::mutex mtx_;
    TextLogger         log_;
};

#ifdef ROBOT_GLOBAL
#  define ROBOT_EXT
#else
#  define ROBOT_EXT extern
#endif
ROBOT_EXT Robot g_robot;
