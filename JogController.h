#pragma once

#include <cstdint>

/**
 * @file   JogController.h
 * @brief  单关节点动控制器（双S型速度曲线 + 位置P控制）
 *
 * 控制逻辑
 * ─────────────────────────────────────────────────────────────
 * 调用 trigger(dir) 切换运动状态：
 *   空闲时  → 向 dir 方向启动，速度按双S曲线加速至 v_max
 *   运动中  → 触发双S减速停止（dir 参数忽略）
 *
 * 每个控制周期调用 update(q_meas) 获取电流指令：
 *   控制器内部维护目标位置 q_target，按双S速度曲线累加；
 *   位置P控制器将误差转换为电流输出。
 *
 * 关节限位保护：在减速停止距离内自动触发减速，不超限位。
 *
 * 状态转换
 * ─────────────────────────────────────────────────────────────
 *
 *             trigger(dir)
 *   IDLE ──────────────────→ ACCEL
 *    ↑                         │ 加速完成
 *    │                         ↓
 *    │       trigger(*)     CRUISE
 *    │    ╭──────────────←    │
 *    │    │                trigger(*)
 *    │    ↓                   ↓
 *    └─── IDLE ←──── DECEL ───╯
 *          速度归零     ↑
 *                       │ 到达限位
 *                   CRUISE/ACCEL
 *
 * 用法示例：
 *   JogController::Params p;
 *   p.v_max = 0.1;   p.a_max = 0.5;   p.j_max = 2.0;
 *   p.q_min_rad = -1.2;   p.q_max_rad = 0.0;
 *   JogController jog(p);
 *
 *   // 按钮按下时调用（开始正方向点动）：
 *   jog.trigger(JogController::Direction::POS);
 *
 *   // 控制循环（须以 p.dt 为周期调用）：
 *   int32_t iq_mA = jog.update(q_meas);
 *   motor.setCurrentMA(iq_mA);
 */
class JogController
{
public:
    enum class State     { IDLE, ACCEL, CRUISE, DECEL };
    enum class Direction { POS = +1, NEG = -1 };

    struct Params
    {
        // ── 速度曲线约束 ──────────────────────────────────────────────────────
        double v_max    = 0.10;   ///< 点动速度（rad/s）
        double a_max    = 0.50;   ///< 峰值加速度（rad/s²）
        double j_max    = 2.00;   ///< 峰值 jerk（rad/s³）

        // ── 关节限位（rad）────────────────────────────────────────────────────
        double q_min_rad = -3.14; ///< 关节角下限
        double q_max_rad =  3.14; ///< 关节角上限

        // ── 位置 P 控制 ───────────────────────────────────────────────────────
        double Kp_mA_rad = 2000.0; ///< 比例增益（mA/rad）
        double iq_max_mA = 3000.0; ///< 电流限幅（mA）

        // ── 控制周期 ──────────────────────────────────────────────────────────
        double dt = 0.01;          ///< 控制周期（s），须与实际调用频率严格一致
    };

    explicit JogController(const Params& p);

    /**
     * @brief 点动触发（按钮按下时调用一次）
     *
     * 空闲 → 启动朝 dir 方向的点动（双S加速）
     * 运动中 → 触发双S减速停止（dir 参数在此时忽略）
     *
     * @param dir  运动方向，空闲时生效
     */
    void trigger(Direction dir);

    /**
     * @brief 控制周期更新（每 dt 秒调用一次）
     *
     * 更新目标位置和速度曲线状态，输出P控制电流指令。
     * 调用频率须与 Params::dt 严格一致。
     *
     * @param q_meas  当前关节角测量值（rad）
     * @return        电流指令（mA），已限幅在 ±iq_max_mA
     */
    int32_t update(double q_meas);

    /**
     * @brief 急停：立即将目标位置锁定在当前位置，归零速度
     * @param q_meas  当前关节角测量值（rad）
     */
    void estop(double q_meas);

    State  getState()     const { return state_; }
    double getTargetPos() const { return q_target_; }
    double getVelocity()  const { return vel_; }
    bool   isIdle()       const { return state_ == State::IDLE; }

private:
    Params p_;
    State  state_    = State::IDLE;
    int    dir_      = +1;      ///< 当前运动方向（+1 / -1）
    double vel_      = 0.0;     ///< 当前速度（rad/s，非负，方向由 dir_ 决定）
    double q_target_ = 0.0;    ///< 目标位置（rad）
    double phase_t_  = 0.0;    ///< 当前段内已过时间（s）

    // 速度斜坡参数（加速/减速段各自规划一次）
    struct VelRamp {
        double v0      = 0.0;
        double v1      = 0.0;
        double T_j     = 0.0;   ///< jerk 段时长
        double T_a     = 0.0;   ///< 恒加减速段时长
        double a_peak  = 0.0;
        double j_max   = 0.0;
        double total_time = 0.0;
        double total_dist = 0.0;  ///< 斜坡期间的总位移（非负）

        void   plan(double v0, double v1, double a_max, double j_max);
        double sampleVel(double t) const;
    };

    VelRamp ramp_;  ///< 当前激活的速度斜坡（ACCEL 或 DECEL 阶段使用）

    void startDecel();                  // 从当前速度规划减速斜坡
    int32_t posCtrl(double q_meas) const;
};
