/**
 * @file   Impedance_controller.h
 * @brief  单关节阻抗控制器（TRANSPARENCY / ASSIST / ZERO_FORCE）
 *
 * 控制模式
 * ────────────────────────────────────────────────────────────
 *   TRANSPARENCY  透明模式
 *     q_d/dq_d 实时跟随编码器，误差恒零，等效纯摩擦补偿。
 *
 *   ASSIST  AAN 变阻抗助力模式
 *     ε = −q_err − β·dq_err
 *     λ = a / (1 + b·ε²)
 *     τ = (ε/λ)·(q_err² + dq_err² + 1)
 *
 *   ZERO_FORCE  导纳零力模式
 *     导纳层（扭矩传感器 → 运动参考）：
 *       τ_eff = deadband(τ_sensor, tau_deadband)
 *       v_d   = τ_eff / B_v
 *       q_d  += v_d · dt          （q_d 限幅在 q ± 1 rad 内）
 *     PD 跟踪（直接用传感器，无观测器）：
 *       τ = Kp_zf·(q_d − q_outer) + Kd_zf·(v_d − dq_inner) + τ_fric
 *
 * 传感器输入（通过 update() 参数传入）
 * ────────────────────────────────────────────────────────────
 *   q_act   ← 外圈编码器单圈角（关节位置，无回程间隙）
 *   dq_act  ← 内圈编码器速度（高频更新，用于速度反馈）
 *   tau_act ← 扭矩传感器（ZERO_FORCE 导纳输入）
 */
#pragma once

#include "robot_struct.h"

/* =========================================================
 * ImpedanceParams  控制器参数
 * ========================================================= */

struct ImpedanceParams
{
    ControlMode mode = ControlMode::ASSIST;  ///< 初始控制模式

    // ── 目标阻抗（TRANSPARENCY / ASSIST）─────────────────────────────────────
    double M       = 0.05;   ///< 虚拟惯量（kg·m²）；ZERO_FORCE 中兼作 J_n
    double K       = 5.0;    ///< 虚拟刚度（N·m/rad）；阻尼 B = 2√(MK) 自动计算

    // ── 电机参数 ──────────────────────────────────────────────────────────────
    double Kt      = 4.53;   ///< 力矩常数（N·m/A）
    double tau_max = 3.0;    ///< 力矩饱和限幅（N·m）

    // ── 摩擦模型 ──────────────────────────────────────────────────────────────
    double B_vis       = 0.0;  ///< 粘性摩擦系数（N·m·s/rad）
    double tau_coulomb = 0.0;  ///< 库伦摩擦幅值（N·m）
    double v_s         = 0.1;  ///< 库伦 tanh 过渡宽度（rad/s）
    double dq_dead     = 0.0;  ///< 速度死区（rad/s）

    // ── 速度滤波 ──────────────────────────────────────────────────────────────
    double dq_lpf_alpha = 0.2;  ///< 内圈速度 EMA 系数（dt=5ms 时 0.2≈7Hz）

    // ── ASSIST 变阻抗 ─────────────────────────────────────────────────────────
    double vi_beta = 0.05;  ///< 滑动面速度权重
    double vi_a    = 0.2;   ///< 可变增益分子
    double vi_b    = 5.0;   ///< 可变增益分母系数

    // ── ZERO_FORCE — 导纳 + PD ───────────────────────────────────────────────
    //   调参顺序：① 标定摩擦  ② 调 B_v（tau_sensor 趋零响应速度）
    //             ③ 调 Kp/Kd（跟踪不振荡，临界阻尼 Kd ≈ 2√Kp）
    double B_v           = 3.0;   ///< 导纳虚拟阻尼（N·m·s/rad）；越小越灵敏
    double Kp_zf         = 20.0;  ///< 位置跟踪增益（N·m/rad）
    double Kd_zf         = 2.0;   ///< 速度跟踪增益（N·m·s/rad）
    double tau_lpf_alpha = 0.3;   ///< 扭矩传感器 EMA 系数
    double tau_deadband  = 0.2;   ///< 导纳死区（N·m）：滤除传感器零点偏置

    // ── 观测器 & 定时 ─────────────────────────────────────────────────────────
    double omega_o = 2.0;   ///< 二阶 ESO 带宽（TRANSPARENCY/ASSIST，rad/s）
    double dt      = 0.01;  ///< 控制周期（s），须与主循环严格一致
};

/* =========================================================
 * ImpedanceController  单关节阻抗控制器
 * ========================================================= */

class ImpedanceController
{
public:

    explicit ImpedanceController(const ImpedanceParams& p);

    /** @brief 清零所有内部状态；控制循环启动前必须调用 */
    void reset();

    /**
     * @brief  执行一个控制步骤（每控制周期调用一次）
     * @param  q_d     期望位置（rad）；TRANSPARENCY / ZERO_FORCE 模式忽略
     * @param  dq_d    期望速度（rad/s）；同上
     * @param  ddq_d   期望加速度（rad/s²）；同上
     * @param  q_act   外圈编码器角度（rad）
     * @param  dq_act  内圈编码器速度（rad/s）
     * @param  tau_act 扭矩传感器读数（N·m）
     */
    void update(double q_d, double dq_d, double ddq_d,
                double q_act, double dq_act, double tau_act);

    /**
     * @brief 切换控制模式
     *
     * 切换到 ZERO_FORCE 时自动标记重新初始化，
     * 下次 update() 将 ADRC 状态锁定到当前关节角，避免位置阶跃。
     */
    void        setMode(ControlMode mode) { mode_ = mode; zf_init_ = false; }
    ControlMode getMode()           const { return mode_; }

    // ── 状态读取 ──────────────────────────────────────────────────────────────
    double getQ()         const { return q_; }          ///< 外圈编码器多圈角（rad）
    double getDq()        const { return dq_; }         ///< 内圈编码器滤波速度（rad/s）
    double getTorque()    const { return tau_cmd_; }    ///< 本周期力矩指令（N·m）
    double getTauSensor() const { return tau_sensor_; } ///< 扭矩传感器滤波值（N·m）
    double getQAdm()      const { return q_adm_; }      ///< 导纳积分位置（调试用）
    double getVAdm()      const { return v_adm_; }      ///< 导纳速度参考（调试用）
    double getB()         const { return B_; }          ///< 临界阻尼 B = 2√(MK)

private:

    ImpedanceParams p_;
    ControlMode     mode_;

    // ── 预计算常数 ────────────────────────────────────────────────────────────
    double B_;      ///< 临界阻尼 2√(MK)
    double beta1_;  ///< 二阶 ESO β₁ = 2ω_o
    double beta2_;  ///< 二阶 ESO β₂ = ω_o²

    // ── 通用状态 ──────────────────────────────────────────────────────────────
    double q_wrapped_prev_;
    bool   initialized_;

    double z1_;          ///< 二阶 ESO 速度估计
    double z2_;          ///< 二阶 ESO 扰动估计
    double q_;           ///< 外圈编码器多圈连续角（rad）
    double dq_;          ///< 内圈编码器 EMA 滤波速度（rad/s）
    double dq_filt_;     ///< EMA 中间状态
    double ddq_est_;     ///< 加速度估计（监控用）
    double tau_cmd_;     ///< 本周期力矩指令（N·m）
    double tau_prev_;    ///< 上周期力矩指令（供 ESO 预测）
    double tau_sensor_;  ///< 扭矩传感器 EMA 滤波值（N·m）

    // ── 零力状态 ──────────────────────────────────────────────────────────────
    double q_adm_;    ///< 导纳积分位置参考（rad）
    double v_adm_;    ///< 导纳速度参考（rad/s）
    bool   zf_init_;  ///< 零力状态初始化标志

    // ── 私有控制律 ────────────────────────────────────────────────────────────
    double computeFriction()         const;
    double computeTransparency(double q_d, double dq_d, double ddq_d) const;
    double computeVariableImpedance(double q_d, double dq_d)          const;
    double computeZeroForce();
};
