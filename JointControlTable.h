/**
 * @file  JointControlTable.h
 * @brief 全身 16 关节阻抗控制参数表（可配置）
 *
 * 使用方式
 * ────────────────────────────────────────────────────────────
 *   // 1. 使用默认参数表
 *   JointControlTable table = JointControlTable::makeDefault();
 *
 *   // 2. 按需修改单关节参数
 *   table[JointID::L_HIP].mode = ControlMode::ASSIST;
 *   table[JointID::L_HIP].vi_a = 0.30;
 *   table[JointID::L_HIP].M    = 0.12;
 *
 *   // 3. 注册到机器人（在 init() 之前调用）
 *   g_robot.setControlTable(table);
 *
 * 默认模式
 * ────────────────────────────────────────────────────────────
 *   所有关节默认 TRANSPARENCY（透明）模式；
 *   需要主动助力时切换到 ASSIST（变阻抗）模式。
 *
 *   M / K / tau_max 按额定力矩比例设置占位默认值，
 *   需根据实际电机与人体动力学标定后替换。
 *
 * 关节参数索引（与 JointID 枚举一致）
 * ────────────────────────────────────────────────────────────
 *   下肢  [0] L_HIP  [1] L_KNEE  [2] L_ANKLE
 *         [3] R_HIP  [4] R_KNEE  [5] R_ANKLE
 *   上肢  [6] L_SHOULDER_FLEX  [7] L_SHOULDER_ABD  [8] L_SHOULDER_ROT
 *         [9] L_ELBOW  [10] L_WRIST
 *        [11] R_SHOULDER_FLEX [12] R_SHOULDER_ABD  [13] R_SHOULDER_ROT
 *        [14] R_ELBOW [15] R_WRIST
 */

#pragma once

#include "Impedance_controller.h"   // ImpedanceParams
#include "robot_struct.h"           // JOINT_NUM, JointID, ControlMode, toIdx()

struct JointControlTable
{
    ImpedanceParams params[JOINT_NUM];

    // ── 访问接口 ──────────────────────────────────────────────────────────────

    ImpedanceParams&       operator[](JointID jid)       { return params[toIdx(jid)]; }
    const ImpedanceParams& operator[](JointID jid) const { return params[toIdx(jid)]; }

    /**
     * @brief 批量写入控制周期（Robot::init() 内自动调用，无需手动设置）
     *
     * @param dt_s  控制周期（s）
     */
    void setDt(double dt_s) noexcept
    {
        for (auto& p : params) p.dt = dt_s;
    }

    // ── 默认参数表 ────────────────────────────────────────────────────────────

    /**
     * @brief 生成全身 16 关节默认参数表
     *
     * 下肢（[0]-[5]）：M/K 较大，对应大力矩髋/膝/踝关节
     * 上肢（[6]-[15]）：M/K 较小，对应轻量肩/肘/腕关节
     * 所有关节初始为 TRANSPARENCY 模式（安全默认）
     * 摩擦参数（B_vis / tau_coulomb）默认为零，标定后填入
     */
    static JointControlTable makeDefault();
};

/* =========================================================
 * 默认参数表实现（inline，无需独立编译单元）
 * ========================================================= */

inline JointControlTable JointControlTable::makeDefault()
{
    JointControlTable t;

    // ── 公共默认值（所有关节共享）────────────────────────────────────────────
    for (auto& p : t.params)
    {
        p.mode         = ControlMode::TRANSPARENCY;
        p.Kt           = 4.53;
        p.v_s          = 0.10;
        p.dq_dead      = 0.0;
        p.dq_lpf_alpha = 0.20;
        p.vi_beta      = 0.05;
        p.vi_a         = 0.20;
        p.vi_b         = 5.00;
        p.omega_o      = 2.00;
        p.dt           = 0.004;
    }

    // ── 下肢参数 ──────────────────────────────────────────────────────────────
    //
    //  关节         M(kg·m²) K(N·m/rad) tau_max  B_vis  tau_coulomb  B_v  tau_db  Kp_zf  Kd_zf
    //  HIP           0.12    18.0        80.0    0.05    0.8         8.0   0.5    20.0    2.0
    //  KNEE          0.10    15.0        80.0    0.04    0.7         8.0   0.5    20.0    2.0
    //  ANKLE         0.05     8.0        40.0    0.02    0.4         4.0   0.3    15.0    1.5

    auto setLeg = [](ImpedanceParams& p,
                     double M, double K, double tau_max,
                     double B_vis, double tau_coulomb,
                     double B_v, double tau_db, double Kp_zf, double Kd_zf)
    {
        p.M           = M;
        p.K           = K;
        p.tau_max     = tau_max;
        p.B_vis       = B_vis;
        p.tau_coulomb = tau_coulomb;
        p.B_v         = B_v;
        p.tau_deadband= tau_db;
        p.Kp_zf       = Kp_zf;
        p.Kd_zf       = Kd_zf;
    };

    setLeg(t.params[0], 0.12, 18.0, 80.0, 0.05, 0.8, 8.0, 0.5, 20.0, 2.0);  // L_HIP
    setLeg(t.params[1], 0.10, 15.0, 80.0, 0.04, 0.7, 8.0, 0.5, 20.0, 2.0);  // L_KNEE
    setLeg(t.params[2], 0.05,  8.0, 40.0, 0.02, 0.4, 4.0, 0.3, 15.0, 1.5);  // L_ANKLE
    t.params[3] = t.params[0];  // R_HIP
    t.params[4] = t.params[1];  // R_KNEE
    t.params[5] = t.params[2];  // R_ANKLE

    // ── 上肢参数 ──────────────────────────────────────────────────────────────
    //
    //  关节                  M       K    tau_max
    //  L_SHOULDER_FLEX [6]   0.06   8.0   40.0
    t.params[6].M = 0.06;  t.params[6].K  =  8.0;  t.params[6].tau_max  = 40.0;
    //  L_SHOULDER_ABD  [7]   0.05   6.0   35.0
    t.params[7].M = 0.05;  t.params[7].K  =  6.0;  t.params[7].tau_max  = 35.0;
    //  L_SHOULDER_ROT  [8]   0.02   3.0   15.0
    t.params[8].M = 0.02;  t.params[8].K  =  3.0;  t.params[8].tau_max  = 15.0;
    //  L_ELBOW         [9]   0.04   5.0   25.0
    t.params[9].M = 0.04;  t.params[9].K  =  5.0;  t.params[9].tau_max  = 25.0;
    //  L_WRIST        [10]   0.01   2.0   10.0
    t.params[10].M = 0.01; t.params[10].K =  2.0;  t.params[10].tau_max = 10.0;
    t.params[11] = t.params[6];   // R_SHOULDER_FLEX
    t.params[12] = t.params[7];   // R_SHOULDER_ABD
    t.params[13] = t.params[8];   // R_SHOULDER_ROT
    t.params[14] = t.params[9];   // R_ELBOW
    t.params[15] = t.params[10];  // R_WRIST

    return t;
}
