#define ROBOT_GLOBAL
#include "robot.h"

#include <cmath>
#include <thread>
#include <chrono>

/* =========================================================
 * 扭矩常数表  Kt（N·m / A），下标与 JointID 枚举严格对应
 *
 * 换算关系：
 *   输入（电机→算法）：tau_Nm  = current_mA / 1000.0 * Kt
 *   输出（算法→电机）：current_mA = (int32_t)(tau_Nm / Kt * 1000.0)
 *
 * 下表为占位标定值，须根据各关节实际电机规格书替换。
 * ========================================================= */
static constexpr double KT[JOINT_NUM] = 
{
//  关节                    Kt (N·m/A)   备注
    4.53,   // L_HIP            大力矩髋关节电机
    4.53,   // L_KNEE           大力矩膝关节电机
    3.20,   // L_ANKLE          中力矩踝关节电机
    4.53,   // R_HIP
    4.53,   // R_KNEE
    3.20,   // R_ANKLE
    2.10,   // L_SHOULDER_FLEX  轻量肩屈伸电机
    2.10,   // L_SHOULDER_ABD
    1.50,   // L_SHOULDER_ROT   旋转自由度力矩需求低
    2.80,   // L_ELBOW
    1.20,   // L_WRIST
    2.10,   // R_SHOULDER_FLEX
    2.10,   // R_SHOULDER_ABD
    1.50,   // R_SHOULDER_ROT
    2.80,   // R_ELBOW
    1.20,   // R_WRIST
};

/* =========================================================
 * 减速比表（无量纲），下标与 JointID 枚举严格对应
 *
 * 换算关系：
 *   输入（内圈编码器→关节空间）：q_joint = rotor_rad / GEAR_RATIO
 *   仿真（关节空间→转子）：      rotor_rad = q_joint  * GEAR_RATIO
 *
 * 下表为占位值，须根据各关节实际减速器规格书替换。
 * ========================================================= */
static constexpr double GEAR_RATIO[JOINT_NUM] =
{
//  关节                    减速比    备注
    50.0,   // L_HIP            谐波减速
    50.0,   // L_KNEE
    36.0,   // L_ANKLE          行星减速
    50.0,   // R_HIP
    50.0,   // R_KNEE
    36.0,   // R_ANKLE
    30.0,   // L_SHOULDER_FLEX  轻量关节
    30.0,   // L_SHOULDER_ABD
    20.0,   // L_SHOULDER_ROT
    36.0,   // L_ELBOW
    20.0,   // L_WRIST
    30.0,   // R_SHOULDER_FLEX
    30.0,   // R_SHOULDER_ABD
    20.0,   // R_SHOULDER_ROT
    36.0,   // R_ELBOW
    20.0,   // R_WRIST
};

/* ── 硬件层缓冲（模拟电机驱动器原始数据）──────────────────────────────────────
 * 真实系统：由底层通信帧填充
 * 仿真：由 applyOutput() 的植物模型反填
 * ──────────────────────────────────────────────────────────────────────────── */
static double  g_q          [JOINT_NUM]{};   // 外圈编码器关节角（rad）
static double  g_rotor_q    [JOINT_NUM]{};   // 内圈编码器转子角（rad，硬件原始值）
static double  g_dq         [JOINT_NUM]{};   // 关节角速度（rad/s，外圈）
static int32_t g_current_mA [JOINT_NUM]{};   // 电机实际电流（mA，驱动器反馈）

/* ── 输入路径：
 *     ① 转子角  → 关节角：q_joint = g_rotor_q / GEAR_RATIO
 *     ② 电流(mA)→ 扭矩(N·m)：tau = current_mA / 1000.0 * KT
 * ──────────────────────────────────────────────────────────────────────────── */
static void readMotorFeedback()
{
    double motor_q_joint[JOINT_NUM]{};   // 内圈编码器换算关节角（rad）
    double g_tau        [JOINT_NUM]{};

    for (int j = 0; j < JOINT_NUM; ++j)
    {
        motor_q_joint[j] = g_rotor_q[j] / GEAR_RATIO[j];
        g_tau[j]         = (g_current_mA[j] / 1000.0) * KT[j];
    }

    g_robot.setFeedback(motor_q_joint, g_q, g_dq, g_tau);
}

/* ── 输出路径：扭矩(N·m) → 电流指令(mA) → 电机驱动器 ───────────────────────
 *
 *   current_mA[j] = (int32_t)(cmd_tau[j] / KT[j] * 1000.0)
 *
 * 真实系统调用：motor_driver[j].setCurrentMA(current_mA)
 * 仿真：将电流指令折算回扭矩，驱动植物模型更新位置/速度，再反填电流反馈。
 * ──────────────────────────────────────────────────────────────────────────── */
static void applyOutput()
{
    const double dt    = g_robot.data.dt_s;
    const double alpha = 0.3;

    for (int j = 0; j < JOINT_NUM; ++j)
    {
        if (!g_robot.isActive(static_cast<JointID>(j))) continue;

        const double cmd_tau = g_robot.output.cmd_tau[j];   // 算法库输出力矩（N·m）

        // ── 输出路径：N·m → mA ──────────────────────────────────────────────
        const int32_t cmd_current_mA = static_cast<int32_t>(cmd_tau / KT[j] * 1000.0);
        // 真实系统：motor_driver[j].setCurrentMA(cmd_current_mA);

        // ── 仿真植物模型：电流→力矩→运动学 ────────────────────────────────
        const double sim_tau = (cmd_current_mA / 1000.0) * KT[j];  // 量化回折

        const double q_ref  = g_robot.output.cmd_q [j];
        const double dq_ref = g_robot.output.cmd_dq[j];
        g_dq[j]      = (q_ref - g_q[j]) * alpha / dt + dq_ref * (1.0 - alpha);
        g_q[j]      += g_dq[j] * dt;
        // 仿真：转子角 = 关节角 × 减速比（真实系统由内圈编码器直接读取）
        g_rotor_q[j] = g_q[j] * GEAR_RATIO[j];

        // 仿真反填电流反馈（真实系统由驱动器帧填充）
        g_current_mA[j] = static_cast<int32_t>(sim_tau / KT[j] * 1000.0);
    }
}

static void updateImuSim(double time_s)
{
    // 绕 Y 轴旋转四元数：w=cos(θ/2), x=0, y=sin(θ/2), z=0
    // 真实系统：从 IMU 驱动层读取解算后的四元数替换此处赋值
    auto rotY = [](double angle) -> Quaternion {
        return { std::cos(angle * 0.5), 0.0, std::sin(angle * 0.5), 0.0 };
    };

    const double w = 2.0 * M_PI * 0.5;

    // 下肢：幅度与步态轨迹一致，相位依次递推
    g_robot.status.imu_quat[toImuIdx(ImuID::L_THIGH)]     = rotY( 0.175 * std::sin(w * time_s));
    g_robot.status.imu_quat[toImuIdx(ImuID::L_SHANK)]     = rotY( 0.140 * std::sin(w * time_s + 0.3));
    g_robot.status.imu_quat[toImuIdx(ImuID::L_FOOT)]      = rotY( 0.087 * std::sin(w * time_s + 0.6));
    g_robot.status.imu_quat[toImuIdx(ImuID::R_THIGH)]     = rotY( 0.175 * std::sin(w * time_s + M_PI));
    g_robot.status.imu_quat[toImuIdx(ImuID::R_SHANK)]     = rotY( 0.140 * std::sin(w * time_s + M_PI + 0.3));
    g_robot.status.imu_quat[toImuIdx(ImuID::R_FOOT)]      = rotY( 0.087 * std::sin(w * time_s + M_PI + 0.6));

    // 上肢：与对侧下肢反相摆动（正常步态手臂协调）
    g_robot.status.imu_quat[toImuIdx(ImuID::L_UPPER_ARM)] = rotY(-0.100 * std::sin(w * time_s + M_PI));
    g_robot.status.imu_quat[toImuIdx(ImuID::L_FOREARM)]   = rotY(-0.080 * std::sin(w * time_s + M_PI + 0.2));
    g_robot.status.imu_quat[toImuIdx(ImuID::R_UPPER_ARM)] = rotY(-0.100 * std::sin(w * time_s));
    g_robot.status.imu_quat[toImuIdx(ImuID::R_FOREARM)]   = rotY(-0.080 * std::sin(w * time_s + 0.2));
}

int main()
{
    JointEnableConfig cfg;
    cfg.enable(JointID::L_HIP,   ControlMode::TRANSPARENCY)        
       .enable(JointID::L_KNEE,  ControlMode::TRANSPARENCY)
       .enable(JointID::L_ANKLE, ControlMode::TRANSPARENCY)
       .enable(JointID::R_HIP,   ControlMode::TRANSPARENCY)       
       .enable(JointID::R_KNEE,  ControlMode::TRANSPARENCY)
       .enable(JointID::R_ANKLE, ControlMode::TRANSPARENCY)
       .enable(JointID::L_ELBOW, ControlMode::TRANSPARENCY)          
       .enable(JointID::L_SHOULDER_ABD, ControlMode::TRANSPARENCY)
       .enable(JointID::L_SHOULDER_FLEX, ControlMode::TRANSPARENCY)
       .enable(JointID::L_SHOULDER_ROT, ControlMode::TRANSPARENCY)
       .enable(JointID::L_WRIST, ControlMode::TRANSPARENCY)
       .enable(JointID::R_ELBOW, ControlMode::TRANSPARENCY)
       .enable(JointID::R_SHOULDER_ABD, ControlMode::TRANSPARENCY)
       .enable(JointID::R_SHOULDER_FLEX, ControlMode::TRANSPARENCY)
       .enable(JointID::R_SHOULDER_ROT, ControlMode::TRANSPARENCY)
       .enable(JointID::R_WRIST, ControlMode::TRANSPARENCY);

    // ── 关节限位（覆盖 Robot 构造时从 RobotConfig 读取的默认值）────────────────
    //   格式：{ tau_max(N·m),  tau_rated(N·m),  dq_max(rad/s),  q_min(rad),  q_max(rad) }
    g_robot.limit_table
        // ── 左下肢 ──────────────────────────────────────────────────────────
        .set(JointID::L_HIP,           { 120.0,  80.0,  4.0,  -0.35,  1.40 })  // 髋：−20°~80°
        .set(JointID::L_KNEE,          { 120.0,  80.0,  4.0,  -2.09,  0.00 })  // 膝：−120°~0°
        .set(JointID::L_ANKLE,         {  60.0,  40.0,  6.0,  -0.52,  0.52 })  // 踝：±30°
        // ── 右下肢 ──────────────────────────────────────────────────────────
        .set(JointID::R_HIP,           { 120.0,  80.0,  4.0,  -0.35,  1.40 })
        .set(JointID::R_KNEE,          { 120.0,  80.0,  4.0,  -2.09,  0.00 })
        .set(JointID::R_ANKLE,         {  60.0,  40.0,  6.0,  -0.52,  0.52 })
        // ── 左上肢 ──────────────────────────────────────────────────────────
        .set(JointID::L_SHOULDER_FLEX, {  60.0,  40.0,  3.0,  -0.52,  2.97 })  // 肩屈伸：−30°~170°
        .set(JointID::L_SHOULDER_ABD,  {  50.0,  35.0,  3.0,  -0.26,  1.57 })  // 肩外展：−15°~90°
        .set(JointID::L_SHOULDER_ROT,  {  20.0,  15.0,  4.0,  -1.57,  1.57 })  // 肩旋转：±90°
        .set(JointID::L_ELBOW,         {  40.0,  25.0,  4.0,   0.00,  2.44 })  // 肘：0°~140°
        .set(JointID::L_WRIST,         {  15.0,  10.0,  5.0,  -1.22,  1.22 })  // 腕：±70°
        // ── 右上肢 ──────────────────────────────────────────────────────────
        .set(JointID::R_SHOULDER_FLEX, {  60.0,  40.0,  3.0,  -0.52,  2.97 })
        .set(JointID::R_SHOULDER_ABD,  {  50.0,  35.0,  3.0,  -0.26,  1.57 })
        .set(JointID::R_SHOULDER_ROT,  {  20.0,  15.0,  4.0,  -1.57,  1.57 })
        .set(JointID::R_ELBOW,         {  40.0,  25.0,  4.0,   0.00,  2.44 })
        .set(JointID::R_WRIST,         {  15.0,  10.0,  5.0,  -1.22,  1.22 });

    if (!g_robot.init(cfg, 0.004, "iron_man_exo")) return 1;
    if (!g_robot.start())                          return 1;

    double time_s = 0.0;
    const double dt = g_robot.data.dt_s;

    // ── 穿戴期：TRANSPARENCY，持续 2 s ──────────────────────────────────────
    g_robot.setModeAll(ControlMode::TRANSPARENCY);
    for (int i = 0; i < 500; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        readMotorFeedback();
        updateImuSim(time_s);
        g_robot.updateAll();
        applyOutput();
        time_s += dt;
    }

    // ── 零力引导：ZERO_FORCE，持续 3 s ──────────────────────────────────────
    g_robot.setModeAll(ControlMode::ZERO_FORCE);
    for (int i = 0; i < 750; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        readMotorFeedback();
        updateImuSim(time_s);
        g_robot.updateAll();
        applyOutput();
        time_s += dt;
    }

    // ── 步态助力：ASSIST，持续运行 ──────────────────────────────────────────
    g_robot.setModeAll(ControlMode::ASSIST);

    const double w         = 2.0 * M_PI * 0.5;
    const double hip_amp   = 0.175;
    const double knee_amp  = 0.140;
    const double ankle_amp = 0.087;

    for (int i = 0; i < 2500; ++i) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));

        readMotorFeedback();
        updateImuSim(time_s);

        g_robot.setTarget(JointID::L_HIP,
            hip_amp   * std::sin(w * time_s),
            hip_amp   * w * std::cos(w * time_s),
           -hip_amp   * w * w * std::sin(w * time_s));
        g_robot.setTarget(JointID::L_KNEE,
            knee_amp  * std::sin(w * time_s + 0.3),
            knee_amp  * w * std::cos(w * time_s + 0.3),
           -knee_amp  * w * w * std::sin(w * time_s + 0.3));
        g_robot.setTarget(JointID::L_ANKLE,
            ankle_amp * std::sin(w * time_s + 0.6),
            ankle_amp * w * std::cos(w * time_s + 0.6),
           -ankle_amp * w * w * std::sin(w * time_s + 0.6));
        g_robot.setTarget(JointID::R_HIP,
            hip_amp   * std::sin(w * time_s + M_PI),
            hip_amp   * w * std::cos(w * time_s + M_PI),
           -hip_amp   * w * w * std::sin(w * time_s + M_PI));
        g_robot.setTarget(JointID::R_KNEE,
            knee_amp  * std::sin(w * time_s + M_PI + 0.3),
            knee_amp  * w * std::cos(w * time_s + M_PI + 0.3),
           -knee_amp  * w * w * std::sin(w * time_s + M_PI + 0.3));
        g_robot.setTarget(JointID::R_ANKLE,
            ankle_amp * std::sin(w * time_s + M_PI + 0.6),
            ankle_amp * w * std::cos(w * time_s + M_PI + 0.6),
           -ankle_amp * w * w * std::sin(w * time_s + M_PI + 0.6));

        g_robot.updateAll();
        applyOutput();
        time_s += dt;

        if (g_robot.getState() == RobotStatusMachine::ERROR) 
        {
            g_robot.clearError();
            if (!g_robot.start()) break;
            g_robot.setModeAll(ControlMode::ASSIST);
        }
    }

    // ── 关机：渐停至 STANDBY（日志由 g_robot 析构自动保存）─────────────────────
    g_robot.pause();
    while (g_robot.getState() == RobotStatusMachine::PAUSING) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        readMotorFeedback();
        updateImuSim(time_s);
        g_robot.updateAll();
        applyOutput();
    }

    return 0;
}
