#define ROBOT_GLOBAL
#include "robot.h"

#include <cmath>
#include <thread>
#include <chrono>

/* =========================================================
 * 单位换算（constexpr，全文件可用）
 *   tau_Nm     = current_mA / 1000.0 * kt
 *   current_mA = (int32_t)(tau_Nm / kt * 1000.0)
 *   q_joint    = rotor_rad / gear
 *   rotor_rad  = q_joint  * gear
 * ========================================================= */
static constexpr double rpm(double r) { return r * (M_PI / 30.0);  }  // rpm → rad/s
static constexpr double deg(double d) { return d * (M_PI / 180.0); }  // °   → rad

/* =========================================================
 * 每组关节参数（L/R 对称，仅填左侧，右侧限位由 mirror() 生成）
 *
 *   Kt(N·m/A)  减速比   tau_max  tau_rated  dq_max        q_min_L    q_max_L
 *                        (N·m)    (N·m)      (rpm→rad/s)   (°→rad)    (°→rad)
 * ========================================================= */
struct JointGroupSpec { double kt; double gear; JointLimit lim; };

static constexpr JointGroupSpec G_HIP   = { 0.136,  64.0, { 120.0,  48.0,  rpm( 39),   deg(-120), deg(   0) } };
static constexpr JointGroupSpec G_KNEE  = {  4.53,  48.0, {  74.0,  26.2,  rpm( 49),   deg(   0), deg(+150) } };
static constexpr JointGroupSpec G_ANKLE = {  1.20,   8.0, {  31.71, 11.4,  rpm(148),   deg( -45), deg( +15) } };
static constexpr JointGroupSpec G_SFLEX = {  0.12,  39.0, {  72.0,  24.0,  rpm( 45),   deg(-150), deg( +45) } };  // 肩屈伸
static constexpr JointGroupSpec G_SABD  = {  1.65,  12.5, {  41.5,  15.8,  rpm(200),   deg(-105), deg( +45) } };  // 肩外展
static constexpr JointGroupSpec G_SROT  = { 0.0124, 35.0, {   5.96,  0.65, rpm(342.6), deg( -45), deg( +45) } };  // 肩旋转
static constexpr JointGroupSpec G_ELBOW = {  0.47,   9.0, {  30.83,  9.8,  rpm(344),   deg(-150), deg(   0) } };
static constexpr JointGroupSpec G_WRIST = { 0.0124, 35.0, {   5.96,  0.65, rpm(342),   deg( -45), deg( +45) } };

/* ── 运行时数组（由 JointGroupSpec 展开，L/R 共用同组参数）─────────────────────── */
static constexpr double KT[JOINT_NUM] = {
    G_HIP.kt,   G_KNEE.kt,   G_ANKLE.kt,                                    //  L 下肢
    G_HIP.kt,   G_KNEE.kt,   G_ANKLE.kt,                                    //  R 下肢
    G_SFLEX.kt, G_SABD.kt,   G_SROT.kt,  G_ELBOW.kt, G_WRIST.kt,           //  L 上肢
    G_SFLEX.kt, G_SABD.kt,   G_SROT.kt,  G_ELBOW.kt, G_WRIST.kt,           //  R 上肢
};
static constexpr double GEAR_RATIO[JOINT_NUM] = {
    G_HIP.gear,   G_KNEE.gear,   G_ANKLE.gear,                              //  L 下肢
    G_HIP.gear,   G_KNEE.gear,   G_ANKLE.gear,                              //  R 下肢
    G_SFLEX.gear, G_SABD.gear,   G_SROT.gear,  G_ELBOW.gear, G_WRIST.gear, //  L 上肢
    G_SFLEX.gear, G_SABD.gear,   G_SROT.gear,  G_ELBOW.gear, G_WRIST.gear, //  R 上肢
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

    // ── 关节限位（正反转约定：面向电机转子，顺时针为正，逆时针为负）─────────────
    auto mirror = [](JointLimit p) -> JointLimit {      // 左→右对称：限位取反并对换
        return { p.tau_max, p.tau_rated, p.dq_max, -p.q_max, -p.q_min };
    };

    g_robot.limit_table
        // ── 左下肢 ──────────────────────────────────────────────────────────
        .set(JointID::L_HIP,           G_HIP.lim)
        .set(JointID::L_KNEE,          G_KNEE.lim)
        .set(JointID::L_ANKLE,         G_ANKLE.lim)
        // ── 右下肢（对称镜像）────────────────────────────────────────────────
        .set(JointID::R_HIP,           mirror(G_HIP.lim))
        .set(JointID::R_KNEE,          mirror(G_KNEE.lim))
        .set(JointID::R_ANKLE,         mirror(G_ANKLE.lim))
        // ── 左上肢 ──────────────────────────────────────────────────────────
        .set(JointID::L_SHOULDER_FLEX, G_SFLEX.lim)
        .set(JointID::L_SHOULDER_ABD,  G_SABD.lim)
        .set(JointID::L_SHOULDER_ROT,  G_SROT.lim)
        .set(JointID::L_ELBOW,         G_ELBOW.lim)
        .set(JointID::L_WRIST,         G_WRIST.lim)
        // ── 右上肢（对称镜像）────────────────────────────────────────────────
        .set(JointID::R_SHOULDER_FLEX, mirror(G_SFLEX.lim))
        .set(JointID::R_SHOULDER_ABD,  mirror(G_SABD.lim))
        .set(JointID::R_SHOULDER_ROT,  mirror(G_SROT.lim))
        .set(JointID::R_ELBOW,         mirror(G_ELBOW.lim))
        .set(JointID::R_WRIST,         mirror(G_WRIST.lim));

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
