#define ROBOT_GLOBAL
#include "robot.h"

#include <cmath>
#include <thread>
#include <chrono>

static double g_q      [JOINT_NUM]{};
static double g_dq     [JOINT_NUM]{};
static double g_tau    [JOINT_NUM]{};
static double g_motor_q[JOINT_NUM]{};

static void applyOutput()
{
    const double dt    = g_robot.data.dt_s;
    const double alpha = 0.3;

    for (int j = 0; j < JOINT_NUM; ++j)
    {
        if (!g_robot.isActive(static_cast<JointID>(j))) continue;

        const double q_ref  = g_robot.output.cmd_q  [j];  // 期望关节角（rad）
        const double dq_ref = g_robot.output.cmd_dq [j];  // 期望关节速度（rad/s）
        const double tau    = g_robot.output.cmd_tau [j];  // 阻抗力矩指令（N·m）

        // 真实系统：motor_driver[j].setCurrentMA((int32_t)(tau / Kt * 1000.0));

        // 仿真植物模型：位置速度混合跟随
        g_dq[j]      = (q_ref - g_q[j]) * alpha / dt + dq_ref * (1.0 - alpha);
        g_q[j]      += g_dq[j] * dt;
        g_motor_q[j] = g_q[j];
        g_tau[j]     = tau;
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
        g_robot.setFeedback(g_motor_q, g_q, g_dq, g_tau);
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
        g_robot.setFeedback(g_motor_q, g_q, g_dq, g_tau);
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

        g_robot.setFeedback(g_motor_q, g_q, g_dq, g_tau);
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
        g_robot.setFeedback(g_motor_q, g_q, g_dq, g_tau);
        updateImuSim(time_s);
        g_robot.updateAll();
        applyOutput();
    }

    return 0;
}
