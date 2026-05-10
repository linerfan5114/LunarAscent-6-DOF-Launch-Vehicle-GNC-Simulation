#include "control.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

PIDController::PIDController(double kp, double ki, double kd, double integral_limit, double output_limit)
    : m_prev_error(0.0), m_integral(0.0), m_derivative(0.0), m_output(0.0)
{
    m_gains.kp = kp;
    m_gains.ki = ki;
    m_gains.kd = kd;
    m_gains.integral_limit = integral_limit;
    m_gains.output_limit = output_limit;
}

double PIDController::update(double setpoint, double measured, double dt) {
    if (dt <= 0.0) return m_output;

    double error = setpoint - measured;
    m_integral += error * dt;

    if (m_gains.integral_limit > 0.0) {
        m_integral = std::max(-m_gains.integral_limit, std::min(m_gains.integral_limit, m_integral));
    }

    m_derivative = (error - m_prev_error) / dt;
    m_prev_error = error;

    m_output = m_gains.kp * error + m_gains.ki * m_integral + m_gains.kd * m_derivative;

    if (m_gains.output_limit > 0.0) {
        m_output = std::max(-m_gains.output_limit, std::min(m_gains.output_limit, m_output));
    }

    return m_output;
}

void PIDController::reset() {
    m_prev_error = 0.0;
    m_integral = 0.0;
    m_derivative = 0.0;
    m_output = 0.0;
}

void PIDController::set_gains(const PIDGains& gains) {
    m_gains = gains;
    m_integral = 0.0;
}

AttitudeControl::AttitudeControl()
    : m_throttle_limit(1.0)
    , m_max_q_throttle(0.7)
    , m_pitch_pid(2.5, 0.1, 0.5, 0.2, 0.14)
    , m_yaw_pid(2.5, 0.1, 0.5, 0.2, 0.14)
    , m_roll_pid(1.5, 0.05, 0.3, 0.1, 0.1)
    , m_rate_pitch_pid(1.0, 0.05, 0.1, 0.1, 0.087)
    , m_rate_yaw_pid(1.0, 0.05, 0.1, 0.1, 0.087)
    , m_rate_roll_pid(0.5, 0.02, 0.05, 0.05, 0.05)
{
    m_gains.pitch_channel = {2.5, 0.1, 0.5, 0.2, 0.14};
    m_gains.yaw_channel = {2.5, 0.1, 0.5, 0.2, 0.14};
    m_gains.roll_channel = {1.5, 0.05, 0.3, 0.1, 0.1};
    m_gains.rate_pitch = {1.0, 0.05, 0.1, 0.1, 0.087};
    m_gains.rate_yaw = {1.0, 0.05, 0.1, 0.1, 0.087};
    m_gains.rate_roll = {0.5, 0.02, 0.05, 0.05, 0.05};

    m_gain_schedule = {
        {0.0, m_gains},
        {1.0, {{4.0, 0.15, 0.8, 0.3, 0.14}, {4.0, 0.15, 0.8, 0.3, 0.14},
               {2.5, 0.08, 0.5, 0.15, 0.1}, {1.5, 0.08, 0.15, 0.15, 0.087},
               {1.5, 0.08, 0.15, 0.15, 0.087}, {0.8, 0.03, 0.08, 0.08, 0.05}}},
        {3.0, {{2.0, 0.08, 0.4, 0.15, 0.1}, {2.0, 0.08, 0.4, 0.15, 0.1},
               {1.2, 0.04, 0.2, 0.08, 0.08}, {0.8, 0.04, 0.08, 0.08, 0.07},
               {0.8, 0.04, 0.08, 0.08, 0.07}, {0.4, 0.02, 0.04, 0.04, 0.04}}},
        {5.0, {{1.5, 0.05, 0.3, 0.1, 0.08}, {1.5, 0.05, 0.3, 0.1, 0.08},
               {1.0, 0.03, 0.15, 0.05, 0.06}, {0.5, 0.03, 0.05, 0.05, 0.05},
               {0.5, 0.03, 0.05, 0.05, 0.05}, {0.3, 0.01, 0.03, 0.03, 0.03}}}
    };

    m_state.attitude_hold_active = true;
    m_state.rate_damping_active = true;
    m_state.max_q_throttle_active = false;
    m_state.aoa_limit_active = false;

    m_commands.gimbal_pitch_cmd_rad = 0.0;
    m_commands.gimbal_yaw_cmd_rad = 0.0;
    m_commands.throttle_cmd = 1.0;
    m_commands.rcs_pitch_up = false;
    m_commands.rcs_pitch_down = false;
    m_commands.rcs_yaw_left = false;
    m_commands.rcs_yaw_right = false;
    m_commands.rcs_roll_cw = false;
    m_commands.rcs_roll_ccw = false;
}

void AttitudeControl::update(double dt, const Quaternion& current_attitude,
                             const Vector3& angular_rate, const Quaternion& target_attitude,
                             const Vector3& target_angular_rate, double mach,
                             double dynamic_pressure) {
    update_gain_schedule(mach);

    double pitch_err, yaw_err, roll_err;
    compute_attitude_error(current_attitude, target_attitude, pitch_err, yaw_err, roll_err);

    m_state.current_pitch_error = pitch_err;
    m_state.current_yaw_error = yaw_err;
    m_state.current_roll_error = roll_err;

    double rate_pitch_cmd = m_rate_pitch_pid.update(0.0, angular_rate.x, dt);
    double rate_yaw_cmd = m_rate_yaw_pid.update(0.0, angular_rate.y, dt);
    double rate_roll_cmd = m_rate_roll_pid.update(0.0, angular_rate.z, dt);

    double pitch_cmd = m_pitch_pid.update(0.0, pitch_err, dt) + rate_pitch_cmd;
    double yaw_cmd = m_yaw_pid.update(0.0, yaw_err, dt) + rate_yaw_cmd;
    double roll_cmd = m_roll_pid.update(0.0, roll_err, dt) + rate_roll_cmd;

    m_commands.gimbal_pitch_cmd_rad = std::max(-m_limits.max_gimbal_angle_rad,
        std::min(m_limits.max_gimbal_angle_rad, pitch_cmd));
    m_commands.gimbal_yaw_cmd_rad = std::max(-m_limits.max_gimbal_angle_rad,
        std::min(m_limits.max_gimbal_angle_rad, yaw_cmd));

    if (std::abs(roll_err) > 0.1) {
        if (roll_err > 0) {
            m_commands.rcs_roll_cw = true;
            m_commands.rcs_roll_ccw = false;
        } else {
            m_commands.rcs_roll_cw = false;
            m_commands.rcs_roll_ccw = true;
        }
    } else {
        m_commands.rcs_roll_cw = false;
        m_commands.rcs_roll_ccw = false;
    }

    if (m_state.max_q_throttle_active && dynamic_pressure > m_limits.max_dynamic_pressure_Pa * 0.8) {
        m_commands.throttle_cmd = m_max_q_throttle;
    } else if (m_state.max_q_throttle_active && dynamic_pressure < m_limits.max_dynamic_pressure_Pa * 0.3) {
        m_state.max_q_throttle_active = false;
        m_commands.throttle_cmd = m_throttle_limit;
    } else {
        m_commands.throttle_cmd = m_throttle_limit;
    }

    m_commands.throttle_cmd = std::max(0.4, std::min(1.0, m_commands.throttle_cmd));
}

void AttitudeControl::compute_attitude_error(const Quaternion& current, const Quaternion& target,
                                             double& pitch_err, double& yaw_err, double& roll_err) {
    Quaternion error = target.conjugate() * current;
    auto euler = error.to_euler();
    roll_err = euler[0];
    pitch_err = euler[1];
    yaw_err = euler[2];
}

void AttitudeControl::update_gain_schedule(double mach) {
    if (m_gain_schedule.empty()) return;

    if (mach <= m_gain_schedule[0].mach) {
        m_gains = m_gain_schedule[0].gains;
        apply_gains();
        return;
    }

    for (size_t i = 0; i < m_gain_schedule.size() - 1; i++) {
        if (mach >= m_gain_schedule[i].mach && mach <= m_gain_schedule[i + 1].mach) {
            double t = (mach - m_gain_schedule[i].mach) /
                      (m_gain_schedule[i + 1].mach - m_gain_schedule[i].mach);

            auto& g0 = m_gain_schedule[i].gains;
            auto& g1 = m_gain_schedule[i + 1].gains;

            m_gains.pitch_channel.kp = g0.pitch_channel.kp + t * (g1.pitch_channel.kp - g0.pitch_channel.kp);
            m_gains.pitch_channel.kd = g0.pitch_channel.kd + t * (g1.pitch_channel.kd - g0.pitch_channel.kd);
            m_gains.rate_pitch.kp = g0.rate_pitch.kp + t * (g1.rate_pitch.kp - g0.rate_pitch.kp);

            apply_gains();
            return;
        }
    }

    m_gains = m_gain_schedule.back().gains;
    apply_gains();
}

void AttitudeControl::apply_gains() {
    m_pitch_pid.set_gains(m_gains.pitch_channel);
    m_yaw_pid.set_gains(m_gains.yaw_channel);
    m_roll_pid.set_gains(m_gains.roll_channel);
    m_rate_pitch_pid.set_gains(m_gains.rate_pitch);
    m_rate_yaw_pid.set_gains(m_gains.rate_yaw);
    m_rate_roll_pid.set_gains(m_gains.rate_roll);
}

void AttitudeControl::set_throttle_for_max_q(double dynamic_pressure, bool enable) {
    if (enable && dynamic_pressure > m_limits.max_dynamic_pressure_Pa * 0.8) {
        m_state.max_q_throttle_active = true;
    }
}

void AttitudeControl::reset() {
    m_pitch_pid.reset();
    m_yaw_pid.reset();
    m_roll_pid.reset();
    m_rate_pitch_pid.reset();
    m_rate_yaw_pid.reset();
    m_rate_roll_pid.reset();

    m_commands = ControlCommands{};
    m_state = ControlState{};
}

void AttitudeControl::emergency_shutdown() {
    m_commands.gimbal_pitch_cmd_rad = 0.0;
    m_commands.gimbal_yaw_cmd_rad = 0.0;
    m_commands.throttle_cmd = 0.0;
    m_commands.rcs_pitch_up = false;
    m_commands.rcs_pitch_down = false;
    m_commands.rcs_yaw_left = false;
    m_commands.rcs_yaw_right = false;
    m_commands.rcs_roll_cw = false;
    m_commands.rcs_roll_ccw = false;

    m_state.attitude_hold_active = false;
    m_state.rate_damping_active = false;
}

void AttitudeControl::load_gain_schedule(const std::vector<GainScheduleEntry>& schedule) {
    m_gain_schedule = schedule;
    std::sort(m_gain_schedule.begin(), m_gain_schedule.end(),
              [](const GainScheduleEntry& a, const GainScheduleEntry& b) {
                  return a.mach < b.mach;
              });
}

Vector3 AttitudeControl::compute_target_angular_rate(const Quaternion& current,
                                                     const Quaternion& target, double dt) {
    Quaternion error = target.conjugate() * current;
    auto euler = error.to_euler();

    double max_rate = m_limits.max_angular_rate_rad_per_s;
    double pitch_rate = std::max(-max_rate, std::min(max_rate, euler[1] / dt));
    double yaw_rate = std::max(-max_rate, std::min(max_rate, euler[2] / dt));
    double roll_rate = std::max(-max_rate, std::min(max_rate, euler[0] / dt));

    return Vector3{pitch_rate, yaw_rate, roll_rate};
}

void AttitudeControl::apply_limits() {
    m_commands.gimbal_pitch_cmd_rad = std::max(-m_limits.max_gimbal_angle_rad,
        std::min(m_limits.max_gimbal_angle_rad, m_commands.gimbal_pitch_cmd_rad));
    m_commands.gimbal_yaw_cmd_rad = std::max(-m_limits.max_gimbal_angle_rad,
        std::min(m_limits.max_gimbal_angle_rad, m_commands.gimbal_yaw_cmd_rad));
}

} // namespace LunarAscent