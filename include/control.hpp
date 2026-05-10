#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "vehicle.hpp"
#include "propulsion.hpp"
#include <cmath>
#include <array>
#include <algorithm>

namespace LunarAscent {

struct PIDGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 0.0;
    double output_limit = 0.0;
};

class PIDController {
public:
    PIDController() = default;
    PIDController(double kp, double ki, double kd, double integral_limit, double output_limit);
    
    double update(double setpoint, double measured, double dt);
    void reset();
    void set_gains(const PIDGains& gains);
    
private:
    PIDGains m_gains;
    double m_prev_error = 0.0;
    double m_integral = 0.0;
    double m_derivative = 0.0;
    double m_output = 0.0;
};

struct ControlGains {
    PIDGains pitch_channel;
    PIDGains yaw_channel;
    PIDGains roll_channel;
    PIDGains rate_pitch;
    PIDGains rate_yaw;
    PIDGains rate_roll;
};

struct GainScheduleEntry {
    double mach;
    ControlGains gains;
};

struct ControlCommands {
    double gimbal_pitch_cmd_rad = 0.0;
    double gimbal_yaw_cmd_rad = 0.0;
    double throttle_cmd = 1.0;
    bool rcs_pitch_up = false;
    bool rcs_pitch_down = false;
    bool rcs_yaw_left = false;
    bool rcs_yaw_right = false;
    bool rcs_roll_cw = false;
    bool rcs_roll_ccw = false;
};

struct ControlLimits {
    double max_gimbal_angle_rad = 0.14;
    double max_gimbal_rate_rad_per_s = 0.087;
    double max_angle_of_attack_rad = 0.175;
    double max_dynamic_pressure_Pa = 40000.0;
    double max_acceleration_m_s2 = 45.0;
    double max_angular_rate_rad_per_s = 0.5;
};

struct ControlState {
    ControlCommands commands;
    double current_pitch_error = 0.0;
    double current_yaw_error = 0.0;
    double current_roll_error = 0.0;
    bool attitude_hold_active = false;
    bool rate_damping_active = false;
    bool max_q_throttle_active = false;
    bool aoa_limit_active = false;
};

class AttitudeControl {
public:
    AttitudeControl();
    
    void update(double dt, const Quaternion& current_attitude,
                const Vector3& angular_rate, const Quaternion& target_attitude,
                const Vector3& target_angular_rate, double mach, double dynamic_pressure);
    
    void set_control_gains(const ControlGains& gains) { m_gains = gains; }
    void load_gain_schedule(const std::vector<GainScheduleEntry>& schedule);
    
    ControlCommands get_commands() const { return m_commands; }
    double get_pitch_command() const { return m_commands.gimbal_pitch_cmd_rad; }
    double get_yaw_command() const { return m_commands.gimbal_yaw_cmd_rad; }
    double get_throttle_command() const { return m_commands.throttle_cmd; }
    
    void compute_attitude_error(const Quaternion& current, const Quaternion& target,
                                double& pitch_err, double& yaw_err, double& roll_err);
    void set_throttle_for_max_q(double dynamic_pressure, bool enable);
    void set_attitude_hold(bool enable) { m_state.attitude_hold_active = enable; }
    void set_rate_damping(bool enable) { m_state.rate_damping_active = enable; }
    void set_throttle_limit(double limit) { m_throttle_limit = limit; }
    
    ControlState get_state() const { return m_state; }
    
    void reset();
    void set_limits(const ControlLimits& limits) { m_limits = limits; }
    void emergency_shutdown();
    
private:
    ControlGains m_gains;
    ControlLatches m_commands;
    ControlState m_state;
    ControlLimits m_limits;
    
    std::vector<GainScheduleEntry> m_gain_schedule;
    
    PIDController m_pitch_pid;
    PIDController m_yaw_pid;
    PIDController m_roll_pid;
    PIDController m_rate_pitch_pid;
    PIDController m_rate_yaw_pid;
    PIDController m_rate_roll_pid;
    
    double m_throttle_limit = 1.0;
    double m_max_q_throttle = 0.7;
    
    void update_gain_schedule(double mach);
    void apply_limits();
    Vector3 compute_target_angular_rate(const Quaternion& current,
                                         const Quaternion& target,
                                         double dt);
};

} // namespace LunarAscent

#endif // CONTROL_HPP