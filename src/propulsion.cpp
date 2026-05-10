#include "propulsion.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

PropulsionSystem::PropulsionSystem(const EngineConfig& engine_config, const RCSConfig& rcs_config)
    : m_engine_config(engine_config)
    , m_rcs_config(rcs_config)
{
    m_state.main_engines.ignited = false;
    m_state.main_engines.shutdown = false;
    m_state.main_engines.throttle_command = 0.0;
    m_state.main_engines.throttle_current = 0.0;
    m_state.main_engines.gimbal_pitch_command_rad = 0.0;
    m_state.main_engines.gimbal_yaw_command_rad = 0.0;
    m_state.main_engines.gimbal_pitch_current_rad = 0.0;
    m_state.main_engines.gimbal_yaw_current_rad = 0.0;
    m_state.main_engines.current_thrust_N = 0.0;
    m_state.main_engines.current_isp_s = 0.0;
    m_state.main_engines.mass_flow_rate_kg_per_s = 0.0;
    m_state.main_engines.total_burn_time_s = 0.0;
    m_state.main_engines.total_impulse_Ns = 0.0;
    m_state.main_engines.engine_temp_K = 300.0;
    m_state.rcs_fuel_consumed_kg = 0.0;
    m_state.main_fuel_consumed_kg = 0.0;
    for (int i = 0; i < 12; i++) {
        m_state.rcs_active[i] = false;
    }
}

void PropulsionSystem::update(double dt, double ambient_pressure_Pa, bool ignition_signal, double throttle_cmd) {
    if (ignition_signal && !m_state.main_engines.ignited && !m_state.main_engines.shutdown) {
        m_state.main_engines.ignited = true;
        m_state.main_engines.throttle_current = 0.0;
    }

    if (!ignition_signal && m_state.main_engines.ignited) {
        m_state.main_engines.shutdown = true;
        m_state.main_engines.ignited = false;
        m_state.main_engines.throttle_command = 0.0;
    }

    if (m_state.main_engines.shutdown) {
        m_state.main_engines.current_thrust_N = 0.0;
        m_state.main_engines.mass_flow_rate_kg_per_s = 0.0;
        return;
    }

    m_state.main_engines.throttle_command = std::max(m_engine_config.throttle_min,
        std::min(m_engine_config.throttle_max, throttle_cmd));

    double throttle_rate = m_engine_config.throttle_rate_per_s * dt;
    double throttle_diff = m_state.main_engines.throttle_command - m_state.main_engines.throttle_current;
    if (std::abs(throttle_diff) < throttle_rate) {
        m_state.main_engines.throttle_current = m_state.main_engines.throttle_command;
    } else {
        m_state.main_engines.throttle_current += (throttle_diff > 0 ? throttle_rate : -throttle_rate);
    }

    double gimbal_rate = m_engine_config.gimbal_max_rate_deg_per_s * M_PI / 180.0 * dt;
    double pitch_diff = m_state.main_engines.gimbal_pitch_command_rad - m_state.main_engines.gimbal_pitch_current_rad;
    if (std::abs(pitch_diff) < gimbal_rate) {
        m_state.main_engines.gimbal_pitch_current_rad = m_state.main_engines.gimbal_pitch_command_rad;
    } else {
        m_state.main_engines.gimbal_pitch_current_rad += (pitch_diff > 0 ? gimbal_rate : -gimbal_rate);
    }
    double yaw_diff = m_state.main_engines.gimbal_yaw_command_rad - m_state.main_engines.gimbal_yaw_current_rad;
    if (std::abs(yaw_diff) < gimbal_rate) {
        m_state.main_engines.gimbal_yaw_current_rad = m_state.main_engines.gimbal_yaw_command_rad;
    } else {
        m_state.main_engines.gimbal_yaw_current_rad += (yaw_diff > 0 ? gimbal_rate : -gimbal_rate);
    }

    double max_angle = m_engine_config.gimbal_max_angle_deg * M_PI / 180.0;
    m_state.main_engines.gimbal_pitch_current_rad = std::max(-max_angle, std::min(max_angle, m_state.main_engines.gimbal_pitch_current_rad));
    m_state.main_engines.gimbal_yaw_current_rad = std::max(-max_angle, std::min(max_angle, m_state.main_engines.gimbal_yaw_current_rad));

    m_state.main_engines.current_thrust_N = compute_thrust(ambient_pressure_Pa) * m_state.main_engines.throttle_current;
    m_state.main_engines.current_isp_s = compute_isp(ambient_pressure_Pa);
    m_state.main_engines.mass_flow_rate_kg_per_s = compute_mass_flow(m_state.main_engines.current_thrust_N, m_state.main_engines.current_isp_s);

    m_state.main_engines.total_burn_time_s += dt;
    m_state.main_engines.total_impulse_Ns += m_state.main_engines.current_thrust_N * dt;
    m_state.main_fuel_consumed_kg += m_state.main_engines.mass_flow_rate_kg_per_s * dt;
    m_state.main_engines.engine_temp_K += m_state.main_engines.current_thrust_N * dt * 0.00001;
}

void PropulsionSystem::ignite() {
    m_state.main_engines.ignited = true;
    m_state.main_engines.shutdown = false;
    m_state.main_engines.throttle_current = 0.0;
}

void PropulsionSystem::shutdown() {
    m_state.main_engines.shutdown = true;
    m_state.main_engines.ignited = false;
    m_state.main_engines.throttle_command = 0.0;
    m_state.main_engines.current_thrust_N = 0.0;
    m_state.main_engines.mass_flow_rate_kg_per_s = 0.0;
}

void PropulsionSystem::set_gimbal(double pitch_rad, double yaw_rad) {
    m_state.main_engines.gimbal_pitch_command_rad = pitch_rad;
    m_state.main_engines.gimbal_yaw_command_rad = yaw_rad;
}

void PropulsionSystem::fire_rcs_pitch_positive() { m_state.rcs_active[0] = true; }
void PropulsionSystem::fire_rcs_pitch_negative() { m_state.rcs_active[1] = true; }
void PropulsionSystem::fire_rcs_yaw_positive()   { m_state.rcs_active[2] = true; }
void PropulsionSystem::fire_rcs_yaw_negative()   { m_state.rcs_active[3] = true; }
void PropulsionSystem::fire_rcs_roll_positive()  { m_state.rcs_active[4] = true; }
void PropulsionSystem::fire_rcs_roll_negative()  { m_state.rcs_active[5] = true; }

void PropulsionSystem::stop_all_rcs() {
    for (int i = 0; i < 12; i++) {
        m_state.rcs_active[i] = false;
    }
}

Vector3 PropulsionSystem::get_rcs_torque() const {
    double torque_magnitude = 0.0;
    if (m_state.rcs_active[0]) torque_magnitude += m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m;
    if (m_state.rcs_active[1]) torque_magnitude -= m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m;
    if (m_state.rcs_active[2]) torque_magnitude += m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m;
    if (m_state.rcs_active[3]) torque_magnitude -= m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m;
    if (m_state.rcs_active[4]) torque_magnitude += m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m * 0.5;
    if (m_state.rcs_active[5]) torque_magnitude -= m_rcs_config.thrust_per_thruster_N * m_rcs_config.moment_arm_m * 0.5;
    return Vector3{torque_magnitude, torque_magnitude * 0.5, torque_magnitude * 0.3};
}

Vector3 PropulsionSystem::compute_thrust_vector_body(const Quaternion& attitude) const {
    double pitch = m_state.main_engines.gimbal_pitch_current_rad;
    double yaw = m_state.main_engines.gimbal_yaw_current_rad;
    double thrust = m_state.main_engines.current_thrust_N;

    double tx = thrust * std::sin(yaw) * std::cos(pitch);
    double ty = thrust * std::sin(pitch);
    double tz = thrust * std::cos(yaw) * std::cos(pitch);

    return Vector3{tx, ty, tz};
}

Vector3 PropulsionSystem::compute_torque_body(const Vector3& thrust_point,
                                              const Vector3& center_of_mass) const {
    Vector3 thrust_vec = compute_thrust_vector_body(Quaternion::identity());
    Vector3 lever_arm = {thrust_point.x - center_of_mass.x,
                         thrust_point.y - center_of_mass.y,
                         thrust_point.z - center_of_mass.z};
    return lever_arm.cross(thrust_vec);
}

double PropulsionSystem::compute_thrust(double ambient_pressure_Pa) const {
    double thrust_vac = m_engine_config.thrust_vacuum_per_engine_N * m_engine_config.num_engines;
    double thrust_sl = m_engine_config.thrust_sea_level_per_engine_N * m_engine_config.num_engines;
    double ref_pressure = 101325.0;
    double pressure_ratio = ambient_pressure_Pa / ref_pressure;
    if (pressure_ratio < 0.0) pressure_ratio = 0.0;
    if (pressure_ratio > 1.0) pressure_ratio = 1.0;
    return thrust_vac - (thrust_vac - thrust_sl) * pressure_ratio;
}

double PropulsionSystem::compute_isp(double ambient_pressure_Pa) const {
    double isp_vac = m_engine_config.isp_vacuum_s;
    double isp_sl = m_engine_config.isp_sea_level_s;
    double ref_pressure = 101325.0;
    double pressure_ratio = ambient_pressure_Pa / ref_pressure;
    if (pressure_ratio < 0.0) pressure_ratio = 0.0;
    if (pressure_ratio > 1.0) pressure_ratio = 1.0;
    return isp_vac - (isp_vac - isp_sl) * pressure_ratio;
}

double PropulsionSystem::compute_mass_flow(double thrust_N, double isp_s) const {
    if (isp_s < 1.0) return 0.0;
    return thrust_N / (isp_s * 9.80665);
}

} // namespace LunarAscent