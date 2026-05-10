#ifndef PROPULSION_HPP
#define PROPULSION_HPP

#include "vehicle.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace LunarAscent {

struct EngineConfig {
    int num_engines = 9;
    double thrust_vacuum_per_engine_N = 934000.0;
    double thrust_sea_level_per_engine_N = 845000.0;
    double isp_vacuum_s = 348.0;
    double isp_sea_level_s = 312.0;
    double chamber_pressure_bar = 97.0;
    double expansion_ratio = 16.0;
    double nozzle_exit_area_m2 = 1.3;
    double gimbal_max_angle_deg = 8.0;
    double gimbal_max_rate_deg_per_s = 5.0;
    double throttle_min = 0.4;
    double throttle_max = 1.0;
    double throttle_rate_per_s = 0.2;
    double ignition_delay_s = 0.3;
    double shutdown_delay_s = 0.1;
    double warmup_time_s = 2.0;
};

struct EngineState {
    bool ignited = false;
    bool shutdown = false;
    double throttle_command = 0.0;
    double throttle_current = 0.0;
    double gimbal_pitch_command_rad = 0.0;
    double gimbal_yaw_command_rad = 0.0;
    double gimbal_pitch_current_rad = 0.0;
    double gimbal_yaw_current_rad = 0.0;
    double current_thrust_N = 0.0;
    double current_isp_s = 0.0;
    double mass_flow_rate_kg_per_s = 0.0;
    double total_burn_time_s = 0.0;
    double total_impulse_Ns = 0.0;
    double engine_temp_K = 300.0;
};

struct RCSConfig {
    int num_thrusters = 8;
    double thrust_per_thruster_N = 500.0;
    double isp_s = 240.0;
    double min_pulse_time_s = 0.05;
    double moment_arm_m = 3.5;
};

struct PropulsionState {
    EngineState main_engines;
    std::array<bool, 12> rcs_active;
    double rcs_fuel_consumed_kg = 0.0;
    double main_fuel_consumed_kg = 0.0;
};

class PropulsionSystem {
public:
    PropulsionSystem(const EngineConfig& engine_config, const RCSConfig& rcs_config);
    
    void update(double dt, double ambient_pressure_Pa, bool ignition_signal, double throttle_cmd);
    void ignite();
    void shutdown();
    void set_gimbal(double pitch_rad, double yaw_rad);
    void fire_rcs_pitch_positive();
    void fire_rcs_pitch_negative();
    void fire_rcs_yaw_positive();
    void fire_rcs_yaw_negative();
    void fire_rcs_roll_positive();
    void fire_rcs_roll_negative();
    void stop_all_rcs();
    
    double get_thrust_N() const { return m_state.main_engines.current_thrust_N; }
    double get_isp_s() const { return m_state.main_engines.current_isp_s; }
    double get_mass_flow_rate() const { return m_state.main_engines.mass_flow_rate_kg_per_s; }
    double get_gimbal_pitch_rad() const { return m_state.main_engines.gimbal_pitch_current_rad; }
    double get_gimbal_yaw_rad() const { return m_state.main_engines.gimbal_yaw_current_rad; }
    bool is_ignited() const { return m_state.main_engines.ignited; }
    bool is_shutdown() const { return m_state.main_engines.shutdown; }
    double get_throttle() const { return m_state.main_engines.throttle_current; }
    double get_total_burn_time() const { return m_state.main_engines.total_burn_time_s; }
    double get_total_impulse() const { return m_state.main_engines.total_impulse_Ns; }
    PropulsionState get_state() const { return m_state; }
    
    Vector3 get_rcs_torque() const;
    double get_rcs_fuel_consumed() const { return m_state.rcs_fuel_consumed_kg; }
    double get_main_fuel_consumed() const { return m_state.main_fuel_consumed_kg; }
    
    Vector3 compute_thrust_vector_body(const Quaternion& attitude) const;
    Vector3 compute_torque_body(const Vector3& thrust_point_of_application, 
                                const Vector3& center_of_mass) const;
    
private:
    EngineConfig m_engine_config;
    RCSConfig m_rcs_config;
    PropulsionState m_state;
    
    double compute_thrust(double ambient_pressure_Pa) const;
    double compute_isp(double ambient_pressure_Pa) const;
    double compute_mass_flow(double thrust_N, double isp_s) const;
};

} // namespace LunarAscent

#endif // PROPULSION_HPP