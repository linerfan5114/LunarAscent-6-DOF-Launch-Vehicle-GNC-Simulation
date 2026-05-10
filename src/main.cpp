#include "vehicle.hpp"
#include "propulsion.hpp"
#include "environment.hpp"
#include "guidance.hpp"
#include "navigation.hpp"
#include "control.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>

using namespace LunarAscent;

struct SimulationConfig {
    double dt = 0.01;
    double simulation_time_max_s = 3600.0;
    bool enable_gps = true;
    bool enable_star_tracker = true;
    bool enable_wind = true;
    bool enable_max_q_throttle = true;
    bool enable_closed_loop_guidance = true;
    bool enable_tli = true;
    bool verbose = true;
    int print_interval_steps = 100;
    std::string output_csv = "trajectory.csv";
};

struct MissionResults {
    double final_altitude_km = 0.0;
    double final_velocity_m_s = 0.0;
    double final_mass_kg = 0.0;
    double max_q_kPa = 0.0;
    double max_acceleration_g = 0.0;
    double max_mach = 0.0;
    double apogee_km = 0.0;
    double perigee_km = 0.0;
    double inclination_deg = 0.0;
    double eccentricity = 0.0;
    double semi_major_axis_km = 0.0;
    double total_burn_time_s = 0.0;
    double total_propellant_used_kg = 0.0;
    double orbit_energy_MJ_per_kg = 0.0;
    bool orbit_achieved = false;
    bool tli_complete = false;
    GuidancePhase final_phase;
    double simulation_time_s = 0.0;
};

struct TelemetryPoint {
    double time_s;
    double altitude_km;
    double velocity_m_s;
    double mach;
    double dynamic_pressure_kPa;
    double acceleration_g;
    double mass_kg;
    double thrust_N;
    double pitch_deg;
    double throttle_pct;
    int stage;
    GuidancePhase phase;
};

class LunarAscentSimulator {
public:
    LunarAscentSimulator(const VehicleConfig& vehicle_config, const SimulationConfig& sim_config);
    
    MissionResults run();
    void save_trajectory(const std::string& filename);
    void print_results(const MissionResults& results) const;
    
private:
    Vehicle m_vehicle;
    PropulsionSystem m_propulsion;
    Environment m_environment;
    Guidance m_guidance;
    Navigation m_navigation;
    AttitudeControl m_control;
    SimulationConfig m_config;
    
    std::vector<TelemetryPoint> m_telemetry;
    
    void initialize();
    void step(double& mission_time, double dt);
    void record_telemetry(double mission_time);
    MissionResults compute_results() const;
    void print_status(double mission_time) const;
};

LunarAscentSimulator::LunarAscentSimulator(const VehicleConfig& vehicle_config,
                                           const SimulationConfig& sim_config)
    : m_vehicle(vehicle_config)
    , m_propulsion(EngineConfig{}, RCSConfig{})
    , m_environment(28.5, -80.6)
    , m_guidance()
    , m_navigation()
    , m_control()
    , m_config(sim_config)
{
    m_telemetry.reserve(static_cast<size_t>(sim_config.simulation_time_max_s / sim_config.dt * 0.1));
}

void LunarAscentSimulator::initialize() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  LunarAscent - Launch Vehicle Simulator" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "Vehicle: " << m_vehicle.get_config().name << std::endl;
    std::cout << "Stages: " << m_vehicle.get_config().num_stages << std::endl;
    std::cout << "Initial mass: " << m_vehicle.get_mass() << " kg" << std::endl;
    std::cout << "Target orbit: " << m_guidance.get_target_orbit().apogee_km
              << "x" << m_guidance.get_target_orbit().perigee_km << " km" << std::endl;
    std::cout << "Time step: " << m_config.dt << " s" << std::endl;
    std::cout << "\nIgnition...\n" << std::endl;
    
    m_propulsion.ignite();
}

void LunarAscentSimulator::step(double& mission_time, double dt) {
    GuidanceCommands guidance_cmd = m_guidance.get_commands();
    
    Quaternion current_att = m_vehicle.get_attitude();
    Vector3 current_omega = m_vehicle.get_angular_velocity();
    
    Quaternion target_att = Quaternion::from_euler(
        0.0, guidance_cmd.commanded_pitch_rad, guidance_cmd.commanded_yaw_rad);
    Vector3 target_omega = Vector3{0.0, 0.0, 0.0};
    
    double mach = m_environment.get_mach();
    double q = m_environment.get_dynamic_pressure();
    
    m_control.update(dt, current_att, current_omega, target_att, target_omega, mach, q);
    ControlCommands ctrl_cmd = m_control.get_commands();
    
    m_propulsion.set_gimbal(ctrl_cmd.gimbal_pitch_cmd_rad, ctrl_cmd.gimbal_yaw_cmd_rad);
    
    double ambient_pressure = m_environment.get_atmosphere(
        m_environment.get_altitude_km() * 1000.0).pressure_Pa;
    bool ignition_signal = (guidance_cmd.phase != GuidancePhase::MISSION_COMPLETE &&
                           guidance_cmd.phase != GuidancePhase::PRE_LAUNCH);
    m_propulsion.update(dt, ambient_pressure, ignition_signal, ctrl_cmd.throttle_cmd);
    
    if (ctrl_cmd.rcs_pitch_up) m_propulsion.fire_rcs_pitch_positive();
    if (ctrl_cmd.rcs_pitch_down) m_propulsion.fire_rcs_pitch_negative();
    if (ctrl_cmd.rcs_yaw_left) m_propulsion.fire_rcs_yaw_positive();
    if (ctrl_cmd.rcs_yaw_right) m_propulsion.fire_rcs_yaw_negative();
    if (ctrl_cmd.rcs_roll_cw) m_propulsion.fire_rcs_roll_positive();
    if (ctrl_cmd.rcs_roll_ccw) m_propulsion.fire_rcs_roll_negative();
    
    Vector3 thrust_body = m_propulsion.compute_thrust_vector_body(current_att);
    Vector3 rcs_torque = m_propulsion.get_rcs_torque();
    
    m_environment.update(m_vehicle.get_position_ecef(),
                        m_vehicle.get_velocity_ecef(),
                        m_vehicle.get_attitude(),
                        m_vehicle.get_config().aero.reference_area,
                        m_vehicle.get_config().aero.drag_coeff_cd);
    
    Vector3 gravity_ecef = m_environment.get_gravity(m_vehicle.get_position_ecef()).acceleration;
    Vector3 aero_force_body{
        m_environment.get_state().aero_forces.drag_body_N.x,
        m_environment.get_state().aero_forces.side_force_body_N.y,
        m_environment.get_state().aero_forces.lift_body_N.z
    };
    
    Vector3 total_force_body = thrust_body + aero_force_body;
    Vector3 total_torque = m_propulsion.compute_torque_body(
        Vector3{0.0, 0.0, -15.0},
        Vector3{0.0, 0.0, -5.0}) + rcs_torque;
    
    m_vehicle.update(dt, total_force_body, total_torque);
    
    m_guidance.update(mission_time,
                     m_vehicle.get_position_ecef(),
                     m_vehicle.get_velocity_ecef(),
                     m_vehicle.get_attitude(),
                     m_vehicle.get_altitude(),
                     m_vehicle.get_velocity_ecef().magnitude(),
                     m_environment.get_mach());
    
    if (m_guidance.get_commands().stage_separation && m_vehicle.get_current_stage() == 0) {
        std::cout << "\n>>> STAGE 1 SEPARATION <<<" << std::endl;
        m_vehicle.separate_stage();
    }
    
    if (m_guidance.get_commands().fairing_jettison && m_vehicle.has_fairing()) {
        std::cout << "\n>>> FAIRING JETTISON <<<" << std::endl;
        m_vehicle.jettison_fairing();
    }
    
    mission_time += dt;
}

void LunarAscentSimulator::record_telemetry(double mission_time) {
    TelemetryPoint tp;
    tp.time_s = mission_time;
    tp.altitude_km = m_environment.get_altitude_km();
    tp.velocity_m_s = m_vehicle.get_velocity_ecef().magnitude();
    tp.mach = m_environment.get_mach();
    tp.dynamic_pressure_kPa = m_environment.get_dynamic_pressure() / 1000.0;
    tp.acceleration_g = m_vehicle.get_acceleration_body().magnitude() / 9.80665;
    tp.mass_kg = m_vehicle.get_mass();
    tp.thrust_N = m_propulsion.get_thrust_N();
    tp.pitch_deg = m_guidance.get_commands().commanded_pitch_rad * 180.0 / M_PI + 90.0;
    tp.throttle_pct = m_propulsion.get_throttle() * 100.0;
    tp.stage = m_vehicle.get_current_stage();
    tp.phase = m_guidance.get_phase();
    
    m_telemetry.push_back(tp);
}

MissionResults LunarAscentSimulator::run() {
    initialize();
    
    double mission_time = 0.0;
    int step_count = 0;
    
    while (mission_time < m_config.simulation_time_max_s) {
        step(mission_time, m_config.dt);
        step_count++;
        
        if (step_count % m_config.print_interval_steps == 0) {
            print_status(mission_time);
        }
        
        if (step_count % 10 == 0) {
            record_telemetry(mission_time);
        }
        
        if (m_guidance.get_phase() == GuidancePhase::MISSION_COMPLETE) {
            std::cout << "\n>>> MISSION COMPLETE <<<" << std::endl;
            break;
        }
    }
    
    std::cout << "\nSaving trajectory data..." << std::endl;
    save_trajectory(m_config.output_csv);
    
    return compute_results();
}

void LunarAscentSimulator::save_trajectory(const std::string& filename) {
    std::ofstream file(filename);
    file << "time_s,altitude_km,velocity_m_s,mach,q_kPa,accel_g,"
         << "mass_kg,thrust_N,pitch_deg,throttle_pct,stage,phase\n";
    
    for (const auto& tp : m_telemetry) {
        file << std::fixed << std::setprecision(3)
             << tp.time_s << ","
             << tp.altitude_km << ","
             << tp.velocity_m_s << ","
             << tp.mach << ","
             << tp.dynamic_pressure_kPa << ","
             << tp.acceleration_g << ","
             << tp.mass_kg << ","
             << tp.thrust_N << ","
             << tp.pitch_deg << ","
             << tp.throttle_pct << ","
             << tp.stage << ","
             << static_cast<int>(tp.phase) << "\n";
    }
    
    file.close();
    std::cout << "Saved " << m_telemetry.size() << " points to " << filename << std::endl;
}

MissionResults LunarAscentSimulator::compute_results() const {
    MissionResults results;
    
    if (!m_telemetry.empty()) {
        const auto& last = m_telemetry.back();
        results.final_altitude_km = last.altitude_km;
        results.final_velocity_m_s = last.velocity_m_s;
        results.final_mass_kg = last.mass_kg;
        results.simulation_time_s = last.time_s;
        results.final_phase = last.phase;
        
        for (const auto& tp : m_telemetry) {
            if (tp.dynamic_pressure_kPa > results.max_q_kPa) {
                results.max_q_kPa = tp.dynamic_pressure_kPa;
            }
            if (tp.acceleration_g > results.max_acceleration_g) {
                results.max_acceleration_g = tp.acceleration_g;
            }
            if (tp.mach > results.max_mach) {
                results.max_mach = tp.mach;
            }
        }
    }
    
    results.total_burn_time_s = m_propulsion.get_total_burn_time();
    results.total_propellant_used_kg = m_propulsion.get_main_fuel_consumed();
    
    results.orbit_achieved = (results.final_altitude_km > 150.0 &&
                              results.final_velocity_m_s > 7000.0);
    results.orbit_energy_MJ_per_kg = results.final_velocity_m_s * results.final_velocity_m_s / 2.0 / 1e6 -
                                     3.986e14 / (6371e3 + results.final_altitude_km * 1000.0) / 1e6;
    
    return results;
}

void LunarAscentSimulator::print_status(double mission_time) const {
    std::cout << std::fixed << std::setprecision(1)
              << "T+" << mission_time << "s | "
              << "Alt: " << m_environment.get_altitude_km() << "km | "
              << "Vel: " << m_vehicle.get_velocity_ecef().magnitude() << "m/s | "
              << "Mach: " << m_environment.get_mach() << " | "
              << "Q: " << m_environment.get_dynamic_pressure()/1000.0 << "kPa | "
              << "Mass: " << m_vehicle.get_mass() << "kg | "
              << "Phase: " << static_cast<int>(m_guidance.get_phase())
              << std::endl;
}

void LunarAscentSimulator::print_results(const MissionResults& results) const {
    std::cout << "\n" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "            MISSION RESULTS             " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Mission time:       " << results.simulation_time_s << " s" << std::endl;
    std::cout << "  Final altitude:     " << results.final_altitude_km << " km" << std::endl;
    std::cout << "  Final velocity:     " << results.final_velocity_m_s << " m/s" << std::endl;
    std::cout << "  Final mass:         " << results.final_mass_kg << " kg" << std::endl;
    std::cout << "  Max Q:              " << results.max_q_kPa << " kPa" << std::endl;
    std::cout << "  Max acceleration:   " << results.max_acceleration_g << " g" << std::endl;
    std::cout << "  Max Mach:           " << results.max_mach << std::endl;
    std::cout << "  Total burn time:    " << results.total_burn_time_s << " s" << std::endl;
    std::cout << "  Propellant used:    " << results.total_propellant_used_kg << " kg" << std::endl;
    std::cout << "  Orbit achieved:     " << (results.orbit_achieved ? "YES" : "NO") << std::endl;
    std::cout << "  Orbital energy:     " << results.orbit_energy_MJ_per_kg << " MJ/kg" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main() {
    VehicleConfig vehicle_config;
    vehicle_config.name = "Falcon 9 v1.2";
    vehicle_config.num_stages = 2;
    
    vehicle_config.stages[0] = {
        25600.0, 395700.0,
        8227000.0, 7607000.0,
        348.0, 312.0,
        162.0,
        8.0, 5.0
    };
    
    vehicle_config.stages[1] = {
        3900.0, 92670.0,
        934000.0, 934000.0,
        348.0, 348.0,
        397.0,
        8.0, 5.0
    };
    
    SimulationConfig sim_config;
    sim_config.dt = 0.01;
    sim_config.simulation_time_max_s = 600.0;
    sim_config.print_interval_steps = 500;
    sim_config.enable_tli = false;
    sim_config.output_csv = "trajectory.csv";
    
    LunarAscentSimulator simulator(vehicle_config, sim_config);
    MissionResults results = simulator.run();
    simulator.print_results(results);
    
    return 0;
}