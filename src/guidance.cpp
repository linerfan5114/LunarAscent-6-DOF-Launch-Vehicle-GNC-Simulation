#include "guidance.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

Guidance::Guidance()
    : m_pitch_program_loaded(false)
    , m_target_set(false)
    , m_initial_launch_azimuth_rad(0.0)
    , m_max_q_throttle_timer(0.0)
    , m_pitch_program_start_time(0.0)
{
    m_commands.commanded_pitch_rad = 0.0;
    m_commands.commanded_yaw_rad = 0.0;
    m_commands.commanded_roll_rad = 0.0;
    m_commands.commanded_throttle = 1.0;
    m_commands.stage_separation = false;
    m_commands.fairing_jettison = false;
    m_commands.tli_burn = false;
    m_commands.phase = GuidancePhase::PRE_LAUNCH;
    
    m_target_orbit.apogee_km = 200.0;
    m_target_orbit.perigee_km = 200.0;
    m_target_orbit.inclination_deg = 28.5;
    m_target_orbit.raan_deg = 0.0;
    m_target_orbit.arg_perigee_deg = 0.0;
    m_target_orbit.circular = true;
    
    m_tli_target.injection_delta_v_m_s = 3150.0;
    m_tli_target.injection_altitude_km = 200.0;
    m_tli_target.flight_time_days = 3.5;
    m_tli_target.arrival_velocity_m_s = 800.0;
}

void Guidance::update(double mission_time_s, const Vector3& position_ecef,
                      const Vector3& velocity_ecef, const Quaternion& attitude,
                      double altitude_m, double speed_m_s, double mach) {
    m_commands.stage_separation = false;
    m_commands.fairing_jettison = false;
    m_commands.tli_burn = false;
    
    double altitude_km = altitude_m / 1000.0;
    
    update_phase(position_ecef, velocity_ecef, altitude_km, speed_m_s, mach);
    
    switch (m_commands.phase) {
        case GuidancePhase::PRE_LAUNCH:
            m_commands.commanded_pitch_rad = 0.0;
            m_commands.commanded_yaw_rad = 0.0;
            m_commands.commanded_throttle = 1.0;
            break;
            
        case GuidancePhase::VERTICAL_ASCENT:
            m_commands.commanded_pitch_rad = 0.0;
            m_commands.commanded_yaw_rad = 0.0;
            m_commands.commanded_throttle = 1.0;
            break;
            
        case GuidancePhase::PITCH_OVER:
            {
                double pitch_time = mission_time_s - m_pitch_program_start_time;
                double pitch_angle_deg = 90.0 - pitch_time * 2.5;
                if (pitch_angle_deg < 65.0) pitch_angle_deg = 65.0;
                m_commands.commanded_pitch_rad = (90.0 - pitch_angle_deg) * M_PI / 180.0;
                m_commands.commanded_yaw_rad = m_initial_launch_azimuth_rad;
                m_commands.commanded_throttle = 1.0;
                
                if (mission_time_s > 180.0) {
                    m_commands.fairing_jettison = true;
                }
            }
            break;
            
        case GuidancePhase::GRAVITY_TURN:
            m_commands.commanded_pitch_rad = compute_gravity_turn_pitch(velocity_ecef, position_ecef);
            m_commands.commanded_yaw_rad = m_initial_launch_azimuth_rad;
            m_commands.commanded_throttle = 1.0;
            break;
            
        case GuidancePhase::MAX_Q_THROTTLE:
            {
                double q = 0.5 * 1.225 * speed_m_s * speed_m_s / 1000.0;
                if (q > 30.0) {
                    m_commands.commanded_throttle = 0.7;
                    m_commands.phase = GuidancePhase::MAX_Q_THROTTLE;
                } else if (q < 15.0 && m_commands.commanded_throttle < 1.0) {
                    m_commands.commanded_throttle = 1.0;
                }
                m_commands.commanded_pitch_rad = compute_gravity_turn_pitch(velocity_ecef, position_ecef);
                m_commands.commanded_yaw_rad = m_initial_launch_azimuth_rad;
            }
            break;
            
        case GuidancePhase::OPEN_LOOP_PITCH:
            m_commands.commanded_pitch_rad = compute_pitch_from_program(mission_time_s);
            m_commands.commanded_yaw_rad = m_initial_launch_azimuth_rad;
            m_commands.commanded_throttle = 1.0;
            break;
            
        case GuidancePhase::CLOSED_LOOP_GUIDANCE:
            m_commands.commanded_pitch_rad = compute_closed_loop_pitch(position_ecef, velocity_ecef, m_target_orbit);
            m_commands.commanded_yaw_rad = m_initial_launch_azimuth_rad;
            m_commands.commanded_throttle = 1.0;
            
            if (is_orbit_insertion_ready()) {
                m_commands.commanded_throttle = 0.0;
                m_commands.phase = GuidancePhase::ORBIT_INJECTION;
            }
            break;
            
        case GuidancePhase::ORBIT_INJECTION:
            {
                double dv_needed = compute_injection_delta_v(velocity_ecef, m_target_orbit);
                if (dv_needed > 10.0) {
                    m_commands.commanded_pitch_rad = 0.0;
                    m_commands.commanded_throttle = 1.0;
                } else {
                    m_commands.commanded_throttle = 0.0;
                    m_commands.phase = GuidancePhase::COAST;
                }
            }
            break;
            
        case GuidancePhase::COAST:
            m_commands.commanded_pitch_rad = 0.0;
            m_commands.commanded_throttle = 0.0;
            break;
            
        case GuidancePhase::TRANS_LUNAR_INJECTION:
            if (is_tli_ready()) {
                m_commands.commanded_pitch_rad = compute_closed_loop_pitch(position_ecef, velocity_ecef, m_target_orbit);
                m_commands.commanded_throttle = 1.0;
                m_commands.tli_burn = true;
                double tli_dv = compute_tli_delta_v(position_ecef, velocity_ecef);
                if (tli_dv > m_tli_target.injection_delta_v_m_s) {
                    m_commands.commanded_throttle = 0.0;
                    m_commands.phase = GuidancePhase::MISSION_COMPLETE;
                }
            }
            break;
            
        case GuidancePhase::MISSION_COMPLETE:
            m_commands.commanded_throttle = 0.0;
            break;
            
        default:
            break;
    }
}

void Guidance::update_phase(const Vector3& position, const Vector3& velocity,
                            double altitude_km, double speed_m_s, double mach) {
    switch (m_commands.phase) {
        case GuidancePhase::PRE_LAUNCH:
            m_commands.phase = GuidancePhase::VERTICAL_ASCENT;
            m_pitch_program_start_time = 0.0;
            break;
            
        case GuidancePhase::VERTICAL_ASCENT:
            if (altitude_km > 0.5) {
                m_commands.phase = GuidancePhase::PITCH_OVER;
                m_pitch_program_start_time = 0.0;
            }
            break;
            
        case GuidancePhase::PITCH_OVER:
            if (altitude_km > 10.0) {
                m_commands.phase = GuidancePhase::GRAVITY_TURN;
            }
            break;
            
        case GuidancePhase::GRAVITY_TURN:
            if (mach > 0.8 && altitude_km < 25.0) {
                m_commands.phase = GuidancePhase::MAX_Q_THROTTLE;
            } else if (altitude_km > 40.0) {
                m_commands.phase = GuidancePhase::OPEN_LOOP_PITCH;
            }
            break;
            
        case GuidancePhase::MAX_Q_THROTTLE:
            if (mach < 0.8 || altitude_km > 40.0) {
                m_commands.commanded_throttle = 1.0;
                m_commands.phase = GuidancePhase::OPEN_LOOP_PITCH;
            }
            break;
            
        case GuidancePhase::OPEN_LOOP_PITCH:
            if (altitude_km > 100.0) {
                m_commands.phase = GuidancePhase::CLOSED_LOOP_GUIDANCE;
            }
            break;
            
        case GuidancePhase::CLOSED_LOOP_GUIDANCE:
            break;
            
        case GuidancePhase::ORBIT_INJECTION:
            break;
            
        case GuidancePhase::COAST:
            break;
            
        case GuidancePhase::TRANS_LUNAR_INJECTION:
            break;
            
        default:
            break;
    }
}

void Guidance::load_pitch_program(const std::vector<PitchProgramEntry>& program) {
    m_pitch_program = program;
    m_pitch_program_loaded = true;
    
    std::sort(m_pitch_program.begin(), m_pitch_program.end(),
              [](const PitchProgramEntry& a, const PitchProgramEntry& b) {
                  return a.time_s < b.time_s;
              });
}

void Guidance::use_default_falcon9_profile() {
    m_pitch_program = {
        {0.0, 90.0},
        {10.0, 90.0},
        {20.0, 78.0},
        {30.0, 68.0},
        {40.0, 58.0},
        {50.0, 48.0},
        {60.0, 38.0},
        {80.0, 28.0},
        {100.0, 20.0},
        {120.0, 15.0},
        {140.0, 10.0},
        {160.0, 5.0},
        {180.0, 2.0},
        {200.0, 1.0},
        {250.0, 0.0}
    };
    m_pitch_program_loaded = true;
}

void Guidance::use_apollo_profile() {
    m_pitch_program = {
        {0.0, 90.0},
        {15.0, 90.0},
        {30.0, 75.0},
        {45.0, 60.0},
        {60.0, 50.0},
        {80.0, 40.0},
        {100.0, 30.0},
        {130.0, 20.0},
        {160.0, 10.0},
        {200.0, 5.0},
        {260.0, 2.0},
        {320.0, 1.0},
        {400.0, 0.0}
    };
    m_pitch_program_loaded = true;
}

double Guidance::compute_pitch_from_program(double mission_time_s) const {
    if (!m_pitch_program_loaded || m_pitch_program.empty()) {
        return 0.0;
    }
    
    if (mission_time_s <= m_pitch_program[0].time_s) {
        return (90.0 - m_pitch_program[0].pitch_degrees) * M_PI / 180.0;
    }
    
    for (size_t i = 0; i < m_pitch_program.size() - 1; i++) {
        if (mission_time_s >= m_pitch_program[i].time_s &&
            mission_time_s <= m_pitch_program[i + 1].time_s) {
            double dt = m_pitch_program[i + 1].time_s - m_pitch_program[i].time_s;
            double dp = m_pitch_program[i + 1].pitch_degrees - m_pitch_program[i].pitch_degrees;
            double t = (mission_time_s - m_pitch_program[i].time_s) / dt;
            double pitch = m_pitch_program[i].pitch_degrees + dp * t;
            return (90.0 - pitch) * M_PI / 180.0;
        }
    }
    
    return (90.0 - m_pitch_program.back().pitch_degrees) * M_PI / 180.0;
}

double Guidance::compute_gravity_turn_pitch(const Vector3& velocity_ecef,
                                            const Vector3& position_ecef) const {
    Vector3 up = position_ecef.normalized();
    Vector3 vel_horiz = velocity_ecef - up * velocity_ecef.dot(up);
    double horiz_mag = vel_horiz.magnitude();
    double vert_mag = velocity_ecef.dot(up);
    
    if (velocity_ecef.magnitude() < 1.0) return 0.0;
    
    double pitch_from_vertical = std::atan2(horiz_mag, vert_mag);
    return pitch_from_vertical;
}

double Guidance::compute_closed_loop_pitch(const Vector3& position,
                                           const Vector3& velocity,
                                           const TargetOrbit& target) const {
    double semi_major_axis = compute_semi_major_axis(position, velocity);
    double target_sma = EARTH_RADIUS + (target.apogee_km + target.perigee_km) / 2.0 * 1000.0;
    double sma_error = target_sma - semi_major_axis;
    
    double alt = position.magnitude() - EARTH_RADIUS;
    double alt_target = target.perigee_km * 1000.0;
    double alt_error = alt_target - alt;
    
    Vector3 up = position.normalized();
    Vector3 velocity_parallel = up * velocity.dot(up);
    Vector3 velocity_perp = velocity - velocity_parallel;
    double vert_speed = velocity.dot(up);
    double horiz_speed = velocity_perp.magnitude();
    
    double target_vert_speed = alt_error * 0.1 + 100.0;
    double vert_error = target_vert_speed - vert_speed;
    
    double desired_flight_path_angle = std::atan2(-vert_error * 0.01, horiz_speed + 1.0);
    return std::max(-0.5, std::min(0.5, desired_flight_path_angle));
}

bool Guidance::is_orbit_insertion_ready() const {
    return m_commands.phase == GuidancePhase::CLOSED_LOOP_GUIDANCE;
}

bool Guidance::is_tli_ready() const {
    return m_commands.phase == GuidancePhase::TRANS_LUNAR_INJECTION ||
           m_commands.phase == GuidancePhase::COAST;
}

double Guidance::compute_injection_delta_v(const Vector3& velocity_vec,
                                           const TargetOrbit& target) const {
    double r = velocity_vec.magnitude();
    double v_circular = std::sqrt(EARTH_MU / (EARTH_RADIUS + target.perigee_km * 1000.0));
    return std::abs(v_circular - r);
}

double Guidance::compute_tli_delta_v(const Vector3& position, const Vector3& velocity) const {
    return m_tli_target.injection_delta_v_m_s;
}

double Guidance::compute_orbital_energy(const Vector3& position, const Vector3& velocity) const {
    double r = position.magnitude();
    double v2 = velocity.magnitude_sq();
    return v2 / 2.0 - EARTH_MU / r;
}

double Guidance::compute_semi_major_axis(const Vector3& position, const Vector3& velocity) const {
    double energy = compute_orbital_energy(position, velocity);
    if (std::abs(energy) < 1e-12) return 1e12;
    return -EARTH_MU / (2.0 * energy);
}

double Guidance::compute_eccentricity(const Vector3& position, const Vector3& velocity) const {
    double r = position.magnitude();
    double v2 = velocity.magnitude_sq();
    double rv = position.dot(velocity);
    double specific_angular_momentum = position.cross(velocity).magnitude();
    double e2 = 1.0 + (2.0 * (v2 / 2.0 - EARTH_MU / r) * specific_angular_momentum * specific_angular_momentum) / (EARTH_MU * EARTH_MU);
    if (e2 < 0.0) e2 = 0.0;
    return std::sqrt(e2);
}

double Guidance::compute_inclination(const Vector3& position, const Vector3& velocity) const {
    Vector3 h = position.cross(velocity);
    double h_mag = h.magnitude();
    if (h_mag < 1e-12) return 0.0;
    return std::acos(h.z / h_mag) * 180.0 / M_PI;
}

Vector3 Guidance::compute_lambert_target(const Vector3& current_position,
                                         const Vector3& current_velocity,
                                         const Vector3& target_position,
                                         double transfer_time_s) const {
    return target_position - current_position;
}

Vector3 Guidance::compute_orbital_normal(const Vector3& position,
                                         const Vector3& velocity) const {
    Vector3 h = position.cross(velocity);
    return h.normalized();
}

double Guidance::compute_required_pitch(double altitude_km, double velocity_m_s) const {
    return 0.0;
}

} // namespace LunarAscent