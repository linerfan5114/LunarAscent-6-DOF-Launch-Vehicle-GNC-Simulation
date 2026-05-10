#ifndef GUIDANCE_HPP
#define GUIDANCE_HPP

#include "vehicle.hpp"
#include "environment.hpp"
#include <vector>
#include <cmath>
#include <array>

namespace LunarAscent {

struct TargetOrbit {
    double apogee_km = 200.0;
    double perigee_km = 200.0;
    double inclination_deg = 28.5;
    double raan_deg = 0.0;
    double arg_perigee_deg = 0.0;
    bool circular = true;
};

struct TransLunarTarget {
    double injection_delta_v_m_s = 3150.0;
    double injection_altitude_km = 200.0;
    double flight_time_days = 3.5;
    double arrival_velocity_m_s = 800.0;
    Vector3 injection_point_ecef;
    bool front_side_arrival = true;
};

enum class GuidancePhase {
    PRE_LAUNCH,
    VERTICAL_ASCENT,
    PITCH_OVER,
    GRAVITY_TURN,
    MAX_Q_THROTTLE,
    OPEN_LOOP_PITCH,
    CLOSED_LOOP_GUIDANCE,
    ORBIT_INJECTION,
    COAST,
    TRANS_LUNAR_INJECTION,
    TERMINAL_GUIDANCE,
    MISSION_COMPLETE
};

struct GuidanceCommands {
    double commanded_pitch_rad = 0.0;
    double commanded_yaw_rad = 0.0;
    double commanded_roll_rad = 0.0;
    double commanded_throttle = 1.0;
    bool stage_separation = false;
    bool fairing_jettison = false;
    bool tli_burn = false;
    GuidancePhase phase = GuidancePhase::PRE_LAUNCH;
};

struct PitchProgramEntry {
    double time_s;
    double pitch_degrees;
};

class Guidance {
public:
    Guidance();
    
    void update(double mission_time_s, const Vector3& position_ecef,
                const Vector3& velocity_ecef, const Quaternion& attitude,
                double altitude_m, double speed_m_s, double mach);
    
    void set_target_orbit(const TargetOrbit& orbit) { m_target_orbit = orbit; }
    void set_trans_lunar_target(const TransLunarTarget& tli) { m_tli_target = tli; }
    
    GuidanceCommands get_commands() const { return m_commands; }
    GuidancePhase get_phase() const { return m_commands.phase; }
    
    void load_pitch_program(const std::vector<PitchProgramEntry>& program);
    void use_default_falcon9_profile();
    void use_apollo_profile();
    
    bool is_in_coast_phase() const { return m_commands.phase == GuidancePhase::COAST; }
    bool is_orbit_insertion_ready() const;
    bool is_tli_ready() const;
    
    double compute_required_pitch(double altitude_km, double velocity_m_s) const;
    double compute_gravity_turn_pitch(const Vector3& velocity_ecef,
                                      const Vector3& position_ecef) const;
    Vector3 compute_lambert_target(const Vector3& current_position,
                                   const Vector3& current_velocity,
                                   const Vector3& target_position,
                                   double transfer_time_s) const;
    double compute_injection_delta_v(const Vector3& velocity_vec,
                                     const TargetOrbit& target) const;
    double compute_tli_delta_v(const Vector3& position, const Vector3& velocity) const;
    double compute_orbital_energy(const Vector3& position, const Vector3& velocity) const;
    double compute_semi_major_axis(const Vector3& position, const Vector3& velocity) const;
    double compute_eccentricity(const Vector3& position, const Vector3& velocity) const;
    double compute_inclination(const Vector3& position, const Vector3& velocity) const;
    
    static constexpr double EARTH_MU = 3.986004418e14;
    static constexpr double EARTH_RADIUS = 6371000.0;
    
    std::vector<PitchProgramEntry> get_pitch_program() const { return m_pitch_program; }
    Vector3 get_target_position() const { return m_target_position; }
    TargetOrbit get_target_orbit() const { return m_target_orbit; }
    
private:
    GuidanceCommands m_commands;
    TargetOrbit m_target_orbit;
    TransLunarTarget m_tli_target;
    Vector3 m_target_position;
    Vector3 m_tli_direction;
    std::vector<PitchProgramEntry> m_pitch_program;
    
    bool m_pitch_program_loaded;
    bool m_target_set;
    double m_initial_launch_azimuth_rad;
    double m_max_q_throttle_timer;
    double m_pitch_program_start_time;
    
    void update_phase(const Vector3& position, const Vector3& velocity,
                      double altitude, double speed, double mach);
    double compute_pitch_from_program(double mission_time_s) const;
    double compute_closed_loop_pitch(const Vector3& position,
                                     const Vector3& velocity,
                                     const TargetOrbit& target) const;
    Vector3 compute_orbital_normal(const Vector3& position,
                                   const Vector3& velocity) const;
};

} // namespace LunarAscent

#endif // GUIDANCE_HPP