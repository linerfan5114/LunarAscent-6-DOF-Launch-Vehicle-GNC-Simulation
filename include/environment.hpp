#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "vehicle.hpp"
#include <array>
#include <cmath>

namespace LunarAscent {

struct WindProfile {
    double speed_m_s = 0.0;
    double direction_deg = 0.0;
    double gust_speed_m_s = 0.0;
    double shear_gradient_per_km = 2.0;
};

struct AtmosphereData {
    double temperature_K = 288.15;
    double pressure_Pa = 101325.0;
    double density_kg_m3 = 1.225;
    double speed_of_sound_m_s = 340.29;
    double dynamic_viscosity = 1.789e-5;
};

struct GravityData {
    Vector3 acceleration; // m/s² in ECEF frame
    double magnitude_m_s2 = 9.80665;
};

struct AerodynamicForces {
    Vector3 drag_body_N;
    Vector3 lift_body_N;
    Vector3 side_force_body_N;
    double dynamic_pressure_Pa = 0.0;
    double mach_number = 0.0;
    double angle_of_attack_rad = 0.0;
    double sideslip_rad = 0.0;
};

struct SolarRadiation {
    double flux_W_m2 = 1361.0;
    bool in_eclipse = false;
    double eclipse_duration_s = 0.0;
};

struct EnvironmentState {
    AtmosphereData atmosphere;
    GravityData gravity;
    AerodynamicForces aero_forces;
    WindProfile wind;
    SolarRadiation solar;
    double altitude_km = 0.0;
    double dynamic_pressure_kPa = 0.0;
};

class Environment {
public:
    Environment();
    explicit Environment(double launch_latitude_deg, double launch_longitude_deg);
    
    void update(const Vector3& position_ecef, const Vector3& velocity_ecef,
                const Quaternion& attitude, double reference_area_m2, double cd);
    void set_wind_profile(const WindProfile& wind) { m_wind = wind; }
    void set_launch_site(double lat_deg, double lon_deg, double alt_m = 0.0);
    
    AtmosphereData get_atmosphere(double altitude_m) const;
    GravityData get_gravity(const Vector3& position_ecef) const;
    WindProfile get_wind() const { return m_wind; }
    
    double get_dynamic_pressure() const { return m_state.dynamic_pressure_kPa * 1000.0; }
    double get_mach() const { return m_state.aero_forces.mach_number; }
    double get_altitude_km() const { return m_state.altitude_km; }
    double get_angle_of_attack() const { return m_state.aero_forces.angle_of_attack_rad; }
    double get_sideslip() const { return m_state.aero_forces.sideslip_rad; }
    EnvironmentState get_state() const { return m_state; }
    
    AerodynamicForces compute_aerodynamics(const Vector3& velocity_body_m_s,
                                           const Quaternion& attitude,
                                           double altitude_m,
                                           double reference_area,
                                           double cd) const;
    
    Vector3 get_wind_vector_ecef(double altitude_m) const;
    bool is_max_q_passed() const { return m_max_q_passed; }
    double get_max_q_kPa() const { return m_max_q_kPa; }
    
private:
    EnvironmentState m_state;
    WindProfile m_wind;
    bool m_max_q_passed;
    double m_max_q_kPa;
    double m_launch_lat_rad;
    double m_launch_lon_rad;
    double m_launch_alt_m;
    
    static constexpr double EARTH_MU = 3.986004418e14;
    static constexpr double EARTH_J2 = 1.08262668e-3;
    static constexpr double EARTH_RADIUS_EQUATORIAL_M = 6378137.0;
    static constexpr double EARTH_FLATTENING = 1.0 / 298.257223563;
    static constexpr double EARTH_ROTATION_RATE = 7.2921150e-5;
    static constexpr double G0 = 9.80665;
    static constexpr double R_GAS = 287.058;
    
    static constexpr int ATMOSPHERE_LAYERS = 8;
    static constexpr double ALTITUDE_TABLE[ATMOSPHERE_LAYERS] = {
        0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 84852.0
    };
    static constexpr double TEMP_TABLE[ATMOSPHERE_LAYERS] = {
        288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.95
    };
    static constexpr double PRESSURE_TABLE[ATMOSPHERE_LAYERS] = {
        101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734
    };
    static constexpr double LAPSE_RATE_TABLE[ATMOSPHERE_LAYERS - 1] = {
        -0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.0020, 0.0
    };
};

} // namespace LunarAscent

#endif // ENVIRONMENT_HPP