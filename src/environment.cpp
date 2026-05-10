#include "environment.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

Environment::Environment()
    : m_max_q_passed(false)
    , m_max_q_kPa(0.0)
    , m_launch_lat_rad(0.0)
    , m_launch_lon_rad(0.0)
    , m_launch_alt_m(0.0)
{
    m_state.atmosphere = {288.15, 101325.0, 1.225, 340.29, 1.789e-5};
    m_state.gravity.acceleration = Vector3{0.0, 0.0, -9.80665};
    m_state.gravity.magnitude_m_s2 = 9.80665;
    m_state.wind = {0.0, 0.0, 0.0, 2.0};
    m_state.solar = {1361.0, false, 0.0};
    m_state.altitude_km = 0.0;
    m_state.dynamic_pressure_kPa = 0.0;
}

Environment::Environment(double launch_latitude_deg, double launch_longitude_deg)
    : Environment()
{
    set_launch_site(launch_latitude_deg, launch_longitude_deg);
}

void Environment::set_launch_site(double lat_deg, double lon_deg, double alt_m) {
    m_launch_lat_rad = lat_deg * M_PI / 180.0;
    m_launch_lon_rad = lon_deg * M_PI / 180.0;
    m_launch_alt_m = alt_m;
}

void Environment::update(const Vector3& position_ecef, const Vector3& velocity_ecef,
                         const Quaternion& attitude, double reference_area_m2, double cd) {
    double r = position_ecef.magnitude();
    m_state.altitude_km = (r - EARTH_RADIUS_EQUATORIAL_M) / 1000.0;

    m_state.gravity = get_gravity(position_ecef);
    m_state.atmosphere = get_atmosphere(std::max(0.0, m_state.altitude_km * 1000.0));

    Vector3 vel_body = attitude.conjugate().rotate(velocity_ecef);
    double aoa = 0.0;
    double sideslip = 0.0;
    if (vel_body.magnitude() > 1.0) {
        aoa = std::atan2(-vel_body.z, vel_body.x);
        sideslip = std::asin(std::max(-1.0, std::min(1.0, vel_body.y / vel_body.magnitude())));
    }

    double speed = velocity_ecef.magnitude();
    double mach = 0.0;
    if (m_state.atmosphere.speed_of_sound_m_s > 1.0) {
        mach = speed / m_state.atmosphere.speed_of_sound_m_s;
    }

    double q = 0.5 * m_state.atmosphere.density_kg_m3 * speed * speed;
    m_state.dynamic_pressure_kPa = q / 1000.0;

    if (m_state.dynamic_pressure_kPa > m_max_q_kPa) {
        m_max_q_kPa = m_state.dynamic_pressure_kPa;
    } else if (m_state.dynamic_pressure_kPa < m_max_q_kPa * 0.5 && m_max_q_kPa > 1.0) {
        m_max_q_passed = true;
    }

    double drag = q * reference_area_m2 * cd;
    double cl_alpha = 2.0 * M_PI * aoa;
    double lift = q * reference_area_m2 * cl_alpha;
    double side = q * reference_area_m2 * 0.1 * sideslip;

    Vector3 body_x_axis = attitude.rotate(Vector3{1.0, 0.0, 0.0});
    Vector3 body_y_axis = attitude.rotate(Vector3{0.0, 1.0, 0.0});
    Vector3 body_z_axis = attitude.rotate(Vector3{0.0, 0.0, 1.0});

    m_state.aero_forces.drag_body_N = { -drag, 0.0, 0.0 };
    m_state.aero_forces.lift_body_N = { 0.0, 0.0, lift };
    m_state.aero_forces.side_force_body_N = { 0.0, side, 0.0 };
    m_state.aero_forces.dynamic_pressure_Pa = q;
    m_state.aero_forces.mach_number = mach;
    m_state.aero_forces.angle_of_attack_rad = aoa;
    m_state.aero_forces.sideslip_rad = sideslip;
}

AtmosphereData Environment::get_atmosphere(double altitude_m) const {
    AtmosphereData atm;

    if (altitude_m < 0.0) altitude_m = 0.0;
    if (altitude_m > 84852.0) {
        atm.temperature_K = 186.95;
        atm.pressure_Pa = 0.0;
        atm.density_kg_m3 = 1e-12;
        atm.speed_of_sound_m_s = std::sqrt(1.4 * R_GAS * atm.temperature_K);
        atm.dynamic_viscosity = 1.789e-5;
        return atm;
    }

    int layer = 0;
    for (int i = 0; i < ATMOSPHERE_LAYERS - 1; i++) {
        if (altitude_m >= ALTITUDE_TABLE[i] && altitude_m < ALTITUDE_TABLE[i + 1]) {
            layer = i;
            break;
        }
    }

    double h_base = ALTITUDE_TABLE[layer];
    double T_base = TEMP_TABLE[layer];
    double p_base = PRESSURE_TABLE[layer];
    double lapse = LAPSE_RATE_TABLE[layer];
    double dh = altitude_m - h_base;

    if (std::abs(lapse) < 1e-9) {
        atm.temperature_K = T_base;
        atm.pressure_Pa = p_base * std::exp(-G0 * dh / (R_GAS * T_base));
    } else {
        atm.temperature_K = T_base + lapse * dh;
        double exp = -G0 / (R_GAS * lapse);
        atm.pressure_Pa = p_base * std::pow(atm.temperature_K / T_base, exp);
    }

    atm.density_kg_m3 = atm.pressure_Pa / (R_GAS * atm.temperature_K);
    atm.speed_of_sound_m_s = std::sqrt(1.4 * R_GAS * atm.temperature_K);
    atm.dynamic_viscosity = 1.458e-6 * std::pow(atm.temperature_K, 1.5) / (atm.temperature_K + 110.4);

    return atm;
}

GravityData Environment::get_gravity(const Vector3& position_ecef) const {
    GravityData grav;
    double x = position_ecef.x;
    double y = position_ecef.y;
    double z = position_ecef.z;
    double r = position_ecef.magnitude();

    if (r < 1.0) {
        grav.acceleration = Vector3{0.0, 0.0, -G0};
        grav.magnitude_m_s2 = G0;
        return grav;
    }

    double r2 = r * r;
    double r3 = r2 * r;
    double r5 = r3 * r2;
    double r7 = r5 * r2;

    double z_over_r = z / r;
    double z_over_r2 = z_over_r * z_over_r;

    double mu_over_r3 = EARTH_MU / r3;
    double j2_term = 1.0 - 1.5 * EARTH_J2 * (EARTH_RADIUS_EQUATORIAL_M * EARTH_RADIUS_EQUATORIAL_M / r2) * (5.0 * z_over_r2 - 1.0);

    double ax = -mu_over_r3 * x * j2_term;
    double ay = -mu_over_r3 * y * j2_term;
    double az = -mu_over_r3 * z * (1.0 - 1.5 * EARTH_J2 * (EARTH_RADIUS_EQUATORIAL_M * EARTH_RADIUS_EQUATORIAL_M / r2) * (5.0 * z_over_r2 - 3.0));

    grav.acceleration = Vector3{ax, ay, az};
    grav.magnitude_m_s2 = grav.acceleration.magnitude();

    return grav;
}

AerodynamicForces Environment::compute_aerodynamics(const Vector3& velocity_body_m_s,
                                                    const Quaternion& attitude,
                                                    double altitude_m,
                                                    double reference_area,
                                                    double cd) const {
    AerodynamicForces aero;

    AtmosphereData atm = get_atmosphere(altitude_m);
    double speed = velocity_body_m_s.magnitude();
    double mach = 0.0;
    if (atm.speed_of_sound_m_s > 1.0) {
        mach = speed / atm.speed_of_sound_m_s;
    }
    double q = 0.5 * atm.density_kg_m3 * speed * speed;

    double aoa = 0.0;
    double sideslip = 0.0;
    if (speed > 1.0) {
        aoa = std::atan2(-velocity_body_m_s.z, velocity_body_m_s.x);
        sideslip = std::asin(std::max(-1.0, std::min(1.0, velocity_body_m_s.y / speed)));
    }

    double drag = q * reference_area * cd;
    double lift = q * reference_area * 2.0 * M_PI * aoa;
    double side = q * reference_area * 0.1 * sideslip;

    aero.drag_body_N = Vector3{-drag, 0.0, 0.0};
    aero.lift_body_N = Vector3{0.0, 0.0, lift};
    aero.side_force_body_N = Vector3{0.0, side, 0.0};
    aero.dynamic_pressure_Pa = q;
    aero.mach_number = mach;
    aero.angle_of_attack_rad = aoa;
    aero.sideslip_rad = sideslip;

    return aero;
}

Vector3 Environment::get_wind_vector_ecef(double altitude_m) const {
    double wind_speed = m_wind.speed_m_s + m_wind.gust_speed_m_s;
    if (altitude_m > 20000.0) {
        wind_speed *= std::exp(-(altitude_m - 20000.0) / 10000.0);
    }
    if (altitude_m > 50000.0) {
        wind_speed = 0.0;
    }
    double wind_dir_rad = m_wind.direction_deg * M_PI / 180.0;
    return Vector3{wind_speed * std::cos(wind_dir_rad),
                   wind_speed * std::sin(wind_dir_rad),
                   0.0};
}

} // namespace LunarAscent