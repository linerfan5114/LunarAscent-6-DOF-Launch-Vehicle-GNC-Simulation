#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <array>
#include <vector>
#include <string>
#include <cmath>

namespace LunarAscent {

struct Vector3 {
    double x = 0.0, y = 0.0, z = 0.0;
    
    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
    double magnitude_sq() const { return x*x + y*y + z*z; }
    Vector3 normalized() const { 
        double m = magnitude();
        return (m > 1e-12) ? *this / m : Vector3{};
    }
    double dot(const Vector3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vector3 cross(const Vector3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
};

struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    
    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    static Quaternion from_axis_angle(const Vector3& axis, double angle);
    static Quaternion from_euler(double roll, double pitch, double yaw);
    static Quaternion identity() { return {1.0, 0.0, 0.0, 0.0}; }
    
    Quaternion operator*(const Quaternion& q) const;
    Vector3 rotate(const Vector3& v) const;
    Quaternion conjugate() const { return {w, -x, -y, -z}; }
    Quaternion normalized() const;
    double magnitude() const { return std::sqrt(w*w + x*x + y*y + z*z); }
    
    std::array<double, 3> to_euler() const;
    std::array<std::array<double, 3>, 3> to_rotation_matrix() const;
};

struct Aerodynamics {
    double reference_area = 10.52;
    double drag_coeff_cd = 0.5;
    double lift_coeff_cl = 0.1;
    double mach_table[10] = {0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 5.0, 10.0};
    double cd_table[10] = {0.3, 0.3, 0.35, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.18};
    
    double get_cd_at_mach(double mach) const;
    double get_drag_force(double dynamic_pressure, double mach) const;
    double get_lift_force(double dynamic_pressure, double alpha_rad) const;
};

struct StageConfig {
    double dry_mass_kg;
    double fuel_mass_kg;
    double thrust_vacuum_N;
    double thrust_sea_level_N;
    double isp_vacuum_s;
    double isp_sea_level_s;
    double burn_time_s;
    double gimbal_range_deg;
    double gimbal_rate_deg_per_s;
};

struct VehicleState {
    Vector3 position;
    Vector3 velocity;
    Quaternion attitude;
    Vector3 angular_velocity;
    double mass_kg;
    double mission_time_s;
    int current_stage;
    bool stage_separated[3];
};

struct VehicleConfig {
    std::string name = "LunarAscent";
    int num_stages = 2;
    StageConfig stages[3];
    double payload_mass_kg = 5000.0;
    double fairing_mass_kg = 2000.0;
    double fairing_jettison_time_s = 180.0;
    double total_length_m = 70.0;
    double diameter_m = 3.7;
    Aerodynamics aero;
    double rcs_thrust_N = 500.0;
    double rcs_isp_s = 240.0;
};

class Vehicle {
public:
    Vehicle(const VehicleConfig& config);
    
    void update(double dt, const Vector3& force_body_N, const Vector3& torque_body_Nm);
    void apply_engine_gimbal(double pitch_rad, double yaw_rad);
    void separate_stage();
    void jettison_fairing();
    
    Vector3 get_position_ecef() const { return m_state.position; }
    Vector3 get_velocity_ecef() const { return m_state.velocity; }
    Vector3 get_velocity_body() const;
    Vector3 get_acceleration_body() const;
    Quaternion get_attitude() const { return m_state.attitude; }
    Vector3 get_angular_velocity() const { return m_state.angular_velocity; }
    double get_mass() const { return m_state.mass_kg; }
    double get_altitude() const;
    double get_mach_number(double speed_of_sound) const;
    double get_dynamic_pressure(double air_density) const;
    double get_angle_of_attack() const;
    double get_sideslip_angle() const;
    int get_current_stage() const { return m_state.current_stage; }
    const StageConfig& get_stage_config() const;
    bool has_fairing() const { return !m_fairing_jettisoned; }
    bool is_stage_separated(int stage) const { return m_state.stage_separated[stage]; }
    VehicleState get_state() const { return m_state; }
    VehicleConfig get_config() const { return m_config; }
    std::array<std::array<double, 3>, 3> get_inertia_tensor() const;
    double get_mass() { return m_state.mass_kg; }
    
private:
    VehicleConfig m_config;
    VehicleState m_state;
    bool m_fairing_jettisoned;
    double m_engine_gimbal_pitch;
    double m_engine_gimbal_yaw;
    
    Quaternion integrate_attitude(const Vector3& omega, double dt);
    void update_mass_properties();
};

} // namespace LunarAscent

#endif // VEHICLE_HPP