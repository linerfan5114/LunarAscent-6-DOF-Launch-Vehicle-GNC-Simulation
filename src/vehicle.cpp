#include "vehicle.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

Quaternion Quaternion::from_axis_angle(const Vector3& axis, double angle) {
    double half_angle = angle * 0.5;
    double s = std::sin(half_angle);
    Vector3 n = axis.normalized();
    return {std::cos(half_angle), n.x * s, n.y * s, n.z * s};
}

Quaternion Quaternion::from_euler(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return {
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    };
}

Vector3 Quaternion::rotate(const Vector3& v) const {
    Quaternion qv{0.0, v.x, v.y, v.z};
    Quaternion q_conj = conjugate();
    Quaternion result = (*this) * qv * q_conj;
    return {result.x, result.y, result.z};
}

Quaternion Quaternion::normalized() const {
    double mag = magnitude();
    if (mag < 1e-12) return {1.0, 0.0, 0.0, 0.0};
    return {w / mag, x / mag, y / mag, z / mag};
}

std::array<double, 3> Quaternion::to_euler() const {
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

std::array<std::array<double, 3>, 3> Quaternion::to_rotation_matrix() const {
    return {{
        {1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z + w*y)},
        {2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)},
        {2.0*(x*z - w*y), 2.0*(y*z + w*x), 1.0 - 2.0*(x*x + y*y)}
    }};
}

double Aerodynamics::get_cd_at_mach(double mach) const {
    if (mach <= mach_table[0]) return cd_table[0];
    if (mach >= mach_table[9]) return cd_table[9];
    for (int i = 0; i < 9; i++) {
        if (mach >= mach_table[i] && mach <= mach_table[i + 1]) {
            double t = (mach - mach_table[i]) / (mach_table[i + 1] - mach_table[i]);
            return cd_table[i] + t * (cd_table[i + 1] - cd_table[i]);
        }
    }
    return cd_table[4];
}

double Aerodynamics::get_drag_force(double dynamic_pressure, double mach) const {
    return dynamic_pressure * reference_area * get_cd_at_mach(mach);
}

double Aerodynamics::get_lift_force(double dynamic_pressure, double alpha_rad) const {
    double cl_alpha = 2.0 * M_PI * alpha_rad;
    return dynamic_pressure * reference_area * cl_alpha;
}

Vehicle::Vehicle(const VehicleConfig& config)
    : m_config(config)
    , m_fairing_jettisoned(false)
    , m_engine_gimbal_pitch(0.0)
    , m_engine_gimbal_yaw(0.0)
{
    m_state.position = Vector3{6371e3 + 0.001, 0.0, 0.0};
    m_state.velocity = Vector3{0.0, 0.0, 0.0};
    m_state.attitude = Quaternion::identity();
    m_state.angular_velocity = Vector3{0.0, 0.0, 0.0};
    m_state.mass_kg = config.payload_mass_kg + config.fairing_mass_kg;
    for (int i = 0; i < config.num_stages; i++) {
        m_state.mass_kg += config.stages[i].dry_mass_kg + config.stages[i].fuel_mass_kg;
    }
    m_state.mission_time_s = 0.0;
    m_state.current_stage = 0;
    for (int i = 0; i < 3; i++) {
        m_state.stage_separated[i] = false;
    }
}

void Vehicle::update(double dt, const Vector3& force_body_N,
                     const Vector3& torque_body_Nm) {
    auto R = m_state.attitude.to_rotation_matrix();
    Vector3 force_ecef{
        R[0][0] * force_body_N.x + R[0][1] * force_body_N.y + R[0][2] * force_body_N.z,
        R[1][0] * force_body_N.x + R[1][1] * force_body_N.y + R[1][2] * force_body_N.z,
        R[2][0] * force_body_N.x + R[2][1] * force_body_N.y + R[2][2] * force_body_N.z
    };

    double mass = m_state.mass_kg;
    if (mass < m_config.payload_mass_kg) mass = m_config.payload_mass_kg;

    Vector3 acceleration = force_ecef / mass;
    m_state.velocity += acceleration * dt;
    m_state.position += m_state.velocity * dt;

    auto I = get_inertia_tensor();
    double det = I[0][0] * (I[1][1] * I[2][2] - I[1][2] * I[2][1])
               - I[0][1] * (I[1][0] * I[2][2] - I[1][2] * I[2][0])
               + I[0][2] * (I[1][0] * I[2][1] - I[1][1] * I[2][0]);
    if (std::abs(det) < 1e-6) det = 1e-6;

    std::array<std::array<double, 3>, 3> I_inv;
    double inv_det = 1.0 / det;
    I_inv[0][0] = (I[1][1] * I[2][2] - I[1][2] * I[2][1]) * inv_det;
    I_inv[0][1] = (I[0][2] * I[2][1] - I[0][1] * I[2][2]) * inv_det;
    I_inv[0][2] = (I[0][1] * I[1][2] - I[0][2] * I[1][1]) * inv_det;
    I_inv[1][0] = (I[1][2] * I[2][0] - I[1][0] * I[2][2]) * inv_det;
    I_inv[1][1] = (I[0][0] * I[2][2] - I[0][2] * I[2][0]) * inv_det;
    I_inv[1][2] = (I[0][2] * I[1][0] - I[0][0] * I[1][2]) * inv_det;
    I_inv[2][0] = (I[1][0] * I[2][1] - I[1][1] * I[2][0]) * inv_det;
    I_inv[2][1] = (I[0][1] * I[2][0] - I[0][0] * I[2][1]) * inv_det;
    I_inv[2][2] = (I[0][0] * I[1][1] - I[0][1] * I[1][0]) * inv_det;

    Vector3 omega = m_state.angular_velocity;
    Vector3 gyro_term{
        omega.y * omega.z * (I[1][1] - I[2][2]) + omega.x * omega.y * I[1][2] - omega.x * omega.z * I[1][2],
        omega.x * omega.z * (I[2][2] - I[0][0]) + omega.x * omega.y * I[0][2] - omega.y * omega.z * I[0][2],
        omega.x * omega.y * (I[0][0] - I[1][1]) + omega.x * omega.z * I[0][1] - omega.y * omega.z * I[0][1]
    };
    Vector3 net_torque{torque_body_Nm.x - gyro_term.x,
                       torque_body_Nm.y - gyro_term.y,
                       torque_body_Nm.z - gyro_term.z};

    Vector3 angular_acc{
        I_inv[0][0] * net_torque.x + I_inv[0][1] * net_torque.y + I_inv[0][2] * net_torque.z,
        I_inv[1][0] * net_torque.x + I_inv[1][1] * net_torque.y + I_inv[1][2] * net_torque.z,
        I_inv[2][0] * net_torque.x + I_inv[2][1] * net_torque.y + I_inv[2][2] * net_torque.z
    };

    m_state.angular_velocity += angular_acc * dt;
    m_state.attitude = integrate_attitude(m_state.angular_velocity, dt);
    m_state.mission_time_s += dt;
}

Quaternion Vehicle::integrate_attitude(const Vector3& omega, double dt) {
    double mag = omega.magnitude();
    if (mag < 1e-12) return m_state.attitude;
    Vector3 axis = omega / mag;
    double angle = mag * dt;
    Quaternion delta = Quaternion::from_axis_angle(axis, angle);
    return (delta * m_state.attitude).normalized();
}

void Vehicle::apply_engine_gimbal(double pitch_rad, double yaw_rad) {
    const auto& stage = m_config.stages[m_state.current_stage];
    double limit = stage.gimbal_range_deg * M_PI / 180.0;
    m_engine_gimbal_pitch = std::max(-limit, std::min(limit, pitch_rad));
    m_engine_gimbal_yaw = std::max(-limit, std::min(limit, yaw_rad));
}

void Vehicle::separate_stage() {
    int current = m_state.current_stage;
    if (current < m_config.num_stages - 1) {
        m_state.stage_separated[current] = true;
        m_state.mass_kg -= (m_config.stages[current].dry_mass_kg +
                           m_config.stages[current].fuel_mass_kg);
        m_state.current_stage++;
    }
}

void Vehicle::jettison_fairing() {
    if (!m_fairing_jettisoned) {
        m_fairing_jettisoned = true;
        m_state.mass_kg -= m_config.fairing_mass_kg;
    }
}

Vector3 Vehicle::get_velocity_body() const {
    return m_state.attitude.conjugate().rotate(m_state.velocity);
}

Vector3 Vehicle::get_acceleration_body() const {
    return Vector3{0.0, 0.0, 0.0};
}

double Vehicle::get_altitude() const {
    return m_state.position.magnitude() - 6371e3;
}

double Vehicle::get_mach_number(double speed_of_sound) const {
    if (speed_of_sound < 1.0) return 0.0;
    return m_state.velocity.magnitude() / speed_of_sound;
}

double Vehicle::get_dynamic_pressure(double air_density) const {
    double speed = m_state.velocity.magnitude();
    return 0.5 * air_density * speed * speed;
}

double Vehicle::get_angle_of_attack() const {
    Vector3 vel_body = get_velocity_body();
    if (vel_body.magnitude() < 1.0) return 0.0;
    return std::atan2(-vel_body.z, vel_body.x);
}

double Vehicle::get_sideslip_angle() const {
    Vector3 vel_body = get_velocity_body();
    if (vel_body.magnitude() < 1.0) return 0.0;
    return std::asin(vel_body.y / vel_body.magnitude());
}

const StageConfig& Vehicle::get_stage_config() const {
    return m_config.stages[m_state.current_stage];
}

std::array<std::array<double, 3>, 3> Vehicle::get_inertia_tensor() const {
    double m = m_state.mass_kg;
    double L = m_config.total_length_m;
    double D = m_config.diameter_m;
    double Ixx = (1.0 / 12.0) * m * (3.0 * D*D/4.0 + L*L);
    double Iyy = Ixx;
    double Izz = (1.0 / 2.0) * m * D*D/4.0;
    return {{
        {Ixx, 0.0, 0.0},
        {0.0, Iyy, 0.0},
        {0.0, 0.0, Izz}
    }};
}

void Vehicle::update_mass_properties() {
}

} // namespace LunarAscent