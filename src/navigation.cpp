#include "navigation.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LunarAscent {

Navigation::Navigation()
    : m_rng(42)
    , m_normal_dist(0.0, 1.0)
{
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            m_state.covariance[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    m_state.estimated_position = Vector3{0, 0, 0};
    m_state.estimated_velocity = Vector3{0, 0, 0};
    m_state.estimated_attitude = Quaternion::identity();
    m_state.estimated_angular_velocity = Vector3{0, 0, 0};
    m_state.estimated_acceleration = Vector3{0, 0, 0};
    m_state.gyro_bias = Vector3{0, 0, 0};
    m_state.accel_bias = Vector3{0, 0, 0};
    m_state.kalman_initialized = false;
    m_state.gps_aiding = false;
    m_state.last_update_time_s = 0.0;
}

void Navigation::initialize(const Vector3& initial_position, const Vector3& initial_velocity,
                            const Quaternion& initial_attitude) {
    m_state.estimated_position = initial_position;
    m_state.estimated_velocity = initial_velocity;
    m_state.estimated_attitude = initial_attitude;
    m_state.kalman_initialized = true;
}

void Navigation::update(double dt_s, const IMU_Measurements& imu_data) {
    if (!m_state.kalman_initialized) return;
    if (dt_s <= 0.0) return;

    IMU_Measurements noisy_imu = add_imu_noise(imu_data);
    propagate_state(dt_s, noisy_imu);
    propagate_covariance(dt_s, noisy_imu);

    m_state.last_update_time_s += dt_s;
}

void Navigation::propagate_state(double dt, const IMU_Measurements& imu) {
    auto R = m_state.estimated_attitude.to_rotation_matrix();
    Vector3 accel_ecef{
        R[0][0] * imu.acceleration.x + R[0][1] * imu.acceleration.y + R[0][2] * imu.acceleration.z,
        R[1][0] * imu.acceleration.x + R[1][1] * imu.acceleration.y + R[1][2] * imu.acceleration.z,
        R[2][0] * imu.acceleration.x + R[2][1] * imu.acceleration.y + R[2][2] * imu.acceleration.z
    };

    double gravity = 9.80665;
    accel_ecef.z -= gravity;

    m_state.estimated_velocity += accel_ecef * dt;
    m_state.estimated_position += m_state.estimated_velocity * dt;

    Vector3 angular_velocity{
        imu.angular_velocity.x - m_state.gyro_bias.x,
        imu.angular_velocity.y - m_state.gyro_bias.y,
        imu.angular_velocity.z - m_state.gyro_bias.z
    };

    double mag = angular_velocity.magnitude();
    if (mag > 1e-12) {
        Vector3 axis = angular_velocity / mag;
        double angle = mag * dt;
        Quaternion delta = Quaternion::from_axis_angle(axis, angle);
        m_state.estimated_attitude = (delta * m_state.estimated_attitude).normalized();
    }

    m_state.estimated_angular_velocity = angular_velocity;
    m_state.estimated_acceleration = accel_ecef;
}

void Navigation::propagate_covariance(double dt, const IMU_Measurements& imu) {
    for (int i = 0; i < 3; i++) {
        m_state.covariance[i][i] += 0.1 * dt;
        m_state.covariance[i + 3][i + 3] += 0.01 * dt;
        m_state.covariance[i + 6][i + 6] += 0.001 * dt;
    }
}

void Navigation::apply_gps_update(const GPSMeasurement& gps) {
    if (!gps.fix_valid) return;

    GPSMeasurement noisy_gps = add_gps_noise(gps);
    kalman_update_position(noisy_gps.position_ecef);
    kalman_update_velocity(noisy_gps.velocity_ecef);
    m_state.gps_aiding = true;
}

void Navigation::apply_star_tracker_update(const StarTrackerMeasurement& star) {
    if (!star.valid) return;

    StarTrackerMeasurement noisy_star = add_star_noise(star);
    kalman_update_attitude(noisy_star.attitude);
}

void Navigation::kalman_update_position(const Vector3& measured_pos) {
    double K = 0.5;
    m_state.estimated_position.x += K * (measured_pos.x - m_state.estimated_position.x);
    m_state.estimated_position.y += K * (measured_pos.y - m_state.estimated_position.y);
    m_state.estimated_position.z += K * (measured_pos.z - m_state.estimated_position.z);

    for (int i = 0; i < 3; i++) {
        m_state.covariance[i][i] *= (1.0 - K);
    }
}

void Navigation::kalman_update_velocity(const Vector3& measured_vel) {
    double K = 0.3;
    m_state.estimated_velocity.x += K * (measured_vel.x - m_state.estimated_velocity.x);
    m_state.estimated_velocity.y += K * (measured_vel.y - m_state.estimated_velocity.y);
    m_state.estimated_velocity.z += K * (measured_vel.z - m_state.estimated_velocity.z);

    for (int i = 3; i < 6; i++) {
        m_state.covariance[i][i] *= (1.0 - K);
    }
}

void Navigation::kalman_update_attitude(const Quaternion& measured_att) {
    m_state.estimated_attitude = Quaternion{
        m_state.estimated_attitude.w * 0.7 + measured_att.w * 0.3,
        m_state.estimated_attitude.x * 0.7 + measured_att.x * 0.3,
        m_state.estimated_attitude.y * 0.7 + measured_att.y * 0.3,
        m_state.estimated_attitude.z * 0.7 + measured_att.z * 0.3
    }.normalized();
}

IMU_Measurements Navigation::add_imu_noise(const IMU_Measurements& true_data) {
    IMU_Measurements noisy = true_data;

    double gyro_noise = m_imu_config.gyro_noise_density_rad_per_s_per_sqrt_Hz * std::sqrt(m_imu_config.update_rate_hz);
    noisy.angular_velocity.x += m_normal_dist(m_rng) * gyro_noise + m_state.gyro_bias.x;
    noisy.angular_velocity.y += m_normal_dist(m_rng) * gyro_noise + m_state.gyro_bias.y;
    noisy.angular_velocity.z += m_normal_dist(m_rng) * gyro_noise + m_state.gyro_bias.z;

    double accel_noise = m_imu_config.accel_noise_density_m_per_s2_per_sqrt_Hz * std::sqrt(m_imu_config.update_rate_hz);
    noisy.acceleration.x += m_normal_dist(m_rng) * accel_noise + m_state.accel_bias.x;
    noisy.acceleration.y += m_normal_dist(m_rng) * accel_noise + m_state.accel_bias.y;
    noisy.acceleration.z += m_normal_dist(m_rng) * accel_noise + m_state.accel_bias.z;

    m_state.gyro_bias.x += m_normal_dist(m_rng) * m_imu_config.gyro_random_walk_rad_per_s_sqrt_s * 0.01;
    m_state.gyro_bias.y += m_normal_dist(m_rng) * m_imu_config.gyro_random_walk_rad_per_s_sqrt_s * 0.01;
    m_state.gyro_bias.z += m_normal_dist(m_rng) * m_imu_config.gyro_random_walk_rad_per_s_sqrt_s * 0.01;

    m_state.accel_bias.x += m_normal_dist(m_rng) * m_imu_config.accel_random_walk_m_per_s2_sqrt_s * 0.01;
    m_state.accel_bias.y += m_normal_dist(m_rng) * m_imu_config.accel_random_walk_m_per_s2_sqrt_s * 0.01;
    m_state.accel_bias.z += m_normal_dist(m_rng) * m_imu_config.accel_random_walk_m_per_s2_sqrt_s * 0.01;

    return noisy;
}

GPSMeasurement Navigation::add_gps_noise(const GPSMeasurement& true_data) {
    GPSMeasurement noisy = true_data;
    noisy.position_ecef.x += m_normal_dist(m_rng) * m_gps_config.position_error_std_m;
    noisy.position_ecef.y += m_normal_dist(m_rng) * m_gps_config.position_error_std_m;
    noisy.position_ecef.z += m_normal_dist(m_rng) * m_gps_config.position_error_std_m * 1.5;
    noisy.velocity_ecef.x += m_normal_dist(m_rng) * m_gps_config.velocity_error_std_m_per_s;
    noisy.velocity_ecef.y += m_normal_dist(m_rng) * m_gps_config.velocity_error_std_m_per_s;
    noisy.velocity_ecef.z += m_normal_dist(m_rng) * m_gps_config.velocity_error_std_m_per_s;
    return noisy;
}

StarTrackerMeasurement Navigation::add_star_noise(const StarTrackerMeasurement& true_data) {
    StarTrackerMeasurement noisy = true_data;
    double noise = m_normal_dist(m_rng) * m_star_config.attitude_error_std_rad;
    Vector3 rand_axis = Vector3{m_normal_dist(m_rng), m_normal_dist(m_rng), m_normal_dist(m_rng)}.normalized();
    Quaternion noise_quat = Quaternion::from_axis_angle(rand_axis, noise);
    noisy.attitude = (noise_quat * true_data.attitude).normalized();
    return noisy;
}

LLA Navigation::ecef_to_lla(const Vector3& ecef) {
    LLA lla;
    double x = ecef.x;
    double y = ecef.y;
    double z = ecef.z;

    double r = std::sqrt(x * x + y * y);
    double e2 = EARTH_ECCENTRICITY_SQ;

    double lat = std::atan2(z, r * (1.0 - e2));
    double lat_prev;
    for (int i = 0; i < 10; i++) {
        lat_prev = lat;
        double N = EARTH_RADIUS_EQUATORIAL / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
        double h = r / std::cos(lat) - N;
        lat = std::atan2(z, r * (1.0 - e2 * N / (N + h)));
        if (std::abs(lat - lat_prev) < 1e-12) break;
    }

    double N = EARTH_RADIUS_EQUATORIAL / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    lla.latitude_deg = lat * 180.0 / M_PI;
    lla.longitude_deg = std::atan2(y, x) * 180.0 / M_PI;
    lla.altitude_m = r / std::cos(lat) - N;

    return lla;
}

Vector3 Navigation::lla_to_ecef(const LLA& lla) {
    double lat = lla.latitude_deg * M_PI / 180.0;
    double lon = lla.longitude_deg * M_PI / 180.0;
    double alt = lla.altitude_m;

    double e2 = EARTH_ECCENTRICITY_SQ;
    double N = EARTH_RADIUS_EQUATORIAL / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));

    return Vector3{
        (N + alt) * std::cos(lat) * std::cos(lon),
        (N + alt) * std::cos(lat) * std::sin(lon),
        (N * (1.0 - e2) + alt) * std::sin(lat)
    };
}

Vector3 Navigation::ned_to_ecef(const Vector3& ned, const LLA& origin) {
    double lat = origin.latitude_deg * M_PI / 180.0;
    double lon = origin.longitude_deg * M_PI / 180.0;

    return Vector3{
        -std::sin(lat) * std::cos(lon) * ned.x - std::sin(lon) * ned.y + std::cos(lat) * std::cos(lon) * ned.z,
        -std::sin(lat) * std::sin(lon) * ned.x + std::cos(lon) * ned.y + std::cos(lat) * std::sin(lon) * ned.z,
        std::cos(lat) * ned.x + std::sin(lat) * ned.z
    };
}

Vector3 Navigation::ecef_to_ned(const Vector3& ecef, const LLA& origin) {
    double lat = origin.latitude_deg * M_PI / 180.0;
    double lon = origin.longitude_deg * M_PI / 180.0;

    Vector3 diff = ecef - lla_to_ecef(origin);
    return Vector3{
        -std::sin(lat) * std::cos(lon) * diff.x - std::sin(lat) * std::sin(lon) * diff.y + std::cos(lat) * diff.z,
        -std::sin(lon) * diff.x + std::cos(lon) * diff.y,
        std::cos(lat) * std::cos(lon) * diff.x + std::cos(lat) * std::sin(lon) * diff.y + std::sin(lat) * diff.z
    };
}

Vector3 Navigation::body_to_ned(const Vector3& body, const Quaternion& attitude) {
    Vector3 ecef_body = attitude.rotate(body);
    LLA origin = ecef_to_lla(Vector3{0, 0, 0});
    return ecef_to_ned(ecef_body, origin);
}

double Navigation::calculate_azimuth(const LLA& from, const LLA& to) {
    double lat1 = from.latitude_deg * M_PI / 180.0;
    double lon1 = from.longitude_deg * M_PI / 180.0;
    double lat2 = to.latitude_deg * M_PI / 180.0;
    double lon2 = to.longitude_deg * M_PI / 180.0;

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double azimuth = std::atan2(y, x);
    if (azimuth < 0) azimuth += 2.0 * M_PI;
    return azimuth;
}

double Navigation::calculate_distance(const LLA& from, const LLA& to) {
    Vector3 p1 = lla_to_ecef(from);
    Vector3 p2 = lla_to_ecef(to);
    return (p2 - p1).magnitude();
}

double Navigation::get_position_error_estimate() const {
    return std::sqrt(m_state.covariance[0][0] + m_state.covariance[1][1] + m_state.covariance[2][2]);
}

double Navigation::get_velocity_error_estimate() const {
    return std::sqrt(m_state.covariance[3][3] + m_state.covariance[4][4] + m_state.covariance[5][5]);
}

} // namespace LunarAscent