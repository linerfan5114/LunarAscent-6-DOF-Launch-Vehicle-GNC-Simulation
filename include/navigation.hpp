#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "vehicle.hpp"
#include "environment.hpp"
#include <array>
#include <cmath>
#include <random>

namespace LunarAscent {

struct IMUConfig {
    double gyro_noise_density_rad_per_s_per_sqrt_Hz = 0.0001;
    double gyro_bias_stability_rad_per_s = 0.00001;
    double gyro_random_walk_rad_per_s_sqrt_s = 0.000005;
    double accel_noise_density_m_per_s2_per_sqrt_Hz = 0.001;
    double accel_bias_stability_m_per_s2 = 0.0001;
    double accel_random_walk_m_per_s2_sqrt_s = 0.00005;
    double update_rate_hz = 100.0;
    double scale_factor_error_ppm = 100.0;
    double misalignment_rad = 0.0001;
};

struct GPSConfig {
    double position_error_std_m = 5.0;
    double velocity_error_std_m_per_s = 0.1;
    double update_rate_hz = 10.0;
    double hdop = 1.0;
    double vdop = 1.5;
    int min_satellites = 4;
};

struct StarTrackerConfig {
    double attitude_error_std_rad = 0.00005;
    double update_rate_hz = 5.0;
    double sun_exclusion_angle_deg = 30.0;
    double earth_exclusion_angle_deg = 25.0;
};

struct IMU Measurements {
    Vector3 angular_velocity;
    Vector3 acceleration;
    double temperature_K = 293.15;
    bool valid = true;
};

struct GPSMeasurement {
    Vector3 position_ecef;
    Vector3 velocity_ecef;
    double hdop = 1.0;
    double vdop = 1.5;
    int satellites = 8;
    bool fix_valid = true;
    double time_s = 0.0;
};

struct StarTrackerMeasurement {
    Quaternion attitude;
    double confidence = 1.0;
    bool valid = true;
    double time_s = 0.0;
};

struct NavigationState {
    Vector3 estimated_position;
    Vector3 estimated_velocity;
    Quaternion estimated_attitude;
    Vector3 estimated_angular_velocity;
    Vector3 estimated_acceleration;
    Vector3 gyro_bias;
    Vector3 accel_bias;
    std::array<std::array<double, 15>, 15> covariance;
    bool kalman_initialized = false;
    bool gps_aiding = false;
    double last_update_time_s = 0.0;
};

struct LLA {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
};

class Navigation {
public:
    Navigation();
    
    void initialize(const Vector3& initial_position, const Vector3& initial_velocity,
                    const Quaternion& initial_attitude);
    void update(double dt_s, const IMU Measurements& imu_data);
    void apply_gps_update(const GPSMeasurement& gps);
    void apply_star_tracker_update(const StarTrackerMeasurement& star);
    
    Vector3 get_position() const { return m_state.estimated_position; }
    Vector3 get_velocity() const { return m_state.estimated_velocity; }
    Quaternion get_attitude() const { return m_state.estimated_attitude; }
    Vector3 get_angular_velocity() const { return m_state.estimated_angular_velocity; }
    Vector3 get_gyro_bias() const { return m_state.gyro_bias; }
    Vector3 get_accel_bias() const { return m_state.accel_bias; }
    NavigationState get_state() const { return m_state; }
    
    static LLA ecef_to_lla(const Vector3& ecef);
    static Vector3 lla_to_ecef(const LLA& lla);
    static Vector3 ned_to_ecef(const Vector3& ned, const LLA& origin);
    static Vector3 ecef_to_ned(const Vector3& ecef, const LLA& origin);
    static Vector3 body_to_ned(const Vector3& body, const Quaternion& attitude);
    static double calculate_azimuth(const LLA& from, const LLA& to);
    static double calculate_distance(const LLA& from, const LLA& to);
    
    bool is_gps_aiding() const { return m_state.gps_aiding; }
    double get_position_error_estimate() const;
    double get_velocity_error_estimate() const;
    
    void set_imu_config(const IMUConfig& config) { m_imu_config = config; }
    void set_gps_config(const GPSConfig& config) { m_gps_config = config; }
    
private:
    NavigationState m_state;
    IMUConfig m_imu_config;
    GPSConfig m_gps_config;
    StarTrackerConfig m_star_config;
    
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normal_dist;
    
    IMU Measurements add_imu_noise(const IMU Measurements& true_data);
    GPSMeasurement add_gps_noise(const GPSMeasurement& true_data);
    StarTrackerMeasurement add_star_noise(const StarTrackerMeasurement& true_data);
    
    void propagate_state(double dt, const IMU Measurements& imu);
    void propagate_covariance(double dt, const IMU Measurements& imu);
    void kalman_update_position(const Vector3& measured_pos);
    void kalman_update_velocity(const Vector3& measured_vel);
    void kalman_update_attitude(const Quaternion& measured_att);
    
    static constexpr double EARTH_RADIUS_EQUATORIAL = 6378137.0;
    static constexpr double EARTH_ECCENTRICITY_SQ = 0.00669437999014;
    static constexpr double EARTH_ROTATION_RATE = 7.2921150e-5;
};

} // namespace LunarAscent

#endif // NAVIGATION_HPP