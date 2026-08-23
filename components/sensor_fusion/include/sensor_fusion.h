#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "mpu6050.h"
#include "eekf_mat.h"
#include "eekf.h"

extern eekf_mat state_x;
extern eekf_mat measurement_z;
extern eekf_mat gyro_input;
extern eekf_mat process_noise_Q;
extern eekf_mat measurement_noise_R;
extern eekf_mat covariance_P;
extern eekf_context sensor_fusion_ctx;

/**
 * @brief Converts accelerometer readings to roll angle in radians.
 *
 * @param [in] ay  Y-axis accelerometer reading (in g)
 * @param [in] az  Z-axis accelerometer reading (in g)
 * @return Roll angle in radians
 */
static float accel_to_roll_rad(float ay, float az);

/**
 * @brief Converts accelerometer readings to pitch angle in radians.
 *
 * @param [in] ax  X-axis accelerometer reading (in g)
 * @param [in] ay  Y-axis accelerometer reading (in g)
 * @param [in] az  Z-axis accelerometer reading (in g)
 * @return Pitch angle in radians
 */
static float accel_to_pitch_rad(float ax, float ay, float az);

/**
 * @brief Transition function for the EKF.
 *
 * @param [out] xp  state prediction (3x1)
 * @param [out] Jf  transition Jacobian (3x3)
 * @param [in]  x   current state (3x1)
 * @param [in]  u   control input, in rad/s (3x1)
 * @param [in]  userData  pointer to a float with dt (seconds)
 */
eekf_return transition_f(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData);

/**
 * @brief Measurement function for the EKF.
 *
 * @param [out] zp  predicted measurement (2x1, roll/pitch in rad)
 * @param [out] jh  measurement Jacobian (2x3)
 * @param [in]  x   current state (3x1)
 */
eekf_return measurement_h(eekf_mat* zp, eekf_mat* jh, eekf_mat const *x, void* userData);

/**
 * @brief Initializes filter context, state, and matrices.
 */
void sensor_fusion_init(void);

/**
 * @brief Updates the filter with a new MPU6050 sample.
 *
 * Input units are the raw units from the MPU6050 driver:
 * accelerometer in g, gyroscope in degrees/second. Conversion to
 * radians is done internally.
 *
 * @param [in] dt      elapsed time since the last update, in seconds
 * @param [in] acce_x  X-axis accelerometer, in g
 * @param [in] acce_y  Y-axis accelerometer, in g
 * @param [in] acce_z  Z-axis accelerometer, in g
 * @param [in] gyro_x  X-axis gyroscope (roll rate), in °/s
 * @param [in] gyro_y  Y-axis gyroscope (pitch rate), in °/s
 * @param [in] gyro_z  Z-axis gyroscope (yaw rate), in °/s
 */
void sensor_fusion_update(float dt, float acce_x, float acce_y, float acce_z, float gyro_x, float gyro_y, float gyro_z);

/** @return estimated roll in degrees */
float get_roll(void);

/** @return estimated pitch in degrees */
float get_pitch(void);

/** @return estimated yaw, in degrees. */
float get_yaw(void);

#endif // SENSOR_FUSION_H