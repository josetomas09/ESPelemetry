#include <stdio.h>
#include <math.h>
#include "sensor_fusion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x) * (float)M_PI / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / (float)M_PI)


// State & Measurement
EEKF_DECL_MAT(state_x, 3, 1);                   // [roll, pitch, yaw] in radians
EEKF_DECL_MAT(measurement_z, 2, 1);             // [roll, pitch] in radians
EEKF_DECL_MAT(gyro_input, 3, 1);                // [roll_rate, pitch_rate, yaw_rate] in radians/sec

EEKF_DECL_MAT(process_noise_Q, 3, 3);
EEKF_DECL_MAT(measurement_noise_R, 2, 2);
EEKF_DECL_MAT(covariance_P, 3, 3);

eekf_context sensor_fusion_ctx;


static float accel_to_roll_rad(float ay, float az) {
    return atan2f(ay, az);
}

static float accel_to_pitch_rad(float ax, float ay, float az) {
    return atan2f(-ax, sqrtf(ay * ay + az * az));
}


eekf_return transition_f(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData) {

    if (xp == NULL || Jf == NULL || x == NULL || u == NULL || userData == NULL) {
        printf("arg\n");
        return eEekfReturnComputationFailed;
    }
    float dt = *((float*) userData);

    for (uint8_t i = 0; i < 3; i++) {
        xp->elements[i] = x->elements[i] + u->elements[i] * dt;
    }

    /* Jacobiano identidad (modelo de transición lineal) */
    *EEKF_MAT_EL(*Jf, 0, 0) = 1.0;  *EEKF_MAT_EL(*Jf, 0, 1) = 0.0;  *EEKF_MAT_EL(*Jf, 0, 2) = 0.0;
    *EEKF_MAT_EL(*Jf, 1, 0) = 0.0;  *EEKF_MAT_EL(*Jf, 1, 1) = 1.0;  *EEKF_MAT_EL(*Jf, 1, 2) = 0.0;
    *EEKF_MAT_EL(*Jf, 2, 0) = 0.0;  *EEKF_MAT_EL(*Jf, 2, 1) = 0.0;  *EEKF_MAT_EL(*Jf, 2, 2) = 1.0;

    return eEekfReturnOk;
}

eekf_return measurement_h(eekf_mat* zp, eekf_mat* jh, eekf_mat const *x, void* userData) {
    if (zp == NULL || jh == NULL || x == NULL) {
        return eEekfReturnComputationFailed;
    }
    (void) userData;

    zp->elements[0] = x->elements[0];   // Roll
    zp->elements[1] = x->elements[1];   // Pitch

    *EEKF_MAT_EL(*jh, 0, 0) = 1.0;  *EEKF_MAT_EL(*jh, 0, 1) = 0.0; *EEKF_MAT_EL(*jh, 0, 2) = 0.0;
    *EEKF_MAT_EL(*jh, 1, 0) = 0.0;  *EEKF_MAT_EL(*jh, 1, 1) = 1.0; *EEKF_MAT_EL(*jh, 1, 2) = 0.0;

    return eEekfReturnOk;
}


void sensor_fusion_init(void) {

    *EEKF_MAT_EL(state_x, 0, 0) = 0.0;
    *EEKF_MAT_EL(state_x, 1, 0) = 0.0;
    *EEKF_MAT_EL(state_x, 2, 0) = 0.0;

    *EEKF_MAT_EL(covariance_P, 0, 0) = 1.0; *EEKF_MAT_EL(covariance_P, 0, 1) = 0.0; *EEKF_MAT_EL(covariance_P, 0, 2) = 0.0;
    *EEKF_MAT_EL(covariance_P, 1, 0) = 0.0; *EEKF_MAT_EL(covariance_P, 1, 1) = 1.0; *EEKF_MAT_EL(covariance_P, 1, 2) = 0.0;
    *EEKF_MAT_EL(covariance_P, 2, 0) = 0.0; *EEKF_MAT_EL(covariance_P, 2, 1) = 0.0; *EEKF_MAT_EL(covariance_P, 2, 2) = 1.0;

    *EEKF_MAT_EL(process_noise_Q, 0, 0) = 0.01; *EEKF_MAT_EL(process_noise_Q, 0, 1) = 0.0;  *EEKF_MAT_EL(process_noise_Q, 0, 2) = 0.0;
    *EEKF_MAT_EL(process_noise_Q, 1, 0) = 0.0;  *EEKF_MAT_EL(process_noise_Q, 1, 1) = 0.01; *EEKF_MAT_EL(process_noise_Q, 1, 2) = 0.0;
    *EEKF_MAT_EL(process_noise_Q, 2, 0) = 0.0;  *EEKF_MAT_EL(process_noise_Q, 2, 1) = 0.0;  *EEKF_MAT_EL(process_noise_Q, 2, 2) = 0.01;

    *EEKF_MAT_EL(measurement_noise_R, 0, 0) = 0.1; *EEKF_MAT_EL(measurement_noise_R, 0, 1) = 0.0;
    *EEKF_MAT_EL(measurement_noise_R, 1, 0) = 0.0; *EEKF_MAT_EL(measurement_noise_R, 1, 1) = 0.1;

    eekf_init(&sensor_fusion_ctx, &state_x, &covariance_P, transition_f, measurement_h, NULL);
}

void sensor_fusion_update(float dt, float acce_x, float acce_y, float acce_z, float gyro_x, float gyro_y, float gyro_z) {

    sensor_fusion_ctx.userData = &dt;

    /* Giro: °/s -> rad/s */
    *EEKF_MAT_EL(gyro_input, 0, 0) = DEG_TO_RAD(gyro_x);
    *EEKF_MAT_EL(gyro_input, 1, 0) = DEG_TO_RAD(gyro_y);
    *EEKF_MAT_EL(gyro_input, 2, 0) = DEG_TO_RAD(gyro_z);

    eekf_predict(&sensor_fusion_ctx, &gyro_input, &process_noise_Q);

    /* Acelerómetro -> ángulo (ya en radianes, atan2 los da así) */
    *EEKF_MAT_EL(measurement_z, 0, 0) = accel_to_roll_rad(acce_y, acce_z);
    *EEKF_MAT_EL(measurement_z, 1, 0) = accel_to_pitch_rad(acce_x, acce_y, acce_z);

    eekf_correct(&sensor_fusion_ctx, &measurement_z, &measurement_noise_R);
}

float get_roll(void) { 
    return RAD_TO_DEG((float) state_x.elements[0]); 
}
float get_pitch(void) {
    return RAD_TO_DEG((float) state_x.elements[1]); 
}
float get_yaw(void) {
    return RAD_TO_DEG((float) state_x.elements[2]); 
}