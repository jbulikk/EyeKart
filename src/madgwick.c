#include "madgwick.h"
#include <math.h>

#define MADGWICK_BETA 0.15f

void madgwick_update(MadgwickState *state,
                     float gx,
                     float gy,
                     float gz,
                     float ax,
                     float ay,
                     float az,
                     float mx,
                     float my,
                     float mz,
                     float dt) {
    float q0 = state->q0, q1 = state->q1, q2 = state->q2, q3 = state->q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float hx, hy, _2bx, _2bz;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Normalise accelerometer measurement
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
        return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = sqrtf(mx * mx + my * my + mz * mz);
    if (recipNorm < 1e-6f)
        return;
    recipNorm = 1.0f / recipNorm;
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Gradient descent algorithm corrective step
    s0 = -2.0f * (q2 * (2.0f * q1q3 - 2.0f * q0 * q2 - ax) + q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay)) +
         -_2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = 2.0f * (q3 * (2.0f * q1q3 - 2.0f * q0 * q2 - ax) + q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay)) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - 2.0f * _2bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -2.0f * (q0 * (2.0f * q1q3 - 2.0f * q0 * q2 - ax) + q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay)) +
         (-2.0f * _2bx * q1 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q0 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q1 - 2.0f * _2bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = 2.0f * (q1 * (2.0f * q1q3 - 2.0f * q0 * q2 - ax) + q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay)) +
         (-2.0f * _2bx * q0 + _2bz * q2) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q3 - 2.0f * _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 + _2bz * q3) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    gx -= MADGWICK_BETA * s1;
    gy -= MADGWICK_BETA * s2;
    gz -= MADGWICK_BETA * s3;

    // Integrate rate of change of quaternion
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * dt);
    q1 += (q0 * gx + q2 * gz - q3 * gy) * (0.5f * dt);
    q2 += (q0 * gy - q1 * gz + q3 * gx) * (0.5f * dt);
    q3 += (q0 * gz + q1 * gy - q2 * gx) * (0.5f * dt);

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    state->q0 = q0 * recipNorm;
    state->q1 = q1 * recipNorm;
    state->q2 = q2 * recipNorm;
    state->q3 = q3 * recipNorm;
}
