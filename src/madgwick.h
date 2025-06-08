#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdint.h>

typedef struct {
    float q0, q1, q2, q3;
} MadgwickState;

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
                     float dt);

#endif // MADGWICK_H
