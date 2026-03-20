#ifndef __QUADRATICINTERPOLATION_H
#define __QUADRATICINTERPOLATION_H

#include "main.h"

typedef struct
{
    double theta_now;
    double theta_set;
    double total_time;
    double sustained_time;
    double theta_out;
		unsigned char finish;
}quadraticInterpolation_t;

void quadraticInterpolation(quadraticInterpolation_t *quadraticInterpolation_param);
void quadraticInterpolation_clear_param(quadraticInterpolation_t *quadraticInterpolation_clear_param);
void quadraticInterpolation_clear_set(quadraticInterpolation_t *quadraticInterpolation_clear_set);

#endif