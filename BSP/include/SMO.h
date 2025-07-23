#ifndef __SMO_PLL_ARM_H
#define __SMO_PLL_ARM_H

#include "main.h"
#include "arm_math.h"

typedef struct {
    float K_slide;
    float F_lpf_bemf;
    float G_lpf_bemf;
    float i_alpha_estimated;
    float i_beta_estimated;
    float E_alpha_filtered;
    float E_beta_filtered;

    float Rs;
    float Ls;
    float Ts;

    float speed_lpf_alpha;

    float theta_estimated;
    float omega_estimated;
    float omega_filtered;

    float pll_kp;
    float pll_ki;
    float pll_integral_state;
    float pll_out_max;
    float pll_out_min;
} Sensorless_Estimator_T;

extern Sensorless_Estimator_T SMO_Controller;

void Estimator_Init(Sensorless_Estimator_T *estimator, float k_slide, float bemf_lpf_hz, float pll_kp, float pll_ki, float rs, float ls, float ts, float speed_lpf_hz);
void Estimator_Step(Sensorless_Estimator_T *estimator, float U_alpha, float U_beta, float I_alpha_measured, float I_beta_measured);

#endif