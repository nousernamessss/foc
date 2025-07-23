#include "SMO.h"

Sensorless_Estimator_T SMO_Controller;
float saturation(float val, float phi)
{
    if (phi <= 0.0f) {
        // 如果phi无效，则退化为符号函数
        if (val > 0.0f) return 1.0f;
        if (val < 0.0f) return -1.0f;
        return 0.0f;
    }

    if (val > phi) {
        return 1.0f;
    } else if (val < -phi) {
        return -1.0f;
    } else {
        return val / phi;
    }
}

void Estimator_Init(Sensorless_Estimator_T *estimator, float k_slide, float bemf_lpf_hz, float pll_kp, float pll_ki, float rs, float ls, float ts, float speed_lpf_hz)
{
    memset(estimator, 0, sizeof(Sensorless_Estimator_T));

    estimator->K_slide = k_slide;
    estimator->F_lpf_bemf = 2.0f * PI * bemf_lpf_hz;
    estimator->G_lpf_bemf = 2.0f * PI * bemf_lpf_hz;
    estimator->speed_lpf_alpha = 2.0f * PI * speed_lpf_hz;

    estimator->Rs = rs;
    estimator->Ls = ls;
    estimator->Ts = ts;

    estimator->pll_kp = pll_kp;
    estimator->pll_ki = pll_ki * estimator->Ts;
    estimator->pll_integral_state = 0.0f;
    estimator->pll_out_max = 2600.f;
    estimator->pll_out_min = -2600.f;
}

void Estimator_Step(Sensorless_Estimator_T *estimator, float U_alpha, float U_beta, float I_alpha_measured, float I_beta_measured)
{
    float i_err_alpha = estimator->i_alpha_estimated - I_alpha_measured;
    float i_err_beta  = estimator->i_beta_estimated - I_beta_measured;

    float Z_alpha = estimator->K_slide * saturation(i_err_alpha, 2.f);
    float Z_beta  = estimator->K_slide * saturation(i_err_beta, 2.f);

    estimator->E_alpha_filtered += estimator->F_lpf_bemf * (Z_alpha - estimator->E_alpha_filtered) * estimator->Ts;
    estimator->E_beta_filtered  += estimator->G_lpf_bemf * (Z_beta  - estimator->E_beta_filtered) * estimator->Ts;

    float di_alpha_dt = (1.0f / estimator->Ls) * (U_alpha - estimator->Rs * estimator->i_alpha_estimated - Z_alpha);
    float di_beta_dt  = (1.0f / estimator->Ls) * (U_beta  - estimator->Rs * estimator->i_beta_estimated  - Z_beta);

    estimator->i_alpha_estimated += di_alpha_dt * estimator->Ts;
    estimator->i_beta_estimated  += di_beta_dt * estimator->Ts;

    float cos_theta, sin_theta;
    arm_sin_cos_f32(estimator->theta_estimated * 180.0f / PI, &sin_theta, &cos_theta); // 注意：arm_sin_cos输入的是角度制
    float E_d_error = -estimator->E_alpha_filtered * cos_theta - estimator->E_beta_filtered * sin_theta;

    float pll_out;
    estimator->pll_integral_state += E_d_error;

    pll_out = estimator->pll_kp * E_d_error + estimator->Ts * estimator->pll_ki * estimator->pll_integral_state;

    if (pll_out > estimator->pll_out_max)
        pll_out = estimator->pll_out_max;
    else if (pll_out < estimator->pll_out_min)
        pll_out = estimator->pll_out_min;

    estimator->omega_estimated = pll_out;
    estimator->omega_filtered += estimator->speed_lpf_alpha * (estimator->omega_estimated - estimator->omega_filtered) * estimator->Ts;
    estimator->theta_estimated += estimator->omega_estimated * estimator->Ts;

    if (estimator->theta_estimated > 2.0f * PI) {
        estimator->theta_estimated -= 2.0f * PI;
    } else if (estimator->theta_estimated < 0.0f) {
        estimator->theta_estimated += 2.0f * PI;
    }
}