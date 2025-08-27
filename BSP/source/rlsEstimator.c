#include "rlsEstimator.h"
#include <string.h> // For memset
#include <math.h>   // For fabsf

RlsIdentifierState rlsIdentifier;
TriWaveGenerator triWaveGenerator;

void triWaveInit(TriWaveGenerator* waveGen, float32_t frequency, float32_t amplitude, float32_t offset, float32_t controlFreq)
{
    waveGen->frequency = frequency;
    waveGen->amplitude = amplitude;
    waveGen->offset = offset;
    waveGen->phase = 0.0f;
    // 计算每个控制周期需要增加的相位
    waveGen->phaseIncrement = waveGen->frequency / controlFreq;
}

float triWaveGenerate(TriWaveGenerator* waveGen)
{
    float32_t normalizedOutput = 0.0f;

    // 核心逻辑：将相位映射到 [-1, 1] 的三角波形
    if (waveGen->phase < 0.5f)
    {
        // 上升沿: y = 4x - 1
        normalizedOutput = 4.0f * waveGen->phase - 1.0f;
    }
    else
    {
        // 下降沿: y = -4x + 3
        normalizedOutput = -4.0f * waveGen->phase + 3.0f;
    }

    // 更新相位
    waveGen->phase += waveGen->phaseIncrement;

    // 相位绕回处理
    if (waveGen->phase >= 1.0f)
    {
        waveGen->phase -= 1.0f;
    }

    // 应用幅值和偏置，并返回最终结果
    return (normalizedOutput * waveGen->amplitude + waveGen->offset);
}

/**
 * @brief 初始化RLS辨识器的状态变量
 */
void identifyParamsInit(RlsIdentifierState* state)
{
    // 使用memset清零整个结构体，确保所有值为0
    memset(state, 0, sizeof(RlsIdentifierState));

    // --- R, L 辨识初始化 ---
    // 初始化P矩阵为 1e6 * I
    state->pLastRL_data[0] = 1e6f; state->pLastRL_data[1] = 0.0f;
    state->pLastRL_data[2] = 0.0f; state->pLastRL_data[3] = 1e6f;

    // 初始化theta矢量为 [0; 0]
    state->thetaLastRL_data[0] = 0.0f; // Rs
    state->thetaLastRL_data[1] = 0.0f; // Ls

    // 关联ARM矩阵实例与数据数组
    arm_mat_init_f32(&state->pLastRL, 2, 2, state->pLastRL_data);
    arm_mat_init_f32(&state->thetaLastRL, 2, 1, state->thetaLastRL_data);

    // --- 磁链辨识初始化 ---
    state->pLastPsi = 1e6f;
    state->thetaLastPsi = 0.0f; // psi_f
}

/**
 * @brief 执行一步PMSM三参数在线辨识
 */
void identifyAllParams(RlsIdentifierState* state,
                         float32_t ud, float32_t uq, float32_t id, float32_t iq,
                         float32_t we, float32_t we_ref, float32_t Ts,
                         float32_t* Rs_est, float32_t* Ls_est, float32_t* psi_f_est) {

    // --- 辨识启动与停止逻辑 ---
    float32_t speed_error;
    arm_sub_f32(&we_ref, &we, &speed_error, 1);
    speed_error = fabsf(speed_error);

    if (state->isIdentificationEnabled && (speed_error > STEADY_STATE_THRESHOLD_STOP)) {
        state->isIdentificationEnabled = 0;
        state->steadyStateCounter = 0;
    }

    if (!state->isIdentificationEnabled)
    {
        if (speed_error < STEADY_STATE_THRESHOLD_START)
        {
            state->steadyStateCounter++;
        }
        else
        {
            state->steadyStateCounter = 0;
        }

        if (state->steadyStateCounter >= STEADY_STATE_COUNT_TARGET) {
            state->isIdentificationEnabled = 1;
        }
    }

    // --- 主辨识逻辑 ---
    if (state->isIdentificationEnabled) {
        state->rlsStepCounter++;

        // --- Step 1: Rs 和 Ls 的辨识 (每次都执行) ---
        // 更新数据缓冲区
        memcpy(&state->dataBufferRL[0][0], &state->dataBufferRL[1][0], 2 * 4 * sizeof(float32_t));
        state->dataBufferRL[2][0] = ud; state->dataBufferRL[2][1] = id;
        state->dataBufferRL[2][2] = iq; state->dataBufferRL[2][3] = we;

        if (state->rlsStepCounter >= 3)
        {
            // 提取数据
            float32_t ud1 = state->dataBufferRL[0][0], id1 = state->dataBufferRL[0][1], iq1 = state->dataBufferRL[0][2], we1 = state->dataBufferRL[0][3];
            float32_t ud2 = state->dataBufferRL[1][0], id2 = state->dataBufferRL[1][1], iq2 = state->dataBufferRL[1][2], we2 = state->dataBufferRL[1][3];
            float32_t id3 = state->dataBufferRL[2][1];

            // RLS计算所需的临时矩阵和数据
            float32_t y_RL_data[2] = {ud1, ud2};
            float32_t H_RL_data[4] = {id1, ((id2 - id1)/Ts - we1*iq1),
                                      id2, ((id3 - id2)/Ts - we2*iq2)};

            arm_matrix_instance_f32 y_RL, H_RL;
            arm_mat_init_f32(&y_RL, 2, 1, y_RL_data);
            arm_mat_init_f32(&H_RL, 2, 2, H_RL_data);

            // 临时计算矩阵
            float32_t H_T_data[4], P_H_T_data[4], Den_data[4], Den_inv_data[4], K_RL_data[4];
            arm_matrix_instance_f32 H_T, P_H_T, Den, Den_inv, K_RL;
            arm_mat_init_f32(&H_T, 2, 2, H_T_data);
            arm_mat_init_f32(&P_H_T, 2, 2, P_H_T_data);
            arm_mat_init_f32(&Den, 2, 2, Den_data);
            arm_mat_init_f32(&Den_inv, 2, 2, Den_inv_data);
            arm_mat_init_f32(&K_RL, 2, 2, K_RL_data);

            // 1. K = P*H' * inv(lambda*I + H*P*H')
            arm_mat_trans_f32(&H_RL, &H_T);
            arm_mat_mult_f32(&state->pLastRL, &H_T, &P_H_T);
            arm_mat_mult_f32(&H_RL, &P_H_T, &Den);
            Den.pData[0] += LAMBDA_RL; Den.pData[3] += LAMBDA_RL; // Den = lambda*I + H*P*H'
            arm_mat_inverse_f32(&Den, &Den_inv);
            arm_mat_mult_f32(&P_H_T, &Den_inv, &K_RL);

            // 2. theta = theta_last + K * (y - H * theta_last)
            float32_t err_data[2], H_theta_data[2], K_err_data[2], theta_RL_data[2];
            arm_matrix_instance_f32 err, H_theta, K_err, theta_RL;
            arm_mat_init_f32(&err, 2, 1, err_data);
            arm_mat_init_f32(&H_theta, 2, 1, H_theta_data);
            arm_mat_init_f32(&K_err, 2, 1, K_err_data);
            arm_mat_init_f32(&theta_RL, 2, 1, theta_RL_data);

            arm_mat_mult_f32(&H_RL, &state->thetaLastRL, &H_theta);
            arm_mat_sub_f32(&y_RL, &H_theta, &err);
            arm_mat_mult_f32(&K_RL, &err, &K_err);
            arm_mat_add_f32(&state->thetaLastRL, &K_err, &theta_RL);

            // 3. P = (I - K*H) * P_last / lambda
            float32_t I_data[4] = {1,0,0,1};
            float32_t KH_data[4], I_KH_data[4], P_RL_data[4];
            arm_matrix_instance_f32 I, KH, I_KH, P_RL;
            arm_mat_init_f32(&I, 2, 2, I_data);
            arm_mat_init_f32(&KH, 2, 2, KH_data);
            arm_mat_init_f32(&I_KH, 2, 2, I_KH_data);
            arm_mat_init_f32(&P_RL, 2, 2, P_RL_data);

            arm_mat_mult_f32(&K_RL, &H_RL, &KH);
            arm_mat_sub_f32(&I, &KH, &I_KH);
            arm_mat_mult_f32(&I_KH, &state->pLastRL, &P_RL);
            arm_mat_scale_f32(&P_RL, 1.0f/LAMBDA_RL, &state->pLastRL); // Update pLastRL

            // Update thetaLastRL
            state->thetaLastRL.pData[0] = theta_RL.pData[0];
            state->thetaLastRL.pData[1] = theta_RL.pData[1];
        }

        // --- Step 2: 磁链 psi_f 的辨识 (降频执行) ---
        memcpy(&state->dataBufferPsi[0][0], &state->dataBufferPsi[1][0], 1 * 4 * sizeof(float32_t));
        state->dataBufferPsi[1][0] = uq; state->dataBufferPsi[1][1] = id;
        state->dataBufferPsi[1][2] = iq; state->dataBufferPsi[1][3] = we;

        if (state->rlsStepCounter >= 2 && (state->rlsStepCounter % PSI_UPDATE_RATE) == 0) {
            float32_t Rs_curr = state->thetaLastRL.pData[0];
            float32_t Ls_curr = state->thetaLastRL.pData[1];

            float32_t uq1 = state->dataBufferPsi[0][0], id1 = state->dataBufferPsi[0][1], iq1 = state->dataBufferPsi[0][2], we1 = state->dataBufferPsi[0][3];
            float32_t iq2 = state->dataBufferPsi[1][2];

            if (fabsf(we1) > 1e-3f) {
                float32_t y_psi = uq1 - we1*Ls_curr*id1 - Rs_curr*iq1 - (Ls_curr/Ts)*(iq2-iq1);
                float32_t H_psi = we1;

                float32_t K_psi = state->pLastPsi * H_psi / (LAMBDA_PSI + H_psi * state->pLastPsi * H_psi);
                state->thetaLastPsi = state->thetaLastPsi + K_psi * (y_psi - H_psi * state->thetaLastPsi);
                state->pLastPsi = (1.0f - K_psi * H_psi) * state->pLastPsi / LAMBDA_PSI;
            }
        }
    }

    // 分配输出 (始终输出最新的有效辨识值)
    *Rs_est = state->thetaLastRL.pData[0];
    *Ls_est = state->thetaLastRL.pData[1];
    *psi_f_est = state->thetaLastPsi;
}
