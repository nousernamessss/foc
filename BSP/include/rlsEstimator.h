#ifndef IDENTIFIER_H
#define IDENTIFIER_H

#include "arm_math.h"

#define LAMBDA_RL (1.0f)      // R, L 辨识的遗忘因子
#define LAMBDA_PSI (1.0f)     // 磁链辨识的遗忘因子
#define PSI_UPDATE_RATE (10)  // 磁链辨识更新频率（每10次调用更新一次）

// --- 稳态判断阈值 (单位: rad/s) ---
#define STEADY_STATE_THRESHOLD_START (20.0f * PI / 30.0f)
#define STEADY_STATE_THRESHOLD_STOP  (100.0f * PI / 30.0f)
#define STEADY_STATE_COUNT_TARGET (10) // 需要连续10次满足条件才启动

typedef struct
{
    // --- R, L 辨识 (Step 1) 的状态变量 ---
    float32_t pLastRL_data[4];      // 2x2 协方差矩阵 P 的数据存储
    float32_t thetaLastRL_data[2];  // 2x1 参数矢量 [Rs; Ls] 的数据存储
    float32_t dataBufferRL[3][4];   // 3x4 数据缓冲区 [ud, id, iq, we]

    arm_matrix_instance_f32 pLastRL; // P 矩阵实例
    arm_matrix_instance_f32 thetaLastRL; // theta 矢量实例

    // --- 磁链辨识 (Step 2) 的状态变量 ---
    float32_t pLastPsi;             // 1x1 协方差 (标量)
    float32_t thetaLastPsi;         // 1x1 参数 [psi_f] (标量)
    float32_t dataBufferPsi[2][4];  // 2x4 数据缓冲区 [uq, id, iq, we]

    // --- 逻辑控制变量 ---
    int32_t rlsStepCounter;         // RLS 步数计数器
    int32_t steadyStateCounter;     // 稳态计数器
    uint8_t isIdentificationEnabled; // 辨识使能标志 (0: false, 1: true)
} RlsIdentifierState;

typedef struct
{
    float32_t frequency;
    float32_t amplitude;
    float32_t offset;
    float32_t phase;
    float32_t phaseIncrement; // 使用小驼峰命名法
} TriWaveGenerator;

extern RlsIdentifierState rlsIdentifier;
extern TriWaveGenerator triWaveGenerator;
/**
 * @brief 初始化RLS辨识器的状态变量
 * @param state 指向RLS状态结构体的指针
 */
void identifyParamsInit(RlsIdentifierState* state);

/**
 * @brief 执行一步PMSM三参数在线辨识
 * @param state 指向RLS状态结构体的指针
 * @param ud, uq 当前d-q轴电压
 * @param id, iq 当前d-q轴电流
 * @param we 当前电机电角速度 (rad/s)
 * @param we_ref 当前电机电角速度参考值 (rad/s)
 * @param Ts 采样周期 (s)
 * @param Rs_est, Ls_est, psi_f_est 指向输出参数的指针
 */
void identifyAllParams(RlsIdentifierState* state,
                         float32_t ud, float32_t uq, float32_t id, float32_t iq,
                         float32_t we, float32_t we_ref, float32_t Ts,
                         float32_t* Rs_est, float32_t* Ls_est, float32_t* psi_f_est);

#endif

void triWaveInit(TriWaveGenerator* waveGen, float32_t frequency, float32_t amplitude, float32_t offset, float32_t controlFreq);
float triWaveGenerate(TriWaveGenerator* waveGen);













