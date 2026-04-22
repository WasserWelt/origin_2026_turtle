/*****************************************************************************************************************************
 * @file: Chassis_Power_Limitor.c
 * @author: Shiki
 * @date: 2025.12.19
 * @brief: 起源哨兵2026赛季底盘功率限制模块源文件 - 全向轮版本
 *
 功率控制模块工作流程：计算底盘当前预测功率->判断是否超过功率上限->若超过则进行拉格朗日乘子法求解->根据求解结果调整电机目标电流
*****************************************************************************************************************************/
#include "Chassis_Power_Limitor.h"
#include "user_common_lib.h"
#include "detect_task.h"
#include "main.h"

/* 功率控制器参数 */
#define MAX_CMD_CURRENT 16384.0f
#define MIN_CMD_CURRENT -16384.0f
#define LAMBDA_INITIAL_UPEER_BOUND 0.05f
#define LAMBDA_INITIAL_LOWER_BOUND 0.0f
#define LAMBDA_UPPER_BOUND_MAX_ITER 10
#define LAMBDA_UPPER_BOUND_STEP 10
#define LAMBDA_MAX_ITER 30
#define POWER_TOLERANCE 0.8f

/* 全向轮权重配置 */
typedef struct {
    float weight_min;
    float weight_max;
    float current_gain;
    float speed_gain;
} wheel_weight_config_t;

static const wheel_weight_config_t WHEEL_WEIGHT_CONFIG = {
    1.0f, 10.0f,
    10.0f / 16384.0f,
    10.0f / 400.0f
};

/* 调试变量 */
float lambda;
int iter;
float power_iter;

/* 内部函数声明 */
static void Lagrange_Solve_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], float P_max);
static float Calculate_Initial_Power(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4]);
static void Calculate_All_Alpha_Coefficients(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4]);
static void Calculate_Alpha(power_limitor_t *power_limiter, float lambda, const chassis_wheel_motor_t wheel_motor[4]);
static float Calculate_Power_With_Alpha(power_limitor_t *power_limitor);
static void Allocate_Motor_Weight(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4]);

/**
 * @description: 功率控制接口
 * @param wheel_motor 轮电机结构体数组指针
 * @param P_max 功率上限
 */
void Chassis_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], float P_max)
{
    power_limiter->chassis_power_predicted = Calculate_Initial_Power(power_limiter, wheel_motor);

    if (power_limiter->chassis_power_predicted <= P_max)
        return;

    for (int i = 0; i < 4; i++)
    {
        if (wheel_motor[i].give_current == 0)
        {
            wheel_motor[i].give_current = (wheel_motor[i].speed_now > 0) ? 1.0f : -1.0f;
        }
    }

    Calculate_All_Alpha_Coefficients(power_limiter, wheel_motor);
    Lagrange_Solve_Power_Control(power_limiter, wheel_motor, P_max);

    for (int i = 0; i < 4; i++)
    {
        wheel_motor[i].give_current = (int16_t)(wheel_motor[i].give_current * power_limiter->wheel_motors.alpha[i]);
    }
}

/**
 * @description: 拉格朗日乘子法求解功率控制
 */
static void Lagrange_Solve_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], float P_max)
{
    float lambda_lower_bound = LAMBDA_INITIAL_LOWER_BOUND;
    float lambda_upper_bound = LAMBDA_INITIAL_UPEER_BOUND;

    Allocate_Motor_Weight(power_limiter, wheel_motor);

    for (int i = 0; i < LAMBDA_UPPER_BOUND_MAX_ITER; i++)
    {
        Calculate_Alpha(power_limiter, lambda_upper_bound, wheel_motor);
        float power = Calculate_Power_With_Alpha(power_limiter);
        if (power > P_max)
        {
            if (i == LAMBDA_UPPER_BOUND_MAX_ITER - 1)
            {
                power_limiter->chassis_power_processed = power;
                power_limiter->final_lambda = lambda_upper_bound;
                return;
            }
            else
            {
                lambda_lower_bound = lambda_upper_bound;
                lambda_upper_bound *= LAMBDA_UPPER_BOUND_STEP;
            }
        }
        else
        {
            break;
        }
    }

    for (iter = 0; iter < LAMBDA_MAX_ITER; iter++)
    {
        lambda = (lambda_lower_bound + lambda_upper_bound) / 2.0f;
        Calculate_Alpha(power_limiter, lambda, wheel_motor);
        power_iter = Calculate_Power_With_Alpha(power_limiter);

        if (power_iter < P_max && (P_max - power_iter) < POWER_TOLERANCE)
        {
            power_limiter->final_lambda = lambda;
            power_limiter->iter_num = iter + 1;
            power_limiter->chassis_power_processed = power_iter;
            return;
        }
        else if (power_iter > P_max)
        {
            lambda_lower_bound = lambda;
        }
        else
        {
            lambda_upper_bound = lambda;
        }
    }
    power_limiter->final_lambda = lambda;
    power_limiter->iter_num = iter + 1;
    power_limiter->chassis_power_processed = power_iter;
}

/**
 * @description: 计算底盘预测功率
 */
static float Calculate_Initial_Power(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4])
{
    float initial_total_power = 0;

    for (int i = 0; i < 4; i++)
    {
        float initial_wheel_power = power_limiter->wheel_motors.k_p * wheel_motor[i].give_current * wheel_motor[i].speed_now +
                                  power_limiter->wheel_motors.k_w * wheel_motor[i].speed_now * wheel_motor[i].speed_now +
                                  power_limiter->wheel_motors.k_t * wheel_motor[i].give_current * wheel_motor[i].give_current +
                                  power_limiter->wheel_motors.p_static;

        if (initial_wheel_power < 0 || toe_is_error(WHEEL_MOTOR_1_TOE + i))
            initial_wheel_power = 0;

        initial_total_power += initial_wheel_power;
    }
    return initial_total_power;
}

/**
 * @description: 计算α系数
 */
static void Calculate_All_Alpha_Coefficients(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.quadratic_coeff[i] = power_limiter->wheel_motors.k_t * wheel_motor[i].give_current * wheel_motor[i].give_current;
        power_limiter->wheel_motors.linear_coeff[i] = power_limiter->wheel_motors.k_p * wheel_motor[i].speed_now * wheel_motor[i].give_current;
        power_limiter->wheel_motors.constant[i] = power_limiter->wheel_motors.k_w * wheel_motor[i].speed_now * wheel_motor[i].speed_now + power_limiter->wheel_motors.p_static;
    }
}

/**
 * @description: 计算给定λ下轮电机的α系数
 */
static void Calculate_Alpha(power_limitor_t *power_limiter, float lambda, const chassis_wheel_motor_t wheel_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.alpha[i] = limit((2.0f * power_limiter->wheel_motors.weight[i] - lambda * power_limiter->wheel_motors.linear_coeff[i]) /
                                                         (2.0f * power_limiter->wheel_motors.weight[i] + 2.0f * power_limiter->wheel_motors.quadratic_coeff[i] * lambda),
                                                     MIN_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current),
                                                     MAX_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current));
    }
}

/**
 * @description: 计算给定λ下底盘总功率
 */
static float Calculate_Power_With_Alpha(power_limitor_t *power_limiter)
{
    float alpha_total_power = 0;

    for (int i = 0; i < 4; i++)
    {
        float alpha_wheel_power = power_limiter->wheel_motors.quadratic_coeff[i] * power_limiter->wheel_motors.alpha[i] * power_limiter->wheel_motors.alpha[i] +
                            power_limiter->wheel_motors.linear_coeff[i] * power_limiter->wheel_motors.alpha[i] +
                            power_limiter->wheel_motors.constant[i];

        if (alpha_wheel_power < 0 || toe_is_error(WHEEL_MOTOR_1_TOE + i))
            alpha_wheel_power = 0;

        alpha_total_power += alpha_wheel_power;
    }
    return alpha_total_power;
}

/**
 * @description: 分配轮电机权重
 */
static void Allocate_Motor_Weight(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4])
{
    const wheel_weight_config_t *cfg = &WHEEL_WEIGHT_CONFIG;

    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.weight[i] = limit(
            wheel_motor[i].give_current * cfg->current_gain + wheel_motor[i].speed_now * cfg->speed_gain,
            cfg->weight_min, cfg->weight_max);
    }
}
