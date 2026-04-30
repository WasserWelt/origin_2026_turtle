# 底盘功率控制算法说明

## 1. 功率模型

单个电机的瞬时功率近似为：

$$
P = k_p \cdot I \cdot \omega + k_w \cdot \omega^2 + k_t \cdot I^2 + P_{\text{static}}
$$

| 表达式 | 物理含义 |
|--------|----------|
| $k_p \cdot I \cdot \omega$ | 电功率，与电流和角速成正比 |
| $k_w \cdot \omega^2$ | 风阻/粘性摩擦损耗 |
| $k_t \cdot I^2$ | 铜损($I^2R$），与电流平方成正比 |
| $P_{\text{static}}$ | 静态功耗（驱动、通信等固定损耗） |

各系数（$k_p$、$k_w$、$k_t$、$P_{\text{static}}$）通过实验标定得到，存储于 `power_limiter` 结构体中。

## 2. 优化问题建模

### 2.1 电流缩放系数

为每个电机引入缩放系数 $\alpha_i$，调整后电流为 $\alpha_i \cdot I_i$。

### 2.2 目标函数

$$
\min \sum_{i=1}^{8} w_i \cdot (\alpha_i - 1)^2
$$

在 $\alpha_i = 1$ 处取得最小值 0，$\alpha_i$ 越小惩罚越大。

### 2.3 约束条件

调整后总功率不超过上限 $P_{\max}$：

$$
\sum_{i=1}^{8} \left( k_t^{(i)} (I_i \cdot \alpha_i)^2 + k_p^{(i)} \cdot I_i \cdot \alpha_i \cdot \omega_i + k_w^{(i)} \cdot \omega_i^2 + P_{\text{static}}^{(i)} \right) \leq P_{\max}
$$

### 2.4 功率的二次表达式

每电机功率可写为 $P_i(\alpha_i) = a_i \alpha_i^2 + b_i \alpha_i + c_i$。

| 系数 | 含义 | 代码对应（见第 191-193 行） |
|------|------|---------------------------|
| $a_i$ (quadratic_coeff) | 二次项 | `kt * I^2` |
| $b_i$ (linear_coeff) | 一次项 | `kp * I * ω` |
| $c_i$ (constant) | 常数项 | `kw * ω^2 + Pstatic` |

```c:189:199:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
static void Calculate_All_Alpha_Coefficients(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.quadratic_coeff[i] = power_limiter->wheel_motors.k_t * wheel_motor[i].give_current * wheel_motor[i].give_current;
        power_limiter->wheel_motors.linear_coeff[i] = power_limiter->wheel_motors.k_p * wheel_motor[i].speed_now * wheel_motor[i].give_current;
        power_limiter->wheel_motors.constant[i] = power_limiter->wheel_motors.k_w * wheel_motor[i].speed_now * wheel_motor[i].speed_now + power_limiter->wheel_motors.p_static;

        power_limiter->steer_motors.quadratic_coeff[i] = power_limiter->steer_motors.k_t * steer_motor[i].give_current * steer_motor[i].give_current;
        power_limiter->steer_motors.linear_coeff[i] = power_limiter->steer_motors.k_p * steer_motor[i].speed_now * steer_motor[i].give_current;
        power_limiter->steer_motors.constant[i] = power_limiter->steer_motors.k_w * steer_motor[i].speed_now * steer_motor[i].speed_now + power_limiter->steer_motors.p_static;
    }
}
```

## 3. 拉格朗日乘子法求解

### 3.1 拉格朗日函数

$$
\mathcal{L}(\alpha_i, \lambda) = \sum_i w_i(\alpha_i - 1)^2 + \lambda \left( \sum_i (a_i \alpha_i^2 + b_i \alpha_i + c_i) - P_{\max} \right)
$$

### 3.2 KKT 条件

对 $\alpha_i$ 求偏导并令为 0：

$$
\frac{\partial \mathcal{L}}{\partial \alpha_i} = 2w_i(\alpha_i - 1) + \lambda(2a_i \alpha_i + b_i) = 0
$$

解得：

$$
\boxed{\alpha_i = \frac{2w_i - \lambda \cdot b_i}{2w_i + 2\lambda \cdot a_i}}
$$

对应代码（见第 208-222 行）：

```c:208:222:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
static void Calculate_Alpha(power_limiter_t *power_limiter, float lambda, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.alpha[i] = limit((2.0f * power_limiter->wheel_motors.weight[i] - lambda * power_limiter->wheel_motors.linear_coeff[i]) /
                                                         (2.0f * power_limiter->wheel_motors.weight[i] + 2.0f * power_limiter->wheel_motors.quadratic_coeff[i] * lambda),
                                                     MIN_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current),
                                                     MAX_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current));

        power_limiter->steer_motors.alpha[i] = limit((2.0f * power_limiter->steer_motors.weight[i] - lambda * power_limiter->steer_motors.linear_coeff[i]) /
                                                         (2.0f * power_limiter->steer_motors.weight[i] + 2.0f * power_limiter->steer_motors.quadratic_coeff[i] * lambda),
                                                     MIN_CMD_CURRENT / my_fabsf((float)steer_motor[i].give_current),
                                                     MAX_CMD_CURRENT / my_fabsf((float)steer_motor[i].give_current));
    }
}
```

### 3.3 $\lambda$ 的物理含义

- $\lambda = 0$：约束不活跃，$\alpha_i = 1$，电机全功率运行
- $\lambda \to +\infty$：约束主导，$\alpha_i \to -\frac{b_i}{2a_i}$，趋向功率最小化方向

### 3.4 $\alpha_i$ 的物理约束

$\alpha_i$ 被限制在 `[-16384, 16384] / |I_i|` 范围内，防止电流指令溢出。

## 4. 二分法搜索 $\lambda$

代码使用二分搜索找到满足功率约束的 $\lambda$。

### 4.1 寻找 $\lambda$ 上界

从 $\lambda = 0.05$ 开始（见第 17 行定义），若功率计算 > $P_{\max}$ 则乘 10 扩大，最多迭代 10 次：

```c:95:118:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
for (int i = 0; i < LAMBDA_UPPER_BOUND_MAX_ITER; i++)
{
    Calculate_Alpha(power_limiter, lambda_upper_bound, wheel_motor, steer_motor);
    float power = Calculate_Power_With_Alpha(power_limiter);
    if (power > P_max)
    {
        if (i == LAMBDA_UPPER_BOUND_MAX_ITER - 1)
        {
            // 说明功率限制过于苛刻，无法找到拉姆达（拉姆达取无穷大也无法满足条件），直接输出当前结果
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
```

### 4.2 二分搜索

在 $[\lambda_{\text{low}}, \lambda_{\text{up}}]$ 区间内二分，最多 30 次迭代，精度 0.8W（见第 19-22 行）：

```c:120:145:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
for (iter = 0; iter < LAMBDA_MAX_ITER; iter++)
{
    lambda = (lambda_lower_bound + lambda_upper_bound) / 2.0f;
    Calculate_Alpha(power_limiter, lambda, wheel_motor, steer_motor);
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
```

搜索目标：找到 $\lambda^*$ 使 $P(\lambda^*) \approx P_{\max}$。

## 5. 动态权重分析

权重 $w_i$ 根据电机当前状态动态计算（见第 287-308 行）。

### 5.1 轮电机权重

$$
\text{weight}_{\text{wheel}} = \text{limit}\left( (90^\circ - |\text{angle\_set} - \text{angle\_now}|) \times \text{gain}, \; \text{min}, \; \text{max} \right)
$$

舵机角度偏差越大，轮电机权重越高，优先保证轮毂出力。

### 5.2 舵电机权重

$$
\text{weight}_{\text{steer}} = \text{limit}\left( |I_{\text{give}}| \times \text{current\_gain} + |\omega| \times \text{speed\_gain}, \; \text{min}, \; \text{max} \right)
$$

所需转矩和转速越大，权重越高。

### 5.3 过颠簸卡舵特殊处理

检测到卡舵时，舵电机权重设为最大值 2 倍，轮电机权重设为最小值，优先保证脱困能力。

对应代码：

```c:287:309:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
static void Allocate_Motor_Weight(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4], weight_allocate_mode_t weight_allocate_mode, const uint8_t steer_stuck_status[4])
{
    const weight_config_t *cfg = (weight_allocate_mode == PASS_BUMPY_ALLOCATE) ?
                                 &WEIGHT_CONFIGS[PASS_BUMPY_ALLOCATE] :
                                 &WEIGHT_CONFIGS[NORMAL_WEIGHT_ALLOCATE];

    for (int i = 0; i < 4; i++)
    {
        power_limiter->steer_motors.weight[i] = limit(steer_motor[i].give_current * cfg->steer_current_gain + steer_motor[i].speed_now * cfg->steer_speed_gain, cfg->steer_min, cfg->steer_max);

        float steer_angle_error = my_fabsf(steer_motor[i].angle_set - steer_motor[i].angle_now);
        power_limiter->wheel_motors.weight[i] = limit((90.0f - steer_angle_error) * cfg->wheel_angle_error_gain, cfg->wheel_min, cfg->wheel_max);

        // --- 过颠簸卡舵特殊权重处理 ---
        if (weight_allocate_mode == PASS_BUMPY_ALLOCATE && steer_stuck_status != NULL && steer_stuck_status[i] == 1)
        {
            power_limiter->steer_motors.weight[i] = cfg->steer_max * 2.0f;
            power_limiter->wheel_motors.weight[i] = cfg->wheel_min;
        }
    }
}
```

## 6. 算法完整流程

```
Chassis_Power_Control()                    (见第 43-74 段)
|
├─ Calculate_Initial_Power()              (见第 152-179 段)
│   计算未限制的预测功率
│
├─ P ≤ P_max ? ──是──> 直接返回（不干预）
│
├─ 零电流特殊处理                         (见第 52-62 段)
│   对 give_current = 0 的电机设为 ±1，避免后续计算除零
│
├─ Calculate_All_Alpha_Coefficients()     (见第 187-199 段)
│   预计算 ai, bi, ci
│
├─ Allocate_Motor_Weight()               (见第 287-309 段)
│   动态计算权重 wi
│
├─ Lagrange_Solve_Power_Control()        (见第 87-145 段)
│   ├─ 寻找 λ 上界
│   └─ 二分搜索找 λ*
│
└─ 更新电机电流                           (见第 68-72 段)
    I_final = I_original × αi
```

## 7. 关键参数定义

```c:15:22:chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c
#define MAX_CMD_CURRENT 16384.0f        // 经过功率控制后的最大控制电流
#define MIN_CMD_CURRENT -16384.0f       // 经过功率控制后的最小控制电流
#define LAMBDA_INITIAL_UPEER_BOUND 0.05f // 拉格朗日乘子初始猜测上界
#define LAMBDA_INITIAL_LOWER_BOUND 0.0f // 拉格朗日乘子初始猜测下界
#define LAMBDA_UPPER_BOUND_MAX_ITER 10  // 寻找拉格朗日乘子上限时最大迭代次数
#define LAMBDA_UPPER_BOUND_STEP 10      // 寻找拉格朗日乘子上限时的迭代步长
#define LAMBDA_MAX_ITER 30              // 二分法寻找lambda时的最大迭代次数
#define POWER_TOLERANCE 0.8f            // 功率限制容限，单位W
```
