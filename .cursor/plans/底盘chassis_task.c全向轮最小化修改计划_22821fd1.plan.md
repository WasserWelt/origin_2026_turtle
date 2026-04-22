---
name: 底盘Chassis_Task.c全向轮最小化修改计划
overview: 在现有 Chassis_Task.c 基础上做最小化改动，将舵轮底盘改为全向轮底盘。只修改必要的函数体，保留所有注释、命名和代码结构不变。
todos:
  - id: fix-power-init
    content: Chassis_Power_Limitor.h - 修正 power_limitor 全局变量初始化
    status: in_progress
  - id: add-wheel-enum
    content: Chassis_Task.c - 添加轮电机索引枚举
    status: pending
  - id: simplify-pid-init
    content: Chassis_Task.c - 精简 Chassis_Motor_Pid_Init
    status: pending
  - id: simplify-data-update
    content: Chassis_Task.c - 精简 Chassis_Data_Update
    status: pending
  - id: add-mecanum-fn
    content: Chassis_Task.c - 新增 Chassis_Vector_To_Mecanum_Speed
    status: pending
  - id: simplify-safe-handler
    content: Chassis_Task.c - 精简 chassis_safe_handler
    status: pending
  - id: simplify-motor-ctrl
    content: Chassis_Task.c - 精简 Chassis_Motor_Control_Current_Set
    status: pending
  - id: simplify-main-loop
    content: Chassis_Task.c - 精简主循环，删除废弃调用
    status: pending
  - id: remove-dead-code
    content: Chassis_Task.c - 删除废弃函数和声明
    status: pending
isProject: false
---

## 总体原则

- 保留所有解释性注释，不删除
- 保留原有命名，不重命名
- 不重写整个文件，只做必要替换
- 注释掉废弃代码，不删除

---

## 修改清单（逐文件）

### 1. `Chassis_Power_Limitor.h` (已有修改，确保正确)

`power_limitor` 全局变量初始化中删除 `steer_motors` 字段：

```cpp
// ============== 当前状态 ==============
power_limitor_t power_limitor =
{
    .wheel_motors = { ... },
    .steer_motors = { ... }  // 需要删除
};
```

需要添加注释说明已移除。

---

### 2. `bsp_can_chassis.h` - 添加轮电机索引枚举

在 `Chassis_Task.c` 引用处添加，或作为局部定义（避免修改头文件造成其他影响）。计划在 `Chassis_Task.c` 顶部添加：

```cpp
/************************全向轮麦克纳姆布局索引***************************/
typedef enum
{
    WHEEL_MOTOR_FR = 0, // 右前
    WHEEL_MOTOR_FL = 1, // 左前
    WHEEL_MOTOR_BR = 2, // 右后
    WHEEL_MOTOR_BL = 3  // 左后
} wheel_motor_index_t;
```

---

### 3. `Chassis_Task.c` - 核心修改

#### 3.1 函数声明部分 — 替换声明

替换 `Chassis_Motor_Pid_Init` 前的注释：
```
* TODO: 全向要改  →  * TODO: 全向要改
```

#### 3.2 `Chassis_Motor_Pid_Init` 函数 — 精简初始化

**改动**：删除轮电机（speed_pid）以外的所有 PID 初始化，删除 `spin_direction` 初始化语句。

```cpp
// ============== 修改后 ==============
for (uint8_t i = 0; i < 4; i++)
{
    chassis_wheel_motor[i].speed_now = 0;
    chassis_wheel_motor[i].speed_set = 0;
    chassis_wheel_motor[i].speed_set_last = 0;
    chassis_wheel_motor[i].give_current = 0;
    chassis_wheel_motor[i].spin_direction = 1;  // 保留
    PID_init(&chassis_wheel_motor[i].speed_pid, PID_POSITION, wheel_motor_speed_pid,
             WHEEL_MOTOR_SPEED_PID_MAX_OUT, WHEEL_MOTOR_SPEED_PID_MAX_IOUT);
}
// 删除: steer_motor speed_pid 和 angle_pid 初始化
// 删除: steer_motor_speed_pid、steer_motor_angle_pid 常量数组
```

#### 3.3 `Chassis_Data_Update` 函数 — 删除转向电机相关

**改动**：删除 `steer_motor_ENC_offset` 数组，删除 `chassis_steer_motor` 更新代码。

```cpp
// ============== 修改后 ==============
for (uint8_t i = 0; i < 4; i++)
{
    chassis_wheel_motor[i].speed_now = toe_is_error(WHEEL_MOTOR_1_TOE + i) ? 0 : motor_measure_wheel[i].speed_rpm;
    chassis_wheel_motor[i].speed_set_last = chassis_wheel_motor[i].speed_set;
}
// 删除: steer_motor 的 speed_now、angle_now、angle_set_last、speed_set_last 更新
```

#### 3.4 删除 `Find_Steer_Min_Angle` 函数

保留函数体但注释掉，或直接删除整个函数定义（不影响编译）。

#### 3.5 新增 `Chassis_Vector_To_Mecanum_Speed` 函数

在 `Chassis_Vector_To_Steer_Angle` 之前插入：

```cpp
/**
 * @description: 通过底盘坐标系下的目标速度解算出四个轮电机的目标速度（麦克纳姆轮运动学解算）
 * @param vx_set 底盘坐标系下x轴目标速度 (rpm)
 * @param vy_set 底盘坐标系下y轴目标速度 (rpm)
 * @param wz_set 底盘坐标系下z轴目标角速度 (rpm)
 * @note 麦克纳姆轮布局:
 *   wheel_speed[0] = 右前轮 (FR)
 *   wheel_speed[1] = 左前轮 (FL)
 *   wheel_speed[2] = 右后轮 (BR)
 *   wheel_speed[3] = 左后轮 (BL)
 */
static void Chassis_Vector_To_Mecanum_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set)
{
    fp32 wheel_speed[4];
    wheel_speed[WHEEL_MOTOR_FR] = (vx_set + vy_set) + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[WHEEL_MOTOR_FL] = (vx_set - vy_set) + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[WHEEL_MOTOR_BR] = (-vx_set + vy_set) + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[WHEEL_MOTOR_BL] = (-vx_set - vy_set) + MOTOR_DISTANCE_TO_CENTER * wz_set;

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_wheel_motor[i].speed_set = wheel_speed[i];
    }
}
```

#### 3.6 `chassis_safe_handler` 函数 — 精简

删除 `chassis_steer_motor` 相关代码：

```cpp
// ============== 修改后 ==============
for (uint8_t i = 0; i < 4; i++)
{
    chassis_wheel_motor[i].give_current = 0;
    PID_clear(&chassis_wheel_motor[i].speed_pid);
}
// 删除: chassis_steer_motor[i].give_current = 0;
// 删除: PID_clear(&chassis_steer_motor[i].speed_pid);
// 删除: PID_clear(&chassis_steer_motor[i].angle_pid);
```

#### 3.7 `Chassis_Motor_Control_Current_Set` 函数 — 删除转向电机电流计算

**改动**：只保留轮电机（wheel）PID 计算，删除所有 steer 相关逻辑。

```cpp
// ============== 修改后 ==============
for (uint8_t i = 0; i < 4; i++)
{
    PID_calc(&chassis_wheel_motor[i].speed_pid, chassis_wheel_motor[i].speed_now, chassis_wheel_motor[i].speed_set);
    chassis_wheel_motor[i].give_current = (int16_t)chassis_wheel_motor[i].speed_pid.out
        + WHEEL_MOTOR_CURRENT_FF * (chassis_wheel_motor[i].speed_set - chassis_wheel_motor[i].speed_set_last);
    chassis_wheel_motor[i].give_current = limit(chassis_wheel_motor[i].give_current, -16000.0f, 16000.0f);
}
// 删除: steer_motor angle_pid、speed_pid 计算
// 删除: 颠簸卡舵补偿逻辑
```

#### 3.8 主循环 `Chassis_Task` — 删除废弃调用

```cpp
// ============== 修改后 ==============
while (1)
{
    Chassis_Data_Update();
    Chassis_Mode_Update(&chassis_control.chassis_mode);
    Chassis_Max_Power_Update(&chassis_control.chassis_max_power);

    Call_Chassis_Mode_Handler(chassis_control.chassis_mode);
    // 删除: Chassis_Vector_To_Steer_Angle(...)
    Chassis_Vector_To_Mecanum_Speed(chassis_target_speed.vx, chassis_target_speed.vy, chassis_target_speed.wz);
    Chassis_Motor_Control_Current_Set(chassis_control.chassis_mode);

    // 删除: power_limitor.weight_allocate_mode = ...;
    Chassis_Power_Control(&power_limitor, chassis_wheel_motor, chassis_control.chassis_max_power);

    // 删除: Allocate_Can_Msg(chassis_steer_motor[...], CAN_STEER_GM6020_CMD);
    Allocate_Can_Msg(chassis_wheel_motor[0].give_current, chassis_wheel_motor[1].give_current,
                     chassis_wheel_motor[2].give_current, chassis_wheel_motor[3].give_current, CAN_WHEEL_M3508_CMD);

    // 删除: cnt % 5 == 0 分支中的 CAN 发送逻辑
    vTaskDelay(2);
}
```

#### 3.9 删除以下函数（保留定义注释掉）

- `Chassis_Vector_To_Steer_Angle` — 整体注释或删除
- `Chassis_Vector_To_Wheel_Speed` — 整体注释或删除
- `Check_Bumpy_Steer_Stuck` — 整体注释或删除

#### 3.10 删除以下声明（函数声明区域）

- `static fp32 Find_Steer_Min_Angle(...)` 声明
- `static void Chassis_Vector_To_Steer_Angle(...)` 声明
- `static void Chassis_Vector_To_Wheel_Speed(...)` 声明
- `static void Check_Bumpy_Steer_Stuck(void)` 声明

#### 3.11 删除废弃全局变量

- `fp32 temp_move_angle_set[4] = {0};` — 删除

---

## 修改顺序

1. `Chassis_Power_Limitor.h` — 修正 `power_limitor` 全局变量初始化
2. `Chassis_Task.c` — 按上述清单逐步修改

## 预计修改量

- 删除行数：约 200 行
- 新增行数：约 40 行
- 修改行数：约 50 行
- 保留（注释不删）：`Find_Chassis_Follow_Gimbal_ZERO`、`Set_Rotate_Wz` 等导航相关函数（待后续导航适配）