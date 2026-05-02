---
name: Gimbal IMU source swap plan
overview: 将云台小pitch和小yaw的角度/速度闭环数据源从板载IMU切换到外置DM IMU，大yaw的速度闭环数据源从DM IMU切换到板载IMU。共需修改2个文件中的5处赋值。
todos:
  - id: mod-small-yaw-speed
    content: 修改 Gimbal_Task.c 中小 yaw 速度赋值
    status: completed
  - id: mod-small-yaw-angle
    content: 修改 Gimbal_Task.c 中小 yaw 角度赋值
    status: completed
  - id: mod-small-pitch-speed
    content: 修改 Gimbal_Task.c 中小 pitch 速度赋值
    status: completed
  - id: mod-small-pitch-angle
    content: 修改 Gimbal_Task.c 中小 pitch 角度赋值
    status: completed
  - id: mod-big-yaw-speed
    content: 修改 Gimbal_Task.c 中大 yaw 速度赋值
    status: completed
isProject: false
---

## 修改目标

- **小 pitch**：角度和速度闭环 → 改用 DM IMU
- **小 yaw**：角度和速度闭环 → 改用 DM IMU（同时大 yaw 角度自动跟随变化，无需单独修改）
- **大 yaw**：速度闭环 → 改用板载 IMU

## 修改文件

### 1. [gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c](gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c) — `Gimbal_Data_Update()` 函数

共 4 处修改，都在 `Gimbal_Data_Update()` 函数（约 175-228 行）内：

**修改 A — 小 yaw 速度（181 行）**

当前：
```c
gimbal_small_yaw_motor.INS_speed_now = (-arm_sin_f32(INS.Pitch * DEGREE_TO_RAD) * INS.Gyro[AXIS_X] + arm_cos_f32(INS.Pitch * DEGREE_TO_RAD) * INS.Gyro[AXIS_Z]) * RAD_TO_DEGREE;
```
改为：
```c
gimbal_small_yaw_motor.INS_speed_now = imu.gyro[2] * RAD_TO_DEGREE;
```

**修改 B — 小 yaw 角度（182 行）**

当前：
```c
gimbal_small_yaw_motor.INS_angle_now = INS.Yaw;
```
改为：
```c
gimbal_small_yaw_motor.INS_angle_now = imu.yaw;
```

> 注：大 yaw 的 `INS_angle_now`（196 行）依赖 `gimbal_small_yaw_motor.INS_angle_now`，会自动跟随变化，无需单独修改。

**修改 C — 小 pitch 速度（203 行）**

当前：
```c
gimbal_pitch_motor.INS_speed_now = -INS.Gyro[AXIS_Y] * RAD_TO_DEGREE;
```
改为：
```c
gimbal_pitch_motor.INS_speed_now = imu.gyro[1] * RAD_TO_DEGREE;
```

**修改 D — 小 pitch 角度（204 行）**

当前：
```c
gimbal_pitch_motor.INS_angle_now = -INS.Pitch;
```
改为：
```c
gimbal_pitch_motor.INS_angle_now = imu.pitch;
```

**修改 E — 大 yaw 速度（195 行）**

当前：
```c
DM_big_yaw_motor.INS_speed_now = imu.gyro[2] * RAD_TO_DEGREE;
```
改为：
```c
DM_big_yaw_motor.INS_speed_now = -INS.Gyro[AXIS_Z] * RAD_TO_DEGREE;
```

### 2. [gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.h](gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.h) — 添加头文件包含

在 `Gimbal_Task.c` 顶部检查是否有 `bsp_DMIMU.h` 的 `#include`。由于 `bsp_DMIMU.c` 中已定义 `imu_t imu;`，`extern imu_t imu;` 在 `bsp_DMIMU.h` 第 90 行声明，`Gimbal_Task.c` 需确保已 `#include "bsp_DMIMU.h"`。

## 符号说明

| 变化 | 原因 |
|---|---|
| 小 yaw/大 yaw 速度去掉了三角投影计算 | DM IMU 的 gyro[2] 已经是机体 yaw 轴角速度，无需投影 |
| 小 pitch 速度/角度去掉了负号 | DM IMU 的 pitch 轴定义（向上为正）与板载 IMU（向上为负）相反 |
| 大 yaw 速度加负号 | 与原来使用 DM IMU gyro[2] 时的方向对齐 |

## 注意事项

1. 改完后实际测试各轴旋转方向是否正确，如反转需加/去负号
2. 两 IMU 的 yaw 零位可能不一致，如云台整体偏斜需在校准或代码中加偏移量
3. 如 DM IMU 断线，需在安全逻辑中检测 `DM_IMU_TOE` 并切回板载 IMU 数据