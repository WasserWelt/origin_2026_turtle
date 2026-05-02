# 哨兵 2026 赛季 CAN 通讯协议报告

> 报告生成日期：2026-04-30
> 代码审查范围：`gimbal_board/` 云台 C 板 + `chassis_board/` 底盘 C 板

> **2026-05-01 更新**：CAN 通道重构已完成，主要变更：
> - 云台板 CAN1：新增大Yaw(DM6006)、拨盘电机、小Pitch(MF6015)、上下板通讯
> - 云台板 CAN2：移除大Yaw、拨盘、小Pitch；保留摩擦轮(M3508 x2)、DM IMU
> - 底盘板 CAN1：新增上下板通讯(0x10/0x11)、功率计接收(0x123)
> - 底盘板 CAN2：移除上下板通讯、大Yaw转发；保留轮电机(M3508 x4)、超电、功率计

---

## 一、硬件与通道分配

| 板卡 | CAN 通道 | 挂载设备 |
|------|----------|---------|
| **云台 C 板** (STM32F407) | CAN1 | 大Yaw电机(DM6006)、大Pitch电机(DM4340)、小Yaw电机(GM6020)、上下板通讯、拨盘电机、小Pitch电机(MF6015) |
| **云台 C 板** | CAN2 | 摩擦轮电机 x2 (M3508)、DM IMU |
| **底盘 C 板** (STM32F407) | CAN1 | 大Yaw电机(DM6006) 反馈(用于底盘跟随云台)、上下板通讯(0x10/0x11)、功率计(0x123) |
| **底盘 C 板** | CAN2 | 轮电机 x4 (M3508)、超电(CAP) |

---

## 二、发送机制概述

CAN 报文的发送采用 **FreeRTOS 队列 + 定时器轮询** 的两级机制：

```
                          云台 / 底盘 业务 Task
  Gimbal_Task (2 ms) / Chassis_Task (2 ms) / Send_Chassis_Task

  Allocate_Can_Msg(...)      ->  FreeRTOS 队列
  Ctrl_DM_Motor(...)           ->  (CAN1/2_send_queue)

                              |
                              | xQueueReceive (非阻塞, 0 ms 等待)
                              |
                          1 ms 轮询定时器回调
  CAN1_Timer_Callback / CAN2_Timer_Callback / DM_IMU_Timer_Callback
  每 1 ms 从队列取 1 帧    ->  HAL_CAN_AddTxMessage()
```

### 定时器配置

| 定时器 | 周期 | 回调函数 | 发送内容 |
|--------|------|----------|---------|
| `CAN1_Timer` | 1 ms | `CAN1_Timer_Callback` | 从 `CAN1_send_queue` 取 1 帧发送 |
| `CAN2_Timer` | 1 ms | `CAN2_Timer_Callback` | 从 `CAN2_send_queue` 取 1 帧发送 |
| `DM_IMU_Timer` | 1 ms | `DM_IMU_Timer_Callback` | 直接调用 `imu_request_gyro()`（不走队列） |

### 队列配置

| 队列 | 队列深度 | 每帧大小 | 用途 |
|------|---------|---------|------|
| `CAN1_send_queue` | 128 | `sizeof(CanTxMsgTypeDef)` | CAN1 所有待发消息 |
| `CAN2_send_queue` | 128 | `sizeof(CanTxMsgTypeDef)` | CAN2 所有待发消息 |

> **注意**: 每 1 ms 定时器只能从队列 **1 帧**发送。Gimbal_Task 每 2 ms 约产生 6~8 帧 CAN 消息（电机控制帧），队列设计可应对突发发送。

---

## 三、云台 C 板 — 完整协议

### 3.1 CAN1 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `BIG_YAW_DM6006_TransID` | `0x01` | ~500 Hz (1 ms 定时器) | 8 | DM MIT 协议 | 大Yaw (DM6006) MIT 控制 |
| `BIG_PITCH_DM4340_CMD` | `0x02` | ~500 Hz (1 ms 定时器) | 8 | DM MIT 协议 | 大Pitch (DM4340) MIT 控制 |
| `SMALL_YAW_AND_PITCH_TransID` | `0x1FE` | ~500 Hz (1 ms 定时器) | 8 | DJI 标准协议 | 小Yaw (GM6020) 电流控制 |
| `GIMBAL_TO_CHASSIS_FIRST_ID` | `0x10` | 100 Hz (Send_Chassis_Task) | 8 | 自定义 | 遥控器数据下发到底盘 |
| `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11` | 50 Hz (Send_Chassis_Task) | 8 | 自定义 | 导航 + 裁判系统数据下发 |
| `GIMBAL_TO_CHASSIS_THIRD_ID` | `0x101` | 预留 | 8 | 自定义 | 预留 |
| `DIAL_TransID` | `0x141` | ~500 Hz (1 ms 定时器) | 8 | LK 协议 | 拨盘电机控制 |
| `SMALL_PITCH_MF6015_CMD` | `0x142` | ~500 Hz (1 ms 定时器) | 8 | LK 协议 | 小Pitch (MF6015) 控制 |

#### 大Yaw DM6006 / 大Pitch DM4340 (0x01 / 0x02) — DM MIT 协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [Pos高8][Pos低8][Vel高8][Vel低4|KP高4][KP低8][KD高4|KdTorq高4][KdTorq低8]
```

| 字段 | 位数 | 范围 | 说明 |
|------|------|------|------|
| `Pos` | 16 bit | `[-12.566, 12.566]` rad | 位置指令 |
| `Vel` | 12 bit | `[-45, 45]` rad/s | 速度前馈 |
| `KP` | 12 bit | `[0, 500]` | 位置刚度 |
| `KD` | 12 bit | `[0, 5]` | 位置阻尼 |
| `Torq` | 12 bit | `[-15, 15]` Nm | 力矩指令 |

#### 小Yaw GM6020 (0x1FE) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [小Yaw高8][小Yaw低8][  0  ][  0  ][  0  ][  0  ][  0  ][  0  ]
```
- `int16_t`，高字节在前；电流范围 卤16000

---

### 3.2 CAN1 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `DIAL_RecID` | `0x141` | 8 | LK 协议 | 拨盘电机反馈 |
| `SMALL_PITCH_MF6015_RecID` | `0x142` | 8 | LK 协议 | 小Pitch MF6015 反馈 |
| `SMALL_YAW_GM6020_RecID` | `0x205` | 8 | DJI 标准 | 小Yaw 电机状态反馈 |
| `BIG_PITCH_DM4340_RecID` | `0x301` | 8 | DM MIT 协议 | 大Pitch DM4340 状态反馈 |

#### 小Pitch MF6015 / 拨盘反馈 (0x141 / 0x142) — LK 协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [  0  ][Temp][Cur低8][Cur高8][Speed低8][Speed高8][ECD低8][ECD高8]
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `temperature` | int8 | 电机温度 |
| `given_current` | int16 | 转矩电流给定 |
| `speed` | int16 | 电机转速 (dps) |
| `ecd` | uint16 | 编码器原始位置 (0~65535) |

#### 大Pitch DM4340 反馈 (0x301)

解析代码 (`bsp_can_gimbal.c` lines 218-232)：
```c
DM_big_pitch_motor.id = (rx_data[0]) & 0x0F;
DM_big_pitch_motor.state = (rx_data[0]) >> 4;
DM_big_pitch_motor.p_int = (rx_data[1] << 8) | rx_data[2];
DM_big_pitch_motor.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
DM_big_pitch_motor.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
```

---

### 3.3 CAN2 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `FRIC_M3508_TransID` | `0x200` | ~500 Hz (1 ms 定时器) | 8 | DJI 标准协议 | 摩擦轮 x2 电流控制 |
| `BIG_YAW_DMIMU_TransID` | `0x15` | 1000 Hz (1 ms 定时器, 直接发送) | 8 | DM IMU 私有协议 | DM IMU 陀螺仪数据请求 |

#### 摩擦轮 M3508 (0x200) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [FRIC1高8][FRIC1低8][FRIC2高8][FRIC2低8][  0  ][  0  ][  0  ][  0  ]
```
- `int16_t`，高字节在前；电流范围 卤16000

#### DM IMU 请求 (0x15) — DM IMU 私有协议

每 1 ms 调用 `imu_request_gyro()`，通过 `HAL_CAN_AddTxMessage` 直接发送（不走队列）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [0xCC][REG=0x02][CMD=0][0xDD][数据0-7][数据8-15][数据16-23][数据24-31]
```
- `imu_read_reg(GYRO_DATA)` -> REG = `0x02`

---

### 3.4 CAN2 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `BIG_YAW_DMIMU_RecID` | `0x16` | 8 | DM IMU 私有协议 | DM IMU 角度/陀螺仪数据 |

#### DM IMU 反馈 (0x16) — DM IMU 私有协议

8 字节数据，第一个字节为数据类型标识，其余为数据：

| pData[0] | 数据类型 | pData[1:7] 格式 | 解析函数 |
|----------|---------|----------------|---------|
| `1` | 加速度 | 3x uint16 (共 16 bit) | `IMU_UpdateAccel()` — accel[3]，范围 卤58.8 m/s? |
| `2` | 陀螺仪 | 3x uint16 (共 16 bit) | `IMU_UpdateGyro()` — gyro[3]，范围 卤34.88 rad/s |
| `3` | 欧拉角 | 3x int16 (共 16 bit) | `IMU_UpdateEuler()` — pitch 卤90掳，yaw 卤180掳，roll 卤180掳 |
| `4` | 四元数 | 4x int (共 14 bit) | `IMU_UpdateQuaternion()` — q[4]，范围 卤1 |

---

### 3.5 云台 — 底盘 通讯帧

#### FIRST 帧 (0x10) — 遥控器数据

**发送方**: `Send_Chassis_Task` (100 Hz)

```
Byte:  0       1       2       3       4       5       6       7
Data: [s[1]高8][s[1]低8|rc_conn][ch[2]高8][ch[2]低8][ch[3]高8][ch[3]低8][ch[4]高8][ch[4]低8]
```

| 字段 | 字节位置 | 数据类型 | 字节序 | 说明 |
|------|---------|---------|--------|------|
| `s[1]` | [0] | uint8 | — | 遥控器左拨杆状态 (高 8 位全 0) |
| `rc_connected` | [1] bit[7] | bool | — | 遥控器连接标志 (`!toe_is_error(DBUS_TOE)`) |
| `ch[2]` | [2:3] | int16 | **big-endian** | 遥控器通道 2 |
| `ch[3]` | [4:5] | int16 | **big-endian** | 遥控器通道 3 |
| `ch[4]` | [6:7] | int16 | **big-endian** | 遥控器通道 4 |

发送代码 (`Send_Chassis_Task.c` line 84)：
```c
Allocate_Can_Msg(
    rc_ctrl.rc.s[1] << 8 | (!toe_is_error(DBUS_TOE)),
    rc_ctrl.rc.ch[2],
    rc_ctrl.rc.ch[3],
    rc_ctrl.rc.ch[4],
    CAN_GIMBAL_TO_CHASSIS_FIRST_CMD
);
```

#### SECOND 帧 (0x11) — 导航 + 裁判系统数据

**发送方**: `Send_Chassis_Task` (50 Hz，每发完 FIRST 帧后 `vTaskDelay(10)` 再发)

使用位域联合体打包数据 (`Send_Chassis_Task.c` lines 25-40)：

```c
union nav_data_u {
    uint16_t packed_data[4];
    struct __attribute__((packed)) {
        uint64_t nav_vx_uint : 16;
        uint64_t nav_vy_uint : 16;
        uint64_t nav_chassis_mode : 2;   // 1=底盘跟随, 2=小陀螺
        uint64_t updownhill_state : 2;
        uint64_t health_state : 1;        // 0=正常, 1=受损
        uint64_t energy_buffer : 6;       // 缓冲能量 (0~63 J)
        uint64_t chassis_max_power : 8;   // 裁判系统功率上限 (0~255 W)
        uint64_t game_start : 1;          // 比赛是否开始 (game_progress==4)
        uint64_t reserved : 12;            // 保留
    } single_data;
};
```

| 字段 | 位数 | 来源 | 范围 |
|------|------|------|------|
| `nav_vx_uint` | 16 | `float_to_uint(NUC_Data_Receive.vx, -10, 10, 12)` | 导航 x 轴目标速度 |
| `nav_vy_uint` | 16 | `float_to_uint(NUC_Data_Receive.vy, -10, 10, 12)` | 导航 y 轴目标速度 |
| `nav_chassis_mode` | 2 | `NUC_Data_Receive.chassis_mode` | 1=跟随, 2=小陀螺 |
| `updownhill_state` | 2 | `NUC_Data_Receive.updownhill_state` | 上下坡状态 |
| `health_state` | 1 | `Health_State_Update()` | 0=正常, 1=被打击受损 |
| `energy_buffer` | 6 | `Power_Heat_Data.buffer_energy` | 缓冲能量，限制到 0~63 |
| `chassis_max_power` | 8 | `Game_Robot_State.chassis_power_limit` | 裁判系统功率上限，限制到 0~255 |
| `game_start` | 1 | `Game_Status.game_progress == 4` | 是否开赛 |

> **注意**: FIRST 帧字节序为 big-endian (高字节在前)，SECOND 帧字节序为 little-endian (低字节在前)。底盘板接收解析时需严格匹配。

---

## 四、底盘 C 板 — 完整协议

### 4.1 CAN1 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `POWER_METER_TransID` | `0x124` | 按需调用 (无队列) | 8 | 自定义 | 向功率计发送数据请求 |

#### 功率计请求 (0x124)

`bsp_can_chassis.c` lines 306-325，直接调用 `HAL_CAN_AddTxMessage`（不走队列）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [yaw高8][yaw低8][pitch高8][pitch低8][shoot高8][shoot低8][rev高8][rev低8]
```

---

### 4.2 CAN1 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `BIG_YAW_DM6006_RecID` | `0x300` | 8 | DM MIT 协议 | 大Yaw DM6006 位置反馈(来自云台板转发，供底盘跟随云台) |
| `GIMBAL_TO_CHASSIS_FIRST_RecID` | `0x10` | 8 | 自定义 | 接收遥控器数据 (100 Hz) |
| `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11` | 8 | 自定义 | 接收导航+裁判数据 (50 Hz) |
| `POWER_METER_RecID` | `0x123` | 8 | 自定义 | 功率计实测功率数据 |

底盘板只解析了电机 ID 和位置整数部分 (`p_int`)，用于计算底盘跟随云台的偏角 (`bsp_can_chassis.c` lines 190-197)：

```c
DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
DM_big_yaw_motor.state = (rx_data[0]) >> 4;
DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
```

---

### 4.3 CAN2 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `WHEEL_M3508_TransID` | `0x200` | 500 Hz (Chassis_Task) | 8 | DJI 标准协议 | 4 个轮电机电流控制 |
| `CAP_TransID` | `0x140` | 500 Hz (Chassis_Task) | 8 | 自定义 | 向超电写入数据 |

#### 轮电机 M3508 (0x200) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [电机1高8][电机1低8][电机2高8][电机2低8][电机3高8][电机3低8][电机4高8][电机4低8]
```

- 电机顺序：TR(右前) / FL(左前) / BR(右后) / BL(左后)
- `int16_t`，高字节在前；电流范围 卤16000
- 来源 `Chassis_Task.c` 500 Hz 调用

#### 超电写入 (0x140)

```
Byte:  0       1       2       3       4       5       6       7
Data: [Val0脳100低8][Val0脳100高8][Val1脳100低8][Val1脳100高8][Val2脳100低8][Val2脳100高8][Val3脳100低8][Val3脳100高8]
```

---

### 4.4 CAN2 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `WHEEL1_M3508_RecID` | `0x201` | 8 | DJI 标准 | 轮电机 1 反馈 |
| `WHEEL2_M3508_RecID` | `0x202` | 8 | DJI 标准 | 轮电机 2 反馈 |
| `WHEEL3_M3508_RecID` | `0x203` | 8 | DJI 标准 | 轮电机 3 反馈 |
| `WHEEL4_M3508_RecID` | `0x204` | 8 | DJI 标准 | 轮电机 4 反馈 |
| `CAP_RecID` | `0x130` | 8 | 自定义 | 超电状态反馈 |

#### 超电反馈 (0x130) — 解析

`bsp_cap.c` lines 5-11：

```
Byte:  0       1       2       3       4       5
Data: [cap_per低8][cap_per高8][chassis_power低8][chassis_power高8][actual_power低8][actual_power高8]
```

| 字段 | 字节位置 | 解析方式 | 说明 |
|------|---------|---------|------|
| `cap_per` | [0:1] | uint16 / 32768.0 | 超电容量百分比 (0~1) |
| `chassis_power` | [2:3] | int16 / 100.0 | 底盘功率 |
| `actual_power` | [4:5] | int16 / 100.0 | 实际功率 |

#### 功率计反馈 (0x123)

```
Byte:  0       1       2       3       4       5       6       7
Data: [IEEE754 float - 32-bit little-endian]
```

```c
uint32_t u32 = (rx_data[0] << 24) | (rx_data[1] << 16) |
               (rx_data[2] << 8) | rx_data[3];
memcpy(&real_power, &u32, sizeof(real_power));
```

---

## 五、完整 CAN ID 一览表

### 云台 C 板

| 方向 | ID 名称 | ID 值 (hex) | CAN 通道 | 协议 | 备注 |
|------|---------|------------|---------|------|------|
| TX | `FRIC_M3508_TransID` | `0x200` | CAN2 | DJI 标准 | 摩擦轮 x2 |
| TX | `BIG_YAW_DM6006_TransID` | `0x01` | CAN1 | DM MIT | 大Yaw DM6006 |
| TX | `BIG_YAW_DMIMU_TransID` | `0x15` | CAN2 | DM IMU | IMU 请求, 1 kHz |
| TX | `BIG_PITCH_DM4340_CMD` | `0x02` | CAN1 | DM MIT | 大Pitch DM4340 |
| TX | `SMALL_YAW_AND_PITCH_TransID` | `0x1FE` | CAN1 | DJI 标准 | 小Yaw GM6020 |
| TX | `SMALL_PITCH_MF6015_CMD` | `0x142` | CAN1 | LK | 小Pitch MF6015 |
| TX | `DIAL_TransID` | `0x141` | CAN1 | LK | 拨盘电机 |
| TX | `GIMBAL_TO_CHASSIS_FIRST_ID` | `0x10` | CAN1 | 自定义 | 遥控器, 100 Hz |
| TX | `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11` | CAN1 | 自定义 | 导航+裁判, 50 Hz |
| TX | `GIMBAL_TO_CHASSIS_THIRD_ID` | `0x101` | CAN1 | 自定义 | 预留 |
| RX | `BIG_YAW_DMIMU_RecID` | `0x16` | CAN2 | DM IMU | 陀螺仪/角度/四元数 |
| RX | `DIAL_RecID` | `0x141` | CAN1 | LK | 拨盘反馈 |
| RX | `SMALL_PITCH_MF6015_RecID` | `0x142` | CAN1 | LK | 小Pitch反馈 |
| RX | `SMALL_YAW_GM6020_RecID` | `0x205` | CAN1 | DJI 标准 | 小Yaw反馈 |
| RX | `BIG_PITCH_DM4340_RecID` | `0x301` | CAN1 | DM MIT | 大Pitch反馈 |

### 底盘 C 板

| 方向 | ID 名称 | ID 值 (hex) | CAN 通道 | 协议 | 备注 |
|------|---------|------------|---------|------|------|
| TX | `WHEEL_M3508_TransID` | `0x200` | CAN2 | DJI 标准 | 轮电机 x4 |
| TX | `CAP_TransID` | `0x140` | CAN2 | 自定义 | 超电写入 |
| TX | `POWER_METER_TransID` | `0x124` | CAN1 | 自定义 | 功率计请求 |
| RX | `BIG_YAW_DM6006_RecID` | `0x300` | CAN1 | DM MIT | 跟随用 |
| RX | `GIMBAL_TO_CHASSIS_FIRST_RecID` | `0x10` | CAN1 | 自定义 | 100 Hz |
| RX | `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11` | CAN1 | 自定义 | 50 Hz |
| RX | `POWER_METER_RecID` | `0x123` | CAN1 | 自定义 | 功率计实测功率 |
| RX | `WHEEL1_M3508_RecID` | `0x201` | CAN2 | DJI 标准 | — |
| RX | `WHEEL2_M3508_RecID` | `0x202` | CAN2 | DJI 标准 | — |
| RX | `WHEEL3_M3508_RecID` | `0x203` | CAN2 | DJI 标准 | — |
| RX | `WHEEL4_M3508_RecID` | `0x204` | CAN2 | DJI 标准 | — |
| RX | `CAP_RecID` | `0x130` | CAN2 | 自定义 | 超电反馈 |

---

## 六、CAN 过滤器配置

### 云台 C 板

```c
can_filter.FilterActivation = ENABLE;
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式，全接收
can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
can_filter.FilterIdHigh = 0x0000;
can_filter.FilterIdLow = 0x0000;
can_filter.FilterMaskIdHigh = 0x0000;
can_filter.FilterMaskIdLow = 0x0000;
```
- **CAN1**: 全接收（掩码全 0）
- **CAN2**: 全接收（掩码全 0）

### 底盘 C 板

```c
// CAN1: 全接收（掩码全 0）
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
can_filter.FilterMaskIdHigh = 0x0000;

// CAN2: 列表模式，仅接收指定 ID
can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
can_filter.FilterIdHigh = (WHEEL1_M3508_RecID << 5);   // 0x201 << 5
can_filter.FilterIdLow  = (WHEEL2_M3508_RecID << 5);   // 0x202 << 5
can_filter.FilterMaskIdHigh = (WHEEL3_M3508_RecID << 5); // 0x203 << 5
can_filter.FilterMaskIdLow  = (CAP_RecID << 5);          // 0x130 << 5
```
- **CAN1 过滤器组 0**: 全接收，接收大Yaw反馈(0x300)、上下板通讯(0x10/0x11)、功率计(0x123)
- **CAN2 过滤器组 14**: 列表模式，接收 ID = 0x201, 0x202, 0x203, 0x130

---

## 七、关键数据流图

```
                              云台 C 板
  遥控器 (DBUS)  ──────> Gimbal_Task ──────> CAN1: 电机控制帧 (500Hz)
                              │
                              │  Send_Chassis_Task (2ms)
                              │  ├── FIRST 帧  (100 Hz) ──> CAN1: 0x10
                              │  └── SECOND 帧 (50 Hz)  ──> CAN1: 0x11
                              │
  NUC (USB)    ──────> Gimbal_Task (2 ms) ──> CAN1: 小Yaw/Pitch (500Hz)
                              │
                              │  ──> CAN1: 大Yaw DM6006 (500Hz)  [通过队列]
                              │  ──> CAN1: 大Pitch DM4340 (500Hz) [通过队列]
                              │  ──> CAN1: 拨盘 + 小Pitch (500Hz) [通过队列]
                              │  ──> CAN2: 摩擦轮 (500Hz)         [通过队列]
                              │  ──> CAN2: DM IMU (1 kHz)          [直接发送]
                              │
                          CAN1 (0x10, 0x11)
                              │
                          ┌─────────────────────────────────────────┐
                          │           物理 CAN 总线                  │
                          └─────────────────────────────────────────┘
                              │                          │
                          CAN1 (0x10, 0x11)           CAN2 (0x200轮电机+0x140超电)
                              │                          │
                          ┌─────────────────────────────────────────┐
                          │           物理 CAN 总线                  │
                          └─────────────────────────────────────────┘
                              │                          │
                              │                     CAN1: 大Yaw反馈(0x300) + 功率计(0x123)
                              │                     CAN2: 轮电机反馈 + 超电反馈
                          底盘 C 板
```

---

## 八、任务优先级与运行周期

| 任务 | 优先级 | 运行周期 | 主要 CAN 操作 |
|------|--------|---------|------------|
| `INS_Task` | `osPriorityRealtime` (最高) | 1 ms | 更新 IMU 数据 |
| `Gimbal_Task` | `osPriorityHigh` | 2 ms | 发送电机控制帧、DM MIT 帧 |
| `Send_Chassis_Task` | `osPriorityHigh` | 2 ms (内部 10 ms 间隔) | 发送云台底板通讯帧 |
| `Shoot_Task` | `osPriorityAboveNormal` | — | 摩擦轮+拨盘控制 |
| `Detect_Task` | `osPriorityAboveNormal` | — | 设备在线检测 |
| `Chassis_Task` | — | 2 ms | 发送轮电机帧、超电写入 |
| `referee_usart` | `osPriorityHigh` | — | 裁判系统 UART 通讯 |

---

## 九、附录：源码文件索引

| 功能 | 文件路径 |
|------|---------|
| 云台 CAN 发送/接收 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c` |
| 云台 CAN ID 定义 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h` |
| 云台主任务 | `gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c` |
| 云台底板发送任务 | `gimbal_board/Gimbal_Board/Tasks/Send_Chassis_Task.c` |
| 云台 CAN 定时器 | `gimbal_board/Gimbal_Board/Tasks/Can_Timer_Task.c` |
| 云台 CAN 定时器头文件 | `gimbal_board/Gimbal_Board/Tasks/Can_Timer_Task.h` |
| DM IMU 驱动 | `gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.c` |
| DM IMU 主头文件 | `gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.h` |
| 电机数据结构 | `gimbal_board/Gimbal_Board/App/motor.c` / `motor.h` |
| 底盘 CAN 发送/接收 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.c` |
| 底盘 CAN ID 定义 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.h` |
| 底盘主任务 | `chassis_board/Chassis_Board/Tasks/Chassis_Task.c` |
| 底盘 CAN 定时器 | `chassis_board/Chassis_Board/Tasks/Can_Timer_Task.c` |
| 超电驱动 | `chassis_board/Chassis_Board/BSP/bsp_cap.c` / `bsp_cap.h` |
