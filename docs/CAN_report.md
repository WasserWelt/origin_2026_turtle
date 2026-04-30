# 哨兵 2026 赛季 CAN 通讯协议报告

> 报告生成日期：2026-04-30
> 代码审查范围：`gimbal_board/` 云台 C 板 + `chassis_board/` 底盘 C 板

---

## 一、硬件与通道概览

| 板卡 | CAN 通道 | 挂载设备 |
|------|----------|---------|
| **云台 C 板** (STM32F407) | CAN1 | 摩擦轮电机 x2 (M3508)、小 Yaw 电机 (GM6020)、大 Pitch 电机 (DM4340) |
| **云台 C 板** | CAN2 | 大 Yaw 电机 (DM6006)、DM IMU、小 Pitch 电机 (MF6015)、拨弹盘电机、与底盘板通讯 |
| **底盘 C 板** (STM32F407) | CAN1 | 大 Yaw 电机 (DM6006) 回读（用于底盘跟随云台） |
| **底盘 C 板** | CAN2 | 轮毂电机 x4 (M3508)、超电 (CAP)、功率计、与云台板通讯 |

---

## 二、发送机制总览

CAN 报文的发送采用 **FreeRTOS 队列 + 软件定时器** 的两级架构：

```
┌──────────────────────────────────────────────────────────────────┐
│                         云台 / 底盘 业务 Task                    │
│  Gimbal_Task (2 ms) / Chassis_Task (2 ms) / Send_Chassis_Task │
│                                                                  │
│  Allocate_Can_Msg(...)  ──?  FreeRTOS 队列                     │
│  Ctrl_DM_Motor(...)      ──?  (CAN1/2_send_queue)              │
└──────────────────────────────────────────────────────────────────┘
                              ▲
                              │ xQueueReceive (非阻塞, 0 ms 等待)
                              │
┌──────────────────────────────────────────────────────────────────┐
│                     1 ms 软件定时器回调                            │
│  CAN1_Timer_Callback / CAN2_Timer_Callback / DM_IMU_Timer_Callback│
│  每 1 ms 从队列取出 1 帧  ──?  HAL_CAN_AddTxMessage()           │
└──────────────────────────────────────────────────────────────────┘
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

> **注意**: 每 1 ms 定时器只能从队列取 **1 帧**发送。Gimbal_Task 每 2 ms 约产生 6~8 帧 CAN 消息（电机控制帧），队列设计可容纳突发发送。

---

## 三、云台 C 板 — 完整协议

### 3.1 CAN1 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `FRIC_M3508_TransID` | `0x200` | ~500 Hz (1 ms 定时器) | 8 | DJI 标准协议 | 摩擦轮 x2 电流控制 |
| `SMALL_YAW_AND_PITCH_TransID` | `0x1FE` | ~500 Hz (1 ms 定时器) | 8 | DJI 标准协议 | 小 Yaw (GM6020) 电流控制 |
| `BIG_PITCH_DM4340_CMD` | `0x02` | ~500 Hz (1 ms 定时器) | 8 | DM MIT 协议 | 大 Pitch (DM4340) MIT 控制 |

#### 摩擦轮 M3508 (0x200) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [FRIC1高8][FRIC1低8][FRIC2高8][FRIC2低8][  0  ][  0  ][  0  ][  0  ]
```
- `int16_t`，高字节在前；电流范围 ±16000

#### 小 Yaw GM6020 (0x1FE) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [小Yaw高8][小Yaw低8][  0  ][  0  ][  0  ][  0  ][  0  ][  0  ]
```
- `int16_t`，高字节在前；电流范围 ±16000

#### 大 Pitch DM4340 (0x02) — DM MIT 协议

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

---

### 3.2 CAN1 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `FRIC1_M3508_RecID` | `0x201` | 8 | DJI 标准 | 摩擦轮 1 状态反馈 |
| `FRIC2_M3508_RecID` | `0x202` | 8 | DJI 标准 | 摩擦轮 2 状态反馈 |
| `SMALL_YAW_GM6020_RecID` | `0x205` | 8 | DJI 标准 | 小 Yaw 电机状态反馈 |
| `BIG_PITCH_DM4340_RecID` | `0x301` | 8 | DM MIT 协议 | 大 Pitch DM4340 状态反馈 |

#### DJI 标准电机反馈 (0x201, 0x202, 0x205)

```
Byte:  0       1       2       3       4       5       6       7
Data: [ECD高][ECD低][Speed高][Speed低][Given高][Given低][Temp][  0  ]
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `ecd` | uint16 | 编码器原始值 |
| `speed_rpm` | int16 | 电机转速 (RPM) |
| `given_current` | int16 | 实际给定电流 |
| `temperate` | uint8 | 电机温度 |

#### 大 Pitch DM4340 反馈 (0x301)

```
Byte:  0       1       2       3       4       5       6       7
Data: [电机状态][Pos高8][Pos低8][Vel高4|KP高4][KP低8|Kd高4|Kd低4|Torq高4][Torq低8]
```

解析代码 (`bsp_can_gimbal.c` lines 218-228)：
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
| `BIG_YAW_DM6006_TransID` | `0x01` | ~500 Hz (1 ms 定时器) | 8 | DM MIT 协议 | 大 Yaw (DM6006) MIT 控制 |
| `BIG_YAW_DMIMU_TransID` | `0x15` | 1000 Hz (1 ms 定时器, 直接发送) | 8 | DM IMU 私有协议 | DM IMU 陀螺仪数据请求 |
| `SMALL_PITCH_MF6015_CMD` | `0x142` | ~500 Hz (1 ms 定时器) | 8 | LK 协议 | 小 Pitch (MF6015) 控制 |
| `DIAL_TransID` | `0x141` | ~500 Hz (1 ms 定时器) | 8 | LK 协议 | 拨弹盘电机控制 |
| `GIMBAL_TO_CHASSIS_FIRST_ID` | `0x10` | 100 Hz (Send_Chassis_Task) | 8 | 自定义 | 遥控器数据下发到底盘 |
| `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11` | 50 Hz (Send_Chassis_Task) | 8 | 自定义 | 导航 + 裁判系统数据下发 |
| `GIMBAL_TO_CHASSIS_THIRD_ID` | `0x101` | 未使用 | 8 | 自定义 | 预留 |

#### 大 Yaw DM6006 (0x01) — DM MIT 协议

格式与大 Pitch DM4340 相同（见 3.1 节），使用相同的参数范围。

#### DM IMU 请求 (0x15) — DM IMU 私有协议

每 1 ms 调用 `imu_request_gyro()`，通过 `HAL_CAN_AddTxMessage` 直接发送（不走队列）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [0xCC][REG=0x02][CMD=0][0xDD][数据0-7][数据8-15][数据16-23][数据24-31]
```
- `imu_read_reg(GYRO_DATA)` → REG = `0x02`

#### 小 Pitch MF6015 (0x142) — LK 协议（当前代码使用 DJI 兼容格式）

```
Byte:  0       1       2       3       4       5       6       7
Data: [Data0低][Data0高][Data1低][Data1高][Data2低][Data2高][Data3低][Data3高]
```

#### 拨弹盘 (0x141) — LK 协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [Data0低][Data0高][Data1低][Data1高][Data2低][Data2高][Data3低][Data3高]
```

---

### 3.4 CAN2 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `BIG_YAW_DM6006_RecID` | `0x300` | 8 | DM MIT 协议 | 大 Yaw DM6006 状态反馈 |
| `BIG_YAW_DMIMU_RecID` | `0x16` | 8 | DM IMU 私有协议 | DM IMU 角度/陀螺仪数据 |
| `SMALL_PITCH_MF6015_RecID` | `0x142` | 8 | LK 协议 | 小 Pitch MF6015 状态反馈 |
| `DIAL_RecID` | `0x141` | 8 | LK 协议 | 拨弹盘电机状态反馈 |

#### 大 Yaw DM6006 反馈 (0x300)

```
Byte:  0       1       2       3       4       5       6       7
Data: [电机状态][Pos高8][Pos低8][Vel高4|KP高4][KP低8|Kd高4|Kd低4|Torq高4][Torq低8][Tmos][Tcoil]
```

解析代码 (`bsp_can_gimbal.c` lines 245-256)：
```c
DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
DM_big_yaw_motor.state = (rx_data[0]) >> 4;
DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
DM_big_yaw_motor.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
DM_big_yaw_motor.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
DM_big_yaw_motor.Tmos = (float)(rx_data[6]);
DM_big_yaw_motor.Tcoil = (float)(rx_data[7]);
```
- `p_int` 再乘以 `57.3248408f` 转换为角度（°）

#### DM IMU 反馈 (0x16) — DM IMU 私有协议

8 字节数据，第一个字节为数据类型标识，其余为数据：

| pData[0] | 数据类型 | pData[1:7] 格式 | 解析函数 |
|----------|---------|----------------|---------|
| `1` | 加速度 | 3x uint16 (各 16 bit) | `IMU_UpdateAccel()` — accel[3]，范围 ±58.8 m/s? |
| `2` | 陀螺仪 | 3x uint16 (各 16 bit) | `IMU_UpdateGyro()` — gyro[3]，范围 ±34.88 rad/s |
| `3` | 欧拉角 | 3x int16 (各 16 bit) | `IMU_UpdateEuler()` — pitch ±90°，yaw ±180°，roll ±180° |
| `4` | 四元数 | 4x int (各 14 bit) | `IMU_UpdateQuaternion()` — q[4]，范围 ±1 |

#### 小 Pitch MF6015 / 拨弹盘反馈 (0x142 / 0x141) — LK 协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [  0  ][Temp][Cur低][Cur高][Speed低][Speed高][ECD低][ECD高]
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `temperature` | int8 | 电机温度 |
| `given_current` | int16 | 转矩电流值 |
| `speed` | int16 | 电机转速 (dps) |
| `ecd` | uint16 | 编码器位置 (0~65535) |

---

### 3.5 云台 → 底盘 通讯帧

#### FIRST 帧 (0x10) — 遥控器数据

**发送端**: `Send_Chassis_Task` (100 Hz)，`bsp_can_gimbal.c` lines 309-318

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

**发送端**: `Send_Chassis_Task` (50 Hz，每发完 FIRST 帧后 `vTaskDelay(10)` 再发)

使用位域联合体打包数据 (`Send_Chassis_Task.c` lines 25-40)：

```c
union nav_data_u {
    uint16_t packed_data[4];
    struct __attribute__((packed)) {
        uint64_t nav_vx_uint : 16;
        uint64_t nav_vy_uint : 16;
        uint64_t nav_chassis_mode : 2;   // 1=底盘跟随, 2=小陀螺
        uint64_t updownhill_state : 2;
        uint64_t health_state : 1;        // 0=正常, 1=受伤
        uint64_t energy_buffer : 6;       // 缓冲能量 (0~63 J)
        uint64_t chassis_max_power : 8;   // 裁判系统功率上限 (0~255 W)
        uint64_t game_start : 1;          // 比赛是否开始 (game_progress==4)
        uint64_t reserved : 12;            // 保留
    } single_data;
};
```

数据排布（uint16 数组，低字节在前）：

| 数组元素 | Byte[0] | Byte[1] | Byte[2] | Byte[3] |
|---------|---------|---------|---------|---------|
| `packed_data[0]` | nav_vx 低 8 | nav_vx 高 8 | nav_vy 低 8 | nav_vy 高 8 |
| `packed_data[1]` | [mode:2][updown:2][health:1][res:3] | [res:5][buffer:3] | buffer 高 3 + power 低 5 | power 高 3 + game_start + res |
| `packed_data[2]` | 0 | 0 | 0 | 0 |
| `packed_data[3]` | 0 | 0 | 0 | 0 |

| 字段 | 位数 | 来源 | 范围 |
|------|------|------|------|
| `nav_vx_uint` | 16 | `float_to_uint(NUC_Data_Receive.vx, -10, 10, 12)` | 导航 x 目标速度 |
| `nav_vy_uint` | 16 | `float_to_uint(NUC_Data_Receive.vy, -10, 10, 12)` | 导航 y 目标速度 |
| `nav_chassis_mode` | 2 | `NUC_Data_Receive.chassis_mode` | 1=跟随, 2=小陀螺 |
| `updownhill_state` | 2 | `NUC_Data_Receive.updownhill_state` | 上下坡状态 |
| `health_state` | 1 | `Health_State_Update()` | 0=正常, 1=被击打受伤 |
| `energy_buffer` | 6 | `Power_Heat_Data.buffer_energy` | 缓冲能量，限制 0~63 |
| `chassis_max_power` | 8 | `Game_Robot_State.chassis_power_limit` | 裁判系统功率上限，限制 0~255 |
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
Data: [yaw高][yaw低][pitch高][pitch低][shoot高][shoot低][rev高][rev低]
```

---

### 4.2 CAN1 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `BIG_YAW_DM6006_RecID` | `0x300` | 8 | DM MIT 协议 | 大 Yaw DM6006 位置回读（供底盘跟随云台） |

底盘板只解析了电机 ID 和位置整数部分 (`p_int`)，用于计算底盘跟随云台的夹角 (`bsp_can_chassis.c` lines 190-197)：

```c
DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
DM_big_yaw_motor.state = (rx_data[0]) >> 4;
DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
```

---

### 4.3 CAN2 — 发送

| 消息名称 | CAN ID (hex) | 发送频率 | DLC | 协议 | 用途 |
|---------|------------|---------|-----|------|------|
| `WHEEL_M3508_TransID` | `0x200` | 500 Hz (Chassis_Task) | 8 | DJI 标准协议 | 4 个轮毂电机电流控制 |
| `CAP_TransID` | `0x140` | 500 Hz (Chassis_Task) | 8 | 自定义 | 向超电写入数据 |

#### 轮毂电机 M3508 (0x200) — DJI 标准协议

```
Byte:  0       1       2       3       4       5       6       7
Data: [电机1高][电机1低][电机2高][电机2低][电机3高][电机3低][电机4高][电机4低]
```

- 电机顺序：FR(右前) / FL(左前) / BR(右后) / BL(左后)
- `int16_t`，高字节在前；电流范围 ±16000
- 来自 `Chassis_Task.c` 500 Hz 调用

#### 超电写入 (0x140)

```
Byte:  0       1       2       3       4       5       6       7
Data: [Val0×100低][Val0×100高][Val1×100低][Val1×100高][Val2×100低][Val2×100高][Val3×100低][Val3×100高]
```

---

### 4.4 CAN2 — 接收

| 消息名称 | CAN ID (hex) | DLC | 协议 | 用途 |
|---------|------------|-----|------|------|
| `WHEEL1_M3508_RecID` | `0x201` | 8 | DJI 标准 | 轮毂电机 1 反馈 |
| `WHEEL2_M3508_RecID` | `0x202` | 8 | DJI 标准 | 轮毂电机 2 反馈 |
| `WHEEL3_M3508_RecID` | `0x203` | 8 | DJI 标准 | 轮毂电机 3 反馈 |
| `WHEEL4_M3508_RecID` | `0x204` | 8 | DJI 标准 | 轮毂电机 4 反馈 |
| `GIMBAL_TO_CHASSIS_FIRST_RecID` | `0x10` | 8 | 自定义 | 接收遥控器数据 (100 Hz) |
| `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11` | 8 | 自定义 | 接收导航+裁判数据 (50 Hz) |
| `CAP_RecID` | `0x130` | 8 | 自定义 | 超电状态反馈 |
| `POWER_METER_RecID` | `0x123` | 8 | 自定义 | 功率计实际功率数据 |

#### 云台→底盘 FIRST 帧 (0x10) — 底盘端解析

`bsp_can_chassis.c` lines 217-230：

```
Byte:  0       1       2       3       4       5       6       7
Data: [s[1] ][rc_conn][ch[2]高][ch[2]低][ch[3]高][ch[3]低][ch[4]高][ch[4]低]
```

| 字段 | 字节位置 | 数据类型 | 字节序 | 说明 |
|------|---------|---------|--------|------|
| `s[1]` | [0] | uint8 | — | 遥控器左拨杆状态 |
| `rc_connected` | [1] | uint8 | — | 遥控器连接标志 |
| `ch[2]` | [2:3] | int16 | **big-endian** | 遥控器通道 2 |
| `ch[3]` | [4:5] | int16 | **big-endian** | 遥控器通道 3 |
| `ch[4]` | [6:7] | int16 | **big-endian** | 遥控器通道 4 |

#### 云台→底盘 SECOND 帧 (0x11) — 底盘端解析

`bsp_can_chassis.c` lines 231-247，使用位域重组数据（注意字节序为 little-endian）：

```c
int nav_vx_int = (rx_data[1] << 8) | rx_data[0];
int nav_vy_int = (rx_data[3] << 8) | rx_data[2];
nav_ctrl.vx = uint_to_float(nav_vx_int, -10.0f, 10.0f, 12);
nav_ctrl.vy = uint_to_float(nav_vy_int, -10.0f, 10.0f, 12);
nav_ctrl.chassis_target_mode = rx_data[4] & 0x03;
nav_ctrl.updownhill_state = (rx_data[4] >> 2) & 0x03;
nav_ctrl.health_state = (rx_data[4] >> 4) & 0x01;
nav_ctrl.buffer_energy_remain = ((rx_data[5] & 0x07) << 3) | (rx_data[4] >> 5);
nav_ctrl.referee_power_limit = ((rx_data[6] & 0x07) << 5) | (rx_data[5] >> 3);
nav_ctrl.game_start = (rx_data[6] >> 3) & 0x01;
```

#### 超电反馈 (0x130) — 解析

`bsp_cap.c` lines 5-11：

```
Byte:  0       1       2       3       4       5
Data: [cap_per低][cap_per高][chassis_power低][chassis_power高][actual_power低][actual_power高]
```

| 字段 | 字节位置 | 解析公式 | 说明 |
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
| TX | `FRIC_M3508_TransID` | `0x200` | CAN1 | DJI 标准 | 摩擦轮 x2 |
| TX | `SMALL_YAW_AND_PITCH_TransID` | `0x1FE` | CAN1 | DJI 标准 | 小 Yaw GM6020 |
| TX | `BIG_PITCH_DM4340_CMD` | `0x02` | CAN1 | DM MIT | 大 Pitch DM4340 |
| TX | `BIG_YAW_DM6006_TransID` | `0x01` | CAN2 | DM MIT | 大 Yaw DM6006 |
| TX | `BIG_YAW_DMIMU_TransID` | `0x15` | CAN2 | DM IMU | IMU 请求, 1 kHz |
| TX | `SMALL_PITCH_MF6015_CMD` | `0x142` | CAN2 | LK | 小 Pitch MF6015 |
| TX | `DIAL_TransID` | `0x141` | CAN2 | LK | 拨弹盘 |
| TX | `GIMBAL_TO_CHASSIS_FIRST_ID` | `0x10` | CAN2 | 自定义 | 遥控器, 100 Hz |
| TX | `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11` | CAN2 | 自定义 | 导航+裁判, 50 Hz |
| TX | `GIMBAL_TO_CHASSIS_THIRD_ID` | `0x101` | CAN2 | 自定义 | 预留 |
| RX | `FRIC1_M3508_RecID` | `0x201` | CAN1 | DJI 标准 | — |
| RX | `FRIC2_M3508_RecID` | `0x202` | CAN1 | DJI 标准 | — |
| RX | `SMALL_YAW_GM6020_RecID` | `0x205` | CAN1 | DJI 标准 | — |
| RX | `BIG_PITCH_DM4340_RecID` | `0x301` | CAN1 | DM MIT | — |
| RX | `BIG_YAW_DM6006_RecID` | `0x300` | CAN2 | DM MIT | — |
| RX | `BIG_YAW_DMIMU_RecID` | `0x16` | CAN2 | DM IMU | — |
| RX | `SMALL_PITCH_MF6015_RecID` | `0x142` | CAN2 | LK | — |
| RX | `DIAL_RecID` | `0x141` | CAN2 | LK | — |

### 底盘 C 板

| 方向 | ID 名称 | ID 值 (hex) | CAN 通道 | 协议 | 备注 |
|------|---------|------------|---------|------|------|
| TX | `WHEEL_M3508_TransID` | `0x200` | CAN2 | DJI 标准 | 轮毂 x4 |
| TX | `CAP_TransID` | `0x140` | CAN2 | 自定义 | 超电写入 |
| TX | `POWER_METER_TransID` | `0x124` | CAN1 | 自定义 | 功率计请求 |
| RX | `BIG_YAW_DM6006_RecID` | `0x300` | CAN1 | DM MIT | 跟随用 |
| RX | `WHEEL1_M3508_RecID` | `0x201` | CAN2 | DJI 标准 | — |
| RX | `WHEEL2_M3508_RecID` | `0x202` | CAN2 | DJI 标准 | — |
| RX | `WHEEL3_M3508_RecID` | `0x203` | CAN2 | DJI 标准 | — |
| RX | `WHEEL4_M3508_RecID` | `0x204` | CAN2 | DJI 标准 | — |
| RX | `GIMBAL_TO_CHASSIS_FIRST_RecID` | `0x10` | CAN2 | 自定义 | 100 Hz |
| RX | `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11` | CAN2 | 自定义 | 50 Hz |
| RX | `CAP_RecID` | `0x130` | CAN2 | 自定义 | 超电反馈 |
| RX | `POWER_METER_RecID` | `0x123` | CAN2 | 自定义 | 功率计反馈 |

---

## 六、CAN 过滤器配置

### 云台 C 板

```c
can_filter.FilterActivation = ENABLE;
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
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
// CAN1: 全接收
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
can_filter.FilterMaskIdHigh = 0x0000;

// CAN2: 列表模式，仅接收指定 ID
can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
can_filter.FilterIdHigh = (GIMBAL_TO_CHASSIS_FIRST_RecID << 5);  // 0x10 << 5
can_filter.FilterIdLow = (GIMBAL_TO_CHASSIS_SECOND_RecID << 5);   // 0x11 << 5
can_filter.FilterMaskIdHigh = (WHEEL1_M3508_RecID << 5);         // 0x201 << 5
can_filter.FilterMaskIdLow = (BIG_YAW_DM6006_RecID << 5);        // 0x300 << 5
```
- **CAN2 过滤器组 14**: 列表模式，接收 ID = 0x10, 0x11, 0x201, 0x300

---

## 七、关键数据流图

```
┌──────────────────────────────────────────────────────────────────────┐
│                          云台 C 板                                    │
│                                                                      │
│  ┌─────────────┐     ┌──────────────┐     ┌──────────────────────┐ │
│  │ Remote Ctrl  │────?│ Send_Chassis │────?│ CAN2: 0x10 (100 Hz)  │ │
│  │  (DBUS)     │     │ _Task (2ms)  │     │ 遥控器原始通道数据     │ │
│  └─────────────┘     └──────────────┘     └──────────────────────┘ │
│                                   │                                 │
│                                   │     ┌──────────────────────┐   │
│                                   └────?│ CAN2: 0x11 (50 Hz)   │   │
│                                         │ 导航+裁判+健康状态     │   │
│                                         └──────────────────────┘   │
│                                                                      │
│  ┌─────────────┐                                                    │
│  │ Referee     │──────────┐                                         │
│  │ (UART)      │          │                                         │
│  └─────────────┘          ▼                                         │
│                      ┌──────────────┐                                │
│  ┌─────────────┐     │ Gimbal_Task │────? CAN1: 电机控制帧 (500Hz) │
│  │ NUC (USB)   │─────?│  (2 ms)     │────? CAN2: DM 大Yaw (500Hz) │
│  └─────────────┘     └──────────────┘────? CAN2: 小Pitch (500Hz)  │
│                                              ──? CAN2: 拨弹盘 (500Hz) │
└──────────────────────────────────────────────────────────────────────┘
                              │ CAN2 (0x10, 0x11)
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│                          底盘 C 板                                    │
│                                                                      │
│  ┌──────────────────────┐     ┌──────────────┐     ┌─────────────┐  │
│  │ CAN2: 0x10 解析       │────?│ Chassis_Task │────?│ CAN2: 0x200│  │
│  │ 遥控器通道 → RC控制    │     │  (2 ms)      │     │ 轮毂电机    │  │
│  └──────────────────────┘     │              │────?│ CAN2: 0x140│  │
│                                 │              │     │ 超电写入    │  │
│  ┌──────────────────────┐     │              │     └─────────────┘  │
│  │ CAN2: 0x11 解析       │────?│              │                      │
│  │ 导航+裁判+健康状态     │     └──────────────┘                      │
│  └──────────────────────┘              │                              │
│                                         ▼                              │
│                              ┌──────────────────────┐               │
│                              │ CAN1: 0x300 接收       │               │
│                              │ 大Yaw位置 → 跟随计算   │               │
│                              └──────────────────────┘               │
│                                                                      │
│  ┌──────────────────────┐     ┌──────────────┐                     │
│  │ CAN2: 0x130 超电反馈  │────?│ 功率控制      │                     │
│  │ CAN2: 0x123 功率计    │────?│ Chassis_Power│                     │
│  └──────────────────────┘     │ _Control     │                     │
│                                └──────────────┘                     │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 八、任务优先级与运行周期

| 任务 | 优先级 | 运行周期 | 主要 CAN 操作 |
|------|--------|---------|------------|
| `INS_Task` | `osPriorityRealtime` (最高) | 1 ms | 更新 IMU 数据 |
| `Gimbal_Task` | `osPriorityHigh` | 2 ms | 发送电机控制帧、DM MIT 帧 |
| `Send_Chassis_Task` | `osPriorityHigh` | 2 ms (内部 10 ms 间隔) | 发送云台→底盘通讯帧 |
| `Shoot_Task` | `osPriorityAboveNormal` | — | 摩擦轮+拨弹盘控制 |
| `Detect_Task` | `osPriorityAboveNormal` | — | 设备在线检测 |
| `Chassis_Task` | — | 2 ms | 发送轮毂电机帧、超电写入 |
| `referee_usart` | `osPriorityHigh` | — | 裁判系统 UART 通讯 |

---

## 九、附录：源码文件索引

| 功能 | 文件路径 |
|------|---------|
| 云台 CAN 发送/接收 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c` |
| 云台 CAN ID 定义 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h` |
| 云台主任务 | `gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c` |
| 云台→底盘发送任务 | `gimbal_board/Gimbal_Board/Tasks/Send_Chassis_Task.c` |
| 云台 CAN 定时器 | `gimbal_board/Gimbal_Board/Tasks/Can_Timer_Task.c` |
| 云台 CAN 定时器头文件 | `gimbal_board/Gimbal_Board/Tasks/Can_Timer_Task.h` |
| DM IMU 驱动 | `gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.c` |
| DM IMU 头文件 | `gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.h` |
| 电机数据结构 | `gimbal_board/Gimbal_Board/App/motor.c` / `motor.h` |
| 底盘 CAN 发送/接收 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.c` |
| 底盘 CAN ID 定义 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.h` |
| 底盘主任务 | `chassis_board/Chassis_Board/Tasks/Chassis_Task.c` |
| 底盘 CAN 定时器 | `chassis_board/Chassis_Board/Tasks/Can_Timer_Task.c` |
| 超电驱动 | `chassis_board/Chassis_Board/BSP/bsp_cap.c` / `bsp_cap.h` |
