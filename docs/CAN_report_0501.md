# 哨兵 2026 赛季 CAN 通讯协议报告

> 报告生成日期：2026-05-01
> 代码审核范围：`gimbal_board/` 云台 C 板 + `chassis_board/` 底盘 C 板
> 芯片型号：STM32F407（两板均为）

---

## 一、硬件与通道分配

### 1.2 两板 CAN 总线负载分配


| 板卡         | CAN 通道 | 挂载设备                                                                                   | 总线角色        |
| ---------- | ------ | -------------------------------------------------------------------------------------- | ----------- |
| **云台 C 板** | CAN1   | 大 Yaw (DM6006) / 大 Pitch (DM4340) / 小 Yaw (GM6020) / 拨盘 + 小 Pitch / 云台→底盘通讯 / 裁判系统数据请求 | 电机控制 + 板间通讯 |
| **云台 C 板** | CAN2   | 摩擦轮 x2 (M3508) / DM IMU / 拨盘 + 小 Pitch                                                 | 电机控制 + IMU  |
| **底盘 C 板** | CAN1   | 大 Yaw 位置直接接收(0x300) / 云台→底盘通讯接收 / 功率计                                                  | 板间通讯 + 功率监控 |
| **底盘 C 板** | CAN2   | 轮电机 x4 (M3508) / 超级电容 (CAP)                                                            | 底盘驱动        |


### 1.3 总线互联拓扑

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                     云台 C 板 (CAN1 + CAN2)             │
                    │                                                         │
                    │  CAN1 (PD0/PD1) ──? 大 Yaw DM6006                      │
                    │              ├─? 大 Pitch DM4340                        │
                    │              ├─? 小 Yaw GM6020 (0x1FE)                  │
                    │              ├─? 拨盘 + 小 Pitch (0x141/0x142)           │
                    │              └─? 云台→底盘通讯 (0x10, 0x11)              │
                    │                                                         │
                    │  CAN2 (PB5/PB6) ──? 摩擦轮 M3508 x2 (0x200)            │
                    │              ├─? DM IMU (0x15→/0x16←)                  │
                    │              └─? 拨盘 + 小 Pitch (已迁移至 CAN1)          │
                    └────────────────────────────┬──────────────────────────────┘
                                                 │
                    ┌────────────────────────────▼──────────────────────────────┐
                    │                     底盘 C 板 (CAN1 + CAN2)               │
                    │                                                         │
                    │  CAN1 (PD0/PD1) ?── 云台→底盘通讯 (0x10, 0x11)           │
                    │              ├─? 大 Yaw 位置 (0x300, 来自 DM6006 原生反馈)│
                    │              └─? 功率计数据请求/反馈 (0x124→/0x123←)     │
                    │                                                         │
                    │  CAN2 (PB5/PB6) ?── 轮电机 M3508 反馈 (0x201~0x204)     │
                    │              ├─? 超级电容 (0x130)                        │
                    │              └─? 轮电机控制 (0x200) / 超电写入 (0x140)    │
                    └─────────────────────────────────────────────────────────┘
```

---

## 二、CAN 总线波特率配置

两板完全一致：


| 参数              | 值            | 说明                                                                                                                                                                |
| --------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 预分频 (Prescaler) | 3            | APB1 时钟 42 MHz / 3 = 14 MHz                                                                                                                                       |
| 同步跳转宽度 (SJW)    | 1 TQ         |                                                                                                                                                                   |
| 时间段1 (BS1)      | 10 TQ        | 采样点位置                                                                                                                                                             |
| 时间段2 (BS2)      | 3 TQ         |                                                                                                                                                                   |
| 总 TQ 数          | 16           |                                                                                                                                                                   |
| **波特率**         | **875 kbps** | 42 MHz / 3 / 16 = 875,000 bps                                                                                                                                     |
| 采样点             | ~87.5%       | (1+10) / 16 ≈ 68.75%... actually (1+10)/16 = 68.75%... 正确：BS1+SS/(总TQ) = (10+1)/16 = 68.75%... 不对 采样点 = (1+BS1)/(1+BS1+BS2) = (1+10)/(1+10+3) = 11/14 ≈ **78.6%** |


---

## 三、发送机制：FreeRTOS 队列 + 定时器调度

### 3.1 两级发送架构

```
  业务 Task (2 ms 周期)
        │
        ▼
  Allocate_Can_Msg(...) / Ctrl_DM_Motor(...)
        │  填充到全局缓冲区
        ▼
  xQueueSend(CANx_send_queue, ...)   ──?  FreeRTOS 队列 (深度 128)
        │                                      │
        │  xQueueReceive (非阻塞, 0 ms)        │
        ▼                                      │
  1 ms 定时器回调                                │
  CAN1_Timer_Callback ──? 从 CAN1_send_queue 取 1 帧 ──? HAL_CAN_AddTxMessage()
  CAN2_Timer_Callback ──? 从 CAN2_send_queue 取 1 帧 ──? HAL_CAN_AddTxMessage()
```

### 3.2 定时器配置


| 定时器            | 周期   | 回调函数                    | 职责                              |
| -------------- | ---- | ----------------------- | ------------------------------- |
| `CAN1_Timer`   | 1 ms | `CAN1_Timer_Callback`   | 从 `CAN1_send_queue` 取 1 帧发送     |
| `CAN2_Timer`   | 1 ms | `CAN2_Timer_Callback`   | 从 `CAN2_send_queue` 取 1 帧发送     |
| `DM_IMU_Timer` | 1 ms | `DM_IMU_Timer_Callback` | 直接调用 `imu_request_gyro()`（不走队列） |


> **注意**：每 1 ms 定时器只能从队列取 **1 帧**发送。业务 Task 每 2 ms 可能产生 6~8 帧 CAN 消息（4 个电机 + DM + 板间通讯等），队列设计可应对突发发送压力，但需避免队列溢出。

### 3.3 队列配置


| 队列                | 队列深度 | 元素大小                      | 使用通道        |
| ----------------- | ---- | ------------------------- | ----------- |
| `CAN1_send_queue` | 128  | `sizeof(CanTxMsgTypeDef)` | CAN1 所有发送消息 |
| `CAN2_send_queue` | 128  | `sizeof(CanTxMsgTypeDef)` | CAN2 所有发送消息 |


---

## 四、云台 C 板 — 完整通讯协议

### 4.1 CAN1 发送


| 消息名称                          | CAN ID (hex) | 频率                 | DLC | 协议     | 用途                      |
| ----------------------------- | ------------ | ------------------ | --- | ------ | ----------------------- |
| `BIG_YAW_DM6006_TransID`      | `0x01`       | ~500 Hz (1 ms 定时器) | 8   | DM MIT | 大 Yaw (DM6006) MIT 控制   |
| `BIG_PITCH_DM4340_CMD`        | `0x02`       | ~500 Hz (1 ms 定时器) | 8   | DM MIT | 大 Pitch (DM4340) MIT 控制 |
| `SMALL_YAW_AND_PITCH_TransID` | `0x1FE`      | ~500 Hz (1 ms 定时器) | 8   | DJI 标准 | 小 Yaw (GM6020) 电流控制     |
| `DIAL_TransID`                | `0x141`      | ~500 Hz (1 ms 定时器) | 8   | LK 协议  | 拨盘电机控制                  |
| `SMALL_PITCH_MF6015_CMD`      | `0x142`      | ~500 Hz (1 ms 定时器) | 8   | LK 协议  | 小 Pitch (MF6015) 控制     |
| `GIMBAL_TO_CHASSIS_FIRST_ID`  | `0x10`       | 100 Hz             | 8   | 自定义    | 遥控器数据下发给底盘              |
| `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11`       | 50 Hz              | 8   | 自定义    | 导航 + 裁判系统数据下发给底盘        |
| `GIMBAL_TO_CHASSIS_THIRD_ID`  | `0x101`      | 预留                 | 8   | 自定义    | 预留                      |


### 4.2 CAN1 接收


| 消息名称                       | CAN ID (hex) | DLC | 协议     | 用途                  |
| -------------------------- | ------------ | --- | ------ | ------------------- |
| `DIAL_RecID`               | `0x141`      | 8   | LK 协议  | 拨盘电机反馈              |
| `SMALL_PITCH_MF6015_RecID` | `0x142`      | 8   | LK 协议  | 小 Pitch MF6015 反馈   |
| `SMALL_YAW_GM6020_RecID`   | `0x205`      | 8   | DJI 标准 | 小 Yaw 电机状态反馈        |
| `BIG_PITCH_DM4340_RecID`   | `0x301`      | 8   | DM MIT | 大 Pitch DM4340 状态反馈 |


### 4.3 CAN2 发送


| 消息名称                    | CAN ID (hex) | 频率                       | DLC | 协议        | 用途            |
| ----------------------- | ------------ | ------------------------ | --- | --------- | ------------- |
| `FRIC_M3508_TransID`    | `0x200`      | ~500 Hz (1 ms 定时器)       | 8   | DJI 标准    | 摩擦轮 x2 电流控制   |
| `BIG_YAW_DMIMU_TransID` | `0x15`       | 1000 Hz (1 ms 定时器, 直接发送) | 8   | DM IMU 私有 | DM IMU 陀螺数据请求 |


### 4.4 CAN2 接收


| 消息名称                   | CAN ID (hex) | DLC | 协议        | 用途                             |
| ---------------------- | ------------ | --- | --------- | ------------------------------ |
| `BIG_YAW_DMIMU_RecID`  | `0x16`       | 8   | DM IMU 私有 | DM IMU 角速度/欧拉角/四元数数据           |
| `BIG_YAW_DM6006_RecID` | `0x300`      | 8   | DM MIT    | **大 Yaw DM6006 原生反馈**（来自底盘板转发） |


---

## 五、底盘 C 板 — 完整通讯协议

### 5.1 CAN1 发送


| 消息名称                  | CAN ID (hex) | 频率                | DLC | 协议  | 用途         |
| --------------------- | ------------ | ----------------- | --- | --- | ---------- |
| `POWER_METER_TransID` | `0x124`      | 按需调用 (直接发送, 不走队列) | 8   | 自定义 | 向功率计发送数据请求 |


### 5.2 CAN1 接收


| 消息名称                             | CAN ID (hex) | DLC | 协议     | 用途                                   |
| -------------------------------- | ------------ | --- | ------ | ------------------------------------ |
| `BIG_YAW_DM6006_RecID`           | `0x300`      | 8   | DM MIT | 大 Yaw 电机位置（来自 DM6006 原生反馈，云台板和底盘板共接） |
| `GIMBAL_TO_CHASSIS_FIRST_RecID`  | `0x10`       | 8   | 自定义    | 遥控器数据 (100 Hz)                       |
| `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11`       | 8   | 自定义    | 导航 + 裁判系统数据 (50 Hz)                  |
| `POWER_METER_RecID`              | `0x123`      | 8   | 自定义    | 功率计实测功率数据 (IEEE754 float)            |


### 5.3 CAN2 发送


| 消息名称                  | CAN ID (hex) | 频率                    | DLC | 协议     | 用途         |
| --------------------- | ------------ | --------------------- | --- | ------ | ---------- |
| `WHEEL_M3508_TransID` | `0x200`      | 500 Hz (Chassis_Task) | 8   | DJI 标准 | 4 个轮电机电流控制 |
| `CAP_TransID`         | `0x140`      | 500 Hz (Chassis_Task) | 8   | 自定义    | 超级电容写入数据   |


### 5.4 CAN2 接收


| 消息名称                 | CAN ID (hex) | DLC | 协议     | 用途       |
| -------------------- | ------------ | --- | ------ | -------- |
| `WHEEL1_M3508_RecID` | `0x201`      | 8   | DJI 标准 | 轮电机 1 反馈 |
| `WHEEL2_M3508_RecID` | `0x202`      | 8   | DJI 标准 | 轮电机 2 反馈 |
| `WHEEL3_M3508_RecID` | `0x203`      | 8   | DJI 标准 | 轮电机 3 反馈 |
| `WHEEL4_M3508_RecID` | `0x204`      | 8   | DJI 标准 | 轮电机 4 反馈 |
| `CAP_RecID`          | `0x130`      | 8   | 自定义    | 超级电容状态反馈 |


---

## 六、详细数据协议解析

### 6.1 DM MIT 协议（DM6006 / DM4340）

用于大 Yaw (DM6006) 和大 Pitch (DM4340) 的发送控制帧：

```
Byte:  0         1         2         3         4         5         6         7
Data: [Pos 高8] [Pos 低8] [Vel 高4|KP 高4] [KP 低8] [KD 高4|KdTorq 高4] [KdTorq 低8]
```


| 字段     | 位数     | 浮点范围                    | 量化范围    | 说明                                     |
| ------ | ------ | ----------------------- | ------- | -------------------------------------- |
| `Pos`  | 16 bit | `[-12.566, 12.566]` rad | 0~65535 | 位置指令（DM6006 用作角度，DM4340 用作大 Pitch 俯仰角） |
| `Vel`  | 12 bit | `[-45, 45]` rad/s       | 0~4095  | 速度前馈                                   |
| `KP`   | 12 bit | `[0, 500]`              | 0~4095  | 位置刚度                                   |
| `KD`   | 12 bit | `[0, 5]`                | 0~4095  | 位置阻尼                                   |
| `Torq` | 12 bit | `[-15, 15]` Nm          | 0~4095  | 力矩指令                                   |


接收反馈帧（DM6006 原生反馈 `0x300`，DM4340 反馈 `0x301`）：

```c
motor.id     = (rx_data[0]) & 0x0F;
motor.state  = (rx_data[0]) >> 4;
motor.p_int  = (rx_data[1] << 8) | rx_data[2];
motor.v_int  = (rx_data[3] << 4) | (rx_data[4] >> 4);
motor.t_int  = ((rx_data[4] & 0xF) << 8) | rx_data[5];
// DM6006 额外字段:
motor.Tmos   = (float)(rx_data[6]);  // 电机温度 (°C)
motor.Tcoil  = (float)(rx_data[7]);  // 线圈温度 (°C)
// DM4340 无温度字段，但额外解析 v_int 和 t_int
```

> **注意**：大 Yaw (DM6006) 的 `0x300` 反馈帧同时被云台板和底盘板接收。云台板用于闭环控制，底盘板用于计算底盘跟随云台的偏角（`Chassis_Task.c` lines 154-160）。

### 6.2 DJI 标准协议（M3508 / GM6020）

**发送帧**（电流控制，int16_t，高字节在前）：

```
Byte:  0       1       2       3       4       5       6       7
M3508: [ID1高8][ID1低8][ID2高8][ID2低8][ID3高8][ID3低8][ID4高8][ID4低8]
GM6020:[Yaw高8][Yaw低8][  0   ][  0   ][  0   ][  0   ][  0   ][  0   ]
```

- `int16_t`，高字节在前；电流范围 ±16000

**接收帧**（DJI 标准电机反馈）：

```
Byte:  0       1       2       3       4       5       6       7
M3508: [转子角度高8][转子角度低8][转速高8][转速低8][实际电流高8][实际电流低8][温度][保留]
GM6020: [转子角度高8][转子角度低8][转速高8][转速低8][实际电流高8][实际电流低8][温度][保留]
```

- `angle`: 0~8191（机械角度，13 bit，编码器一圈）
- `speed_rpm`: int16，电机输出轴转速（rpm）
- `given_current`: int16，闭环电流输出
- `temperature`: uint8，电机温度

### 6.3 LK 协议（MF6015 / MF4010 拨盘）

**发送帧**（`0x141` / `0x142`）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [  0   ][  0   ][  0   ][  0   ][  0   ][  0   ][  0   ][  0   ]
```

> 当前代码中拨盘和小 Pitch 发送帧直接填充全 0（使用 DM 电机控制链路中的 int16_t 发送函数），实际控制数据由 LK 协议电流值填充到低字节位置。

**接收帧**：

```
Byte:  0       1       2       3       4       5       6       7
Data: [  0   ][温度][电流低8][电流高8][速度低8][速度高8][编码器低8][编码器高8]
```


| 字段              | 类型     | 说明                |
| --------------- | ------ | ----------------- |
| `temperature`   | int8   | 电机温度              |
| `given_current` | int16  | 实际电流              |
| `speed_rpm`     | int16  | 电机转速 (dps)        |
| `ecd`           | uint16 | 编码器原始位置 (0~65535) |


### 6.4 云台→底盘通讯协议

#### FIRST 帧 (0x10) — 遥控器数据，100 Hz

```
Byte:  0       1       2       3       4       5       6       7
Data: [s[1]   ][rc_conn|ch[2]高8][ch[2]低8][ch[3]高8][ch[3]低8][ch[4]高8][ch[4]低8]
```


| 字段             | 字节位置       | 数据类型  | 字节序            | 说明       |
| -------------- | ---------- | ----- | -------------- | -------- |
| `s[1]`         | [0]        | uint8 | —              | 遥控器左拨杆状态 |
| `rc_connected` | [1] bit[7] | bool  | —              | 遥控器连接标志  |
| `ch[2]`        | [2:3]      | int16 | **big-endian** | 遥控器通道 2  |
| `ch[3]`        | [4:5]      | int16 | **big-endian** | 遥控器通道 3  |
| `ch[4]`        | [6:7]      | int16 | **big-endian** | 遥控器通道 4  |


#### SECOND 帧 (0x11) — 导航 + 裁判系统数据，50 Hz

使用位域打包（bit packing），总 64 bit（8 字节）：

```c
union nav_data_u {
    uint16_t packed_data[4];
    struct __attribute__((packed)) {
        uint64_t nav_vx_uint : 16;        // 导航 x 轴目标速度 (m/s)
        uint64_t nav_vy_uint : 16;        // 导航 y 轴目标速度 (m/s)
        uint64_t nav_chassis_mode : 2;    // 底盘目标模式 (1=跟随, 2=小陀螺)
        uint64_t updownhill_state : 2;    // 上下坡状态
        uint64_t health_state : 1;        // 哨兵健康状态 (0=正常, 1=受损)
        uint64_t energy_buffer : 6;        // 缓冲能量 (0~63 J)
        uint64_t chassis_max_power : 8;    // 裁判系统功率上限 (0~255 W)
        uint64_t game_start : 1;           // 比赛是否开始 (game_progress==4)
        uint64_t reserved : 12;            // 保留
    } single_data;
};
```


| 字段                  | 位数  | 浮点转换                            | 范围         | 说明            |
| ------------------- | --- | ------------------------------- | ---------- | ------------- |
| `nav_vx`            | 16  | `uint_to_float(v, -10, 10, 12)` | -10~10 m/s | 导航 x 目标速度     |
| `nav_vy`            | 16  | `uint_to_float(v, -10, 10, 12)` | -10~10 m/s | 导航 y 目标速度     |
| `nav_chassis_mode`  | 2   | 直接使用                            | 1~2        | 1=跟随云台, 2=小陀螺 |
| `updownhill_state`  | 2   | 直接使用                            | 0~3        | 上下坡状态         |
| `health_state`      | 1   | 直接使用                            | 0/1        | 0=正常, 1=受击    |
| `energy_buffer`     | 6   | 直接使用                            | 0~63       | 缓冲能量 (J)      |
| `chassis_max_power` | 8   | 直接使用                            | 0~255      | 裁判系统功率上限 (W)  |
| `game_start`        | 1   | 直接使用                            | 0/1        | 比赛开始标志        |


> **注意**：FIRST 帧字节序为 **big-endian**（高字节在前），SECOND 帧字节序为 **little-endian**（低字节在前）。底盘板接收解析时需严格匹配。

### 6.5 DM IMU 私有协议

**发送请求帧** (`0x15`) — 1 kHz，直接发送（不走队列）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [0xCC][REG=0x02][CMD=0][0xDD][数据0-7][数据8-15][数据16-23][数据24-31]
```

**接收反馈帧** (`0x16`) — 数据类型由 pData[0] 决定：


| pData[0] | 数据类型 | 格式        | 解析函数                                                    |
| -------- | ---- | --------- | ------------------------------------------------------- |
| `1`      | 加速度  | 3x uint16 | `IMU_UpdateAccel()` — accel[3]                          |
| `2`      | 角速度  | 3x uint16 | `IMU_UpdateGyro()` — gyro[3]                            |
| `3`      | 欧拉角  | 3x int16  | `IMU_UpdateEuler()` — pitch ±90°, yaw ±180°, roll ±180° |
| `4`      | 四元数  | 4x int14  | `IMU_UpdateQuaternion()` — q[4]                         |


### 6.6 超级电容 (CAP) 协议

**底盘→CAP 写入帧** (`0x140`) — `int16_t × 100`，低字节在前：

```
Byte:  0       1       2       3       4       5       6       7
Data: [Val0低8][Val0高8][Val1低8][Val1高8][Val2低8][Val2高8][Val3低8][Val3高8]
```

**CAP→底盘反馈帧** (`0x130`)：

```
Byte:  0       1       2       3       4       5
Data: [cap_per低8][cap_per高8][chassis_power低8][chassis_power高8][actual_power低8][actual_power高8]
```


| 字段              | 类型     | 转换        | 说明            |
| --------------- | ------ | --------- | ------------- |
| `cap_per`       | uint16 | / 32768.0 | 电容容量百分比 (0~1) |
| `chassis_power` | int16  | / 100.0   | 底盘功率          |
| `actual_power`  | int16  | / 100.0   | 实际功率          |


### 6.7 功率计协议

**底盘→功率计请求帧** (`0x124`) — 直接发送（不走队列）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [yaw高8][yaw低8][pitch高8][pitch低8][shoot高8][shoot低8][rev高8][rev低8]
```

- int16_t，高字节在前

**功率计→底盘反馈帧** (`0x123`) — IEEE754 单精度浮点（大端）：

```
Byte:  0       1       2       3       4       5       6       7
Data: [Float MSB ← ──────────────────────────────── Float LSB]
```

```c
uint32_t u32 = (rx_data[0] << 24) | (rx_data[1] << 16) |
               (rx_data[2] << 8)  | rx_data[3];
memcpy(&real_power, &u32, sizeof(real_power));
```

---

## 七、完整 CAN ID 一览表

### 7.1 云台 C 板


| 方向  | ID 名称                         | ID 值 (hex) | CAN 通道 | 协议        | 频率      |
| --- | ----------------------------- | ---------- | ------ | --------- | ------- |
| TX  | `FRIC_M3508_TransID`          | `0x200`    | CAN2   | DJI 标准    | ~500 Hz |
| TX  | `BIG_YAW_DM6006_TransID`      | `0x01`     | CAN1   | DM MIT    | ~500 Hz |
| TX  | `BIG_YAW_DMIMU_TransID`       | `0x15`     | CAN2   | DM IMU 私有 | 1000 Hz |
| TX  | `BIG_PITCH_DM4340_CMD`        | `0x02`     | CAN1   | DM MIT    | ~500 Hz |
| TX  | `SMALL_YAW_AND_PITCH_TransID` | `0x1FE`    | CAN1   | DJI 标准    | ~500 Hz |
| TX  | `SMALL_PITCH_MF6015_CMD`      | `0x142`    | CAN1   | LK        | ~500 Hz |
| TX  | `DIAL_TransID`                | `0x141`    | CAN1   | LK        | ~500 Hz |
| TX  | `GIMBAL_TO_CHASSIS_FIRST_ID`  | `0x10`     | CAN1   | 自定义       | 100 Hz  |
| TX  | `GIMBAL_TO_CHASSIS_SECOND_ID` | `0x11`     | CAN1   | 自定义       | 50 Hz   |
| TX  | `GIMBAL_TO_CHASSIS_THIRD_ID`  | `0x101`    | CAN1   | 自定义       | 预留      |
| RX  | `BIG_YAW_DMIMU_RecID`         | `0x16`     | CAN2   | DM IMU 私有 | —       |
| RX  | `DIAL_RecID`                  | `0x141`    | CAN1   | LK        | —       |
| RX  | `SMALL_PITCH_MF6015_RecID`    | `0x142`    | CAN1   | LK        | —       |
| RX  | `SMALL_YAW_GM6020_RecID`      | `0x205`    | CAN1   | DJI 标准    | —       |
| RX  | `BIG_PITCH_DM4340_RecID`      | `0x301`    | CAN1   | DM MIT    | —       |
| RX  | `BIG_YAW_DM6006_RecID`        | `0x300`    | CAN2   | DM MIT    | —       |


### 7.2 底盘 C 板


| 方向  | ID 名称                            | ID 值 (hex) | CAN 通道 | 协议     | 频率     |
| --- | -------------------------------- | ---------- | ------ | ------ | ------ |
| TX  | `WHEEL_M3508_TransID`            | `0x200`    | CAN2   | DJI 标准 | 500 Hz |
| TX  | `CAP_TransID`                    | `0x140`    | CAN2   | 自定义    | 500 Hz |
| TX  | `POWER_METER_TransID`            | `0x124`    | CAN1   | 自定义    | 按需调用   |
| RX  | `BIG_YAW_DM6006_RecID`           | `0x300`    | CAN1   | DM MIT | —      |
| RX  | `GIMBAL_TO_CHASSIS_FIRST_RecID`  | `0x10`     | CAN1   | 自定义    | 100 Hz |
| RX  | `GIMBAL_TO_CHASSIS_SECOND_RecID` | `0x11`     | CAN1   | 自定义    | 50 Hz  |
| RX  | `POWER_METER_RecID`              | `0x123`    | CAN1   | 自定义    | —      |
| RX  | `WHEEL1_M3508_RecID`             | `0x201`    | CAN2   | DJI 标准 | —      |
| RX  | `WHEEL2_M3508_RecID`             | `0x202`    | CAN2   | DJI 标准 | —      |
| RX  | `WHEEL3_M3508_RecID`             | `0x203`    | CAN2   | DJI 标准 | —      |
| RX  | `WHEEL4_M3508_RecID`             | `0x204`    | CAN2   | DJI 标准 | —      |
| RX  | `CAP_RecID`                      | `0x130`    | CAN2   | 自定义    | —      |


---

## 八、CAN 过滤器配置

### 8.1 云台 C 板

两路 CAN 均使用**掩码模式全接收**：

```c
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码模式，全接收
can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
can_filter.FilterIdHigh = 0x0000;
can_filter.FilterIdLow = 0x0000;
can_filter.FilterMaskIdHigh = 0x0000;
can_filter.FilterMaskIdLow = 0x0000;
```

- **CAN1 过滤器组 0**：全接收
- **CAN2 过滤器组 14**：全接收（`SlaveStartFilterBank = 14`）

### 8.2 底盘 C 板

```c
// CAN1: 掩码模式全接收 (过滤器组 0)
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 全接收

// CAN2: 列表模式，仅接收指定 ID (过滤器组 14)
can_filter.FilterMode = CAN_FILTERMODE_IDLIST;  // 列表模式
can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
can_filter.FilterIdHigh = (WHEEL1_M3508_RecID << 5);   // 0x201 << 5
can_filter.FilterIdLow  = (WHEEL2_M3508_RecID << 5);   // 0x202 << 5
can_filter.FilterMaskIdHigh = (WHEEL3_M3508_RecID << 5); // 0x203 << 5
can_filter.FilterMaskIdLow  = (CAP_RecID << 5);           // 0x130 << 5
```

- **CAN1 过滤器组 0**：全接收，接收大 Yaw 反馈 (0x300)、云台→底盘通讯 (0x10/0x11)、功率计反馈 (0x123)
- **CAN2 过滤器组 14**：列表模式，仅接收轮电机 1~~3 反馈 (0x201~~0x203) 和超电反馈 (0x130)

> **注意**：列表模式下 16 位缩放，每两个 ID 共用 32 位掩码位置。当前配置将 4 个 ID 压缩为 2 个 32 位槽位，但 `WHEEL4_M3508_RecID (0x204)` 和 `POWER_METER_RecID (0x123)` 未被显式配置——这可能意味着它们通过全接收掩码也被接收，或存在配置缺失。建议验证 0x204 轮电机反馈是否被底盘板正确接收。

---

## 九、任务调度与优先级


| 任务                  | 板卡      | 优先级                     | 周期                 | 主要 CAN 操作              |
| ------------------- | ------- | ----------------------- | ------------------ | ---------------------- |
| `INS_Task`          | 云台 + 底盘 | `osPriorityRealtime`    | 1 ms               | 更新 IMU 数据              |
| `Gimbal_Task`       | 云台      | `osPriorityHigh`        | 2 ms               | 发送电机控制帧 (DM MIT + DJI) |
| `Send_Chassis_Task` | 云台      | `osPriorityHigh`        | 2 ms (内部 10 ms 间隔) | 发送云台→底盘通讯帧             |
| `Chassis_Task`      | 底盘      | `osPriorityAboveNormal` | 2 ms               | 发送轮电机 + 超电控制帧          |
| `Detect_Task`       | 云台 + 底盘 | `osPriorityAboveNormal` | —                  | 设备在线检测                 |
| `referee_usart`     | 云台      | `osPriorityHigh`        | —                  | 裁判系统 UART 通讯           |


---

## 十、关键数据流图

```
 云台 C 板
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │  遥控器 (DBUS)  ──?  Gimbal_Task (2ms)  ──┬──? Ctrl_DM_Motor()     ──? CAN1  │
 │                                            ├──? Allocate_Can_Msg()   ──? CAN1  │
 │  NUC (USB)  ──?  Gimbal_Task (2ms)  ─────┤                                  │
 │                                            │                                  │
 │  Send_Chassis_Task (2ms)  ──┬──? FIRST 帧 (0x10) 100 Hz  ──? CAN1 ─────────┤
 │                              └──? SECOND 帧 (0x11) 50 Hz   ──? CAN1 ─────────┤
 │                                                                             │
 │  DM IMU (CAN2)  ?──? 1 kHz 直接请求 ──? 陀螺/欧拉角/四元数                   │
 │                                                                             │
 │  大 Yaw 反馈 (0x300)  ?── CAN2 ───────────────────────────────────────────┤
 └─────────────────────────────────────────────────────────────────────────────┘
                                         │
                                         │ 物理 CAN 总线 (CAN1)
                                         ▼
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │  底盘 C 板                                                                     │
 │                                                                             │
 │  FIRST 帧 (0x10)  ──? 解析 s[1], ch[2~4] ──? Chassis_Task                  │
 │  SECOND 帧 (0x11) ──? 解析 nav_vx/vy, 裁判系统数据 ──? Chassis_Task         │
 │  大 Yaw 位置 (0x300) ──? 归一化 ──? 底盘跟随云台偏角计算                    │
 │  功率计反馈 (0x123) ──? IEEE754 float ──? 功率限制                         │
 │                                                                             │
 │  Chassis_Task (2ms)  ──? Allocate_Can_Msg() ──? CAN2 ──? 轮电机 M3508 x4    │
 │                        ──? Allocate_Can_Msg() ──? CAN2 ──? 超级电容写入      │
 └─────────────────────────────────────────────────────────────────────────────┘
                                         │
                                         │ 物理 CAN 总线 (CAN2)
                                         ▼
                         ┌──────────────────────────────┐
                         │  轮电机 M3508 x4 (0x201~204) │
                         │  超级电容 CAP (0x130)         │
                         └──────────────────────────────┘
```

---

## 十一、源代码文件索引


| 功能           | 文件路径                                                      |
| ------------ | --------------------------------------------------------- |
| 云台 CAN 发送/接收 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c`          |
| 云台 CAN ID 定义 | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h`          |
| 云台主任务        | `gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c`           |
| 云台→底盘发送任务    | `gimbal_board/Src/freertos.c` (freertos 中创建)              |
| 云台 CAN 定时器   | `gimbal_board/Src/freertos.c` (freertos 中创建)              |
| DM IMU 驱动    | `gimbal_board/Gimbal_Board/BSP/bsp_DMIMU.c`               |
| 云台 CAN 外设初始化 | `gimbal_board/Src/can.c`                                  |
| 底盘 CAN 发送/接收 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.c`       |
| 底盘 CAN ID 定义 | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.h`       |
| 底盘主任务        | `chassis_board/Chassis_Board/Tasks/Chassis_Task.c`        |
| 底盘 CAN 定时器   | `chassis_board/Src/freertos.c` (freertos 中创建)             |
| 超级电容驱动       | `chassis_board/Chassis_Board/BSP/bsp_cap.c`               |
| 底盘 CAN 外设初始化 | `chassis_board/Src/can.c`                                 |
| 功率限制器        | `chassis_board/Chassis_Board/App/Chassis_Power_Limitor.c` |


