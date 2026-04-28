---
name: 云台大Pitch+小Pitch修改终版计划
overview: 审核修正终版：新增大Pitch(DM4340)控制和替换小Pitch为MF6015(LK协议)。修正了字段命名、类型替换、限制器移除、大Pitch函数合并入原有函数等问题。
todos:
  - id: motor-h
    content: "motor.h: 修改motor_measure_pitch类型 + 添加DM_big_pitch_motor extern"
    status: pending
  - id: motor-c
    content: "motor.c: 修改motor_measure_pitch实例化类型 + 添加DM_big_pitch_motor实例"
    status: pending
  - id: can-gimbal-h
    content: "bsp_can_gimbal.h: 添加CAN ID宏 + 枚举扩展 + 函数声明"
    status: pending
  - id: can-gimbal-c
    content: "bsp_can_gimbal.c: 队列宏/消息实例 + Can_Msg_Init + CAN1/CAN2回调 + Allocate_Can_Msg分支 + 4个新函数"
    status: pending
  - id: gimbal-task
    content: "Gimbal_Task.c: 新增extern/PID宏/初始化/数据更新/使能检查/电机控制/安全处理/CAN发送"
    status: pending
  - id: detect-h
    content: "detect_task.h: 枚举末尾追加两个TOE"
    status: pending
  - id: detect-c
    content: "detect_task.c: set_item数组追加两项"
    status: pending
isProject: false
---

# 云台大Pitch + 小Pitch 修改计划 — 审核修正终版v3.0

## 一、修改目标概述

1. **大Pitch (DM4340)**：新增，CAN1 + MIT协议，独立控制
2. **小Pitch (MF6015)**：替换原小Pitch (GM6020)，协议从 DJI 改为 LK，接收/发送ID均为 `0x142`，变量名 `motor_measure_pitch` 保持不变，但类型从 `motor_measure_t` 改为 `LK_motor_measure_t`

### CAN通道布局（修改后）


| CAN通道    | 电机                       | 协议      | 接收ID      | 发送ID      |
| -------- | ------------------------ | ------- | --------- | --------- |
| CAN1     | Big Yaw (DM6006)         | MIT     | 0x300     | 0x01      |
| **CAN1** | **Big Pitch (DM4340)**   | **MIT** | **0x301** | **0x02**  |
| CAN1     | Small Yaw (GM6020)       | DJI     | 0x205     | 0x1FE     |
| CAN1     | Fric1 (3508)             | DJI     | 0x201     | 0x200     |
| CAN1     | Fric2 (3508)             | DJI     | 0x202     | 0x200     |
| CAN2     | Big IMU                  | -       | 0x16      | 0x15      |
| **CAN2** | **Small Pitch (MF6015)** | **LK**  | **0x142** | **0x142** |
| CAN2     | Dial (MF4010)            | LK      | 0x141     | 0x141     |


### 关键行为说明

- **大Pitch**：旋转基座，折叠（`BIG_PITCH_ANGLE_MIN = -60°`）和展开（`BIG_PITCH_ANGLE_MAX = +30°`）
- **控制方式**：Q键边沿触发切换折叠/展开状态
- **自瞄行为**：不受影响，自瞄控制小pitch、小yaw、大yaw（DM_big_yaw_motor）；大Pitch独立控制
- **DM4340 state字段**：与DM6006定义完全一致

---

## 二、涉及文件清单


| 序号  | 文件路径                                             | 操作     |
| --- | ------------------------------------------------ | ------ |
| 1   | `gimbal_board/Gimbal_Board/App/motor.h`          | 修改     |
| 2   | `gimbal_board/Gimbal_Board/App/motor.c`          | 修改     |
| 3   | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h` | 修改     |
| 4   | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c` | 修改     |
| 5   | `gimbal_board/Gimbal_Board/Tasks/Gimbal_Task.c`  | 修改     |
| 6   | `gimbal_board/Gimbal_Board/Tasks/detect_task.h`  | **修改** |


---

## 三、详细修改内容

### 3.1 `motor.h` — 两处修改

**修改1**：`motor_measure_pitch` 类型变更（line 171）

```diff
- extern motor_measure_t motor_measure_pitch;     // pitch 6020
+ extern LK_motor_measure_t motor_measure_pitch;  // pitch MF6015 (LK协议)
```

**修改2**：`DM_big_pitch_motor` 全局实例声明（在 `DM_big_yaw_motor` 之后）

```c
extern DM_motor_data_t DM_big_pitch_motor;  // 大Pitch DM4340，复用DM_motor_data_t
```

---

### 3.2 `motor.c` — 两处修改

**修改1**：`motor_measure_pitch` 实例化类型变更（line 15）

```diff
- motor_measure_t motor_measure_pitch = {0};     // pitch 6020
+ LK_motor_measure_t motor_measure_pitch = {0}; // pitch MF6015 (LK协议)
```

**修改2**：`DM_big_pitch_motor` 全局实例（在 `DM_big_yaw_motor` 之后）

```c
DM_motor_data_t DM_big_pitch_motor = {0};  // 大Pitch DM4340
```

---

### 3.3 `bsp_can_gimbal.h` — 三处修改

**修改1**：新增CAN ID宏定义（在文件适当位置）

```c
// 大Pitch (DM4340) - CAN1, MIT协议
#define BIG_PITCH_DM4340_RecID  0x301    // 接收ID (CAN1)
#define BIG_PITCH_DM4340_CMD     0x02    // 发送ID (CAN1)

// 小Pitch (MF6015) - CAN2, LK协议，替换原PITCH_GM6020_RecID(0x206, CAN1)
#define SMALL_PITCH_MF6015_RecID  0x142  // 接收ID (CAN2)
#define SMALL_PITCH_MF6015_CMD    0x142  // 发送ID (CAN2)
```

**修改2**：`CAN_CMD_ID` 枚举体扩展

```diff
 typedef enum {
     CAN_GIMBAL_TO_CHASSIS_FIRST_CMD,
     CAN_GIMBAL_TO_CHASSIS_SECOND_CMD,
     CAN_GIMBAL_TO_CHASSIS_THIRD_CMD,
     CAN_BIG_YAW_CMD,
     CAN_SMALL_YAW_AND_PITCH_CMD,
     CAN_FRIC_CMD,
     CAN_DIAL_CMD,
+    CAN_BIG_PITCH_CMD,   // 新增：大Pitch MIT发送
+    CAN_SMALL_PITCH_CMD,  // 新增：小Pitch LK发送
 } CAN_CMD_ID;
```

**修改3**：新增函数声明

```c
void Ctrl_DM_BigPitch(float _pos, float _vel, float _KP, float _KD, float _torq);
void enable_DM_BigPitch(void);
```

---

### 3.4 `bsp_can_gimbal.c` — 多处修改

#### 3.4.1 新增队列宏和消息实例

```c
#define BIG_PITCH_SEND_QUEUE CAN1_send_queue
#define SMALL_PITCH_SEND_QUEUE CAN2_send_queue

CanTxMsgTypeDef big_pitch_send_msg;     // 大Pitch MIT
CanTxMsgTypeDef small_pitch_send_msg;   // 小Pitch LK
```

#### 3.4.2 `Can_Msg_Init()` — 新增缓冲区初始化

在 `buffer_list` 数组中添加：

```c
{&big_pitch_send_msg, BIG_PITCH_DM4340_CMD},
{&small_pitch_send_msg, SMALL_PITCH_MF6015_CMD},
```

- Big Pitch 初始化：同 Big Yaw（MIT零位转换，使用 `BIG_YAW_DM6006_TransID` 相同的逻辑分支）
- Small Pitch 初始化：`memset(data, 0, 8)`

#### 3.4.3 CAN1接收回调 — 新增大Pitch分支 + 注释原小Pitch

**注释掉**原 CAN1 上 `case PITCH_GM6020_RecID:` 分支（line 212-216），**替换为**：

```c
case BIG_PITCH_DM4340_RecID:
{
    DM_big_pitch_motor.id = (rx_data[0]) & 0x0F;
    DM_big_pitch_motor.state = (rx_data[0]) >> 4;
    DM_big_pitch_motor.p_int = (rx_data[1] << 8) | rx_data[2];
    DM_big_pitch_motor.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    DM_big_pitch_motor.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];

    DM_big_pitch_motor.pos = uint_to_float(DM_big_pitch_motor.p_int, P_MIN, P_MAX, 16);
    DM_big_pitch_motor.vel = uint_to_float(DM_big_pitch_motor.v_int, V_MIN, V_MAX, 12);
    DM_big_pitch_motor.toq = uint_to_float(DM_big_pitch_motor.t_int, T_MIN, T_MAX, 12);

    detect_hook(BIG_PITCH_DM4340_TOE);
    break;
}
```

> 注：位解析与现有 Big Yaw (DM6006) 完全一致，state字段含义相同。

#### 3.4.4 CAN2接收回调 — 新增小Pitch LK协议分支

在 CAN2 switch 中、DIAL_RecID 之前添加：

```c
case SMALL_PITCH_MF6015_RecID:
{
    get_motor_measure_LK(&motor_measure_pitch, rx_data);
    detect_hook(SMALL_PITCH_MF6015_TOE);
    break;
}
```

> 注：`motor_measure_pitch` 已改为 `LK_motor_measure_t` 类型，`get_motor_measure_LK` 可正确填充 `ecd` 字段供 `gimbal_pitch_motor.ENC_angle_now` 计算使用。`motor_measure_pitch.speed`（dps）不参与小Pitch控制（`gimbal_pitch_motor.INS_speed_now` 来自IMU数据）。

#### 3.4.5 `Allocate_Can_Msg()` — 新增大Pitch发送分支 + 新增小Pitch LK发送分支

> 注意：`Ctrl_DM_Motor()` 保持不变，用于发送 Big Yaw (DM6006) 的 CAN 消息（调用 `CAN_BIG_YAW_CMD`）。

```c
case CAN_BIG_PITCH_CMD:
{
    for (int i = 0; i < 4; i++)
    {
        big_pitch_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF;
        big_pitch_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;
    }
    xQueueSend(BIG_PITCH_SEND_QUEUE, &big_pitch_send_msg, 0);
    break;
}
case CAN_SMALL_PITCH_CMD:
{
    // MF6015 LK协议：DATA[0]=命令字节 0xA1（位置模式），DATA[4]/DATA[5]=转矩电流 iqControl
    int16_t iq = data1;
    small_pitch_send_msg.data[0] = 0xA1;                    // 命令字节：0xA1=位置模式
    small_pitch_send_msg.data[1] = 0x00;
    small_pitch_send_msg.data[2] = 0x00;
    small_pitch_send_msg.data[3] = 0x00;
    small_pitch_send_msg.data[4] = (uint8_t)(iq & 0xFF);          // iqControl 低字节
    small_pitch_send_msg.data[5] = (uint8_t)((iq >> 8) & 0xFF);    // iqControl 高字节
    small_pitch_send_msg.data[6] = 0x00;
    small_pitch_send_msg.data[7] = 0x00;
    xQueueSend(SMALL_PITCH_SEND_QUEUE, &small_pitch_send_msg, 0);
    break;
}
```

#### 3.4.6 新增 `Ctrl_DM_BigPitch()`

```c
void Ctrl_DM_BigPitch(float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint16_t data1, data2, data3, data4;
    data1 = pos_tmp;
    data2 = ((vel_tmp >> 4) << 8) | ((vel_tmp & 0x0F) << 4 | (kp_tmp >> 8));
    data3 = ((kp_tmp & 0xFF) << 8) | (kd_tmp >> 4);
    data4 = ((kd_tmp & 0x0F) << 12) | tor_tmp;

    Allocate_Can_Msg(data1, data2, data3, data4, CAN_BIG_PITCH_CMD);
}
```

> 注：字节打包格式与现有 `Ctrl_DM_Motor()` 完全一致。

#### 3.4.7 新增 `enable_DM_BigPitch()`

```c
void enable_DM_BigPitch(void)
{
    uint8_t TX_Data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Tx_Msg;
    Tx_Msg.StdId = 0x000;
    Tx_Msg.IDE = CAN_ID_STD;
    Tx_Msg.RTR = CAN_RTR_DATA;
    Tx_Msg.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, TX_Data, &send_mail_box);
}
```

---

### 3.5 `Gimbal_Task.c` — 控制逻辑

#### 3.5.1 顶部新增

```c
extern DM_motor_data_t DM_big_pitch_motor;  // 新增

static void Big_Pitch_Motor_Control(void);

// 大Pitch状态标志：0=折叠, 1=展开
static uint8_t big_pitch_up_flag = 0;
```

#### 3.5.2 PID参数宏定义

**文件**: `Gimbal_Task.h`（与现有 `BIG_YAW_MOTOR_`*、`PITCH_MOTOR_*` 等宏定义放同一位置）

```c
// ============== 大Pitch DM4340 PID参数 ==============
#define BIG_PITCH_SPEED_KP        0.1f
#define BIG_PITCH_SPEED_KI        0.0001f
#define BIG_PITCH_SPEED_KD        0.0f
#define BIG_PITCH_SPEED_MAX_OUT   10.0f
#define BIG_PITCH_SPEED_MAX_IOUT  0.1f

#define BIG_PITCH_ANGLE_KP        20.0f
#define BIG_PITCH_ANGLE_KI        0.0f
#define BIG_PITCH_ANGLE_KD        10.0f
#define BIG_PITCH_ANGLE_MAX_OUT   150.0f
#define BIG_PITCH_ANGLE_MAX_IOUT  0.0f

#define BIG_PITCH_ANGLE_MAX       30.0f   // 展开最大角度(度)
#define BIG_PITCH_ANGLE_MIN     -60.0f   // 折叠最大角度(度)

// ============== 小Pitch MF6015 编码器适配 ==============
#define MF6015_ENC_RESOLUTION     65535   // 16位编码器，满量程
#define MF6015_ENC_TO_DEGREE     (360.0f / MF6015_ENC_RESOLUTION)  // 编码器弧度转角度系数

// 跳变阈值：距离编码器0点和满量程点的边界值（电机实际编码器值，非角度值）
#define MF6015_ECD_ANGLE_MAX     60000   // 编码器上限阈值
#define MF6015_ECD_ANGLE_MIN     5000    // 编码器下限阈值
```

> 注：与现有代码风格保持一致，宏定义统一放在 `Gimbal_Task.h` 文件中。

#### 3.5.3 `Gimbal_Task()` 主循环修改

**初始化部分**（在 `Gimbal_Motor_Control_Init();` 之后）：

> 大Pitch PID初始化直接写在 `Gimbal_Motor_Control_Init()` 函数**末尾**：

```c
// 在 Gimbal_Motor_Control_Init() 末尾添加：
PID_init(&DM_big_pitch_motor.speed_pid, PID_POSITION,
         BIG_PITCH_SPEED_KP, BIG_PITCH_SPEED_KI, BIG_PITCH_SPEED_KD,
         BIG_PITCH_SPEED_MAX_OUT, BIG_PITCH_SPEED_MAX_IOUT);
PID_init(&DM_big_pitch_motor.nav_angle_pid, PID_POSITION,  // 复用 nav_angle_pid
         BIG_PITCH_ANGLE_KP, BIG_PITCH_ANGLE_KI, BIG_PITCH_ANGLE_KD,
         BIG_PITCH_ANGLE_MAX_OUT, BIG_PITCH_ANGLE_MAX_IOUT);
```

**循环内**：

```c
Check_Big_Yaw_DM_Auto_Enable(); // 包含两个达妙电机大Yaw和大Pitch，且都在CAN1
// enable_DM_BigPitch 和enable_DM需要注意ID的不同和CAN通道
if (!DM_big_pitch_motor.state)  // 大Pitch自动使能检查，合并到原Check函数逻辑
{
    uint8_t enable_send_count = 5;
    while (enable_send_count > 0)
    {
        enable_DM_BigPitch();
        vTaskDelay(1);
        enable_send_count--;
    }
}

Gimbal_Data_Update();

// ========== 在 Gimbal_Data_Update() 末尾添加大Pitch数据更新 ==========
// 将DM4340原始数据转换为角度/速度单位（使用Gimbal_Task.h中定义的RAD_TO_DEGREE）
DM_big_pitch_motor.INS_angle_now = DM_big_pitch_motor.pos * RAD_TO_DEGREE;
DM_big_pitch_motor.INS_speed_now = DM_big_pitch_motor.vel * RAD_TO_DEGREE;
// ====================================================================

// ========== 将小Pitch编码器跳变处理替换为MF6015版本 ==========
// 替换原有的 GM6020 (13位, 0-8191) 跳变处理为 MF6015 (16位, 0-65535) 版本
if (motor_measure_pitch.ecd > MF6015_ECD_ANGLE_MAX)
    gimbal_pitch_motor.ENC_angle_now = (motor_measure_pitch.ecd - MF6015_ENC_RESOLUTION) * MF6015_ENC_TO_DEGREE;
else if (motor_measure_pitch.ecd < MF6015_ECD_ANGLE_MIN)
    gimbal_pitch_motor.ENC_angle_now = (motor_measure_pitch.ecd + MF6015_ENC_RESOLUTION) * MF6015_ENC_TO_DEGREE;
else
    gimbal_pitch_motor.ENC_angle_now = motor_measure_pitch.ecd * MF6015_ENC_TO_DEGREE;
// ================================================================

gimbal_control.gimbal_mode = Gimbal_Mode_Update();

// 小Pitch离线保护：离线时直接进入安全模式
if (toe_is_error(SMALL_PITCH_MF6015_TOE))
{
    gimbal_control.gimbal_mode = GIMBAL_SAFE;
}

// Q键边沿触发：切换大Pitch折叠/展开状态
static uint8_t last_key_q = 0;
if ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) && !last_key_q)
{
    big_pitch_up_flag = big_pitch_up_flag ? 0 : 1;
}
last_key_q = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) ? 1 : 0;

Call_Gimbal_Mode_Handler(gimbal_control.gimbal_mode);
Big_Pitch_Motor_Control();  // 新增
```

**CAN发送部分**：

```c
Ctrl_DM_Motor(0, 0, 0, 0, DM_big_yaw_motor.target_current);
Ctrl_DM_BigPitch(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    DM_big_pitch_motor.target_current
);
// Small Yaw: CAN1, DJI协议，原有发送不变
Allocate_Can_Msg(gimbal_small_yaw_motor.give_current, 0, 0, 0, CAN_SMALL_YAW_AND_PITCH_CMD);
// Small Pitch: CAN2, LK协议，独立发送，
Allocate_Can_Msg(gimbal_pitch_motor.give_current, 0, 0, 0, CAN_SMALL_PITCH_CMD);
```

#### 3.5.4 `Big_Pitch_Motor_Control()` — 核心控制

> 注：使用 `nav_angle_pid` 作为大Pitch的角度环PID（`DM_motor_data_t` 中 `nav_angle_pid` 在大Pitch控制中不会被使用，可直接复用）。

```c
static void Big_Pitch_Motor_Control(void)
{
    if (gimbal_control.gimbal_mode == GIMBAL_SAFE)
    {
        DM_big_pitch_motor.target_current = 0;
        return;
    }

    fp32 target_angle = (big_pitch_up_flag == 1) ? BIG_PITCH_ANGLE_MAX : BIG_PITCH_ANGLE_MIN;

    // 角度环（复用 nav_angle_pid 作为角度环PID）
    PID_calc(&DM_big_pitch_motor.nav_angle_pid,
             DM_big_pitch_motor.INS_angle_now,
             target_angle);

    // 速度环（输出作为力矩）
    PID_calc(&DM_big_pitch_motor.speed_pid,
             DM_big_pitch_motor.INS_speed_now,
             DM_big_pitch_motor.nav_angle_pid.out);

    DM_big_pitch_motor.target_current = DM_big_pitch_motor.speed_pid.out;
}
```

#### 3.5.5 `gimbal_safe_handler()` — 扩展

```c
DM_big_pitch_motor.target_current = 0;
PID_clear(&DM_big_pitch_motor.speed_pid);
PID_clear(&DM_big_pitch_motor.nav_angle_pid);  // 复用 nav_angle_pid 作为角度环
// 保留原有的gimbal_pitch_motor/gimbal_small_yaw_motor/DM_big_yaw_motor清零代码
```

---

### 3.6 `detect_task.h`

在 `enum errorList` 末尾添加：

```c
BIG_PITCH_DM4340_TOE,    // 新增
SMALL_PITCH_MF6015_TOE,  // 新增
```

---

### 3.7 `detect_task.c` — 扩展 `set_item` 数组

```c
uint16_t set_item[ERROR_LIST_LENGHT][3] = {
    {30, 40, 15},  // DBUS
    {5, 40, 15},   // DM_IMU
    {10, 30, 14},  // DIAL
    {7, 3, 7},     // BOARD_GYRO
    {7, 5, 7},     // BOARD_ACCEL
    {50, 50, 14},  // NUC_DATA
    {5, 40, 10},   // BIG_PITCH_DM4340   新增
    {5, 40, 10},   // SMALL_PITCH_MF6015 新增
};
```

---

## 四、大Pitch控制策略

```
big_pitch_up_flag = 0 (折叠态) -> 目标角度 = BIG_PITCH_ANGLE_MIN (-60度)
big_pitch_up_flag = 1 (展开态) -> 目标角度 = BIG_PITCH_ANGLE_MAX (+30度)

Q键边沿触发切换，当前仅手动控制。
未来可扩展：导航模式下根据导航信息自动切换折叠/展开状态。
```

- 大Pitch独立于各云台模式的固定位置控制
- 大Pitch展开/折叠不影响小pitch和小yaw的瞄准控制
- 自瞄行为不变：自瞄控制小pitch、小yaw、大yaw（DM_big_yaw_motor）
- 大Pitch不受自瞄控制，独立保持当前折叠/展开状态

---

## 五、优先级


| 优先级 | 文件                                      |
| --- | --------------------------------------- |
| 高   | `motor.h` (类型修改 + extern)               |
| 高   | `motor.c` (实例化修改 + 新增)                  |
| 高   | `bsp_can_gimbal.h` (CAN ID + 枚举 + 函数声明) |
| 高   | `bsp_can_gimbal.c` (CAN收发核心)            |
| 高   | `detect_task.h` (枚举扩展)                  |
| 高   | `detect_task.c` (set_item扩展)            |
| 中   | `Gimbal_Task.c` (控制逻辑)                  |


---

## 六、测试验证清单

1. Big Pitch CAN1通信正常（观察 `DM_big_pitch_motor.pos` 值随电机转动变化）
2. Q键折叠/展开状态切换正常
3. 折叠目标角度 = -60度，展开目标角度 = +30度
4. 失能模式正常（力矩归零，PID清零）
5. 小Pitch (MF6015) CAN2通信正常
6. 自瞄控制小pitch、小yaw、大yaw正常（与原有行为一致）
7. 大Pitch折叠/展开时，自瞄对小pitch、小yaw、大yaw的控制不受影响
8. 其他电机功能不受影响

