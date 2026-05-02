---
name: CAN拨盘小pitch和功率计修改
overview: 将云台板拨盘和小Pitch移至CAN2，底盘板功率计接收从CAN2改至CAN1，并修正相关注释和过滤器配置。
todos:
  - id: gimbal-h-comment
    content: 云台板 bsp_can_gimbal.h - 修正 DIAL/SMALL_PITCH ID 注释（CAN1→CAN2）
    status: completed
  - id: chassis-h-comment
    content: 底盘板 bsp_can_chassis.h - 修正 POWER_METER 注释（CAN2→CAN1）
    status: completed
  - id: chassis-c-rx
    content: 底盘板 bsp_can_chassis.c - 功率计接收从 CAN2 移至 CAN1
    status: completed
  - id: chassis-c-filter
    content: 底盘板 bsp_can_chassis.c - CAN2 过滤器修正（移除上下板通讯ID）
    status: completed
  - id: can-report-doc
    content: 文档 CAN_report_0501.md - 更新底盘板功率计 CAN 通道标注
    status: in_progress
isProject: false
---

# CAN 修改计划：云台拨盘/小Pitch → CAN2，底盘功率计 → CAN1

## 一、问题总览

根据用户需求：

1. **云台板**：`DIAL` (拨盘) 和 `SMALL_PITCH` (小Pitch MF6015) 需要在 **CAN2**
2. **底盘板**：`POWER_METER` (功率计) 接收需要在 **CAN2**

对照 `CAN_report_0501.md` 文档（规定均在 CAN1），实际代码与文档一致。但用户实际需求是 CAN2。因此需要做以下修改。

---

## 二、修改清单

### 步骤 1：云台板 — `bsp_can_gimbal.h` 注释修正

**文件**：`[gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h](gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h)`


| 行号  | 当前注释                                     | 修改为                                            |
| --- | ---------------------------------------- | ---------------------------------------------- |
| 21  | `DIAL_RecID 0x141 // CAN1, 拨盘电机反馈`       | `DIAL_RecID 0x141 // CAN2, 拨盘电机反馈`             |
| 28  | `SMALL_PITCH_MF6015_RecID 0x142 // CAN1` | `SMALL_PITCH_MF6015_RecID 0x142 // CAN2`       |
| 29  | `SMALL_PITCH_MF6015_CMD 0x142 // CAN1`   | `SMALL_PITCH_MF6015_CMD 0x142 // CAN2`         |
| 38  | `DIAL_TransID 0x141 // CAN2,拨盘电机`        | `DIAL_TransID 0x141 // CAN2,拨盘电机`（已是CAN2，确认不变） |


### 步骤 2：云台板 — `bsp_can_gimbal.c` 确认接收处理位置

**文件**：`[gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c](gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.c)`

当前代码中 `DIAL_RecID` (lines 281-290) 和 `SMALL_PITCH_MF6015_RecID` (lines 275-280) 均已在 **CAN2 接收处理块** (`if (hcan == &hcan2)`) 中。

**结论**：代码路由已正确（拨盘和小Pitch在CAN2），无需修改代码逻辑，仅修正注释。

### 步骤 3：底盘板 — `bsp_can_chassis.h` 注释修正

**文件**：`[chassis_board/Chassis_Board/BSP/bsp_can_chassis.h](chassis_board/Chassis_Board/BSP/bsp_can_chassis.h)`


| 行号  | 当前注释                                    | 修改为                                                         |
| --- | --------------------------------------- | ----------------------------------------------------------- |
| 33  | `POWER_METER_RecID 0x123 // CAN2,功率计`   | `POWER_METER_RecID 0x123 // CAN1,功率计`                       |
| 38  | `POWER_METER_TransID 0x124 // CAN2,功率计` | `POWER_METER_TransID 0x124 // CAN1,功率计`（代码中已用&hcan1，此处修正注释） |


### 步骤 4：底盘板 — `bsp_can_chassis.c` 功率计接收移至 CAN1

**文件**：`[chassis_board/Chassis_Board/BSP/bsp_can_chassis.c](chassis_board/Chassis_Board/BSP/bsp_can_chassis.c)`

**当前状态**：`POWER_METER_RecID` (0x123) 在 **CAN2 接收处理块** (lines 248-252)。

**修改**：将此 case 从 CAN2 接收块移至 **CAN1 接收处理块**。

**操作**：

1. 在 CAN1 接收处理块 (`if (hcan == &hcan1)`, lines 184-225) 末尾，`}` 之前，新增 `POWER_METER_RecID` case：

```c
case POWER_METER_RecID:
{
    real_power = EnergyDataToFloat(rx_data);
    break;
}
```

1. 从 CAN2 接收处理块 (`if (hcan == &hcan2)`, lines 226-258) 中**删除** `POWER_METER_RecID` case。

### 步骤 5：底盘板 — `bsp_can_chassis.c` 过滤器配置修正

**文件**：`[chassis_board/Chassis_Board/BSP/bsp_can_chassis.c](chassis_board/Chassis_Board/BSP/bsp_can_chassis.c)`

当前 CAN2 过滤器 (lines 79-91) 配置了 IDLIST 模式，接收 `GIMBAL_TO_CHASSIS_FIRST_RecID` 和 `GIMBAL_TO_CHASSIS_SECOND_RecID`：

```c:83:86:chassis_board/Chassis_Board/BSP/bsp_can_chassis.c
    can_filter.FilterIdHigh = (GIMBAL_TO_CHASSIS_FIRST_RecID << 5);
    can_filter.FilterIdLow = (GIMBAL_TO_CHASSIS_SECOND_RecID << 5);
    can_filter.FilterMaskIdHigh = (WHEEL1_M3508_RecID << 5);
    can_filter.FilterMaskIdLow = (BIG_YAW_DM6006_RecID << 5);
```

由于 `GIMBAL_TO_CHASSIS_FIRST/SECOND_RecID` 已在 CAN1 接收，这些 ID 不应出现在 CAN2 过滤器中。修改为仅接收 CAN2 实际需要的 ID：

```c
    can_filter.FilterIdHigh = (WHEEL1_M3508_RecID << 5);    // 0x201
    can_filter.FilterIdLow = (WHEEL2_M3508_RecID << 5);    // 0x202
    can_filter.FilterMaskIdHigh = (WHEEL3_M3508_RecID << 5); // 0x203
    can_filter.FilterMaskIdLow = (CAP_RecID << 5);          // 0x130
```

同时，CAN1 应增加接收 `POWER_METER_RecID` 的能力。当前 CAN1 过滤器使用 IDMASK 全接收模式，无需修改过滤器配置，但接收路由已改为 CAN1 即可生效。

### 步骤 6：更新 `CAN_report_0501.md` 文档

**文件**：`[docs/CAN_report_0501.md](docs/CAN_report_0501.md)`

更新底盘板 CAN ID 表（7.2 节），将 `POWER_METER_RecID` 从 CAN2 改为 CAN1。

---

## 三、修改文件汇总


| 优先级 | 文件                                                  | 修改内容                                     |
| --- | --------------------------------------------------- | ---------------------------------------- |
| 高   | `gimbal_board/Gimbal_Board/BSP/bsp_can_gimbal.h`    | 修正 DIAL/SMALL_PITCH 相关 ID 的注释（CAN1→CAN2） |
| 高   | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.h` | 修正 POWER_METER 注释（CAN2→CAN1）             |
| 高   | `chassis_board/Chassis_Board/BSP/bsp_can_chassis.c` | 功率计接收从 CAN2 移至 CAN1 + 修正过滤器              |
| 低   | `docs/CAN_report_0501.md`                           | 更新底盘板 ID 表中功率计的 CAN 通道标注                 |


---

## 四、修改后目标

```

  云台板 CAN1: 大Pitch(DM4340) + 小Yaw(GM6020) + 大Yaw(DM6006) + 上下板通信
  云台板 CAN2: 拨盘(0x141) + 小Pitch(0x142) + 摩擦轮(M3508)
  底盘板 CAN1: 大Yaw(0x300) + 上下板通讯(0x10/0x11) 
  底盘板 CAN2: 轮电机(M3508) + 超电(CAP) + 功率计(0x123)
```

