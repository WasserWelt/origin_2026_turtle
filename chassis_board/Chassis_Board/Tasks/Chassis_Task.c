/****************************************************************
 * @file: 	Chassis_Task.c
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵舵轮底盘任务
 * @attention:
 ******************************************************************/
#include "Chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_cap.h"
#include "arm_math.h"
#include "motor.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "INS_Task.h"
/*************************底盘模式枚举体****************************/
typedef enum
{
	FOLLOW_GIMBAL, // 底盘跟随云台移动模式
	ROTATE,		   // 小陀螺移动模式
	STEER_LOCK,    // 舵轮自锁模式（相邻两个舵轮
	CHASSIS_SAFE   // 失能模式
} chassis_mode_t;
/*******************************************************************/

/**********************在chassis_task.c中的全局变量及常量区定义底盘模式表，在不同的底盘模式下设置对应底盘控制逻辑***************************/
typedef struct
{
	chassis_mode_t mode;   // 底盘模式
	void (*handler)(void); // 不同底盘模式对应的处理函数
} chassis_command_t;
/*******************************************************************/
typedef struct // 底盘坐标系目标速度结构体
{
	fp32 vx;
	fp32 vy;
	fp32 vw;
} chassis_speed_t;

typedef struct // 底盘控制参数结构体
{
	chassis_speed_t chassis_target_speed;

	fp32 chassis_follow_gimbal_angle;
	fp32 chassis_power_limit;
	fp32 init_chassis_power;
	bool_t chassis_follow_gimbal_zerochange;

	pid_type_def chassis_follow_gimbal_pid;
} chassis_control_t;

/********************电机功率控制参数**********************/
const fp32 toque_coefficient = 1.99688994e-6f;
const fp32 k1 = 1.23e-07;  // k1
const fp32 k2 = 1.453e-07; // k2
const fp32 constant = 4.081f;
/*****************************************************/

/****************************底盘模式处理函数声明，原因是使用函数名给函数指针赋值之前，该函数必须已经被声明***************************************/
void chassis_follow_gimbal_handler(void);
void chassis_rotate_handler(void);
void chassis_safe_handler(void);
/************************全局变量及常量区*****************************/
chassis_command_t chassis_commands[] = {{FOLLOW_GIMBAL, chassis_follow_gimbal_handler}, {ROTATE, chassis_rotate_handler}, {CHASSIS_SAFE, chassis_safe_handler}}; // 初始化底盘控制命令数组
chassis_motor_t chassis_m3508[4] = {0};
chassis_control_t chassis_control = {0};
chassis_real_speed_t chassis_real_speed;
chassis_mode_t chassis_mode = CHASSIS_SAFE;
/*******************************************************************/

static void Chassis_Motor_Pid_Init(void)
{
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_init(&chassis_m3508[i].pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	PID_init(&chassis_control.chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
}

static void Chassis_Motor_Data_Update(void)
{
	static float lpf_ratio = 0.0f; // 一阶低通滤波参数，越大响应越滞后
	static fp32 chassis_m3508_last_speed[4];
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_m3508[i].speed = motor_measure_chassis[i].speed_rpm;

		chassis_m3508[i].speed = (chassis_m3508[i].speed) * (1 - lpf_ratio) + chassis_m3508_last_speed[i] * lpf_ratio;
		chassis_m3508_last_speed[i] = motor_measure_chassis[i].speed_rpm;
	}
}

/**
 * @description: 更新底盘模式
 * @return 底盘当前模式
 */
static chassis_mode_t Chassis_Mode_Update()
{
	bool_t rc_ctrl_follow_gimbal = ((rc_ctrl.rc.s[1] == RC_SW_MID) && (rc_ctrl.rc.ch[4] < 500) && (rc_ctrl.rc.ch[4] > -500)); // 是否满足遥控器控制时底盘跟随云台模式，下面以此类推
	bool_t rc_ctrl_rotate = ((rc_ctrl.rc.s[1] == RC_SW_MID) && !rc_ctrl_follow_gimbal);
	bool_t rc_ctrl_safe = ((rc_ctrl.rc.s[1] == RC_SW_DOWN) || toe_is_error(DBUS_TOE));
	bool_t nav_follow_gimbal = ((AutoAim_Data_Receive.rotate == 0) && (rc_ctrl.rc.s[1] == RC_SW_UP)); // 是否满足导航模式下底盘跟随云台模式，下面以此类推
	bool_t nav_rotate = ((AutoAim_Data_Receive.rotate != 0) && (rc_ctrl.rc.s[1] == RC_SW_UP));
	bool_t nav_safe = ((Game_Status.game_progress != 4) && (rc_ctrl.rc.s[1] == RC_SW_UP));

	if (rc_ctrl_safe)
	{
		return CHASSIS_SAFE; // 失能模式的优先级最高，需要优先判断
	}
	else if (rc_ctrl_rotate || nav_rotate)
	{
		return ROTATE;
	}
	else if (rc_ctrl_follow_gimbal || nav_follow_gimbal)
	{
		return FOLLOW_GIMBAL;
	}
}

static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 *wheel0, fp32 *wheel1, fp32 *wheel2, fp32 *wheel3, chassis_mode_t mode)
{
	if (wheel0 == NULL || wheel1 == NULL || wheel2 == NULL || wheel3 == NULL || mode == CHASSIS_SAFE)
	{
		return;
	}
	*wheel0 = (+vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel1 = (+vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel2 = (-vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel3 = (-vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief  设置底盘四个3508电机的电流
 */
static void chassis_motor_current_set(chassis_mode_t mode)
{
	if (mode == CHASSIS_SAFE)
		return;
	for (uint8_t i = 0; i < 4; i++)
	{
		PID_calc(&chassis_m3508[i].pid, chassis_m3508[i].speed, chassis_m3508[i].speed_set);
		chassis_m3508[i].give_current = chassis_m3508[i].pid.out;
	}
}

static fp32 Angle_Z_Suit_ZERO_Get(fp32 Target_Angle)
{
	float angle_front_err = Limit_To_180((float)(Target_Angle - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f);

	if (angle_front_err > 135 || angle_front_err <= -135)
		return CHASSIS_FOLLOW_GIMBAL_ANGLE_BACK_ZERO;
	if (angle_front_err > 45 && angle_front_err <= 135)
		return CHASSIS_FOLLOW_GIMBAL_ANGLE_LEFT_ZERO;
	if (angle_front_err > -45 && angle_front_err <= 45)
		return CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	if (angle_front_err > -135 && angle_front_err <= -45)
		return CHASSIS_FOLLOW_GIMBAL_ANGLE_RIGHT_ZERO;
}

/**
 * @brief  以yaw轴朝向作为正方向设置底盘xy轴速度，在小陀螺模式和跟随云台模式下通用
 */
static void Set_Chassis_VxVy(fp32 yaw_nearest_zero_rad, fp32 *chassis_vx, fp32 *chassis_vy)
{
	fp32 vx, vy;
	fp32 sin_yaw = arm_sin_f32(yaw_nearest_zero_rad);
	fp32 cos_yaw = arm_cos_f32(yaw_nearest_zero_rad);
	// todo
	if (rc_ctrl.rc.s[1] == RC_SW_MID) // 遥控器控制模式
	{
		vx = ramp_control(vx, rc_ctrl.rc.ch[2] * 9, 0.7f);
		vy = ramp_control(vy, rc_ctrl.rc.ch[3] * 9, 0.7f);
	}
	else // 进入Set_FollowGimbal_VxVy函数时不是遥控器控制模式就是导航模式，所以不用再判断一次是否为导航模式
	{
		vx = -ramp_control(vx, (float)AutoAim_Data_Receive.vy * factor[0] * NAV_SPEED_FAST, 0.9f);
		vy = ramp_control(vy, (float)AutoAim_Data_Receive.vx * factor[1] * NAV_SPEED_FAST, 0.9f);
	}
	*chassis_vx = cos_yaw * vx + sin_yaw * vy;
	*chassis_vy = -sin_yaw * vx + cos_yaw * vy;
}

/**
 * @brief  设置底盘跟随云台时的底盘角速度，只在chassis_follow_gimbal_handler中调用
 */
static fp32 Set_FollowGimbal_Wz(fp32 follow_gimbal_angle, fp32 *wz)
{
	PID_calc(&chassis_control.chassis_follow_gimbal_pid, follow_gimbal_angle, 0);
	*wz = -chassis_control.chassis_follow_gimbal_pid.out;
	return *wz;
}

/**
 * @brief  设置小陀螺时的底盘角速度，只在chassis_rotate_handler中调用
 */
static fp32 Set_Rotate_Wz(fp32 *wz)
{
	if (rc_ctrl.rc.s[1] == RC_SW_MID && rc_ctrl.rc.ch[4] <= -500)
		*wz = ROTATE_WZ_MAX;
	else if (rc_ctrl.rc.s[1] == RC_SW_MID && rc_ctrl.rc.ch[4] >= 500)
		*wz = ROTATE_WZ_MIN;
	else // 导航模式下的小陀螺角速度设置
	{
		if (AutoAim_Data_Receive.uphill_flag == 2)
			*wz = 0;

		else if (health_state == HEALTH_HURT)
		{
			*wz = -(float)AutoAim_Data_Receive.rotate;
		}
		else
		{
			*wz = -(float)AutoAim_Data_Receive.rotate * ROTATE_WEAK;
		}
	}
	return *wz;
}

/**
 * @brief  跟随云台模式下的控制函数，在控制函数中解算出以yaw轴朝向为正方向的机器人x y轴速度和角速度wz
 */
static void chassis_follow_gimbal_handler(void)
{
	static fp32 chassis_follow_gimbal_zero_actual = CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;

	if (chassis_control.chassis_follow_gimbal_zerochange == TRUE)
	{
		chassis_follow_gimbal_zero_actual = Angle_Z_Suit_ZERO_Get(gimbal_m6020[0].ENC_angle);
		chassis_control.chassis_follow_gimbal_zerochange = FALSE;
	}
	chassis_control.chassis_follow_gimbal_angle = Limit_To_180((float)(((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - (uint16_t)chassis_follow_gimbal_zero_actual)) % 8192) / 8192.0f * 360.0f);
	fp32 chassis_yaw_offset_zero_rad = -(chassis_control.chassis_follow_gimbal_angle + Limit_To_180((float)(chassis_follow_gimbal_zero_actual - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f)) / 180.0f * 3.14159f;

	Set_FollowGimbal_Wz(chassis_control.chassis_follow_gimbal_angle, &chassis_control.wz);
	Set_Chassis_VxVy(chassis_yaw_offset_zero_rad, &chassis_control.vx, &chassis_control.vy);
}

/**
 * @brief  小陀螺模式下的控制函数，在控制函数中解算出以yaw轴朝向为正方向的机器人x y轴速度和角速度wz
 */
static void chassis_rotate_handler(void)
{
	chassis_control.chassis_follow_gimbal_zerochange = TRUE; // 进入小陀螺模式当前底盘零点会切换

	chassis_control.chassis_follow_gimbal_angle = Limit_To_180((float)((uint16_t)((uint16_t)gimbal_m6020[0].ENC_angle - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO + 8192  - chassis_control.wz * ROTATE_MOVE_FF) % 8192) / 8192.0f * 360.0f);
	fp32 rotate_yaw_rad = -(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f; // 小陀螺模式下当前零点距离yaw轴的弧度差

	Set_Rotate_Wz(&chassis_control.wz);
	Set_Chassis_VxVy(rotate_yaw_rad, &chassis_control.vx, &chassis_control.vy);
}

/**
 * @brief  失能模式下的控制函数，对底盘电机电流置零
 */
static void chassis_safe_handler(void)
{
	chassis_control.chassis_follow_gimbal_zerochange = TRUE; // 进入失能模式当前底盘零点可能会切换，模式切换到底盘跟随云台后需要重新寻找零点
	chassis_control.chassis_follow_gimbal_pid.Iout = 0;		 // 清零iout，防止积分饱和
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_m3508[i].give_current = 0;
		chassis_m3508[i].pid.Iout = 0; // 清零iout，防止积分饱和
	}
}

/**
 * @brief  根据不同底盘模式执行对应底盘控制函数，给以yaw轴朝向为正方向的机器人x y轴速度和wz轴速度赋值
 *         注意：1.在小陀螺模式和底盘跟随模式下，xy轴的速度解算逻辑相同，角速度解算逻辑不同
 * 				 2.失能模式下，直接对底盘电机电流值赋0，在task的while(1)中后续调用chassis_motor_current_set函数和chassis_vector_to_omni_wheel_speed函数时会直接返回
 *         综上：在小陀螺模式和底盘跟随模式下xy轴速度解算调用同一函数 Set_Chassis_VxVy（），角速度解算调用不同函数
 */
static void chassis_vector_set(chassis_mode_t mode)
{
	int size = sizeof(chassis_commands) / sizeof(chassis_command_t);
	for (int i = 0; i < size; i++) // 寻找匹配当前模式的控制函数
	{
		if (chassis_commands[i].mode == mode)
		{
			chassis_commands[i].handler();
			return;
		}
	}
}
void Chassis_Task(void const *argument)
{
	Chassis_Motor_Pid_Init();

	vTaskDelay(200);

	chassis_mode = CHASSIS_SAFE;
	while (1)
	{
		chassis_mode = Chassis_Mode_Update();
		Chassis_Motor_Data_Update();
		Health_Monitor_Update();
		chassis_vector_set(chassis_mode);

		chassis_vector_to_omni_wheel_speed(chassis_control.vx, chassis_control.vy, chassis_control.wz, &chassis_m3508[0].speed_set, &chassis_m3508[1].speed_set, &chassis_m3508[2].speed_set, &chassis_m3508[3].speed_set, chassis_mode);

		chassis_motor_current_set(chassis_mode);
		Allocate_Can_Buffer(Game_Robot_State.chassis_power_limit - 5, 0, Power_Heat_Data.buffer_energy, 0, CAN_CAP_CMD);

		Allocate_Can_Buffer(chassis_m3508[0].give_current, chassis_m3508[1].give_current, chassis_m3508[2].give_current, chassis_m3508[3].give_current, CAN_CHASSIS_CMD);
		vTaskDelay(2);
	}
}
