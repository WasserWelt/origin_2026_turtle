/****************************************************************
 * @file: 	Chassis_Task.h
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵舵轮底盘任务，存放底盘的各种控制参数的宏定义
 * @attention:
 ******************************************************************/
#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#define MOTOR_DISTANCE_TO_CENTER 0.237f // 舵轮与地面的接触点到车体中心的距离

#define ROTATE_WZ_MAX 22000  // 遥控模式下小陀螺正向速度
#define ROTATE_WZ_MIN -10000 // 遥控模式下小陀螺反向速度
#define ROTATE_WEAK 0.3f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_BACK_ZERO 4450
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_RIGHT_ZERO 6498
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_LEFT_ZERO 2402
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 8544
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
#define BUFFER_TOTAL_CURRENT_LIMIT 30000.0f
#define POWER_TOTAL_CURRENT_LIMIT 30000.0f
#define WARNING_POWER_BUFF 60.0f

#define M3505_MOTOR_SPEED_PID_KP 10.0f
#define M3505_MOTOR_SPEED_PID_KI 0.006f
#define M3505_MOTOR_SPEED_PID_KD 10.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 320.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.002f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 50.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1500.0f

#define ROTATE_MOVE_FF 0.012f // 小陀螺模式下的前馈系数

#define NAV_SPEED_FAST 800.0f // 导航发过来的速度乘以的系数，非上坡用

void Chassis_Task(void const *argument);

#endif
