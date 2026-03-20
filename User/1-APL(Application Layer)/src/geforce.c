#include "geforce.h"                 // 内核主头文件
#include "Callback_Button.h"        // 按钮回调函数
#include "dm4310_ctrl.h"            // DM4310电机控制
#include "dm4310_drv.h"             // DM4310电机驱动
#include "Kernel_behaviour.h"       // 内核行为模式
#include "math.h"

void motor_t_set(motor_t *motor, float max, float min, float torque)
{
    // 优化：加入 else if 减少一次不必要的判断；移除了无用的 speed 变量
	if (torque > max) torque = max;
	else if (torque < min) torque = min;
	
	motor->cmd.tor_set = torque;
}

float calculate_theta(float theta_A_rad, float phi_rad, float ang_5) 
{
    // 优化：缓存三角函数运算结果，大幅减少计算开销
    float sA = sinf(theta_A_rad), cA = cosf(theta_A_rad);
    float sP = sinf(phi_rad),     cP = cosf(phi_rad);
    float s5 = sinf(ang_5),       c5 = cosf(ang_5);

	// 计算杆子B的方向向量
	float B_x = sA * cP * c5 + cA * s5;
	float B_y = sP * c5;
	float B_z = cA * cP * c5 + sA * s5;

	// 优化：使用单精度 sqrtf 和字面量 0.01f
	float magnitude_B = sqrtf(B_x * B_x + B_y * B_y + B_z * B_z);
	if (magnitude_B < 0.01f)
		magnitude_B = 0.01f;

	float cos_theta = B_z / magnitude_B;

	// 优化：使用单精度 asinf
	return asinf(cos_theta);
}

float calculate_theta_X(float theta_A_rad, float phi_rad, float ang_5) 
{
	return sinf(theta_A_rad) * cosf(phi_rad) * sinf(ang_5) + cosf(theta_A_rad) * cosf(ang_5);
}

void motor_coords_get(CoordinateSystem *sys)
{
    // 优化：全部追加 'f'，避免双精度隐式转换
	sys->motor_coords[motor_2].z = 1.57f;
	sys->motor_coords[motor_3].z = 1.57f;

	sys->motor_coords[motor_2].x = -(motor[motor_2].para.pos+ 1.57f);
	sys->motor_coords[motor_3].x = sys->motor_coords[motor_2].x + motor[motor_3].para.pos+ 1.57f;
	sys->motor_coords[motor_4].z = sys->motor_coords[motor_3].x + 1.57f;
	sys->motor_coords[motor_4].x = motor[motor_4].para.pos;

	sys->motor_coords[motor_5].z = calculate_theta(sys->motor_coords[motor_3].x, sys->motor_coords[motor_4].x, motor[motor_5].para.pos);
	sys->motor_coords[motor_5].x = sys->motor_coords[motor_4].z + motor[motor_5].para.pos;
}

void world_coords_get(CoordinateSystem *sys)
{
	sys->world_coords[motor_2].x = 0.0f;
	sys->world_coords[motor_2].y = 0.0f;
	sys->world_coords[motor_2].z = 0.0f;
    
    // 优化：直接复用 motor_coords 中已存在的值
	sys->world_coords[motor_3].x = sinf(sys->motor_coords[motor_2].x) * dis_2_3;
	sys->world_coords[motor_4].x = sys->world_coords[motor_3].x + sinf(sys->motor_coords[motor_3].x) * dis_3_4;
	sys->world_coords[motor_5].x = sys->world_coords[motor_4].x + sinf(sys->motor_coords[motor_4].z) * dis_4_5;

	sys->world_coords[motor_6].x = sys->world_coords[motor_5].x + calculate_theta_X(sys->motor_coords[motor_3].x, sys->motor_coords[motor_4].x, motor[motor_5].para.pos) * dis_5_6;
	sys->world_coords[motor_6].y = sinf(motor[motor_5].para.pos) * dis_5_6;
}

void ge_off(CoordinateSystem *sys)
{
	motor_coords_get(sys);
	world_coords_get(sys);
	
    // 常数 3 加 f 变成 3.0f
	float t_motor_5 = sinf(sys->motor_coords[motor_5].z) * std_mg * dis_5_6;
	float t_motor_4 = sinf(sys->motor_coords[motor_4].x) * sys->world_coords[motor_6].y * std_mg;
	float t_motor_3 = (3.0f * sys->world_coords[motor_3].x - sys->world_coords[motor_6].x
		- sys->world_coords[motor_5].x - sys->world_coords[motor_4].x) * std_mg;
	float t_motor_2 = (sys->world_coords[motor_6].x + sys->world_coords[motor_5].x + sys->world_coords[motor_4].x
		+ sys->world_coords[motor_3].x) * std_mg;

	motor_t_set(&motor[1], 2.5f, -2.5f,  t_motor_2);
	motor_t_set(&motor[2], 1.8f, -1.8f,  t_motor_3);
	motor_t_set(&motor[3], 0.5f, -0.5f,  -t_motor_4);
	motor_t_set(&motor[4], 0.5f, -0.5f,  t_motor_5);	
}












