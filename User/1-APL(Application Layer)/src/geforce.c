#include "geforce.h"                 // 内核主头文件

#include "dm4310_ctrl.h"            // DM4310电机控制

#include "dm4310_drv.h"             // DM4310电机驱动

#include "math.h"



void motor_t_set(motor_t *motor, float max, float min, float torque)

{

if (torque > max) torque = max;

if (torque < min) torque = min;

float speed = motor->para.vel;

float tor_compensate = speed * 0.006;


motor->cmd.tor_set = torque;// - tor_compensate;

}



void tor_compensate(motor_t *motor) // 惯性&摩擦力补偿

{


}



float calculate_theta(float theta_A_rad, float phi_rad, float ang_5) 

{

// 计算杆子B的方向向量（垂直于杆子A）

// 绕杆子A旋转phi角度

float B_x = sinf(theta_A_rad) * cosf(phi_rad) * cosf(ang_5) + cosf(theta_A_rad) * sinf(ang_5);

float B_y = sinf(phi_rad) * cosf(ang_5) ;

float B_z = cosf(theta_A_rad) * cosf(phi_rad) * cosf(ang_5) + sinf(theta_A_rad) * sinf(ang_5);



// 使用点积公式：cosθ = (B · Z) / (|B| |Z|)

// Z轴方向向量为 (0, 0, 1)

float dot_product = B_z;  // B_x*0 + B_y*0 + B_z*1

float magnitude_B = sqrt(B_x * B_x + B_y * B_y + B_z * B_z);

if (magnitude_B < 0.01)

magnitude_B = 0.01;

// 计算夹角的余弦值

float cos_theta = dot_product / magnitude_B;



// 计算夹角（弧度）

float z_theta_rad = asin(cos_theta);



return z_theta_rad;

}



float calculate_theta_X(float theta_A_rad, float phi_rad, float ang_5) {

float B_x = sinf(theta_A_rad) * cosf(phi_rad) * sinf(ang_5) + cosf(theta_A_rad) * cosf(ang_5); // cos*sin

return B_x;

}



void motor_coords_get(CoordinateSystem *sys) // 获取欧拉角

{

sys->motor_coords[motor_2].z = 1.57;

sys->motor_coords[motor_3].z = 1.57;



sys->motor_coords[motor_2].x = -(motor[motor_2].para.pos + 1.45); // 电机二x轴实际转角，2-3臂与z轴夹角

sys->motor_coords[motor_3].x = sys->motor_coords[motor_2].x + motor[motor_3].para.pos + 1.585 ; // 电机三x轴实际转角，3-4臂与z轴夹角

sys->motor_coords[motor_4].z = sys->motor_coords[motor_3].x + 1.57; // 电机4z轴朝向（x轴转角）4-5臂与z轴夹角

sys->motor_coords[motor_4].x = motor[motor_4].para.pos + 1.57; // 电机朝向



sys->motor_coords[motor_5].z = calculate_theta(sys->motor_coords[motor_3].x, sys->motor_coords[motor_4].x, motor[motor_5].para.pos); // 电机5z轴与世界z轴夹角

sys->motor_coords[motor_5].x = sys->motor_coords[motor_4].z + motor[motor_5].para.pos;

}



void world_coords_get(CoordinateSystem *sys) // 电机位置

{

sys->world_coords[motor_2].x = 0;

sys->world_coords[motor_2].y = 0;

sys->world_coords[motor_2].z = 0;

sys->world_coords[motor_3].x = sinf(sys->motor_coords[motor_2].x) * dis_2_3;

sys->world_coords[motor_4].x = sys->world_coords[motor_3].x + sinf(sys->motor_coords[motor_3].x) * dis_3_4;

sys->world_coords[motor_5].x = sys->world_coords[motor_4].x + sinf(sys->motor_coords[motor_4].z) * dis_4_5;



sys->world_coords[motor_6].x = sys->world_coords[motor_5].x + calculate_theta_X(sys->motor_coords[motor_3].x, sys->motor_coords[motor_4].x, motor[motor_5].para.pos) * dis_5_6;

sys->world_coords[motor_6].y = sinf(motor[motor_5].para.pos) * dis_5_6;

}



/**

 * @brief 自定义关节重力补偿核心计算 (修正版)

 * @param w2~w6 各关节组件的重量权重系数 (对应原先的 std_mg 估计值，默认传入 4.0f)

 */

void ge_off(CoordinateSystem *sys, float w2, float w3, float w4, float w5, float w6)

{

motor_coords_get(sys);

world_coords_get(sys);



// 1. 电机 5 力矩补偿

float t_motor_5 = sinf(sys->motor_coords[motor_5].z) * w6 * dis_5_6;



// 2. 电机 4 力矩补偿

float t_motor_4 = sinf(sys->motor_coords[motor_4].x) * sys->world_coords[motor_6].y * w6;



// 3. 电机 3 力矩补偿 (J1)

float t_motor_3 = ((w4 + w5 + w6) * sys->world_coords[motor_3].x

- w4 * sys->world_coords[motor_4].x

- w5 * sys->world_coords[motor_5].x

- w6 * sys->world_coords[motor_6].x) ;



// 4. 电机 2 力矩补偿

float t_motor_2 = (w3 * sys->world_coords[motor_3].x 

+ w4 * sys->world_coords[motor_4].x 

+ w5 * sys->world_coords[motor_5].x 

+ w6 * sys->world_coords[motor_6].x);


// 输出给执行器，正负方向及限幅完全还原

motor_t_set(&motor[1], 2.9f, -2.9f, t_motor_2);

motor_t_set(&motor[2], 2.2f, -2.2f, t_motor_3);

motor_t_set(&motor[3], 0.5f, -0.5f, t_motor_4);

motor_t_set(&motor[4], 0.5f, -0.5f, -t_motor_5);

}