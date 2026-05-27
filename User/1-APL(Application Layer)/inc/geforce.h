#ifndef __GEFORCE_H
#define __GEFORCE_H

// 确保 vec3 定义在 CoordinateSystem 之前，防止报 unknown type name 错误
typedef struct {
    float x;
    float y;
    float z;
} vec3;

#define dis_2_3 0.121        // 电机2-3间距
#define dis_3_4 0.085                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
#define dis_4_5 0.124
#define dis_5_6 0.052  
#define dis_3_5 0.154

#define NUM_MOTORS 6         // 定义电机数量
#define tor_uf  0.01         // 测得摩擦力

typedef struct {
    vec3 origin;                      // 原点坐标系
    vec3 world_coords[NUM_MOTORS];    // 各电机的世界坐标
    vec3 motor_coords[NUM_MOTORS];    // 各电机的电机坐标,(弧度值表示)
    float tor_compensate[NUM_MOTORS]; // 惯性&摩擦力补偿参数
    float inertia[NUM_MOTORS];        // 转动惯量
    float Torque[NUM_MOTORS];         // 力矩
} CoordinateSystem;

enum MotorID {
    motor_2 = 1,
    motor_3 = 2,
    motor_4 = 3,
    motor_5 = 4,
    motor_6 = 5
};

void geforce_off_4mo();

/**
 * @brief 自定义关节重力补偿计算
 * @param sys 坐标系结构体指针
 * @param m2 电机2组件质量 (kg)
 * @param m3 电机3组件质量 (kg)
 * @param m4 电机4组件质量 (kg)
 * @param m5 电机5组件质量 (kg)
 * @param m6 电机6及末端负载总质量 (kg)
 */
// geforce.h 中的函数声明修改为：
void ge_off(CoordinateSystem *sys, float w2, float w3, float w4, float w5, float w6);

#endif