#ifndef __GEFORCE_H
#define __GEFORCE_H





#define std_mg 4.1 //电机+打印件重量*g 估计值
#define dis_2_3 0.117//电机2-3间距
#define dis_3_4 0.091                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
#define dis_4_5 0.120
#define dis_5_6 0.065  
#define dis_3_5 0.146

#define NUM_MOTORS 6  // 定义电机数量

#define tor_uf  0.01 //测得摩擦力

typedef struct {
    float x;
    float y;
    float z;
} vec3;

typedef struct {
    vec3 origin;                 // 原点坐标系
    vec3 world_coords[NUM_MOTORS];  // 各电机的世界坐标
    vec3 motor_coords[NUM_MOTORS];   // 各电机的电机坐标,(弧度值表示)
		float tor_compensate[NUM_MOTORS]; //惯性&摩擦力补偿参数
		float inertia[NUM_MOTORS];//转动惯量
		float Torque[NUM_MOTORS];//力矩
		
} CoordinateSystem;

enum MotorID {
    motor_2 = 1,
    motor_3 = 2,
    motor_4 = 3,
    motor_5 = 4,
    motor_6 = 5
};

void geforce_off_4mo();
void ge_off(CoordinateSystem *sys);

#endif
