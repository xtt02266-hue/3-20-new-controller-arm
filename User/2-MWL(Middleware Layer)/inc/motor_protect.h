#ifndef __MOTOR_PROTECT_H
#define __MOTOR_PROTECT_H

typedef struct {
    float max_tor;      // 最大力矩
    float max_vel;      // 最大速度
    float max_pos;      // 最大位置
    float min_pos;      // 最小位置
} motor_limit_t;

motor_limit_t motor_limits[6] = {
    {.max_tor = 0.5, .max_vel = 2.0, .max_pos = 3, .min_pos = -3},
    {.max_tor = 2.8, .max_vel = 2.0, .max_pos = 1.5, .min_pos = -1.5},
    {.max_tor = 2.3, .max_vel = 2.0, .max_pos = 2, .min_pos = -2},
    {.max_tor = 1.2, .max_vel = 2.0, .max_pos = 2, .min_pos = -2},
    {.max_tor = 1.2, .max_vel = 2.0, .max_pos = 2, .min_pos = -2},
    {.max_tor = 0.5, .max_vel = 2.0, .max_pos = 5, .min_pos = -5},
};


#endif

