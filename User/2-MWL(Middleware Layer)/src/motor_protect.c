#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"


static void motor_disable_detect(void);
static void motor_pos_protect(void);
static void motor_vel_protect(void);    
static void motor_tor_protect(void);
static void motor_overtime_protect(void);
static void motor_safe_mode(void);

uint8_t global_system_halt_flag = 0; // 全局系统停机标志位

void motor_protect_run(void)
{
    // 如果系统已经被确诊熔断，则直接持续维持安全模式，屏蔽其他一切控制和保护逻辑
    if (global_system_halt_flag) {
        motor_safe_mode();
        return; 
    }

    motor_pos_protect();
    motor_vel_protect();
    motor_tor_protect();
    motor_disable_detect();

    // 监控上述保护是否能在一定时间内使所有参数降回正常
    // 若在设定的保护超时内仍处于异常，则触发全局熔断标志
    motor_overtime_protect();
}

static void motor_pos_protect(void)
{
    for (int i = 0; i < motor_num; i++) {
        if (motor[i].para.pos > motor_limits[i].max_pos) {
            motor[i].cmd.pos_set = motor_limits[i].max_pos;
            motor[i].cmd.vel_set = 0;
            // 保持原本被计算出的其他力矩补偿，只需要覆盖位置和刚度即可
            motor[i].cmd.kp_set = 10.0f;
            motor[i].cmd.kd_set = 2.0f; 
        }

        else if (motor[i].para.pos < motor_limits[i].min_pos) {
            motor[i].cmd.pos_set = motor_limits[i].min_pos;
            motor[i].cmd.vel_set = 0;
            motor[i].cmd.kp_set = 10.0f;
            motor[i].cmd.kd_set = 2.0f; 
        }
    }
}

static void motor_vel_protect(void)
{
    for (int i = 0; i < motor_num; i++) {
        // 【主动预防】：限制给定位置的误差，防止目标突变导致速度失控
        // 原理：MIT模式下稳态速度极限 V = (Kp/Kd) * pos_err，由于Kp产生的最大力矩 T_p = Kp * pos_err
        if (motor[i].cmd.kp_set > 0.001f) {
            // 根据最大速度计算出的允许位置误差
            float max_err_v = (motor[i].cmd.kd_set / motor[i].cmd.kp_set) * motor_limits[i].max_vel;
            // 根据最大力矩计算出的允许位置误差
            float max_err_t = motor_limits[i].max_tor / motor[i].cmd.kp_set;
            
            // 取允许条件里更严格(更小)的一个误差范围
            float limit_err = max_err_v < max_err_t ? max_err_v : max_err_t;
            
            float pos_err = motor[i].cmd.pos_set - motor[i].para.pos;
            if (pos_err > limit_err) {
                motor[i].cmd.pos_set = motor[i].para.pos + limit_err;
            } else if (pos_err < -limit_err) {
                motor[i].cmd.pos_set = motor[i].para.pos - limit_err;
            }
        }

        // 【被动保护】：如果受外力等情况实际速度仍然超限，进入纯阻尼刹车模式
        if (motor[i].para.vel > motor_limits[i].max_vel) {
            motor[i].cmd.vel_set = motor_limits[i].max_vel;
            motor[i].cmd.kp_set  = 0.0f;
            motor[i].cmd.kd_set  = 5.0f;
        }
        // 负向超速同理，把目标速度设为 -max_vel，产生反向阻尼力矩
        else if (motor[i].para.vel < -motor_limits[i].max_vel) {
            motor[i].cmd.vel_set = -motor_limits[i].max_vel;
            motor[i].cmd.kp_set  = 0.0f;
            motor[i].cmd.kd_set  = 5.0f;
        }
    }
}

static void motor_tor_protect(void)
{
    for (int i = 0; i < motor_num; i++) {
        // 直接限幅下发的前馈力矩指令，不修改其他结构体参数
        if (motor[i].para.tor > motor_limits[i].max_tor) {
            motor[i].cmd.tor_set = motor_limits[i].max_tor-0.01f; 
        }
        else if (motor[i].para.tor < -motor_limits[i].max_tor) {
            motor[i].cmd.tor_set = -motor_limits[i].max_tor+0.01f; 
        }
    }
}

static void motor_disable_detect(void) 
{
    static int detect_cnt = 0;
    detect_cnt++;
    
    if (detect_cnt > 30)  // 每30个周期检测一次
    {
        for (int i = 0; i < 3; i++) {
            // 解析出 state 为 0x0D(即十进制13) 代表 D——通讯丢失
            if (motor[i].para.state == 0x0D) { 
                dm4310_clear_err(&hcan1, &motor[i]); // 步骤1：发送清除错误帧
                dm4310_enable(&hcan1, &motor[i]);    // 步骤2：重新使能电机
            }
        }
        for(int i = 3; i < motor_num; i++) {
            if (motor[i].para.state == 0x0D) { 
                dm4310_clear_err(&hcan2, &motor[i]); // 步骤1：发送清除错误帧
                dm4310_enable(&hcan2, &motor[i]);    // 步骤2：重新使能电机
            }
        }
        detect_cnt = 0; // 重置计数器
    }
}

static void motor_overtime_protect(void)
{
    // 每个电机有一个独立的超时计数器
    static int abnormal_cnt[motor_num] = {0};
    uint8_t force_safe_flag = 0; // 是否有任何一个电机触发最终安全模式

    for (int i = 0; i < motor_num; i++) {
        // 判断自身是否处于异常区（包含位置越界、速度超限、力矩超限）
        if (motor[i].para.pos > motor_limits[i].max_pos ||
            motor[i].para.pos < motor_limits[i].min_pos ||
            fabs(motor[i].para.vel) > motor_limits[i].max_vel ||
            fabs(motor[i].cmd.tor_set) > motor_limits[i].max_tor) 
        {
            abnormal_cnt[i]++;
        } 
        else {
            if (abnormal_cnt[i] > 0) {
                // 如果回到了正常状态，快速下降清零，防止在高频震荡（临界点抽搐）中被错误累加触发保护
                abnormal_cnt[i] -= 10; 
                if (abnormal_cnt[i] < 0) {
                    abnormal_cnt[i] = 0;
                }
            }
        }

        // 如果异常持续超过 200 次（约1000ms），强制熔断
        if (abnormal_cnt[i] > 200) {
            force_safe_flag = 1;
        }
    }

    // 若任何电机保护超时失败，强制剥夺所有控制权，全体切入安全纯阻尼宕机模式
    if (force_safe_flag) {
        global_system_halt_flag = 1; // 设置全局停机标志
    }
}

static void motor_safe_mode(void) 
{
    for (int i = 0; i < motor_num; i++) {
        Set_MIT_PVT(&motor[i], 0, 0, 0); 
        Set_MIT_PD(&motor[i], 0, 5); 
    }
}

