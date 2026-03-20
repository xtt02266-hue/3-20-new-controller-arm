#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "motor_protect.h"

static void motor_disable_detect(void);
static void motor_pos_protect(void);
static void motor_vel_protect(void);    
static void motor_tor_protect(void);
static void motor_overtime_protect(void);
static void motor_safe_mode(void);

uint8_t global_system_halt_flag = 0; // 全局系统停机标志位

/**
 ************************************************************************
 * @brief:      motor_protect_run: 顶层电机综合保护运行函数
 * @param:      void
 * @retval:     void
 * @details:    该函数在定时器中断或主控制循环中周期性调用，负责按顺序调用：
 *              1. 系统熔断降级判断
 *              2. 位置、速度、力矩等多维度的电机边界保护
 *              3. 通讯丢失异常监测
 *              4. 异常超时计算及全局停机决策
 ************************************************************************
 **/
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
    motor_overtime_protect();
}

/**
 ************************************************************************
 * @brief:      motor_pos_protect: 电机位置保护函数
 * @param:      void
 * @retval:     void
 * @details:    监控所有电机的位置，若超出配置的最大/最小限制，则强制修改
 *              控制命令（限位，并将速度设为0），同时设置刚度和阻尼以维持在限位处。
 *              采用指针自增遍历优化数组寻址性能。
 ************************************************************************
 **/
static void motor_pos_protect(void)
{
    motor_t *m = motor;
    motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        float max_p = lim->max_pos;
        float min_p = lim->min_pos;

        if (m->para.pos > max_p) {
            m->cmd.pos_set = max_p;
            m->cmd.vel_set = 0;
            // 保持原本被计算出的其他力矩补偿，只需要覆盖位置和刚度即可
            m->cmd.kp_set = 10.0f;
            m->cmd.kd_set = 2.0f; 
        }
        else if (m->para.pos < min_p) {
            m->cmd.pos_set = min_p;
            m->cmd.vel_set = 0;
            m->cmd.kp_set = 10.0f;
            m->cmd.kd_set = 2.0f; 
        }
    }
}

/**
 ************************************************************************
 * @brief:      motor_vel_protect: 电机会速度保护与防飞车函数
 * @param:      void
 * @retval:     void
 * @details:    包含主动预防和被动保护：
 *              1. 主动预防：基于设定刚度和速度极限，反推允许的最大位置误差，防止目标设定值突变导致的瞬间飞车。
 *                 使用了除法转乘法优化(预计算 inv_kp)，提升浮点运算性能。
 *              2. 被动保护：检测到实际速度超限时，强制进入纯阻尼刹车模式。
 ************************************************************************
 **/
static void motor_vel_protect(void)
{
    motor_t *m = motor;
    motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        float max_v = lim->max_vel;

        // 【主动预防】：限制给定位置的误差，防止目标突变导致速度失控
        // 原理：MIT模式下稳态速度极限 V = (Kp/Kd) * pos_err，由于Kp产生的最大力矩 T_p = Kp * pos_err
        if (m->cmd.kp_set > 0.001f) {
            float inv_kp = 1.0f / m->cmd.kp_set; // 将除法转为乘法，硬件浮点乘法远快于除法
            // 根据最大速度计算出的允许位置误差
            float max_err_v = (m->cmd.kd_set * inv_kp) * max_v;
            // 根据最大力矩计算出的允许位置误差
            float max_err_t = lim->max_tor * inv_kp;
            
            // 取允许条件里更严格(更小)的一个误差范围
            float limit_err = max_err_v < max_err_t ? max_err_v : max_err_t;
            
            float pos_err = m->cmd.pos_set - m->para.pos;
            if (pos_err > limit_err) {
                m->cmd.pos_set = m->para.pos + limit_err;
            } else if (pos_err < -limit_err) {
                m->cmd.pos_set = m->para.pos - limit_err;
            }
        }

        // 【被动保护】：如果受外力等情况实际速度仍然超限，进入纯阻尼刹车模式
        if (m->para.vel > max_v) {
            m->cmd.vel_set = max_v;
            m->cmd.kp_set  = 0.0f;
            m->cmd.kd_set  = 5.0f;
        }
        // 负向超速同理，把目标速度设为 -max_vel，产生反向阻尼力矩
        else if (m->para.vel < -max_v) {
            m->cmd.vel_set = -max_v;
            m->cmd.kp_set  = 0.0f;
            m->cmd.kd_set  = 5.0f;
        }
    }
}

/**
 ************************************************************************
 * @brief:      motor_tor_protect: 电力矩限幅保护函数
 * @param:      void
 * @retval:     void
 * @details:    对下发的前馈力矩进行绝对值限幅，确保硬件不超过最大扭矩。
 *              采用指针自增遍历优化数组寻址性能。
 ************************************************************************
 **/
static void motor_tor_protect(void)
{
    motor_t *m = motor;
    motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        float max_t = lim->max_tor;
        // 直接限幅下发的前馈力矩指令，不修改其他结构体参数
        if (m->para.tor > max_t) {
            m->cmd.tor_set = max_t - 0.01f; 
        }
        else if (m->para.tor < -max_t) {
            m->cmd.tor_set = -max_t + 0.01f; 
        }
    }
}

/**
 ************************************************************************
 * @brief:      motor_disable_detect: 电机通讯丢失检测与恢复函数
 * @param:      void
 * @retval:     void
 * @details:    定期（每30个控制周期）检查电机的状态码。如果检测到通讯丢失
 *              （state 为 0x0D），则向对应 CAN 总线发送清除错误帧并重新使能。
 ************************************************************************
 **/
static void motor_disable_detect(void) 
{
    static int detect_cnt = 0;
    detect_cnt++;
    
    if (detect_cnt > 30)  // 每30个周期检测一次
    {
        motor_t *m = motor; // 指向第一个电机motor[0]
        for (int i = 0; i < 3; i++, m++) {
            // 解析出 state 为 0x0D(即十进制13) 代表 D——通讯丢失
            if (m->para.state == 0x0D) { 
                dm4310_clear_err(&hcan1, m); // 步骤1：发送清除错误帧
                dm4310_enable(&hcan1, m);    // 步骤2：重新使能电机
            }
        }
        
        m = &motor[3]; // [防御性编程] 显式重新对齐指针，防止后续有人修改上面的循环导致指针错位
        for(int i = 3; i < motor_num; i++, m++) {
            if (m->para.state == 0x0D) { 
                dm4310_clear_err(&hcan2, m); // 步骤1：发送清除错误帧
                dm4310_enable(&hcan2, m);    // 步骤2：重新使能电机
            }
        }
        detect_cnt = 0; // 重置计数器
    }
}

/**
 ************************************************************************
 * @brief:      motor_overtime_protect: 电机全局超时熔断保护函数
 * @param:      void
 * @retval:     void
 * @details:    对每个电机维持一个独立的异常时序计数器。若位置、速度或力矩
 *              长时间（例如 200 个控制周期以上）处于异常状态无法自行恢复，
 *              则触发全局系统的熔断标志（global_system_halt_flag = 1）。
 *              使用了单精度硬件函数(fabsf)以加速浮点运算。
 ************************************************************************
 **/
static void motor_overtime_protect(void)
{
    // 每个电机有一个独立的超时计数器
    static int abnormal_cnt[motor_num] = {0};
    uint8_t force_safe_flag = 0; // 是否有任何一个电机触发最终安全模式
    
    motor_t *m = motor;
    motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        // 判断自身是否处于异常区（包含位置越界、速度超限、力矩超限）
        if (m->para.pos > lim->max_pos ||
            m->para.pos < lim->min_pos ||
            fabsf(m->para.vel) > lim->max_vel ||
            fabsf(m->cmd.tor_set) > lim->max_tor) 
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

/**
 ************************************************************************
 * @brief:      motor_safe_mode: 电机安全模式（宕机模式）函数
 * @param:      void
 * @retval:     void
 * @details:    用于在系统触发全局熔断后被调用。强制所有电机进入 
 *              纯阻尼（重度刹车）且 0 目标位置、0 力矩模式，放弃所有的主动运动。
 ************************************************************************
 **/
static void motor_safe_mode(void) 
{
    motor_t *m = motor;
    for (int i = 0; i < motor_num; i++, m++) {
        Set_MIT_PVT(m, 0, 0, 0); 
        Set_MIT_PD(m, 0, 5); 
    }
}

