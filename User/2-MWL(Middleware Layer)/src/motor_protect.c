#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "motor_protect.h"

void motor_disable_detect(void);
static void motor_pos_protect(void);
static void motor_vel_protect(void);    
static void motor_tor_protect(void);
static void motor_overtime_protect(void);
static void motor_safe_mode(void);

volatile uint8_t global_system_halt_flag = 0; // 全局系统停机标志位

const motor_limit_t motor_limits[6] = {
    {.max_tor = 0.5f, .max_vel = 30, .max_pos = 3.0f, .min_pos = -3.0f},
    {.max_tor = 2.9f, .max_vel = 30, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 2.3f, .max_vel = 30, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 1.2f, .max_vel = 30, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 1.2f, .max_vel = 30, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 0.5f, .max_vel = 30, .max_pos = 5.0f, .min_pos = -5.0f},
};

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
    for (int i = 0; i < motor_num; i++) {
        if(motor[i].cmd.kd_set == 0.0f) {
            motor[i].cmd.kd_set = 0.1f; // 最小阻尼，防止完全失控
        }
    
    }
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
    const motor_limit_t *lim = motor_limits;

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
    const motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        float max_v = lim->max_vel;

        // 【主动预防】：限制给定位置的误差，防止目标突变导致失控
        if (m->cmd.kp_set > 0.001f) {
            float inv_kp = 1.0f / m->cmd.kp_set; 
            
            // 错误点1已修复：去除了原先基于(Kd/Kp)*max_v的误差限制。
            // 因为当系统设定的Kd较小时，原算法会把运行误差限制得极其微小(甚至接近0)，
            // 导致电机只要出现一点正常误差，目标位置就会被强行拉扯到当前位置，
            // 从而使得电机无法输出必要的弹力（P力矩被吃掉），表现为其在持续抵抗重力或外力时疯狂抽搐、掉力。
            // 现在只根据电机的最大力矩能力来限制最大位置突变。
            float limit_err = lim->max_tor * inv_kp;
            
            float pos_err = m->cmd.pos_set - m->para.pos;
            if (pos_err > limit_err) {
                m->cmd.pos_set = m->para.pos + limit_err;
            } else if (pos_err < -limit_err) {
                m->cmd.pos_set = m->para.pos - limit_err;
            }
        }

        // 【被动保护】：如果受外力等情况实际速度仍然超限，进入超速抑制
        // 错误点2已修复：对受重力的机械臂，绝对不能把 kp_set 设为 0！
        // 突然把位置刚度Kp清零会导致整个机械臂突然丧失支撑力从而下坠，并在速度恢复瞬间P力矩恢复，带来极其暴力的反复震荡。
        // 正确做法：只加大阻尼Kd进行刹车，但不剥夺原本的位置支撑刚度。
        if (m->para.vel > max_v) {
            m->cmd.vel_set = max_v;
            if (m->cmd.kd_set < 3.0f) {
                m->cmd.kd_set = 3.0f; // 增加阻尼压制速度
            }
        }
        else if (m->para.vel < -max_v) {
            m->cmd.vel_set = -max_v;
            if (m->cmd.kd_set < 3.0f) {
                m->cmd.kd_set = 3.0f; // 增加阻尼压制速度
            }
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
    const motor_limit_t *lim = motor_limits;

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
 * @details:    定期检查电机状态码，使用高频状态机进行极其短暂的延时恢复，避免长掉电抖动
 ************************************************************************
 **/
void motor_disable_detect(void) 
{
    static int detect_cnt = 0;
    static uint16_t recovery_timer[6] = {0}; 
    motor_t *m = motor;
    
    // 1. 高频恢复状态机（每一个控制周期都会运行，不受 30 次阈值的阻塞）
    // 给电机留出极短的时间（如 3 毫秒）复位，肉眼和机械臂的惯性无法察觉，不会导致掉力矩抖动
    for (int i = 0; i < motor_num; i++, m++) {
        if (recovery_timer[i] > 0) {
            recovery_timer[i]++;
            
            // 发出 clear_err 后，等待 3 个控制周期 再发 enable
            if (recovery_timer[i] == 3) { 
                if (i < 3) dm4310_enable(&hcan1, m);
                else       dm4310_enable(&hcan2, m);
            }
            // 恢复完成，重置计时器结束状态机
            else if (recovery_timer[i] > 5) {
                recovery_timer[i] = 0;
            }
        }
        
        m = &motor[3]; // [防御性编程] 显式重新对齐指针，防止后续有人修改上面的循环导致指针错位
        for(int i = 3; i < motor_num; i++, m++) {
            if (m->para.state == 0x0D) { 
                dm4310_clear_err(&hcan2, m); // 步骤1：发送清除错误帧
                dm4310_enable(&hcan2, m);    // 步骤2：重新使能电机
            }
        }
        detect_cnt = 0; // 重置轮询计数器
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
    const motor_limit_t *lim = motor_limits;

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
        Set_MIT_PD(m, 0, 1); 
    }
}

