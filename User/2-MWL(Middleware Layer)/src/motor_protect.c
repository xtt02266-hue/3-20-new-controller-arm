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
static int motor_anomaly_detect(void);

#define MOTOR_PROTECT_NUM 6
#define PROTECT_POS_HARD_MARGIN        0.20f
#define PROTECT_VEL_HARD_SCALE         1.50f
#define PROTECT_TOR_HARD_SCALE         1.35f
#define PROTECT_TOR_VEL_MIN_RATIO      0.20f
#define PROTECT_OVERTIME_TRIP_COUNT    180
#define PROTECT_OVERTIME_DECAY_STEP    8
#define PROTECT_STARTUP_GRACE_CYCLES   200
#define DISABLE_DETECT_NEED_CONSECUTIVE 2

volatile uint8_t global_system_halt_flag = 0; // 全局系统停机标志位

const motor_limit_t motor_limits[6] = {
    {.max_tor = 0.6f, .max_vel = 50, .max_pos = 3.0f, .min_pos = -3.0f},
    {.max_tor = 2.9f, .max_vel = 50, .max_pos = 2.0f, .min_pos = -3.0f},
    {.max_tor = 2.3f, .max_vel = 50, .max_pos = 1.5f, .min_pos = -1.5f},
    {.max_tor = 1.2f, .max_vel = 50, .max_pos = 3.0f, .min_pos = -3.0f},
    {.max_tor = 1.2f, .max_vel = 50, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 0.6f, .max_vel = 50, .max_pos = 5.0f, .min_pos = -5.0f},
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
    
    // 强制每周期运行保护函数：
    // 否则一旦实际位置/速度退回正常区间，恢复控制权（例如解除刚度墙）的逻辑就不会执行，导致电机一直卡在保护的粘滞状态。
    motor_pos_protect();
   // motor_vel_protect();
    motor_tor_protect();
    motor_overtime_protect();
    
    motor_disable_detect();
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
static uint8_t in_pos_limit[6] = {0};
static float backup_kp[6] = {0};
static float backup_kd[6] = {0};

static void motor_pos_protect(void)
{
    motor_t *m = motor;
    const motor_limit_t *lim = motor_limits;

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        float max_p = lim->max_pos;
        float min_p = lim->min_pos;

        // 1. 无条件钳制目标指令：防止上层下发越界的目标位置
        if (m->cmd.pos_set > max_p) {
            m->cmd.pos_set = max_p;
        } else if (m->cmd.pos_set < min_p) {
            m->cmd.pos_set = min_p;
        }

        // 2. 对于物理超过限位的情况（比如外力推动），增加虚拟墙阻力
        if (m->para.pos > max_p) {
            if (!in_pos_limit[i]) {
                backup_kp[i] = m->cmd.kp_set;
                backup_kd[i] = m->cmd.kd_set;
                in_pos_limit[i] = 1;
            }
            m->cmd.pos_set = max_p;
            m->cmd.vel_set = 0;
            if (m->cmd.kp_set < 10.0f) m->cmd.kp_set = 10.0f;
            if (m->cmd.kd_set < 2.0f) m->cmd.kd_set = 2.0f; 
        }
        else if (m->para.pos < min_p) {
            if (!in_pos_limit[i]) {
                backup_kp[i] = m->cmd.kp_set;
                backup_kd[i] = m->cmd.kd_set;
                in_pos_limit[i] = 1;
            }
            m->cmd.pos_set = min_p;
            m->cmd.vel_set = 0;
            if (m->cmd.kp_set < 10.0f) m->cmd.kp_set = 10.0f;
            if (m->cmd.kd_set < 2.0f) m->cmd.kd_set = 2.0f; 
        }
        else {
            // 当退回限位以内时，恢复应用原先设置的参数，防止粘滞感
            if (in_pos_limit[i]) {
                m->cmd.kp_set = backup_kp[i];
                m->cmd.kd_set = backup_kd[i];
                in_pos_limit[i] = 0;
            }
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
static uint8_t in_vel_limit[6] = {0};
static float backup_vel_kd[6] = {0};

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
        // 正确做法：只加大阻尼Kd进行刹车，但不剥夺原本的位置支撑刚度。同时要在恢复时归还控制权。
        if (m->para.vel > max_v) {
            if (!in_vel_limit[i]) {
                backup_vel_kd[i] = m->cmd.kd_set;
                in_vel_limit[i] = 1;
            }
            m->cmd.vel_set = max_v;
            if (m->cmd.kd_set < 3.0f) {
                m->cmd.kd_set = 2.0f; // 增加阻尼压制速度
            }
        }
        else if (m->para.vel < -max_v) {
            if (!in_vel_limit[i]) {
                backup_vel_kd[i] = m->cmd.kd_set;
                in_vel_limit[i] = 1;
            }
            m->cmd.vel_set = -max_v;
            if (m->cmd.kd_set < 3.0f) {
                m->cmd.kd_set = 2.0f; // 增加阻尼压制速度
            }
        }
        else {
            // 速度平稳后归还原先的阻尼系数
            if (in_vel_limit[i]) {
                m->cmd.kd_set = backup_vel_kd[i];
                in_vel_limit[i] = 0;
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
        // 直接限幅下发的前馈力矩指令（对目标指令进行限幅，而不是依赖反馈力矩）
        // 否则容易导致外界推力大时产生错误的正反馈粘滞力
        if (m->cmd.tor_set > max_t) {
            m->cmd.tor_set = max_t - 0.01f; 
        }
        else if (m->cmd.tor_set < -max_t) {
            m->cmd.tor_set = -max_t + 0.01f; 
        }
    }
}

/**
 ************************************************************************
 * @brief:      motor_disable_detect: 电机通讯丢失检测与恢复函数
 * @param:      void
 * @retval:     void
 * @details:    
 ************************************************************************
 **/
void motor_disable_detect(void) 
{
    static int detect_cnt = 0;
    static uint8_t offline_cnt[MOTOR_PROTECT_NUM] = {0};
    detect_cnt++;
    
    if (detect_cnt > 30)  // 每30个周期检测一次
    {
        for (int i = 0; i < 3; i++) {
            if (motor[i].para.state == 0) {
                if (offline_cnt[i] < 24) {
                    offline_cnt[i]++;
                }
            } else {
                offline_cnt[i] = 0;
            }

            if (offline_cnt[i] >= DISABLE_DETECT_NEED_CONSECUTIVE) {
                dm4310_enable(&hcan1, &motor[i]);    // 重新使能电机
                offline_cnt[i] = 0;
            }
        }
        for(int i = 3; i < motor_num; i++) {
            if (motor[i].para.state == 0) {
                if (offline_cnt[i] < 24) {
                    offline_cnt[i]++;
                }
            } else {
                offline_cnt[i] = 0;
            }

            if (offline_cnt[i] >= DISABLE_DETECT_NEED_CONSECUTIVE) {
                dm4310_enable(&hcan2, &motor[i]);    
                offline_cnt[i] = 0;
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
    static uint16_t startup_grace_cnt = 0;
    uint8_t force_safe_flag = 0; // 是否有任何一个电机触发最终安全模式
    
    motor_t *m = motor;
    const motor_limit_t *lim = motor_limits;

    // 上电/模式切换初期给一个宽限窗口，避免重力补偿切入时的瞬态被误判为硬故障
    if (startup_grace_cnt < PROTECT_STARTUP_GRACE_CYCLES) {
        startup_grace_cnt++;
        for (int i = 0; i < motor_num; i++) {
            if (abnormal_cnt[i] > 0) {
                abnormal_cnt[i] -= PROTECT_OVERTIME_DECAY_STEP;
                if (abnormal_cnt[i] < 0) {
                    abnormal_cnt[i] = 0;
                }
            }
        }
        return;
    }

    for (int i = 0; i < motor_num; i++, m++, lim++) {
        // 熔断只关注“硬异常”: 传感实测超过带裕量阈值；不使用 cmd.tor_set，避免重力补偿时误判
        uint8_t hard_pos = (m->para.pos > (lim->max_pos + PROTECT_POS_HARD_MARGIN)) ||
                           (m->para.pos < (lim->min_pos - PROTECT_POS_HARD_MARGIN));
        uint8_t hard_vel = fabsf(m->para.vel) > (lim->max_vel * PROTECT_VEL_HARD_SCALE);
        uint8_t hard_tor = (fabsf(m->para.tor) > (lim->max_tor * PROTECT_TOR_HARD_SCALE)) &&
                           (fabsf(m->para.vel) > (lim->max_vel * PROTECT_TOR_VEL_MIN_RATIO));

        if (hard_pos || hard_vel || hard_tor)
        {
            abnormal_cnt[i]++;
        } 
        else {
            if (abnormal_cnt[i] > 0) {
                // 如果回到了正常状态，快速下降清零，防止在高频震荡（临界点抽搐）中被错误累加触发保护
                abnormal_cnt[i] -= PROTECT_OVERTIME_DECAY_STEP;
                if (abnormal_cnt[i] < 0) {
                    abnormal_cnt[i] = 0;
                }
            }
        }

        // 只有“硬异常”连续持续才触发熔断
        if (abnormal_cnt[i] > PROTECT_OVERTIME_TRIP_COUNT) {
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

static int motor_anomaly_detect(void)
{
    for(int i = 0; i < motor_num; i++) {
        if (motor[i].para.pos > motor_limits[i].max_pos ||
            motor[i].para.pos < motor_limits[i].min_pos ||
            fabsf(motor[i].para.vel) > motor_limits[i].max_vel ||
            fabsf(motor[i].para.tor) > motor_limits[i].max_tor) 
        {
            return 1; // 只要有一个电机异常就返回1
        }
    }
    return 0; // 必须返回0，否则会产生未定义行为导致保护机制紊乱
}

void protected_from_geforce(void) //直接使用重力补偿力矩检测电机是否处于异常状态
{
   
}
