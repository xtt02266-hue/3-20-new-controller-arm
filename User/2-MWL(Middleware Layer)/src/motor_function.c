#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "motor_function.h"

uint8_t lock_flag;
static uint8_t motor_pos_init_reset_request = 0;
static uint8_t motor_pos_run_first_run = 1;
static float motor_pos_run_p_des[6] = {0};
/* motor_pos_init 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_init[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
/* motor_pos_run 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_run[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void motor_pos_run_reset(void)
{
    motor_pos_run_first_run = 1;
}

void motor_pos_init_reset(void)
{
    motor_pos_init_reset_request = 1;
}

void motor_wait_feedback_update(void)
{
    for (int i = 0; i < motor_num; i++) {
        motor[i].cmd.pos_set = motor[i].para.pos;
        motor[i].cmd.vel_set = 0.0f;
        motor[i].cmd.tor_set = 0.0f;
        motor[i].cmd.kp_set = 0.0f;
        motor[i].cmd.kd_set = 0.0f;
    }
    ctrl_set();
    ctrl_send();
}

void motor_pos_reset(void) 
{
    uint8_t mode_id = 0;// 0: MIT模式   1: 位置速度模式   2: 速度模式
    for (int i = 0; i < 3; i++) {
        save_pos_zero(&hcan1, motor[i].id, mode_id);
    }
    for (int i = 3; i < 6; i++) {
        save_pos_zero(&hcan2, motor[i].id, mode_id);
    }
}

/**
 * @brief 电机位置锁定函数
 * @note 记录进入锁定状态瞬间的位置，并持续输出该位置指令
 */
void motor_lock(void) 
{ 
    static float motor_lock_pos[6]; 
    
    if (lock_flag == 0) {
        for(int i = 0; i < motor_num; i++) {
            motor_lock_pos[i] = motor[i].para.pos; 
        }
        lock_flag = 1;
    }
    
    for(int i = 0; i < motor_num; i++) {
        Set_MIT_PVT(&motor[i], motor_lock_pos[i], 0, 0); 
        Set_MIT_PD(&motor[i], 35, 1); 
    }
		//motor_par_send();
}


// 返回 1 表示回零完成，返回 0 表示正在回零
// 全局强制单次运行：完成回零后彻底自锁，整个开机周期不再重复执行
uint8_t motor_pos_init(void) 
{
    static uint8_t is_init_completed = 0;
    static uint8_t first_run = 1;
    static float p_des[6] = {0};
    if (motor_pos_init_reset_request) {
        first_run = 1;
        is_init_completed = 0;
        motor_pos_init_reset_request = 0;
    }
    // 如果已经完成过回零，直接返回1，防止整个MCU生命周期内被反复触发
    if (is_init_completed == 1) {
        return 1;
    }

    // ======== 运动学参数 (绝对防震荡方案) ========
    const float v_speed = 1.2f;             // 匀速回零速度 (rad/s)
    const float dt = 0.005f;                // 控制周期 (假定 5ms)
    const float max_step = v_speed * dt;    // 单个周期允许的最大平移步长
    
    if (first_run) {
        for (int i = 0; i < motor_num; i++) {
            p_des[i] = motor[i].para.pos; 
        }
        first_run = 0;
    }

    // =============== 局部状态判定 ===============
    uint8_t all_traj_done = 1; // 假设大家都完工了

    for (int i = 0; i < motor_num; i++) {
        float p_target = motor_reset_target_pos_init[i];
        
        // 1. 计算虚拟轨迹剩余误差
        float error = p_target - p_des[i];

        // 2. 匀速逼近与死区吸附处理
        if (fabsf(error) <= max_step) {

            p_des[i] = p_target;

        } else {
            all_traj_done = 0;
            float dir = (error > 0) ? 1.0f : -1.0f;
            p_des[i] += dir * max_step;
        }

        motor[i].cmd.pos_set = p_des[i];
        motor[i].cmd.vel_set = 0.0f; 
        motor[i].cmd.kp_set = 15.0f;  
        motor[i].cmd.kd_set = 1.0f;
    }   
    
    // 4. 收尾判定
    if (all_traj_done == 1) {
        is_init_completed = 1;  // 设置开机级别的高级防御标志位，自杀式锁死本函数！
        return 1; // 回零宣告彻底收工
    }
    
    return 0; // 仍在带路回零中...
}

uint8_t motor_pos_run(void) 
{
    // ======== 运动学参数 (绝对防震荡方案) ========
    const float v_speed = 1.0f;             // 匀速回零速度 (rad/s)
    const float dt = 0.005f;                // 控制周期 (假定 5ms)
    const float max_step = v_speed * dt;    // 单个周期允许的最大平移步长
    
    // 初次执行时，将当前实际位置作为轨迹起点
    if (motor_pos_run_first_run) {
        for (int i = 0; i < motor_num; i++) {
            motor_pos_run_p_des[i] = motor[i].para.pos;
        }
        motor_pos_run_first_run = 0;
    }

    // =============== 局部状态判定 ===============
    uint8_t all_traj_done = 1; // 假设大家都完工了

    for (int i = 0; i < motor_num; i++) {
        float p_target = motor_reset_target_pos_run[i];
        
        // 1. 计算虚拟轨迹剩余误差
        float error = p_target - motor_pos_run_p_des[i];

        // 2. 计算物理真实误差 (实际位置距离虚拟期望位置的差值)
        float current_pos_err = motor_pos_run_p_des[i] - motor[i].para.pos;

        // 3. 匀速逼近与死区吸附处理
        if (fabsf(error) <= max_step) {

            motor_pos_run_p_des[i] = p_target;

            if (fabsf(current_pos_err) > 0.05f) {
                all_traj_done = 0;
            }
        } else {
            all_traj_done = 0;
            float dir = (error > 0) ? 1.0f : -1.0f;
            motor_pos_run_p_des[i] += dir * max_step;
        }

        motor[i].cmd.pos_set = motor_pos_run_p_des[i];
        motor[i].cmd.vel_set = 0.0f; 
        motor[i].cmd.kp_set = 9.0f;  
        motor[i].cmd.kd_set = 1.0f;
    }   
    
    // 4. 收尾判定
    if (all_traj_done == 1) {
        return 1; // 回零宣告彻底收工
    }
    
    return 0; // 仍在带路回零中...
}
