#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "button_status_read.h" // 引入我们写好的高阶按键驱动模块
#include "motor_protect.h"
#include "motor_function.h"

CoordinateSystem sys;


static void motor_par_send(void);
static uint8_t motor_pos_init(void);

// ================= 状态机核心 =================
void fsm_run(fsm_t* fsm) {
	
    // 置顶调用！每次跑状态机前，先去喂狗并获取最新的门控状态
    // 如果返回 0 说明操作员开机后还没拨过哪怕一次开关，系统处于防误触静默期
    int is_switch_ready = fsm_enable();

    switch (fsm->state)
    {
        case wait_switch:
            motor_par_send();
            //  操作员明确拨过一次开关(返回1)
            if ( is_switch_ready == 1) {
                fsm->state = fsm_pos_init;
            }
            break;

        case fsm_pos_init:
            motor_par_send();
            if (motor_pos_init()) { // 回零完成后自动切入下一个状态
                lock_flag = 0; // 确保进入下一个状态前锁定标志位被重置
                fsm->state = fsm_judge;
            }
            break;

        case fsm_judge:
            break;

        case fsm_lock:			
            break;
        
        case fsm_geforce_off:

            break;


        default:
					fsm->state = wait_switch;
            break;
    }
}
void fsm_run_test(fsm_t* fsm) {
    int is_switch_ready = fsm_enable();

    switch (fsm->state)
    {
        case wait_switch:

            if ( is_switch_ready == 1) {
                fsm->state = fsm_geforce_off;
            }
            break;
        case fsm_geforce_off:
               motor_par_send();
            break;
        default:
					fsm->state = wait_switch;
            break;
    }
}

static void motor_par_send(void)
{
	ge_off(&sys);
    motor_protect_run(); // 先跑保护，确保所有电机命令都在安全范围内
	ctrl_set();  
    ctrl_send(); 
}

// ================= 业务函数区 =================

// 返回 1 表示回零完成，返回 0 表示正在回零
static uint8_t motor_pos_init(void) 
{
    static uint8_t first_run = 1;
    // 使用静态数组记录各个电机的规划位置和速度，保证下次进入循环能接续积分
    static float p_des[6] = {0};
    static float v_des[6] = {0};
    
    // ======== 运动学参数 (请根据实际硬件调整) ========
    const float a_max = 3.0f;      // 最大加速度 (rad/s^2)
    const float v_max = 25.0f;      // 最大速度 (rad/s)
    const float dt = 0.005f;       // 控制周期 (默认假设 1ms = 0.001s)
    
    // 初次执行时，将当前实际位置作为轨迹起点
    if (first_run) {
        for (int i = 0; i < motor_num; i++) {
            p_des[i] = motor[i].para.pos; 
            v_des[i] = 0.0f;
        }
        first_run = 0;
    }

    float pos_int_err = 0; 
    
    for (int i = 0; i < motor_num; i++) {
        float p_target = 0.0f; // 目标位置为0
        
        // 1. 计算当前期望位置到目标位置的距离和方向
        float error = p_target - p_des[i];
        float dir = (error > 0) ? 1.0f : -1.0f; // 确定运动方向：error大于0（目标在正方向）时dir为1，否则为-1

        // 2. 计算如果现在开始全力减速，需要的刹车距离
        float brake_distance = (v_des[i] * v_des[i]) / (2.0f * a_max);

        // 3. 决定当前的期望加速度 (a_des)
        float a_des = 0.0f;
        if (fabs(error) <= brake_distance) {
            // 【减速段】距离不够了，必须开始刹车
            a_des = -dir * a_max; 
        } else {
            // 【加速或匀速段】距离还够
            if (fabs(v_des[i]) < v_max) {
                a_des = dir * a_max;
            } else {
                a_des = 0.0f;
                v_des[i] = dir * v_max; // 钳制速度
            }
        }

        // 4. 死区处理：防止到达终点时反复震荡 (收敛判定)
        if (fabsf(error) < 0.002f && fabsf(v_des[i]) < 0.01f) {
            p_des[i] = p_target;
            v_des[i] = 0.0f;
            a_des = 0.0f;
        } else {
            // 5. 积分更新期望速度和期望位置
            v_des[i] += a_des * dt;
            p_des[i] += v_des[i] * dt;  
        }

        // 7. 直接给结构体成员赋值下发 MIT 控制指令
        motor[i].cmd.pos_set = p_des[i];
        motor[i].cmd.vel_set = v_des[i];
        motor[i].cmd.kp_set = 2.5f;
        motor[i].cmd.kd_set = 0.5f;

        // 将当前电机真值误差累加用于统一计算是否整体到达目标
        pos_int_err += fabs(motor[i].para.pos - p_target);
    }   
    
    // 如果整体位置偏差足够小，视为回零完成
    if (pos_int_err < 0.06f) {
        first_run = 1; // 重置首次运行标志位，以便下次状态切换时重新初始化 
        return 1; // 回零完成
    }
    return 0; // 仍在回零中
}

