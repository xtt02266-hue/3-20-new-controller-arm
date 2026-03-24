#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "button_status_read.h" // 引入我们写好的高阶按键驱动模块
#include "motor_protect.h"
#include "motor_function.h"

CoordinateSystem sys;
float q[6];
float arm[6];
float q_ikine[6];


static void motor_par_send(void);
static void fsm_param_get(fsm_t *fsm);

// ================= 状态机核心 =================
void fsm_run(fsm_t* fsm) {
	


    switch (fsm->state)
    {
        case wait_switch:
            /* 在等待拨动开关的无操作静默期，什么控制指令也不发，直接待机 */
            if ( 1) {
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
                motor_par_send();
            break;


        default:
					fsm->state = wait_switch;
            break;
    }
}
void fsm_run_test(fsm_t* fsm) {
    static uint8_t is_fsm_started = 0;
    fsm_param_get(fsm); 
    if (is_fsm_started == 0) {
        fsm->state = wait_switch; // 强制开机第一拍进入等待开关状态
        is_fsm_started = 1;
    }

    switch (fsm->state)
    {
        case wait_switch:
            /* 在等待拨动开关的无操作静默期，什么控制指令也不发，直接待机 */
            if ( lock_button_enable() == 1) {
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
                // 初次判断跳入对应分支，之后不再回到 judge 状态，而是在下方两个状态相互跳转
                if(lock_button_judge() == 1) {
                    fsm->state = fsm_lock;
                } else {
                    fsm->state = fsm_geforce_off;
                }
                break;

            case fsm_lock:
                if (lock_button_judge() == 0) {
                    lock_flag = 0; 
                    fsm->state = fsm_geforce_off;
                    break; 
                }
                
                motor_lock();
                motor_par_send();			
                break;

            case fsm_geforce_off:
                if (lock_button_judge() == 1) {
                    fsm->state = fsm_lock;
                    break; 
                }
                for(int i = 0; i < motor_num; i++) {
                    motor[i].cmd.pos_set = 0; 
                    motor[i].cmd.vel_set = 0;
                    motor[i].cmd.kp_set = 0;
                    motor[i].cmd.kd_set = 0.08;
                }
                motor_par_send();
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

void fsm_param_get(fsm_t *fsm)
{
	static uint8_t i;
	Switch_Callback(fsm);

	q[0] = motor[0].para.pos;
	q[1] = motor[1].para.pos;
	q[2] = motor[2].para.pos;
	q[3] = motor[3].para.pos;
	q[4] = motor[4].para.pos;
	q[5] = motor[5].para.pos;

	for(i=0;i<motor_num;i++)
	{
		fsm->to_manipulator_data.param.motor[i].num=motor[i].para.pos;
	}
}
