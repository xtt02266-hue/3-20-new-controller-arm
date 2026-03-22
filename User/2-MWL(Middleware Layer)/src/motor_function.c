#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "motor_function.h"

uint8_t lock_flag;

static void motor_pos_reset(void) 
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
static void motor_lock(void) 
{
    static float motor_lock_pos[6]; 
    
    // lock_flag 在切入本状态前已被置 0 (比如在回零完成或 geforce_off 时)
    // 所以刚进入锁定的第一个周期，会记录一次当前位置
    if (lock_flag == 0) {
        for(int i = 0; i < motor_num; i++) {
            motor_lock_pos[i] = motor[i].para.pos; 
        }
        lock_flag = 1; // 标记已经记录完毕，后续周期不再刷新目标位置,后续需要加这个标志位防护，确保每次进入前都会为0
    }
    
    // 持续向所有电机发送锁定位置的指令
    for(int i = 0; i < motor_num; i++) {
        Set_MIT_PVT(&motor[i], motor_lock_pos[i], 0, 0); 
        Set_MIT_PD(&motor[i], 40, 2); // 刚性较高的PD参数，保持位置死锁
    }
    
    // 统一打包发送，在 for 循环外部
		//motor_par_send();
}