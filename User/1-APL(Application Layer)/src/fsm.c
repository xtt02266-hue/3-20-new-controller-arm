#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"

CoordinateSystem sys;
static uint8_t lock_button_flag;
static uint8_t lock_flag;


// ================= 函数声明区 =================
// 注意：有返回值的函数用于向状态机报告执行进度或状态
static uint8_t motor_pos_init(void); 
static void motor_protect(float max_t, float max_v);
static void lock_button_init(void);
static void motor_lock(void);
static void lock_button_judge(fsm_t* fsm); 
static void motor_fault_detect(fsm_t* fsm, float max_t, float max_v);
static void motor_safe_mode(void);
static void motor_par_send(void);

// ================= 状态机核心 =================
void fsm_run(fsm_t* fsm) {
	
 

    switch (fsm->state)
    {
        case fsm_pos_init:
            if (motor_pos_init() == 1) {
                lock_flag = 0;
                lock_button_init(); // 回零完成后再初始化按钮
                fsm->state = fsm_judge;
            }
            break;

        case fsm_judge:
            lock_button_judge(fsm);
            motor_disable_detect();
            break;

        case fsm_lock:
            motor_lock();
            lock_button_judge(fsm);
						motor_disable_detect(); 				
            break;
        
        case fsm_geforce_off:
            lock_flag = 0; // 解除锁定标志
						motor_par_send();
            lock_button_judge(fsm); 
						motor_disable_detect(); 
            break;
        case fsm_protect:
            motor_safe_mode();
            break;

        default:
            fsm->state = fsm_pos_init; // 容错处理
            break;
    }
}

// ================= 业务函数区 =================

// 返回 1 表示回零完成，返回 0 表示正在回零
static uint8_t motor_pos_init(void) 
{
    float pos_int_err = 0; 
    
    for (int i = 0; i < motor_num; i++) {
        pos_int_err += fabs(motor[i].para.pos);
        

        float current_pos = motor[i].para.pos;
        float target_pos = 0;
        
        float max_step = 0.01f; 
        
        if (current_pos > max_step) {
            target_pos = current_pos - max_step; // 稳步递减
        } else if (current_pos < -max_step) {
            target_pos = current_pos + max_step; // 稳步递增
        } else {
            target_pos = 0; 
        }

        Set_MIT_PVT(&motor[i], target_pos, 0, 0);
        Set_MIT_PD(&motor[i], 20, 3); 
    }
    
    ge_off(&sys);
    ctrl_set();  
    ctrl_send(); 
    
    if (pos_int_err < 0.06) {
        return 1; // 回零完成
    }
    return 0; // 仍在回零中
}
// 传入 fsm 指针以控制状态跳转
static void lock_button_judge(fsm_t* fsm) 
{
    static uint8_t button_judge_flag = 0;
    
    button_judge_flag++;
    if (button_judge_flag >= 4) 
    {
        button_judge_flag = 0;
        // 判断当前物理按键电平与初始化时的电平是否一致
        // 假设：GPIO_PIN_SET 和 lock_button_flag=1 对应
        if(lock_button_flag == 1) {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
                fsm->state = fsm_lock;
            } else {
                fsm->state = fsm_geforce_off;
            }
        } else {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
                fsm->state = fsm_geforce_off;
            } else {
                fsm->state = fsm_lock;
            }
        }
    }
}





/**
 * @brief 锁定按钮初始化
 * @note 记录系统上电/回零完成时的按键初始电平状态
 */
static void lock_button_init(void) 
{
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
        lock_button_flag = 1; 
    } else {
        lock_button_flag = 0; 
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
        lock_flag = 1; // 标记已经记录完毕，后续周期不再刷新目标位置
    }
    
    // 持续向所有电机发送锁定位置的指令
    for(int i = 0; i < motor_num; i++) {
        Set_MIT_PVT(&motor[i], motor_lock_pos[i], 0, 0); 
        Set_MIT_PD(&motor[i], 80, 1); // 刚性较高的PD参数，保持位置死锁
    }
    
    // 统一打包发送，必须放在 for 循环外部！
		motor_par_send();
}

static void motor_par_send(void)
{
		ge_off(&sys);
	 ctrl_set();  
     ctrl_send(); 
}

