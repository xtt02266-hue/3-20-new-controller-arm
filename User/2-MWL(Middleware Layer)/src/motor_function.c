#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "geforce.h"
#include "fsm.h"
#include "math.h"
#include "motor_function.h"

uint8_t lock_flag;

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
        Set_MIT_PD(&motor[i], 30, 1); // 刚性较高的PD参数，保持位置死锁
    }
    
    // 统一打包发送，在 for 循环外部
		//motor_par_send();
}


// 返回 1 表示回零完成，返回 0 表示正在回零
// 全局强制单次运行：完成回零后彻底自锁，整个开机周期不再重复执行
uint8_t motor_pos_init(void) 
{
    static uint8_t is_init_completed = 0;
    // 如果已经完成过回零，直接返回1，防止整个MCU生命周期内被反复触发
    if (is_init_completed == 1) {
        return 1;
    }

    static uint8_t first_run = 1;
    // 使用静态数组记录各个电机的虚拟期望位置，匀速插值不需要期望速度缓存
    static float p_des[6] = {0};
    
    // ======== 运动学参数 (绝对防震荡方案) ========
    const float v_speed = 1.5f;             // 匀速回零速度 (rad/s)
    const float dt = 0.005f;                // 控制周期 (假定 5ms)
    const float max_step = v_speed * dt;    // 单个周期允许的最大平移步长
    
    // 初次执行时，将当前实际位置作为轨迹起点
    if (first_run) {
        for (int i = 0; i < motor_num; i++) {
            p_des[i] = motor[i].para.pos; 
        }
        first_run = 0;
    }

    // =============== 局部状态判定 ===============
    uint8_t all_traj_done = 1; // 假设大家都完工了

    for (int i = 0; i < motor_num; i++) {
        float p_target = 0.0f; // 目标位置统一为0
        
        // 1. 计算虚拟轨迹剩余误差
        float error = p_target - p_des[i];

        // 2. 计算物理真实误差 (实际位置距离虚拟期望位置的差值)
        float current_pos_err = p_des[i] - motor[i].para.pos;

        // 3. 匀速逼近与死区吸附处理
        if (fabsf(error) <= max_step) {
            // 如果虚拟轨迹的剩余误差不到一步就能走完，虚拟轨迹锁定到目标
            p_des[i] = p_target;

            // 虚拟轨迹虽然到了，但必须检查电机的“物理位置”是否也已经跟上来了？
            // 我们允许 0.08 弧度（约 4.6 度）的位置宽容度，
            // 只要电机进入这个物理死区，就认为它真的到位了
            if (fabsf(current_pos_err) > 0.08f) {
                // 如果虚拟轨迹到了目标，但电机物理上还卡在半路（误差>0.08），
                // 此时本电机依然没有完成，不能收工！必须等它一点点贴近。
                all_traj_done = 0;
            }
        } else {
            // 虚拟规划还在路上，显然还没有完工
            all_traj_done = 0;
            // 判断前进方向并按预定步长前进
            float dir = (error > 0) ? 1.0f : -1.0f;
            p_des[i] += dir * max_step;
        }

        // 4. 赋值下发 MIT 控制指令
        motor[i].cmd.pos_set = p_des[i];
        motor[i].cmd.vel_set = 0.0f; // MIT的虚拟速度环给0即可，位置闭环本身有随动特性
        motor[i].cmd.kp_set = 8.0f;  // 回零刚度，不宜过大防止甩动
        motor[i].cmd.kd_set = 1.0f;
    }   
    
    // 4. 收尾判定
    if (all_traj_done == 1) {
        is_init_completed = 1;  // 设置开机级别的高级防御标志位，自杀式锁死本函数！
        return 1; // 回零宣告彻底收工
    }
    
    return 0; // 仍在带路回零中...
}
