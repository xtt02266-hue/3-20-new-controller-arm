#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

extern uint8_t lock_flag;
static void motor_pos_reset(void);
void motor_lock(void);
uint8_t motor_pos_init(void);


#endif