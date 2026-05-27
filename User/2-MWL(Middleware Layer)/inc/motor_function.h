#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

extern uint8_t lock_flag;
void motor_pos_reset(void);
void motor_lock(void);
uint8_t motor_pos_init(void);
uint8_t motor_pos_run(void) ;
void motor_pos_init_reset(void);
void motor_pos_run_reset(void);
void motor_wait_feedback_update(void);

#endif
