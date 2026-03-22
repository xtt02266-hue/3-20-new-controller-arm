#ifndef __FSM_H
#define __FSM_H

typedef enum
{
    fsm_pos_init,
    fsm_lock,
    fsm_geforce_off,
    fsm_judge,
		fsm_protect,
		wait_switch,
} fsm_state_t;
typedef struct
{
    fsm_state_t state;
		int TEST;
} fsm_t;

void fsm_run(fsm_t* fsm);
void fsm_run_test(fsm_t* fsm); 


#endif
