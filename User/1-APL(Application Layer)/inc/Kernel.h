#ifndef __KERNEL_H
#define __KERNEL_H

#include "main.h"
//#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "quadraticInterpolation.h"

extern float arm[6]; // ppppppppppppppppp
extern float q[6];	 // ppppppppppppppppp
extern float q_ikine[6]; // ppppppppppppppppp
typedef enum
{
	Kernel_Lock,
	Kernel_Free,
	Kernel_Reset,
	Kernel_Transit,
} Kernel_mode_e;

typedef enum
{
	//这里一定要enable是因为只有单片机发消息，才能获得电机状态
	only_enable,
	enable_and_control,
}Kernel_cmd_to_motor_mode_e;

typedef struct
{
	float num;
} __attribute__((packed)) custom_motor_t;

typedef union
{
	uint8_t data[30];
	struct
	{
		custom_motor_t motor[motor_num]; 
		uint8_t Button_state[2];//Button_num
	}__attribute__((packed)) param;
		
}Kernel_param_t;

typedef struct
{
	Kernel_mode_e mode;
	Kernel_mode_e last_mode;
	Kernel_cmd_to_motor_mode_e cmd_to_motor_mode;

	uint8_t ret;

	Kernel_param_t to_manipulator_data;
	Kernel_param_t to_customized_comtroller_data;
	Kernel_param_t last_Kernel_data;

	quadraticInterpolation_t Kernel_cmd_to_manipulator_quadraticInterpolation_param[motor_num];

}Kernel_t;

extern Kernel_t Kernel_task;

void Kernel_run(Kernel_t *Kernel_run);

#endif