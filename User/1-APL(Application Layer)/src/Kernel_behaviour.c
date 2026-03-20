#include "Kernel_behaviour.h"
#include "Kernel.h"
#include "dm4310_drv.h"
#include "geforce.h"

CoordinateSystem coord_sys;

static void Kernel_cmd_to_motor_set(Kernel_t * Kernel_cmd_to_motor_set);
static void Kernel_cmd_to_motor(Kernel_t *Kernel_cmd_to_motor);
static void Kernel_Reset_Behaviour(Kernel_t *Kernel_Reset_Behaviour);
static void Kernel_Free_Behaviour(Kernel_t *Kernel_Free_Behaviour);
static void Kernel_Lock_Behaviour(Kernel_t *Kernel_Lock_Behaviour);
static void Kernel_Transit_Behaviour(Kernel_t *Kernel_Transit_Behaviour);

void Kernel_mode_control(Kernel_t *Kernel_mode_control)
{
    Kernel_cmd_to_motor_set(Kernel_mode_control);
    if(Kernel_mode_control->mode == Kernel_Reset)
    {
        Kernel_Reset_Behaviour(Kernel_mode_control);
    }
    else if(Kernel_mode_control->mode == Kernel_Free)
    {
        Kernel_Free_Behaviour(Kernel_mode_control);
    }
    else if(Kernel_mode_control->mode == Kernel_Lock)
    {
        Kernel_Lock_Behaviour(Kernel_mode_control);
    }
    else if(Kernel_mode_control->mode == Kernel_Transit)
    {
        Kernel_Transit_Behaviour(Kernel_mode_control);
    }
    Kernel_cmd_to_motor(Kernel_mode_control);
}


static void Kernel_cmd_to_motor_set(Kernel_t * Kernel_cmd_to_motor_set)
{
    if(Kernel_cmd_to_motor_set->mode == Kernel_Reset)
    {
        Kernel_cmd_to_motor_set->cmd_to_motor_mode = enable_and_control;
    }
    else if(Kernel_cmd_to_motor_set->mode == Kernel_Lock)
    {
        Kernel_cmd_to_motor_set->cmd_to_motor_mode = enable_and_control;
    }
    else if(Kernel_cmd_to_motor_set->mode == Kernel_Free)
    {
        Kernel_cmd_to_motor_set->cmd_to_motor_mode = only_enable;
    } 
    else if(Kernel_cmd_to_motor_set->mode == Kernel_Transit)
    {
        Kernel_cmd_to_motor_set->cmd_to_motor_mode = enable_and_control;
    }
}


static void Kernel_cmd_to_motor(Kernel_t *Kernel_cmd_to_motor)
{
    if(Kernel_cmd_to_motor->cmd_to_motor_mode == only_enable)
    {
        ctrl_enable();
    }
    else if(Kernel_cmd_to_motor->cmd_to_motor_mode == enable_and_control)
    {
				for(int i=0;i<motor_num;i++)
				{
						dm4310_clear_para(&motor[i]);
				}
        //ctrl_set();
				ge_off(&coord_sys);
        ctrl_send();
    }
}


static void Kernel_Reset_Behaviour(Kernel_t *Kernel_Reset_Behaviour)
{
    static uint8_t i;
    for(i = 0;i < motor_num;i++)
    {
        quadraticInterpolation(&Kernel_Reset_Behaviour->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i]);//不确定传参对不对
        Set_MIT_PVT(&motor[i],Kernel_Reset_Behaviour->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i].theta_out,0,0);
        Set_MIT_PD(&motor[i],50,0.9);
    }
		if(Kernel_Reset_Behaviour->Kernel_cmd_to_manipulator_quadraticInterpolation_param[5].finish == 1)
		{
			Kernel_Reset_Behaviour->ret = 2;
			for(i = 0;i < motor_num;i++)
			{
				quadraticInterpolation_clear_param(&Kernel_Reset_Behaviour->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i]);
			}
		}
}

static void Kernel_Free_Behaviour(Kernel_t *Kernel_Free_Behaviour)
{
    //
	
		
}

static void Kernel_Lock_Behaviour(Kernel_t *Kernel_Lock_Behaviour)
{
	static uint8_t i;
    for(i = 0 ; i < motor_num ; i++)
    {
        Kernel_Lock_Behaviour->to_customized_comtroller_data.param.motor[i].num = Kernel_Lock_Behaviour->last_Kernel_data.param.motor[i].num;
        Set_MIT_PVT(&motor[i], Kernel_Lock_Behaviour->to_customized_comtroller_data.param.motor[i].num,0,0);
        Set_MIT_PD(&motor[i],60,0.9);
    }
}

static void Kernel_Transit_Behaviour(Kernel_t *Kernel_Transit_Behaviour)
{
    //
}

