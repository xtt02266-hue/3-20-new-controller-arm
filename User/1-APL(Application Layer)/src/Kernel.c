#include "Kernel.h"
#include "Callback_Button.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "Kernel_behaviour.h"
#include "quadraticInterpolation.h"

#include "arm_tool.h"
#include "geforce.h"

Kernel_t Kernel_task;

float arm[6];//ppppppppppppppppp   x,y,z,pitch,yaw,roll
float q[6] = {0, 0, 0, 0, 0, 0}; //{-0.2, -0.5, -0.3, -0.6, 0.5, 0.2}  // ppppppppppppppppp
float q_ikine[6] = {0, 0, 0, 0, 0, 0};

static uint8_t Kernel_init(Kernel_t *Kernel_init);
static void Kernel_param_get(Kernel_t *Kernel_param_get);
static void Kernel_param_update(Kernel_t *Kernel_param_update);
static void Kernel_mode_change_control_transit(Kernel_t *Kernel_mode_change_control_transit);


/**
 * @brief ״̬������
 * @param *Kernel_run ����ָ��
 */
void Kernel_run(Kernel_t *Kernel_run)
{
	
	/*****��ʼ��*****/
	if(Kernel_run->ret == 0)
	{
		Kernel_run->ret  = Kernel_init(Kernel_run);
		arm_tool_init();//ppppppppppp
	}
	/*****״̬��1*****/
	else if(Kernel_run->ret == 1)
	{
		Kernel_param_get(Kernel_run);
		Kernel_mode_control(Kernel_run);//	Kernel_run->ret =
	
		
	}
	/*****״̬��2*****/
	else if(Kernel_run->ret == 2)
	{	
		Button_Callback(Kernel_run);

		


	}
//	else if(Kernel_run->ret == -1)
//	{
//		
//	}
	//���ݸ���
	Kernel_param_update(Kernel_run);
}
	
/**
 * @brief Kernel��ʼ��
 * @param *Kernel_init ����ָ��
 * @return ret״̬ ��ʼ������򷵻�1
 */	
static uint8_t Kernel_init(Kernel_t *Kernel_init)
{
	const double Kernel_reset_theta_set[6]={0,0,0,0,0,0};
	const double Kernel_reset_total_time[6]={200,200,200,200,200,200};
	static uint8_t i;
	static uint8_t Kernel_init_cnt;
	ctrl_enable();
	for(i=0;i<motor_num;i++)
	{
		Kernel_init->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i].theta_set = Kernel_reset_theta_set[i];
		Kernel_init->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i].total_time = Kernel_reset_total_time[i];
	}
	Kernel_init_cnt++;
	if(Kernel_init_cnt >= 50)
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}

/**
 * @brief ���ݻ�ȡ
 * @param *Kernel_param_get ����ָ��
 */
static void Kernel_param_get(Kernel_t *Kernel_param_get)
{
	static uint8_t i;
	Button_Callback(Kernel_param_get);

	q[0] = motor[0].para.pos;
	q[1] = motor[1].para.pos;
	q[2] = motor[2].para.pos;
	q[3] = motor[3].para.pos;
	q[4] = motor[4].para.pos;
	q[5] = motor[5].para.pos;//2 3 4 ���Գ�����

	for(i=0;i<motor_num;i++)
	{
		Kernel_param_get->Kernel_cmd_to_manipulator_quadraticInterpolation_param[i].theta_now=motor[i].para.pos;
		Kernel_param_get->to_manipulator_data.param.motor[i].num=motor[i].para.pos;
	}
}

/**
 * @brief ���ݸ���
 * @param *Kernel_param_update ����ָ��
 */
static void Kernel_param_update(Kernel_t *Kernel_param_update)
{
    static uint8_t i;
    // for(i=0;i<motor_num;i++)
    // {
    //     Kernel_param_update->last_Kernel_data.param.motor[i].num = Kernel_param_update->to_manipulator_data.param.motor[i].num;
    // }//pppppppppppppppppppppppppppppp������ȥ��
    for(i=0;i<Button_num;i++)
    {
        Kernel_param_update->last_Kernel_data.param.Button_state[i] = Kernel_param_update->to_manipulator_data.param.Button_state[i];
    }
	Kernel_param_update->last_mode = Kernel_param_update->mode;
}

/**
 * @brief ģʽ���ƹ���
 * @param *Kernel_mode_change_control_transit ����ָ��
 */
static void Kernel_mode_change_control_transit(Kernel_t *Kernel_mode_change_control_transit)
{
    static uint8_t i;
    if(Kernel_mode_change_control_transit->last_mode == Kernel_Reset && Kernel_mode_change_control_transit->mode == Kernel_Free)
	{
		for(i=0;i<motor_num;i++)
		{
			dm4310_clear_para(&motor[i]);
		}
		Kernel_mode_change_control_transit->mode = Kernel_Transit;
	}
	else if(Kernel_mode_change_control_transit->last_mode == Kernel_Reset && Kernel_mode_change_control_transit->mode == Kernel_Lock)
	{
		//
	}
	else if(Kernel_mode_change_control_transit->last_mode == Kernel_Lock && Kernel_mode_change_control_transit->mode == Kernel_Free)
	{
		for(i=0;i<motor_num;i++)
		{
			dm4310_clear_para(&motor[i]);
		}
		Kernel_mode_change_control_transit->mode = Kernel_Transit;
	}
	else if (Kernel_mode_change_control_transit->last_mode == Kernel_Free && Kernel_mode_change_control_transit->mode == Kernel_Lock)
	{
		for (i = 0; i < motor_num; i++)
		{
			Kernel_mode_change_control_transit->last_Kernel_data.param.motor[i].num = Kernel_mode_change_control_transit->to_manipulator_data.param.motor[i].num;
		}
		
	}
}

