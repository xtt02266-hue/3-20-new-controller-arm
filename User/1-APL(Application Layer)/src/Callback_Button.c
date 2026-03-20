#include "Callback_Button.h"
#include "Kernel.h"


void Button_Callback(Kernel_t * Kernel_button_callback)
{
    if(Kernel_button_callback->ret == 2)
    {
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) == GPIO_PIN_SET)
        { 
            Kernel_button_callback->mode = Kernel_Free;
            Kernel_button_callback->to_manipulator_data.param.Button_state[0] = Kernel_Free;
        }
        else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) == GPIO_PIN_RESET)
        {
            Kernel_button_callback->mode = Kernel_Lock;
            Kernel_button_callback->to_manipulator_data.param.Button_state[0] = Kernel_Lock;
        }
    }
    else if(Kernel_button_callback->ret == 1)
    {
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == GPIO_PIN_SET)
        {
            Kernel_button_callback->mode = Kernel_Free;
            Kernel_button_callback->to_manipulator_data.param.Button_state[1] = Kernel_Free;
        }
        else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == GPIO_PIN_RESET)
        {
            Kernel_button_callback->mode = Kernel_Reset;        
            Kernel_button_callback->to_manipulator_data.param.Button_state[1] = Kernel_Reset;
        }
    }

}


