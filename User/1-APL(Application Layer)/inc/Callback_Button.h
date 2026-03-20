#ifndef __CALLBACK_BUTTON_H
#define __CALLBACK_BUTTON_H

#include "main.h"
#include "Kernel.h"


typedef enum 
{
    lock_Button,
    Reset_Button,
    Button_num,
}Button_e;

void Button_Callback(Kernel_t * Kernel_button_callback);

#endif