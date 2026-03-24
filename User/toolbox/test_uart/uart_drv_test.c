#include "uart_drv_test.h"
#include "string.h"
#include "usart.h" 
#include "fsm.h"
PackFromTestUnionDef PackFromTest[6];
// #define UART_TO_Q
#define UART_TO_ARM
uint8_t rxbuffer[7];////修改第七位作为夹爪开关
float unluck(char *buffer);
int unluck_int(char *buffer);
PackFromTestUnionDef PackTest;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
#ifdef UART_TO_Q
        memcpy(PackFromTest->UsartData, rxbuffer, sizeof(rxbuffer));
        switch (rxbuffer[0])
        {
        case 0x30:
            q[0] = unluck(rxbuffer);
            break;
        case 0x31:
            q[1] = unluck(rxbuffer);
            break;
        case 0x32:
            q[2] = unluck(rxbuffer);
            break;
        case 0x33:
            q[3] = unluck(rxbuffer);
            break;
        case 0x34:
            q[4] = unluck(rxbuffer);
            break;
        case 0x35:
            q[5] = unluck(rxbuffer);
            break;
        }
#endif
#ifdef UART_TO_ARM
        memcpy(PackFromTest->UsartData, rxbuffer, sizeof(rxbuffer));
        switch (rxbuffer[0])
        {
        case 0x30:
            if (unluck_int(rxbuffer) == -1)
                return;
            arm[0] = (float)unluck_int(rxbuffer);
            break;
        case 0x31:
            if (unluck_int(rxbuffer) == -1)
                return;
            arm[1] = (float)unluck_int(rxbuffer);
            break;
        case 0x32:
            if (unluck_int(rxbuffer) == -1)
                return;
            arm[2] = (float)unluck_int(rxbuffer);
            break;
        case 0x33:
            arm[3] = unluck(rxbuffer);
            break;
        case 0x34:
            arm[4] = unluck(rxbuffer);
            break;
        case 0x35:
            arm[5] = unluck(rxbuffer);
            break;
        }
#endif
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxbuffer, sizeof(rxbuffer));
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxbuffer, sizeof(rxbuffer));
    }
}
int char_to_int(char i)
{
    switch (i)
    {
    case 0x30:
        return 0;
        break;
    case 0x31:
        return 1;
        break;
    case 0x32:
        return 2;
        break;
    case 0x33:
        return 3;
        break;
    case 0x34:
        return 4;
        break;
    case 0x35:
        return 5;
        break;
    case 0x36:
        return 6;
        break;
    case 0x37:
        return 7;
        break;
    case 0x38:
        return 8;
        break;
    case 0x39:
        return 9;
        break;

    default:
        return 0;
        break;
    }
}

float unluck(char *buffer)
{
    return 1.0f * (float)char_to_int(buffer[2]) + 0.1f * (float)char_to_int(buffer[4]) + 0.01f * (float)char_to_int(buffer[5]);
}

int unluck_int(char *buffer)
{  
    int i = 0;
    for (i = 0; i < 7; i++)
    {
        if(buffer[i] == 0x0A)
            break;
    }
    // i++;
    switch (i)
    {
        case 3:
            return char_to_int(buffer[i - 1]);
            break;
        case 4:
            return char_to_int(buffer[i - 1]) + 10 * char_to_int(buffer[i - 2]);
            break;
        case 5:
            return char_to_int(buffer[i - 1]) + 10 * char_to_int(buffer[i - 2]) + 100 * char_to_int(buffer[i - 3]);
            break;
        default:
            break;
    }
    return -1;
}
