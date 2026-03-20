#ifndef UART_DRV_TEST
#define UART_DRV_TEST
#include "main.h"
typedef struct
{
    uint8_t head;
    float num[7];
} PackFromTestDef;

typedef union
{
    uint8_t UsartData[sizeof(PackFromTestDef)];
    PackFromTestDef PackFromTest;
} PackFromTestUnionDef;
extern PackFromTestUnionDef PackTest;
extern uint8_t rxbuffer[7];
#endif