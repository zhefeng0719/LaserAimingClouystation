#ifndef __MYUSART_H
#define __MYUSART_H
#include "main.h"
typedef struct
{
    uint8_t len;
	uint16_t buff;
    uint8_t data;
    uint8_t *Receive_data;
    uint8_t *prcData;
} Uart;
typedef struct
{
    Uart camera;
    Uart L1;
} MyUartGroup;
typedef struct 
{
int x;
int y;
}coordinate;

void CameraPrc();
void L1Prc();
void MyUartGroup_Init(MyUartGroup *group);
void MsgSendPrc();
#endif // __MYUSART_H