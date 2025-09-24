#include "myusart.h"
int scan;

MyUartGroup uartGroup;//定义串口组
coordinate aim;
// 滑动窗口缓存
static uint16_t aim_x_buf[20] = {
320,320,320,320,320,320,320,320,320,320,   
320,320,320,320,320,320,320,320,320,320
};
static uint16_t aim_y_buf[20] = {
240,240,240,240,240,240,240,240,240,240,
240,240,240,240,240,240,240,240,240,240
};
static int filter_idx = 0;

void MyUartGroup_Init(MyUartGroup *group)
{
    aim.x=320;
    aim.y=240;

    group->camera.len = 6;
    group->camera.buff = 128;
    group->camera.Receive_data = (uint8_t *)malloc(group->camera.buff);
    group->camera.prcData = (uint8_t *)malloc(group->camera.len);

    group->L1.len = 8;
    group->L1.buff = 128;
    group->L1.Receive_data = (uint8_t *)malloc(group->L1.buff);
    group->L1.prcData = (uint8_t *)malloc(group->L1.len);

}
uint16_t moving_average(uint16_t *buf, uint16_t new_val)
{
    buf[filter_idx] = new_val;
    uint32_t sum = 0;
    for (int i = 0; i < 20; i++) {
        sum += buf[i];
    }
    return (uint16_t)(sum / 20);
}
void CameraPrc()
{
    // 原始数据
    if(uartGroup.camera.prcData[0]==0x07)
    {
    uint16_t raw_aim_x   = (uartGroup.camera.prcData[2] << 8 ) | uartGroup.camera.prcData[3];
    uint16_t raw_aim_y   = (uartGroup.camera.prcData[4] << 8 ) | uartGroup.camera.prcData[5];
    // 滑动滤波
    if(raw_aim_x==320|raw_aim_y==240)
    {
    aim.x=320;
    aim.y=240;   
    return;
    }
    aim.x   = moving_average(aim_x_buf, raw_aim_x);
    aim.y   = moving_average(aim_y_buf, raw_aim_y);
    // 更新滑动窗口索引
    filter_idx++;
    if (filter_idx >= 20) filter_idx = 0;
    }
}
void L1Prc()
{
    if (uartGroup.L1.prcData[2] == 0x04)
    {
        // 组包scan（第3~6字节，scan为int/uint32_t类型）
        scan = (uartGroup.L1.prcData[3] << 24)  |
                (uartGroup.L1.prcData[4] << 16) |
                (uartGroup.L1.prcData[5] << 8)  |
                (uartGroup.L1.prcData[6]);
    }
    // eff不是0x04则不处理 
}
void MsgSendPrc()
{
    uint8_t txbuf[9];
    txbuf[0] = 0x07;
    txbuf[1] = 0x19;
    txbuf[2] = (aim.x >> 8) & 0xFF;
    txbuf[3] = aim.x & 0xFF;
    txbuf[4] = (aim.y >> 8) & 0xFF;
    txbuf[5] = aim.y & 0xFF;
    txbuf[6] = (scan >> 8) & 0xFF;   // scan高8位
    txbuf[7] = scan & 0xFF;          // scan低8位
    txbuf[8] = 0x01;
    HAL_UART_Transmit(&huart4, txbuf, 9, 10);
}