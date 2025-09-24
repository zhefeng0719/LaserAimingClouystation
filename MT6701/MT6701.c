#include "MT6701.h"
#include "main.h"
#include "i2c.h"
#include "math.h"
/*
 * IIC 方式读取角度信息
 * 返回数据为 0 ~ 360 之间的浮点数
 * STM32 开启 IIC 高速模式
 */
float MT6701read(void)//弧度获取
{
    uint32_t angle = 0;
    double fangle = 0;
    uint8_t ReadBuffer1,ReadBuffer2;

    HAL_I2C_Mem_Read(&hi2c3,SlaveAddress,ReadAddress1,I2C_MEMADD_SIZE_8BIT,&ReadBuffer1,1,0XFF);
    angle = ReadBuffer1;
    angle <<= 8;
    HAL_I2C_Mem_Read(&hi2c3,SlaveAddress,ReadAddress2,I2C_MEMADD_SIZE_8BIT,&ReadBuffer2,1,0XFF);
    angle += ReadBuffer2;
    angle >>= 2;            //取数据高 14 位
    fangle = (float)(angle * 360.0f) / 16384.0f;
	// 转换为弧度
    float fradians = fangle * PI / 180.0f;
    return fradians;
}
float MT6701read2(void)//弧度获取
{
    uint32_t angle = 0;
    double fangle = 0;
    uint8_t ReadBuffer1,ReadBuffer2;

    HAL_I2C_Mem_Read(&hi2c2,SlaveAddress,ReadAddress1,I2C_MEMADD_SIZE_8BIT,&ReadBuffer1,1,0XFF);
    angle = ReadBuffer1;
    angle <<= 8;
    HAL_I2C_Mem_Read(&hi2c2,SlaveAddress,ReadAddress2,I2C_MEMADD_SIZE_8BIT,&ReadBuffer2,1,0XFF);
    angle += ReadBuffer2;
    angle >>= 2;            //取数据高 14 位
    fangle = (float)(angle * 360.0f) / 16384.0f;
	// 转换为弧度
    float fradians = fangle * PI / 180.0f;
    return fradians;
}


