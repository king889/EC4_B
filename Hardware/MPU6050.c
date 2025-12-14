#include "stm32f10x.h"
#include "MyI2C.h"
#include "Delay.h"
#include "OLED.h"
#define MPU6050_ADDRESS 0xD0
#include "MPU6050_Reg.h"
#include <math.h>

// ====== 全局变量：零偏 ======
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float accel_offset_x = 0, accel_offset_y = 0;  // ← 新增加速度计偏差

#define ACCEL_SCALE  2048.0f
#define GYRO_SCALE   16.4f
#define DEG_TO_RAD   0.017453292519943295f

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendACK(1);
    MyI2C_Stop();
    return Data;
}

void MPU6050_Init()
{
    MyI2C_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                     int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz)
{
    uint8_t Data[14];
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MPU6050_ACCEL_XOUT_H);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    
    for (uint8_t i = 0; i < 14; i++)
    {
        Data[i] = MyI2C_ReceiveByte();
        MyI2C_SendACK(i < 13 ? 0 : 1);
    }
    
    MyI2C_Stop();
    
    *Accx  = (Data[0]  << 8) | Data[1];
    *Accy  = (Data[2]  << 8) | Data[3];
    *Accz  = (Data[4]  << 8) | Data[5];
    *Gyrox = (Data[8]  << 8) | Data[9];
    *Gyroy = (Data[10] << 8) | Data[11];
    *Gyroz = (Data[12] << 8) | Data[13];
}

// ====== 完整校准（陀螺仪 + 加速度计）======
void MPU6050_CalibrateGyro(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0;
    int16_t AX, AY, AZ, gx, gy, gz;
    
    OLED_ShowString(1, 1, "Calibrating...");
    OLED_ShowString(2, 1, "Keep Still!");
    OLED_ShowString(3, 1, "Level Place!");
    
    // 采集 2000 个样本（更准确）
    for (int i = 0; i < 2000; i++)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &gx, &gy, &gz);
        
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        
        sum_ax += AX;
        sum_ay += AY;
        
        Delay_ms(2);
        
        // 显示进度
        if (i % 100 == 0)
        {
            OLED_ShowNum(4, 1, i / 20, 3);  // 显示百分比
            OLED_ShowString(4, 4, "%");
        }
    }
    
    // 计算平均零偏
    gyro_offset_x = sum_gx / 2000.0f;
    gyro_offset_y = sum_gy / 2000.0f;
    gyro_offset_z = sum_gz / 2000.0f;
    
   accel_offset_x = sum_ax / 2000.0f;
   accel_offset_y = sum_ay / 2000.0f;
    
    OLED_Clear();
}

// ====== 读取校准后的数据 ======
void MPU6050_GetDataScaled(float *ax, float *ay, float *az,
                           float *gx, float *gy, float *gz)
{
    static uint8_t gz_is_moving = 0;  
    int16_t Accx, Accy, Accz;
    int16_t Gyrox, Gyroy, Gyroz;
    
    MPU6050_GetData(&Accx, &Accy, &Accz, &Gyrox, &Gyroy, &Gyroz);
    

    *ax = ((float)Accx - accel_offset_x) / ACCEL_SCALE;
    *ay = ((float)Accy - accel_offset_y) / ACCEL_SCALE;
    *az = ((float)Accz) / ACCEL_SCALE;
    

    *gx = (((float)Gyrox - gyro_offset_x) / GYRO_SCALE) * DEG_TO_RAD;
    *gy = (((float)Gyroy - gyro_offset_y) / GYRO_SCALE) * DEG_TO_RAD;
    

    float gz_dps = ((float)Gyroz - gyro_offset_z) / GYRO_SCALE;
    
    if (!gz_is_moving)
    {
        if (fabs(gz_dps) > 1.0f)
        {
            gz_is_moving = 1;
            *gz = gz_dps * DEG_TO_RAD;
        }
        else
        {
            *gz = 0;  // 保持静止
        }
    }
    else
    {
        if (fabs(gz_dps) < 0.5f)
        {
            gz_is_moving = 0;
            *gz = 0;
        }
        else
        {
            *gz = gz_dps * DEG_TO_RAD;
        }
    }
}