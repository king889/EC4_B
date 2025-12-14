#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

void MPU6050_Init(void);

// 读取原始值（int16_t）
void MPU6050_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                     int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz);

// 读取换算后的值（float，单位：g 和 rad/s）
void MPU6050_GetDataScaled(float *ax, float *ay, float *az,
                           float *gx, float *gy, float *gz);

// 陀螺仪零漂校准
void MPU6050_CalibrateGyro(void);

#endif