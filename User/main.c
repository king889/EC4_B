
#include "stm32f10x.h"  
#include "Delay.h"  
#include "Serial.h"  
#include "MPU6050.h"  
#include "MadgwickAHRS.h"  
#include "Timer.h"  
#include "OLED.h"  
#include <math.h>  

// === 全局变量 ===  
volatile float pitch, roll, yaw;  
volatile float ax, ay, az, gx, gy, gz;  
extern volatile float q0, q1, q2, q3;

// 连接状态相关
volatile uint8_t B_Link_Status = 1;
volatile uint8_t Missed_ACK_Count = 0;
volatile uint8_t Got_ACK_Flag = 0;
volatile uint8_t RxBit = 0;

int main(void)  
{  

    OLED_Init();
    OLED_Clear();
    

    OLED_ShowString(1, 1, "System Init...");
    OLED_ShowString(2, 1, "Please wait");
    
    Serial_Init();
    MPU6050_Init();
    

    OLED_ShowString(3, 1, "MPU Warming up");
    Delay_ms(3000);
    

    OLED_ShowString(3, 1, "Calibrating...  ");
    MPU6050_CalibrateGyro();

    OLED_ShowString(3, 1, "Calibrate OK!   ");
    Delay_ms(1000);
    

    OLED_Clear();
    OLED_ShowString(1, 1, "Starting...");
    OLED_ShowString(2, 1, "Display OFF");
    OLED_ShowString(3, 1, "in 2 seconds");
    Delay_ms(2000);
    

    
    OLED_Clear();

    Timer_Init();

    while (1)  
    {  

    }  
}  

// === 定时器中断服务函数 ===  
void TIM2_IRQHandler(void)  
{  
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)  
    {  
        static uint8_t task_counter = 0;
        static uint8_t heartbeat_counter = 0;
        static uint8_t send_counter = 0;
        
        task_counter++;
        heartbeat_counter++;
        
        // ===== 任务1: 每 10ms 执行一次姿态解算 (100Hz) =====
        if (task_counter >= 10)  
        {  
            task_counter = 0;
            
            // 1. 读取传感器  
            MPU6050_GetDataScaled(&ax, &ay, &az, &gx, &gy, &gz);  
            
            // 2. 姿态解算
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);  
            
            // 3. 计算欧拉角
            pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.295779513f;  
            roll = atan2f(2.0f * (q0 * q1 + q2 * q3),  
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.295779513f;  
            yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),  
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.295779513f;  
            
            // 4. 每5次解算(50ms)发送一次欧拉角数据
            send_counter++;
            if (send_counter >= 5)
            {
                send_counter = 0;
                
                int16_t pitch_int = (int16_t)(pitch * 100);
                int16_t roll_int = (int16_t)(roll * 100);
                int16_t yaw_int = (int16_t)(yaw * 100);
                
                Serial_SendByte(0xAA);
                Serial_SendByte(0x03);
                Serial_SendByte((pitch_int >> 8) & 0xFF);
                Serial_SendByte(pitch_int & 0xFF);
                Serial_SendByte((roll_int >> 8) & 0xFF);
                Serial_SendByte(roll_int & 0xFF);
                Serial_SendByte((yaw_int >> 8) & 0xFF);
                Serial_SendByte(yaw_int & 0xFF);
                Serial_SendByte(0xFF);
            }
        }  
        

        if (heartbeat_counter >= 50)
        {
            heartbeat_counter = 0;
            
            Serial_SendByte(0xAA);  
            Serial_SendByte(0x01);  
            Serial_SendByte(B_Link_Status);
            Serial_SendByte(0xFF);
            
            Got_ACK_Flag = 0;
        }
        

        if (heartbeat_counter == 40)
        {
            if (Got_ACK_Flag == 1)  
            {  
                Missed_ACK_Count = 0;
                B_Link_Status = 1;
            }  
            else  
            {  
                Missed_ACK_Count++;
                if (Missed_ACK_Count > 4)  
                {  
                    B_Link_Status = 0;
                    Missed_ACK_Count = 4;
                }  
            }
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  
    }  
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t b = USART_ReceiveData(USART1);
        
        if (RxBit == 0 && b == 0xAA)
            RxBit = 1;  
        else if (RxBit == 1 && b == 0x02)
            RxBit = 2;  
        else if (RxBit == 2 && b == 0xFF)  
        {  
            Got_ACK_Flag = 1;
            RxBit = 0;  
        }  
        else
            RxBit = 0;
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}