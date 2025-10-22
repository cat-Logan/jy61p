#ifndef __BSP_GYRO_H__
#define __BSP_GYRO_H__
#include <STC32G.H>

#include <string.h>
#include <stdio.h>
#include "intrins.h"
#include "math.h"

// 定义一个结构体来存储所有传感器数据
typedef struct {
    // 角度数据 (单位: 度)
    float x;  // Roll 横滚角
    float y;  // Pitch 俯仰角  
    float z;  // Yaw 航向角
    
    // 加速度数据 (单位: m/s2)
    float ax; // X轴加速度
    float ay; // Y轴加速度
    float az; // Z轴加速度
    
    // 角速度数据 (单位: °/s)
    float gx; // X轴角速度
    float gy; // Y轴角速度
    float gz; // Z轴角速度
	
} Gyro_Struct;

extern volatile Gyro_Struct Gyro_Structure;

// 模块地址
#define IIC_ADDR		0x50

// 寄存器地址定义
#define YAW_REG_ADDR	0x3F	    // 航向角地址
#define ROLL_REG_ADDR   0x3D        // 横滚角地址
#define PITCH_REG_ADDR  0x3E        // 俯仰角地址

// 加速度和角速度寄存器地址
#define AX_REG_ADDR     0x34        // X轴加速度
#define AY_REG_ADDR     0x36        // Y轴加速度  
#define AZ_REG_ADDR     0x38        // Z轴加速度
#define GX_REG_ADDR     0x3A        // X轴角速度
#define GY_REG_ADDR     0x3C        // Y轴角速度
#define GZ_REG_ADDR     0x3E        // Z轴角速度

// 控制寄存器
#define UN_REG			0x69	    // 寄存器解锁
#define SAVE_REG		0x00	    // 保存寄存器
#define ANGLE_REFER_REG	0x01        // 角度参考寄存器
#define speed_rate	    0x03        // 速率设置

// 量程定义
#define ACCEL_RANGE     (16.0f * 9.8f)     // 加速度量程 ±16g -> m/s2
#define GYRO_RANGE      2000.0f            // 角速度量程 ±2000°/s

// 引脚定义
sbit SCL = P2^4;
sbit SDA = P2^3;

// 引脚操作宏
#define SDA_OUT()   P4M1 &= ~0x02, P4M0 |= 0x02   // P4.1 推挽输出
#define SDA_IN()    P4M1 |= 0x02, P4M0 &= ~0x02   // P4.1 高阻输入
#define SDA_GET()   (SDA)  // 直接读取引脚状态
#define SDA(x)      (SDA = (x))  // 设置引脚输出
#define SCL(x)      (SCL = (x))  // 设置引脚输出

// 函数声明
void jy61pInit(void);
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *dat, uint32_t length);
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* dat, uint32_t length);

// 数据读取函数
void get_all_angles(void);
void get_all_sensor_data(void);
void get_gyro_data(float *roll, float *pitch, float *yaw);
float get_angle(void);

// 校准函数
void calibrate_accelerometer(void);
float get_calibrated_ax(void);
float get_calibrated_ay(void);
float get_calibrated_az(void);

#endif