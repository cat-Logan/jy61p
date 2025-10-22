#ifndef __BSP_GYRO_H__
#define __BSP_GYRO_H__
#include <STC32G.H>

#include <string.h>
#include <stdio.h>
#include "intrins.h"
#include "math.h"

// ����һ���ṹ�����洢���д���������
typedef struct {
    // �Ƕ����� (��λ: ��)
    float x;  // Roll �����
    float y;  // Pitch ������  
    float z;  // Yaw �����
    
    // ���ٶ����� (��λ: m/s2)
    float ax; // X����ٶ�
    float ay; // Y����ٶ�
    float az; // Z����ٶ�
    
    // ���ٶ����� (��λ: ��/s)
    float gx; // X����ٶ�
    float gy; // Y����ٶ�
    float gz; // Z����ٶ�
	
} Gyro_Struct;

extern volatile Gyro_Struct Gyro_Structure;

// ģ���ַ
#define IIC_ADDR		0x50

// �Ĵ�����ַ����
#define YAW_REG_ADDR	0x3F	    // ����ǵ�ַ
#define ROLL_REG_ADDR   0x3D        // ����ǵ�ַ
#define PITCH_REG_ADDR  0x3E        // �����ǵ�ַ

// ���ٶȺͽ��ٶȼĴ�����ַ
#define AX_REG_ADDR     0x34        // X����ٶ�
#define AY_REG_ADDR     0x36        // Y����ٶ�  
#define AZ_REG_ADDR     0x38        // Z����ٶ�
#define GX_REG_ADDR     0x3A        // X����ٶ�
#define GY_REG_ADDR     0x3C        // Y����ٶ�
#define GZ_REG_ADDR     0x3E        // Z����ٶ�

// ���ƼĴ���
#define UN_REG			0x69	    // �Ĵ�������
#define SAVE_REG		0x00	    // ����Ĵ���
#define ANGLE_REFER_REG	0x01        // �ǶȲο��Ĵ���
#define speed_rate	    0x03        // ��������

// ���̶���
#define ACCEL_RANGE     (16.0f * 9.8f)     // ���ٶ����� ��16g -> m/s2
#define GYRO_RANGE      2000.0f            // ���ٶ����� ��2000��/s

// ���Ŷ���
sbit SCL = P2^4;
sbit SDA = P2^3;

// ���Ų�����
#define SDA_OUT()   P4M1 &= ~0x02, P4M0 |= 0x02   // P4.1 �������
#define SDA_IN()    P4M1 |= 0x02, P4M0 &= ~0x02   // P4.1 ��������
#define SDA_GET()   (SDA)  // ֱ�Ӷ�ȡ����״̬
#define SDA(x)      (SDA = (x))  // �����������
#define SCL(x)      (SCL = (x))  // �����������

// ��������
void jy61pInit(void);
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *dat, uint32_t length);
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* dat, uint32_t length);

// ���ݶ�ȡ����
void get_all_angles(void);
void get_all_sensor_data(void);
void get_gyro_data(float *roll, float *pitch, float *yaw);
float get_angle(void);

// У׼����
void calibrate_accelerometer(void);
float get_calibrated_ax(void);
float get_calibrated_ay(void);
float get_calibrated_az(void);

#endif