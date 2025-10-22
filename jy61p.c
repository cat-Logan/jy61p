#include "jy61p.h"
#include "stdio.h"
#include "string.h"
#include "delay.h"

volatile Gyro_Struct Gyro_Structure;

// 加速度校准偏移量
static float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;

/******************************************************************
 * 函 数 名 称：jy61pInit
 * 函 数 说 明：初始化JY61P传感器
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void jy61pInit(void)
{
    /*================Z轴归零==================*/
    unsigned char unlock_reg1[2] = {0x88,0xB5};
    unsigned char z_axis_reg[2] = {0x04,0x00};
    unsigned char save_reg1[2] = {0x00,0x00};
    unsigned char unlock_reg[2] = {0x88,0xB5};
    unsigned char angle_reg[2] = {0x08,0x00};
    unsigned char save_reg[2] = {0x00,0x00};    
    unsigned char speed[2] = {0x00,0x11}; 
    
    // 寄存器解锁
    writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg1,2);
    delay_ms(200);
    // Z轴归零
    writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,z_axis_reg,2);
    delay_ms(200);
    // 保存
    writeDataJy61p(IIC_ADDR,speed_rate,speed,2);
    delay_ms(200);
    writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg1,2);
    delay_ms(200);

    /*================角度归零==================*/
    // 寄存器解锁
    writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg,2);
    delay_ms(200);
    // 角度归零
    writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,angle_reg,2);
    delay_ms(200);
    // 保存
    writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg,2);
    delay_ms(200);

    // 清空结构体
    memset((void *)&Gyro_Structure,0,sizeof(Gyro_Structure));
    
    // 执行加速度计校准
    calibrate_accelerometer();
}

/******************************************************************
 * 函 数 名 称：IIC_Start
 * 函 数 说 明：IIC起始时序
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void IIC_Start(void)
{
    SCL = 1;
    SDA = 1;
    delay_us(5);
    SDA = 0;
    delay_us(5);
    SCL = 0;
    delay_us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Stop
 * 函 数 说 明：IIC停止信号
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void IIC_Stop(void)
{
    SCL = 0;
    SDA = 0;
    delay_us(5);
    SCL = 1;
    delay_us(5);
    SDA = 1;
    delay_us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Send_Ack
 * 函 数 说 明：主机发送应答或者非应答信号
 * 函 数 形 参：0发送应答  1发送非应答
 * 函 数 返 回：无
******************************************************************/
void IIC_Send_Ack(unsigned char ack)
{
    SCL = 0;
    if(!ack) 
        SDA = 0;
    else     
        SDA = 1;
    delay_us(5);
    SCL = 1;
    delay_us(5);
    SCL = 0;
    SDA = 1;
}

/******************************************************************
 * 函 数 名 称：I2C_WaitAck
 * 函 数 说 明：等待从机应答
 * 函 数 形 参：无
 * 函 数 返 回：0有应答  1超时无应答
******************************************************************/
unsigned char I2C_WaitAck(void)
{
    unsigned char ack = 0;
    unsigned char ack_flag = 50;

    SDA = 1;  // 释放SDA线，设置为输入
    delay_us(5);
    
    while((SDA == 1) && (ack_flag))
    {
        ack_flag--;
        delay_us(5);
    }

    if(ack_flag == 0)
    {
        IIC_Stop();
        return 1;
    }
    else
    {
        SCL = 1;
        delay_us(5);
        SCL = 0;
    }
    return ack;
}

/******************************************************************
 * 函 数 名 称：Send_Byte
 * 函 数 说 明：写入一个字节
 * 函 数 形 参：dat要写人的数据
 * 函 数 返 回：无
******************************************************************/
void Send_Byte(unsigned char dat)
{
    unsigned char i = 0;
    SCL = 0;  // 拉低时钟开始数据传输

    for(i = 0; i < 8; i++)
    {
        if((dat & 0x80) >> 7)
            SDA = 1;
        else
            SDA = 0;
        delay_us(2);

        SCL = 1;
        delay_us(5);

        SCL = 0;
        delay_us(5);

        dat <<= 1;
    }
}

/******************************************************************
 * 函 数 名 称：Read_Byte
 * 函 数 说 明：IIC读时序
 * 函 数 形 参：无
 * 函 数 返 回：读到的数据
******************************************************************/
unsigned char Read_Byte(void)
{
    unsigned char i, receive = 0;
    
    for(i = 0; i < 8; i++)
    {
        SCL = 0;
        delay_us(5);
        SCL = 1;
        delay_us(5);
        
        receive <<= 1;
        if(SDA)
        {
            receive |= 1;
        }
        delay_us(5);
    }

    return receive;
}

/******************************************************************
 * 函 数 名 称：writeDataJy61p
 * 函 数 说 明：写数据
 * 函 数 形 参：dev 设备地址
                reg 寄存器地址
                data 数据首地址
                length 数据长度
 * 函 数 返 回：返回0则写入成功
******************************************************************/
unsigned char writeDataJy61p(unsigned char dev, unsigned char reg, unsigned char* dat, unsigned long length)
{
    unsigned long count = 0;

    IIC_Start();

    Send_Byte(dev << 1);
    if(I2C_WaitAck() == 1) return 0;

    Send_Byte(reg);
    if(I2C_WaitAck() == 1) return 0;

    for(count = 0; count < length; count++)
    {
        Send_Byte(dat[count]);
        if(I2C_WaitAck() == 1) return 0;
    }

    IIC_Stop();

    return 1;
}

/******************************************************************
 * 函 数 名 称：readDataJy61p
 * 函 数 说 明：读数据数据
 * 函 数 形 参：dev 设备地址
                reg 寄存器地址
                data 数据存储地址
                length 数据长度
 * 函 数 返 回：返回0则写入成功
******************************************************************/
unsigned char readDataJy61p(unsigned char dev, unsigned char reg, unsigned char *dat, unsigned long length)
{
    unsigned long count = 0;

    IIC_Start();

    Send_Byte((dev << 1) | 0);
    if(I2C_WaitAck() == 1) return 0;

    Send_Byte(reg);
    if(I2C_WaitAck() == 1) return 0;

    delay_us(5);

    IIC_Start();

    Send_Byte((dev << 1) | 1);
    if(I2C_WaitAck() == 1) return 0;

    for(count = 0; count < length; count++)
    {
        if(count != length - 1)
        {
            dat[count] = Read_Byte();
            IIC_Send_Ack(0);
        }
        else
        {
            dat[count] = Read_Byte();
            IIC_Send_Ack(1);
        }
    }

    IIC_Stop();

    return 1;
}

/******************************************************************
 * 函 数 名 称：get_all_angles
 * 函 数 说 明：读取所有角度数据
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void get_all_angles(void)
{
    unsigned char sda_angle[6] = {0};
    int ret = 0;

    // 清空数据缓存
    memset((void *)sda_angle, 0, sizeof(sda_angle));

    // 从0x3D开始连续读取6字节（包含Roll、Pitch、Yaw）
    ret = readDataJy61p(IIC_ADDR, 0x3D, sda_angle, 6);

    if(ret == 1) // 读取成功
    {
        // 计算RollX角度
        Gyro_Structure.x = (float)(((short)(sda_angle[1] << 8) | sda_angle[0]) / 32768.0 * 180.0);
        if (Gyro_Structure.x > 180.0) Gyro_Structure.x -= 360.0;
        else if (Gyro_Structure.x < -180.0) Gyro_Structure.x += 360.0;

        // 计算PitchY角度
        Gyro_Structure.y = (float)(((short)(sda_angle[3] << 8) | sda_angle[2]) / 32768.0 * 180.0);
        if (Gyro_Structure.y > 180.0) Gyro_Structure.y -= 360.0;
        else if (Gyro_Structure.y < -180.0) Gyro_Structure.y += 360.0;

        // 计算YawZ角度
        Gyro_Structure.z = (float)(((short)(sda_angle[5] << 8) | sda_angle[4]) / 32768.0 * 180.0);
        if (Gyro_Structure.z > 180.0) Gyro_Structure.z -= 360.0;
        else if (Gyro_Structure.z < -180.0) Gyro_Structure.z += 360.0;
    }
}

/******************************************************************
 * 函 数 名 称：get_all_sensor_data
 * 函 数 说 明：读取所有传感器数据（加速度、角速度）
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void get_all_sensor_data(void)
{
    unsigned char sda_data[12] = {0}; // 6个轴 × 2字节 = 12字节
    int ret = 0;

    // 清空数据缓存
    memset(sda_data, 0, sizeof(sda_data));

    // 从0x34开始连续读取12字节（包含6个轴的加速度和角速度）
    ret = readDataJy61p(IIC_ADDR, 0x34, sda_data, 12);

    if(ret == 1) // 读取成功
    {
        // 解析加速度数据 (单位: m/s2)
        Gyro_Structure.ax = (float)((short)(sda_data[1] << 8) | sda_data[0]) / 32768.0 * ACCEL_RANGE;
        Gyro_Structure.ay = (float)((short)(sda_data[3] << 8) | sda_data[2]) / 32768.0 * ACCEL_RANGE;
        Gyro_Structure.az = (float)((short)(sda_data[5] << 8) | sda_data[4]) / 32768.0 * ACCEL_RANGE;

        // 解析角速度数据 (单位: °/s)
        Gyro_Structure.gx = (float)((short)(sda_data[7] << 8) | sda_data[6]) / 32768.0 * GYRO_RANGE;
        Gyro_Structure.gy = (float)((short)(sda_data[9] << 8) | sda_data[8]) / 32768.0 * GYRO_RANGE;
        Gyro_Structure.gz = (float)((short)(sda_data[11] << 8) | sda_data[10]) / 32768.0 * GYRO_RANGE;
    }
}

/******************************************************************
 * 函 数 名 称：get_gyro_data
 * 函 数 说 明：获取陀螺仪角度数据
 * 函 数 形 参：roll, pitch, yaw - 指向存储角度值的指针
 * 函 数 返 回：无
******************************************************************/
void get_gyro_data(float *roll, float *pitch, float *yaw)
{
    get_all_angles();  // 更新角度数据
    if(roll != NULL) *roll = Gyro_Structure.x;
    if(pitch != NULL) *pitch = Gyro_Structure.y;
    if(yaw != NULL) *yaw = Gyro_Structure.z;
}

/******************************************************************
 * 函 数 名 称：get_angle
 * 函 数 说 明：获取航向角（兼容旧代码）
 * 函 数 形 参：无
 * 函 数 返 回：航向角角度值
******************************************************************/
float get_angle(void)
{
    get_all_angles();
    return Gyro_Structure.z;
}

/******************************************************************
 * 函 数 名 称：calibrate_accelerometer
 * 函 数 说 明：加速度计校准（假设传感器水平放置）
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void calibrate_accelerometer(void)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 100;
    int i;
    
    for(i = 0; i < samples; i++) {
        get_all_sensor_data();
        sum_x += Gyro_Structure.ax;
        sum_y += Gyro_Structure.ay;
        sum_z += Gyro_Structure.az;
        delay_ms(10);
    }
    
    // 计算偏移量（假设水平放置时ax=0, ay=0, az=9.8）
    accel_offset_x = sum_x / samples;
    accel_offset_y = sum_y / samples; 
    accel_offset_z = (sum_z / samples) - 9.8f;
}

/******************************************************************
 * 函 数 名 称：get_calibrated_ax
 * 函 数 说 明：获取校准后的X轴加速度
 * 函 数 形 参：无
 * 函 数 返 回：校准后的X轴加速度
******************************************************************/
float get_calibrated_ax(void) {
    return Gyro_Structure.ax - accel_offset_x;
}

/******************************************************************
 * 函 数 名 称：get_calibrated_ay
 * 函 数 说 明：获取校准后的Y轴加速度
 * 函 数 形 参：无
 * 函 数 返 回：校准后的Y轴加速度
******************************************************************/
float get_calibrated_ay(void) {
    return Gyro_Structure.ay - accel_offset_y;
}

/******************************************************************
 * 函 数 名 称：get_calibrated_az
 * 函 数 说 明：获取校准后的Z轴加速度
 * 函 数 形 参：无
 * 函 数 返 回：校准后的Z轴加速度
******************************************************************/
float get_calibrated_az(void) {
    return Gyro_Structure.az - accel_offset_z;
}