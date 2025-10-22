#include "jy61p.h"
#include "stdio.h"
#include "string.h"
#include "delay.h"

volatile Gyro_Struct Gyro_Structure;

// ���ٶ�У׼ƫ����
static float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;

/******************************************************************
 * �� �� �� �ƣ�jy61pInit
 * �� �� ˵ ������ʼ��JY61P������
 * �� �� �� �Σ���
 * �� �� �� �أ���
******************************************************************/
void jy61pInit(void)
{
    /*================Z�����==================*/
    unsigned char unlock_reg1[2] = {0x88,0xB5};
    unsigned char z_axis_reg[2] = {0x04,0x00};
    unsigned char save_reg1[2] = {0x00,0x00};
    unsigned char unlock_reg[2] = {0x88,0xB5};
    unsigned char angle_reg[2] = {0x08,0x00};
    unsigned char save_reg[2] = {0x00,0x00};    
    unsigned char speed[2] = {0x00,0x11}; 
    
    // �Ĵ�������
    writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg1,2);
    delay_ms(200);
    // Z�����
    writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,z_axis_reg,2);
    delay_ms(200);
    // ����
    writeDataJy61p(IIC_ADDR,speed_rate,speed,2);
    delay_ms(200);
    writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg1,2);
    delay_ms(200);

    /*================�Ƕȹ���==================*/
    // �Ĵ�������
    writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg,2);
    delay_ms(200);
    // �Ƕȹ���
    writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,angle_reg,2);
    delay_ms(200);
    // ����
    writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg,2);
    delay_ms(200);

    // ��սṹ��
    memset((void *)&Gyro_Structure,0,sizeof(Gyro_Structure));
    
    // ִ�м��ٶȼ�У׼
    calibrate_accelerometer();
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Start
 * �� �� ˵ ����IIC��ʼʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ���
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
 * �� �� �� �ƣ�IIC_Stop
 * �� �� ˵ ����IICֹͣ�ź�
 * �� �� �� �Σ���
 * �� �� �� �أ���
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
 * �� �� �� �ƣ�IIC_Send_Ack
 * �� �� ˵ ������������Ӧ����߷�Ӧ���ź�
 * �� �� �� �Σ�0����Ӧ��  1���ͷ�Ӧ��
 * �� �� �� �أ���
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
 * �� �� �� �ƣ�I2C_WaitAck
 * �� �� ˵ �����ȴ��ӻ�Ӧ��
 * �� �� �� �Σ���
 * �� �� �� �أ�0��Ӧ��  1��ʱ��Ӧ��
******************************************************************/
unsigned char I2C_WaitAck(void)
{
    unsigned char ack = 0;
    unsigned char ack_flag = 50;

    SDA = 1;  // �ͷ�SDA�ߣ�����Ϊ����
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
 * �� �� �� �ƣ�Send_Byte
 * �� �� ˵ ����д��һ���ֽ�
 * �� �� �� �Σ�datҪд�˵�����
 * �� �� �� �أ���
******************************************************************/
void Send_Byte(unsigned char dat)
{
    unsigned char i = 0;
    SCL = 0;  // ����ʱ�ӿ�ʼ���ݴ���

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
 * �� �� �� �ƣ�Read_Byte
 * �� �� ˵ ����IIC��ʱ��
 * �� �� �� �Σ���
 * �� �� �� �أ�����������
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
 * �� �� �� �ƣ�writeDataJy61p
 * �� �� ˵ ����д����
 * �� �� �� �Σ�dev �豸��ַ
                reg �Ĵ�����ַ
                data �����׵�ַ
                length ���ݳ���
 * �� �� �� �أ�����0��д��ɹ�
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
 * �� �� �� �ƣ�readDataJy61p
 * �� �� ˵ ��������������
 * �� �� �� �Σ�dev �豸��ַ
                reg �Ĵ�����ַ
                data ���ݴ洢��ַ
                length ���ݳ���
 * �� �� �� �أ�����0��д��ɹ�
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
 * �� �� �� �ƣ�get_all_angles
 * �� �� ˵ ������ȡ���нǶ�����
 * �� �� �� �Σ���
 * �� �� �� �أ���
******************************************************************/
void get_all_angles(void)
{
    unsigned char sda_angle[6] = {0};
    int ret = 0;

    // ������ݻ���
    memset((void *)sda_angle, 0, sizeof(sda_angle));

    // ��0x3D��ʼ������ȡ6�ֽڣ�����Roll��Pitch��Yaw��
    ret = readDataJy61p(IIC_ADDR, 0x3D, sda_angle, 6);

    if(ret == 1) // ��ȡ�ɹ�
    {
        // ����RollX�Ƕ�
        Gyro_Structure.x = (float)(((short)(sda_angle[1] << 8) | sda_angle[0]) / 32768.0 * 180.0);
        if (Gyro_Structure.x > 180.0) Gyro_Structure.x -= 360.0;
        else if (Gyro_Structure.x < -180.0) Gyro_Structure.x += 360.0;

        // ����PitchY�Ƕ�
        Gyro_Structure.y = (float)(((short)(sda_angle[3] << 8) | sda_angle[2]) / 32768.0 * 180.0);
        if (Gyro_Structure.y > 180.0) Gyro_Structure.y -= 360.0;
        else if (Gyro_Structure.y < -180.0) Gyro_Structure.y += 360.0;

        // ����YawZ�Ƕ�
        Gyro_Structure.z = (float)(((short)(sda_angle[5] << 8) | sda_angle[4]) / 32768.0 * 180.0);
        if (Gyro_Structure.z > 180.0) Gyro_Structure.z -= 360.0;
        else if (Gyro_Structure.z < -180.0) Gyro_Structure.z += 360.0;
    }
}

/******************************************************************
 * �� �� �� �ƣ�get_all_sensor_data
 * �� �� ˵ ������ȡ���д��������ݣ����ٶȡ����ٶȣ�
 * �� �� �� �Σ���
 * �� �� �� �أ���
******************************************************************/
void get_all_sensor_data(void)
{
    unsigned char sda_data[12] = {0}; // 6���� �� 2�ֽ� = 12�ֽ�
    int ret = 0;

    // ������ݻ���
    memset(sda_data, 0, sizeof(sda_data));

    // ��0x34��ʼ������ȡ12�ֽڣ�����6����ļ��ٶȺͽ��ٶȣ�
    ret = readDataJy61p(IIC_ADDR, 0x34, sda_data, 12);

    if(ret == 1) // ��ȡ�ɹ�
    {
        // �������ٶ����� (��λ: m/s2)
        Gyro_Structure.ax = (float)((short)(sda_data[1] << 8) | sda_data[0]) / 32768.0 * ACCEL_RANGE;
        Gyro_Structure.ay = (float)((short)(sda_data[3] << 8) | sda_data[2]) / 32768.0 * ACCEL_RANGE;
        Gyro_Structure.az = (float)((short)(sda_data[5] << 8) | sda_data[4]) / 32768.0 * ACCEL_RANGE;

        // �������ٶ����� (��λ: ��/s)
        Gyro_Structure.gx = (float)((short)(sda_data[7] << 8) | sda_data[6]) / 32768.0 * GYRO_RANGE;
        Gyro_Structure.gy = (float)((short)(sda_data[9] << 8) | sda_data[8]) / 32768.0 * GYRO_RANGE;
        Gyro_Structure.gz = (float)((short)(sda_data[11] << 8) | sda_data[10]) / 32768.0 * GYRO_RANGE;
    }
}

/******************************************************************
 * �� �� �� �ƣ�get_gyro_data
 * �� �� ˵ ������ȡ�����ǽǶ�����
 * �� �� �� �Σ�roll, pitch, yaw - ָ��洢�Ƕ�ֵ��ָ��
 * �� �� �� �أ���
******************************************************************/
void get_gyro_data(float *roll, float *pitch, float *yaw)
{
    get_all_angles();  // ���½Ƕ�����
    if(roll != NULL) *roll = Gyro_Structure.x;
    if(pitch != NULL) *pitch = Gyro_Structure.y;
    if(yaw != NULL) *yaw = Gyro_Structure.z;
}

/******************************************************************
 * �� �� �� �ƣ�get_angle
 * �� �� ˵ ������ȡ����ǣ����ݾɴ��룩
 * �� �� �� �Σ���
 * �� �� �� �أ�����ǽǶ�ֵ
******************************************************************/
float get_angle(void)
{
    get_all_angles();
    return Gyro_Structure.z;
}

/******************************************************************
 * �� �� �� �ƣ�calibrate_accelerometer
 * �� �� ˵ �������ٶȼ�У׼�����贫����ˮƽ���ã�
 * �� �� �� �Σ���
 * �� �� �� �أ���
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
    
    // ����ƫ����������ˮƽ����ʱax=0, ay=0, az=9.8��
    accel_offset_x = sum_x / samples;
    accel_offset_y = sum_y / samples; 
    accel_offset_z = (sum_z / samples) - 9.8f;
}

/******************************************************************
 * �� �� �� �ƣ�get_calibrated_ax
 * �� �� ˵ ������ȡУ׼���X����ٶ�
 * �� �� �� �Σ���
 * �� �� �� �أ�У׼���X����ٶ�
******************************************************************/
float get_calibrated_ax(void) {
    return Gyro_Structure.ax - accel_offset_x;
}

/******************************************************************
 * �� �� �� �ƣ�get_calibrated_ay
 * �� �� ˵ ������ȡУ׼���Y����ٶ�
 * �� �� �� �Σ���
 * �� �� �� �أ�У׼���Y����ٶ�
******************************************************************/
float get_calibrated_ay(void) {
    return Gyro_Structure.ay - accel_offset_y;
}

/******************************************************************
 * �� �� �� �ƣ�get_calibrated_az
 * �� �� ˵ ������ȡУ׼���Z����ٶ�
 * �� �� �� �Σ���
 * �� �� �� �أ�У׼���Z����ٶ�
******************************************************************/
float get_calibrated_az(void) {
    return Gyro_Structure.az - accel_offset_z;
}