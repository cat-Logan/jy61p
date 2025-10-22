​
最近想搓个无人机就搞了个jy61p当陀螺仪，想用高速i2c，但奈何官网看的有点看不懂，查询几天后做出来了，发出来当个经验。

I2C读写时序，不用多说，可以在这个网站中查看所有的寄存器地址和协议说明https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy?#UFGm8



ADDR(HEX)就是寄存器地址
FUNCTION是寄存器作用
SERIAL I/F 是寄存器读写权限【R/W寄存器代表可读可写】【R代表只能读不能写入数据】
参考https://wiki.lckfb.com/zh-hans/dmx/module/sensor/jy61p-measurement-sensor.html

得知对传感器操作必须解锁，我只需要角度，加速度，角速度清零，初始化即为

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

官方给的说明：



读取出来的数据/32768*180;

同理



加速度角速度也可以这样处理

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

最后就能读出来了


​
