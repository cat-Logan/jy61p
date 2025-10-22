#include <STC32G.H>
#include <board.h>
#include <intrins.h>
#include "delay.h"
#include "jy61p.h"
void UART_init(void){
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器时钟1T模式
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0x9E;			//设置定时初始值
	TH1 = 0xFF;			//设置定时初始值
	ET1 = 0;			//禁止定时器中断
	TR1 = 1;			//定时器1开始计时
}
void UART1_SendData(unsigned char dat)
{
    SBUF = dat;         // 将数据写入发送缓冲区
    while(!TI);         // 等待发送完成
    TI = 0;             // 清除发送中断标志
}

// UART1发送字符串函数
void UART1_SendString(char *s)
{

    while(*s)           // 检测字符串结束符
    {
        UART1_SendData(*s++); // 发送当前字符
    }

}
void main(){
	   float roll, pitch, yaw;
    char str[120]; 
    clock_init(SYSTEM_CLOCK_45_1584M);
    board_init();	// 增加缓冲区大小以容纳所有数据
	    delay_init();
    UART_init();
    jy61pInit();
	
	
	while(1){
	    get_all_angles();        // 读取角度数据
    get_all_sensor_data();   // 读取加速度和角速度数据
    
    // 获取角度数据（通过参数传递）
    get_gyro_data(&roll, &pitch, &yaw);
    
    // 格式化输出所有数据
    sprintf(str, "R:%.2f,P:%.2f,Y:%.2f,ax:%.2f,ay:%.2f,az:%.2f,gx:%.2f,gy:%.2f,gz:%.2f\r\n", 
            roll, pitch, yaw,
            Gyro_Structure.ax, Gyro_Structure.ay, Gyro_Structure.az,
            Gyro_Structure.gx, Gyro_Structure.gy, Gyro_Structure.gz);
    
    UART1_SendString(str);
		delay_ms(200);
	}
	
	
	
	
}