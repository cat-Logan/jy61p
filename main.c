#include <STC32G.H>
#include <board.h>
#include <intrins.h>
#include "delay.h"
#include "jy61p.h"
void UART_init(void){
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��ʱ��1Tģʽ
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//���ö�ʱ��ģʽ
	TL1 = 0x9E;			//���ö�ʱ��ʼֵ
	TH1 = 0xFF;			//���ö�ʱ��ʼֵ
	ET1 = 0;			//��ֹ��ʱ���ж�
	TR1 = 1;			//��ʱ��1��ʼ��ʱ
}
void UART1_SendData(unsigned char dat)
{
    SBUF = dat;         // ������д�뷢�ͻ�����
    while(!TI);         // �ȴ��������
    TI = 0;             // ��������жϱ�־
}

// UART1�����ַ�������
void UART1_SendString(char *s)
{

    while(*s)           // ����ַ���������
    {
        UART1_SendData(*s++); // ���͵�ǰ�ַ�
    }

}
void main(){
	   float roll, pitch, yaw;
    char str[120]; 
    clock_init(SYSTEM_CLOCK_45_1584M);
    board_init();	// ���ӻ�������С��������������
	    delay_init();
    UART_init();
    jy61pInit();
	
	
	while(1){
	    get_all_angles();        // ��ȡ�Ƕ�����
    get_all_sensor_data();   // ��ȡ���ٶȺͽ��ٶ�����
    
    // ��ȡ�Ƕ����ݣ�ͨ���������ݣ�
    get_gyro_data(&roll, &pitch, &yaw);
    
    // ��ʽ�������������
    sprintf(str, "R:%.2f,P:%.2f,Y:%.2f,ax:%.2f,ay:%.2f,az:%.2f,gx:%.2f,gy:%.2f,gz:%.2f\r\n", 
            roll, pitch, yaw,
            Gyro_Structure.ax, Gyro_Structure.ay, Gyro_Structure.az,
            Gyro_Structure.gx, Gyro_Structure.gy, Gyro_Structure.gz);
    
    UART1_SendString(str);
		delay_ms(200);
	}
	
	
	
	
}