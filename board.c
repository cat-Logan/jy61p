#include "board.h"
#include "delay.h"
//#include "uart.h"
//内核频率
int32 sys_clk = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      STC32G设置系统频率
//  @param      NULL          	空值
//  @return     void        	系统频率
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 set_clk(uint32 clock)
{
	uint32 temp_sys_clk = 0;
	
	
    switch(clock)
    {
		case SYSTEM_CLOCK_500K:
        //选择500KHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
        _nop_();
        _nop_();
        _nop_();
        CLKDIV = 48;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_3M:
        //选择3MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 8;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_55_296M:
        //选择5.5296MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 4;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_6M:
        //选择6MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        WTST = 0;       //设置程序读取等待控制寄存器
        CLKDIV = 4;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_11_0592M:
        //选择11.0592MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_12M:
        //选择12MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_22_1184M:
        //选择22.1184MHz
        CLKDIV = 0x04;
        IRTRIM = T22M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_24M:
        //选择24MHz
        CLKDIV = 0x04;
        IRTRIM = T24M_ADDR;
        VRTRIM = VRT24M_ADDR;
        IRCBAND = IRCBAND_24M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_30M:
        //选择30MHz
        CLKDIV = 0x04;
        IRTRIM = T60M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_32M:
        //选择32MHz
        CLKDIV = 0x04;
        IRTRIM = T64M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 0;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 2;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_40M:
        //选择40MHz
        CLKDIV = 0x04;
        IRTRIM = T40M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_45_1584M:
        //选择45.1584MHz
        CLKDIV = 0x04;
        IRTRIM = T45M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_48M:
        //选择48MHz
        CLKDIV = 0x04;
        IRTRIM = T48M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_50M:
        //选择50.8032MHz
        CLKDIV = 0x04;
        IRTRIM = T50M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_52M:
        //选择52MHz
        CLKDIV = 0x04;
        IRTRIM = T52M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_56M:
        //选择56MHz
        CLKDIV = 0x04;
        IRTRIM = T56M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

    case SYSTEM_CLOCK_60M:
        //选择60MHz
        CLKDIV = 0x04;
        IRTRIM = T60M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = clock;
        break;

//    case SYSTEM_CLOCK_64M:
//        //选择64MHz
//        CLKDIV = 0x04;
//        IRTRIM = T64M_ADDR;
//        VRTRIM = VRT44M_ADDR;
//        IRCBAND = IRCBAND_44M;
//        WTST = 2;       //设置程序读取等待控制寄存器
//        CLKDIV = 1;
//        break;

    default:
        //选择56MHz
        CLKDIV = 0x04;
        IRTRIM = T56M_ADDR;
        VRTRIM = VRT44M_ADDR;
        IRCBAND = IRCBAND_44M;
        WTST = 1;       //设置程序读取等待控制寄存器
		_nop_();
		_nop_();
		_nop_();
		_nop_();
        CLKDIV = 1;
		temp_sys_clk = SYSTEM_CLOCK_56M;
        break;
	}
	
	return temp_sys_clk;
}







void DisableGlobalIRQ(void)
{
	EA = 0;
}


void EnableGlobalIRQ(void)
{
	EA = 1;
}

void enalbe_icache(void)
{
	EA = 0;
	_nop_();
	_nop_();
	TA = 0xaa; 		//写入触发命令序列1
					//此处不能有其他任何指令
	TA = 0x55; 		//写入触发命令序列2
					//此处不能有其他任何指令
	ICHECR = 0x01; 	//写保护暂时关闭，可以修改ICHECR 中的EN 位
					//EN 位再次进入写保护状态
	_nop_();
	_nop_();
	EA = 1;
}

void disalbe_icache(void)
{
	EA = 0;
	_nop_();
	_nop_();
	TA = 0xaa; 		//写入触发命令序列1
					//此处不能有其他任何指令
	TA = 0x55; 		//写入触发命令序列2
					//此处不能有其他任何指令
	ICHECR = 0x00; 	//写保护暂时关闭，可以修改ICHECR 中的EN 位
					//EN 位再次进入写保护状态
	_nop_();
	_nop_();
	EA = 1;
}


void ICacheOn() //打开ICACHE 功能
{
	_nop_();
	_nop_();
	TA = 0xaa; //写入触发命令序列1
	//此处不能有其他任何指令
	TA = 0x55; //写入触发命令序列2
	//此处不能有其他任何指令
	ICHECR = 0x01; //写保护暂时关闭，可以修改ICHECR 中的EN 位
	//EN 位再次进入写保护状态
	_nop_();
	_nop_();
}

void clock_init(uint32 clock)
{
	P_SW2 = 0x80; // 开启特殊地址访问
	
	#if (1 == EXTERNAL_CRYSTA_ENABLE)
	{
		sys_clk = clock;
	}
	#else
	{
		sys_clk = set_clk(clock);
	}
	#endif

}


void board_init(void)
{

	SET_P54_RESRT;			// 使P54为复位引脚

	
	P0M0 = 0x00;
	P0M1 = 0x00;
	P1M0 = 0x00;
	P1M1 = 0x00;
	P2M0 = 0x00;
	P2M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P4M0 = 0x00;
	P4M1 = 0x00;
	P5M0 = 0x00;
	P5M1 = 0x00;   
//	P6M0 = 0x00;
//	P6M1 = 0x00;
//	P7M0 = 0x00;
//	P7M1 = 0x00;
	

#if (1 == EXTERNAL_CRYSTA_ENABLE)
{

	XOSCCR = 0xc0; 			//启动外部晶振
	while(!(XOSCCR & 1)); 	//等待时钟稳定

	CLKDIV = 0x00; 			//时钟不分频
	CLKSEL = 0x01; 			//选择外部晶振
}

#endif

	delay_init();			//延时函数初始化

	if(sys_clk > SYSTEM_CLOCK_32M)
	{
		//STC32F 系列单片机内部硬件逻辑及外设可运行在60MHz 的主频下，但内部的程序存储器（ROM）
		//相对较慢，时钟只能在33M 左右，为解决这个时钟不一致的问题，STC32F 系列增加了ICACHE 技术，
		//即程序存储器高速缓存技术。
		ICacheOn();
	}
	


	ADCCFG = 0;
	AUXR = 0;
	SCON = 0;
	S2CON = 0;
	S3CON = 0;
	S4CON = 0;
	P_SW1 = 0;
	IE2 = 0;
	TMOD = 0;
//	MCLKOCR =1<<7| 10;
	//uart_init(UART1_RX_P30, UART1_TX_P31, 115200, TIM_1);
	EnableGlobalIRQ();
}