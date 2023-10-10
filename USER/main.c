 /**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 
 知乎：张巧龙 
**************************************************************************/
#include "sys.h"
/****************************全局变量*************************************/    
float Voltage;  															 //电池电压采样相关的变量
float SR04_Distance;                 //超声波测距
int   Encoder_Left,Encoder_Right;         		 //左右编码器的脉冲计数
int 	Moto1=0,Moto2=0,Target_A=0,Target_B=0;												 //计算出来的最终赋给电机的PWM

u8 		CTRL_MODE=97,Mode_Change=0;   //在此选择小车模式  //97超声波避障；98蓝牙；99红外循迹；100 PS2；101自动跟随

u8 		Flag_Direction=0;
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2相关变量
u8 PS2_Plugin=0;

int Velocity=0,Turn=0;

int Uart_Receive=0;
u8 TkSensor=0;
/***********************************************************************/
int mode = 0;

int main(void)
{
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====初始化与按键连接的IO
	delay_init();	    	           //=====延时函数初始化	
	uart1_init(115200);	           //=====串口1初始化
	uart3_init(9600);            //=====初始化串口3
	uart2_init(9600);            //=====初始化串口2
	PS2_Init();
	NVIC_Configuration();					 //=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	Adc_Init();                    //=====初始化ADC
	Encoder_Init_TIM2();           //=====初始化编码器2
	Encoder_Init_TIM4();           //=====初始化编码器4
	TIM1_PWM_Init(7199,0);   			 //=====初始化PWM 10KHZ,用于驱动电机。 
	OLED_Init();
	OLED_Clear();
	oled_first_show();
	Motor_Init();									 //=====初始化与电机连接的硬件IO接口 
	Timer3_Init(100,7199);           //=====10MS进一次中断服务函数，中断服务函数在control.c
	Mode_Change=1;
  while(1)	
	{
		oled_show();
//		if(Mode_Change)
//		{
//			Mode_Change=0;
//			Mode_Init();
//		}
		KEY_Press(1);
			if(mode == 0)
		{
			Target_A = 30;
			Target_B = 30;
		}
		else if(mode == 1)
		{
			Target_A = 50;
			Target_B = 50;
		}
		else if(mode == 2)
		{
			Target_A = 70;
			Target_B = 70;
		}
		else if(mode == 3)
		{
			Target_A = 30;
			Target_B = 30;
		}
		else if(mode == 4)
		{
			Target_A = 50;
			Target_B = 50;
		}
		else if(mode == 5)
		{
			Target_A = 70;
			Target_B = 70;
		}
		
	}
}
void Mode_Init(void)//根据模式不同进行IO初始化
{
	OLED_Clear();									//OLED清屏
	oled_first_show();						//只需要显示一次的字符,在此刷新一次即可。
	switch(CTRL_MODE)
	{
		case 97:
			SR04_Configuration();
			Velocity=20;
			break;
		case 98:
			uart2_init(9600);							 //串口2初始化即蓝牙初始化
			break;
		case 99:
			Tracking_Init();
			break;
		case 100:
			PS2_Init();
			break;
	}
}
void Tracking_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
} 


