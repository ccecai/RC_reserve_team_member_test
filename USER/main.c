 /**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
 
 ΢�Ź��ںš���������ˡ�
 ��̨�ظ���ƽ��С��������ȡƽ��С��ȫ��DIY����
 ��̨�ظ������ӿ������ߡ�����ȡ���ӹ���ʦ�ر���������
 ��̨�ظ�������������ϡ�����ȡ����������ϰ�
 
 ֪���������� 
**************************************************************************/
#include "sys.h"
/****************************ȫ�ֱ���*************************************/    
float Voltage;  															 //��ص�ѹ������صı���
float SR04_Distance;                 //���������
int   Encoder_Left,Encoder_Right;         		 //���ұ��������������
int 	Moto1=0,Moto2=0,Target_A=0,Target_B=0;												 //������������ո��������PWM

u8 		CTRL_MODE=97,Mode_Change=0;   //�ڴ�ѡ��С��ģʽ  //97���������ϣ�98������99����ѭ����100 PS2��101�Զ�����

u8 		Flag_Direction=0;
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2��ر���
u8 PS2_Plugin=0;

int Velocity=0,Turn=0;

int Uart_Receive=0;
u8 TkSensor=0;
/***********************************************************************/
int mode = 0;

int main(void)
{
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	KEY_Init();                    //=====��ʼ���밴�����ӵ�IO
	delay_init();	    	           //=====��ʱ������ʼ��	
	uart1_init(115200);	           //=====����1��ʼ��
	uart3_init(9600);            //=====��ʼ������3
	uart2_init(9600);            //=====��ʼ������2
	PS2_Init();
	NVIC_Configuration();					 //=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	Adc_Init();                    //=====��ʼ��ADC
	Encoder_Init_TIM2();           //=====��ʼ��������2
	Encoder_Init_TIM4();           //=====��ʼ��������4
	TIM1_PWM_Init(7199,0);   			 //=====��ʼ��PWM 10KHZ,������������� 
	OLED_Init();
	OLED_Clear();
	oled_first_show();
	Motor_Init();									 //=====��ʼ���������ӵ�Ӳ��IO�ӿ� 
	Timer3_Init(100,7199);           //=====10MS��һ���жϷ��������жϷ�������control.c
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
void Mode_Init(void)//����ģʽ��ͬ����IO��ʼ��
{
	OLED_Clear();									//OLED����
	oled_first_show();						//ֻ��Ҫ��ʾһ�ε��ַ�,�ڴ�ˢ��һ�μ��ɡ�
	switch(CTRL_MODE)
	{
		case 97:
			SR04_Configuration();
			Velocity=20;
			break;
		case 98:
			uart2_init(9600);							 //����2��ʼ����������ʼ��
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��PA�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	           //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��PA�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	           //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
} 


