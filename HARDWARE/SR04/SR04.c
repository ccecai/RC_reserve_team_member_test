#include "SR04.h"

/**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
 ΢�Ź��ںš���������ˡ�
 ��̨�ظ���ƽ��С��������ȡƽ��С��ȫ��DIY����
 ��̨�ظ������ӿ������ߡ�����ȡ���ӹ���ʦ�ر���������
 ��̨�ظ�������������ϡ�����ȡ����������ϰ�
 ֪���������� 
**************************************************************************/
#define	TRIG_PIN       GPIO_Pin_10   //TRIG       
#define	ECHO_PIN       GPIO_Pin_11		//ECHO   
/*
 * ��������SR04_Configuration
 * ����  ��������ģ��ĳ�ʼ��
 * ����  ����
 * ���  ����	
 */
void SR04_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;					 //PB10��TRIG
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     //��Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(GPIOB, &GPIO_InitStructure);	                 //��ʼ������GPIO 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     //PB11��ECH0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		 //��Ϊ����
  GPIO_Init(GPIOB,&GPIO_InitStructure);						 //��ʼ��GPIOB
	
	 //GPIOB.11	  �ж����Լ��жϳ�ʼ������
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11);

 	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���	
}


void EXTI15_10_IRQHandler(void)
{
	u8 fac_us=SystemCoreClock/8000000;
	u32 told,tnow;
	float tcnt=0;
	Flag_Direction=16;
	delay_us(10);		                      //��ʱ10us
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
			SysTick->LOAD=65535;					//LOAD��ֵ
			SysTick->VAL=65535;        					//�ս���ʱ�ļ�����ֵ
			SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	
			while(GPIO_ReadInputDataBit(GPIOB,ECHO_PIN));	       				//�ȴ��͵�ƽ
			tnow=SysTick->VAL;
			SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
			tcnt+=65535-tnow;
			SR04_Distance=tcnt*340/fac_us/1000/7.25;			//������룬δ��ɴ�����
	}
	EXTI_ClearITPendingBit(EXTI_Line11);  //���EXTI11��·����λ
}		

/*
 * ��������SR04_StartMeasure
 * ����  ����ʼ��࣬����һ��>10us�����壬Ȼ��������صĸߵ�ƽʱ��
 * ����  ����
 * ���  ����	
 */
void SR04_StartMeasure(void)
{
  GPIO_SetBits(GPIOB,TRIG_PIN); 		  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����˵
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(GPIOB,TRIG_PIN);
}

/******************* (C) 1209 Lab *****END OF FILE************/
