#include "control.h"	
/**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 知乎：张巧龙 
**************************************************************************/
/**************************************************************************
函数功能：所有的控制代码都在这里面
         Timer3 10ms定时中断
**************************************************************************/

#define SPEED_Y 40 //前后最大设定速度
#define SPEED_Z 100//左右最大设定速度 


//针对不同车型参数，在sys.h内设置define的电机类型
float velocity_KP=SPD_KP;     // 小车速度环PI参数
float velocity_KI=SPD_KI;

void TIM3_IRQHandler(void)   //TIM3中断
{
	static u8 Voltage_Counter=0;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		  Led_Flash(100);
			Encoder_Right=-Read_Encoder(2);                           //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
			Encoder_Left=Read_Encoder(4);                           //===读取编码器的值
		
			Voltage_Counter++;
			if(Voltage_Counter==10)									 //===100ms读取一次超声波的数据以及电压
			{
				Voltage_Counter=0;
				Voltage=Get_battery_volt();		                         //===读取电池电压		
			}
//			if(KEY_Press(100))																				//长按按键切换模式并触发模式切换初始化
//			{
//				if(++CTRL_MODE>=101) CTRL_MODE=97;
//				Mode_Change=1;
//			}
			
//			Get_RC();
//			Kinematic_Analysis(Velocity,Turn);
			
			Moto1=Incremental_PI_A(Encoder_Left,Target_A);                 //===计算左轮电机最终PWM
			Moto2=Incremental_PI_B(Encoder_Right,Target_B);                 //===计算右轮电机最终PWM
			Xianfu_Pwm();  																					 //===PWM限幅
			Turn_Off(Voltage);																 //===检查角度以及电压是否正常
			Set_Pwm(Moto1,Moto2);                                    //===赋值给PWM寄存器  
	}
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=velocity_KP*(Bias-Last_bias)+velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=velocity_KP*(Bias-Last_bias)+velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}


/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
void Kinematic_Analysis(int velocity,int turn)
{
		Target_A=velocity+turn; 
		Target_B=velocity-turn;      //后轮差速
}

void Get_RC()
{
	static u8 SR04_Counter =0;
	static int MAX_VEL=80,MAX_TURN=40;
	float RATE_VEL = 128/MAX_VEL;
	float RATE_TURN = 128/MAX_TURN;
	float step=1;  //设置速度控制步进值。
	float LY,RX;  //PS2手柄控制变量
	int Yuzhi=2;  		//PS2控制防抖阈值
	
	if(CTRL_MODE ==98)//蓝牙APP控制
	{
		switch(Flag_Direction)   //方向控制
		{
			case 0: 
				if(myabs(Velocity)>step)
				{
					if(Velocity>0) Velocity-=step;
					else Velocity+=step;
				}
				else Velocity=0;
				Turn=0;        	 
				break;	 //停止
			case 1:      Velocity+=step;  	 				 Turn=0;       		 break;//前进
			case 2:      Velocity+=step;  	 	 			 Turn+=step;   	 	 break;//右前
			case 3:      Velocity=0;      				 	 Turn+=step;   		 break;//右转
			case 4:      Velocity-=step;  					 Turn-=step; 			 break;//右后
			case 5:      Velocity-=step;  					 Turn=0;       		 break;//后退
			case 6:      Velocity-=step;  					 Turn+=step;   		 break;//左后
			case 7:      Velocity=0;     	 			 	 Turn-=step;			 break;//左转
			case 8:      Velocity+=step; 	 				 Turn-=step;			 break; //左前
			default:     break;	 
		}
	}
	else if(CTRL_MODE == 100)//PS2控制
	{
		if(PS2_Plugin)
		{
			LY=PS2_LY-128; //获取偏差
			RX=PS2_RX-128; //获取偏差
			if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //设置小角度的死区
			if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //设置小角度的死区
			if(Velocity>-LY/RATE_VEL) Velocity-=step;
			else if(Velocity<-LY/RATE_VEL) Velocity+=step;
			Turn=RX/RATE_TURN;
		}
		else
		{
			Velocity=0,Turn=0;
		}
	}
	else if(CTRL_MODE == 97)
	{
		SR04_Counter++;
		if(SR04_Counter>=20)									         //100ms读取一次超声波的数据
		{
			SR04_Counter=0;
			SR04_StartMeasure();												 //读取超声波的值
		}
		if(SR04_Distance<=100)
			Turn=40;
		else
			Turn=0;
	}
	else if(CTRL_MODE == 99)
	{
		Tracking();
		switch(TkSensor)
		{
			case 15:
				Velocity=0;
				Turn=0;
				break;
			case 9:
				Velocity--;
				Turn=0;
				break;
			case 2://向右转
				Velocity--;
				Turn=15;
				break;
			case 4://向左转
				Velocity--;
				Turn=-15;
				break;
			case 8:
				Velocity=-10;
				Turn=-80;
				break;
			case 1:
				Velocity=-10;
				Turn=80;
				break;
		}
	}
	
	if(Velocity>80)Velocity=MAX_VEL;
	else if(Velocity<-80)Velocity=-MAX_VEL;
	if(Turn>40)Turn=MAX_TURN;
	else if(Turn<-40)Turn=-MAX_TURN;
}
void Tracking()
{
	TkSensor=0;
	TkSensor+=(C1<<3);
	TkSensor+=(C2<<2);
	TkSensor+=(C3<<1);
	TkSensor+=C4;
}
