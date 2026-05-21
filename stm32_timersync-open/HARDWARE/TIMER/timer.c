#include "timer.h"
#include "led.h"
#include "usart.h"
#include "sys.h"
#include <stdio.h>
#include <string.h>

//1,����TIM3_PWM_Init������
//2,����LED0_PWM_VAL�궨�壬����TIM3_CH2����									  
//////////////////////////////////////////////////////////////////////////////////  
   	  
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,���������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}




vu16 varl=0;
//��ʱ��3�жϷ������
vu16 var_Exp=0;
vu16 global_time;
char snum[7];
vu16 shorttt=0;

char gprmcStr[7]="$GPRMC,";
int chckNum=0;
char chckNumChar[2];

int ss=0;
int mm=0;
int hh=0;

unsigned char result;
int i;
int checkNum(const char *gprmcContext)
{
    if (gprmcContext == NULL) 
    {
        // printf("Input is NULL.\n");
        return -1;
		}

    result = gprmcContext[1];

    for (i = 2; gprmcContext[i] != '*' && gprmcContext[i] != '\0'; i++)
    {
        // printf("Processing character: %c (ASCII: %d)\n", gprmcContext[i], gprmcContext[i]);
        result ^= gprmcContext[i];
    }

    if (gprmcContext[i] != '*') 
    {
        // printf("No '*' found in the string.\n");
        return -1;
    }

    //printf("Final result before returning: %02X\n", result);
    return result;
}

char value_1[100]="";
char value_2[100]="";
char value_time[10]="";

char test[100]="$GPRMC,004015,A,2812.0498,N,11313.1361,E,0.0,180.0,150122,3.9,W,A*";
void TIM3_IRQHandler(void)   //TIM3�ж�
{


		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		LED1=!LED1;
			
			//var_Exp �����ع�ʱ�������Ʋ���
		TIM2->CNT=TIM2->ARR/2;

		PCout(13)=0;
		}
		
		
		//global_time++;
		//shorttt=0;
		//sprintf(snum, "%06d", global_time); //������"000011D7"
		//snum[6]=0;
		//printf("$GPRMC,");
	  //printf(snum);
		//printf(".00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*23\n");
		

		
		//************************************* add *********************************************
		//UTCtime format: hhmmss
		
    if(ss<59){
				ss++;
		}else{
			  ss=0;
			 if(mm<59){
				 mm++;
			 }else{
				 mm=0;
				 if(hh<23){
					 hh++;
				 }else{
					 hh=0;
				 }
			 }
		}
		
    sprintf(value_2, "%s%02d%02d%02d%s", gprmcStr, hh, mm, ss, ".00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*");
		strcpy(value_1,value_2);
	  chckNum =checkNum(value_1);
	  sprintf(chckNumChar, "%02X", chckNum);
		printf("%s", value_2);
    printf("%s\n", chckNumChar);
	 
		
	  //**********************************************************************************

		/**
		chckNum=checkNum(test);	
	  sprintf(chckNumChar, "%02X", chckNum);
    printf("%s\n", chckNumChar);
    */
}



void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	TIM_DeInit(TIM2);
 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH1  B6 pin out
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //??????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//???GPIO
 
   //???TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	
	//???TIM4 Channel1 PWM??	 
	TIM_OCInitStructure.TIM_Pulse = 990; // HIGH = 10 ticks = 1ms (PWM2: HIGH when CNT >= CCR)
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //???????:TIM????????2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //????:TIM???????
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //??TIM4?CCR2????????
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //??T??????????TIM4 OC2
	
	//???TIM4 Channel2 PWM??	 
//	TIM_OCInitStructure.TIM_Pulse = 500; //????????????????
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //???????:TIM????????2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //????:TIM???????
//	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //??TIM4?CCR2????????
//	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //??T??????????TIM4 OC2

 	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);  //??TIM4
	


 //	TIM_SetCompare2(TIM2,50);	 // ccr  set pwm value
TIM_SetCompare2(TIM2,990);
}



//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��

//
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
 
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	
	TIM_SetCompare2(TIM3,TIM3->ARR/2);	

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,���������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}
