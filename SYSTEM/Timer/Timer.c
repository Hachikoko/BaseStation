#include "Timer.h"
#include "NRF24L01.h"
#include "delay.h"
#include "main.h"

extern u8 node_num;
extern u8 add_extra_nodes_count;
extern u8 curent_search_bandwidth;
extern char ret_words[200];
extern u8 add_extra_nodes_flag;
extern int show_node_nums_count;
extern u8 add_extra_node_set_flag;
extern u8 add_extra_node_single_flag;


extern int count_for_send;
extern int record_for_extra_node;

u8 buf[6];
static unsigned int absolute_frame_num = 0;

void Timer7_init(void){
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision =  TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 40*(node_num+1);
	TIM_TimeBaseInitStruct.TIM_Prescaler = 8399;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStruct);          //计数器初始化
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );                //中断使能
	TIM_Cmd(TIM7,DISABLE);
	return;
}

void TIM7_IRQHandler(void){

	u8 ret = 0;
	buf[0] = 'S';
	buf[1] = 'T';
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{
//		add_extra_nodes_count++;
//		show_node_nums_count++;
		absolute_frame_num++;
		(*((unsigned int *)(buf+2))) = absolute_frame_num;
		if(TX_OK==Wireless_Send_Data(buf)){
			count_for_send++;
		}else {
			Uart1_SendString((u8*)" . 射频模块发射失败......\r\n");
		}	
	}		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
}



