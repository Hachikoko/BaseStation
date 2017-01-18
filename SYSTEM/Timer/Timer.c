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
extern int coutn_for_receive_1;
extern int coutn_for_receive_2;
extern int coutn_for_receive_3;
extern int coutn_for_receive_4;
extern int coutn_for_receive_5;
extern int coutn_for_receive_6;
extern int coutn_for_receive_7;
extern int record_for_extra_node;

void Timer7_init(void){
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision =  TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 30*(node_num+1);
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
	u8 buf[5] = {'S','T','A','R',0};
	u8 ret = 0;
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{
		add_extra_nodes_count++;
		show_node_nums_count++;
		
		if(count_for_send == 90000){
			TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
			TIM_Cmd(TIM7,DISABLE); 
			sprintf(ret_words,"all:%d, 1:%d, 2:%d, 3:%d, 4:%d, 5:%d, 6:%d, extra:%d\r\n",count_for_send,coutn_for_receive_1,coutn_for_receive_2
			,coutn_for_receive_3,coutn_for_receive_4,coutn_for_receive_5,coutn_for_receive_6,coutn_for_receive_7);
			Uart1_SendString((u8*)ret_words);
			return;
		}

		//添加额外单个节点
		//if(add_extra_node_single_flag == 1){
		if(count_for_send == 60000){
			count_for_send++;
			TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
			TIM_Cmd(TIM7,DISABLE); 
			#ifdef TEST_VERSION
			Uart1_SendString((u8*)"添加单个节点");
			#endif
			add_extra_nodes_flag = 1;  //重新规划时隙标记
			ret = search_extra_node();
			if (ret == 1){
				sprintf(ret_words,"add new extra node %d successfully!!\r\n",node_num-1);
				Uart1_SendString((u8*)ret_words);
				ret_words[0] = 0;
				
				add_extra_nodes_count = 1;
				record_for_extra_node = count_for_send; //记录添加时的
			}
			add_extra_node_single_flag = 0;
			return;
		}
		//添加一批节点
		if(count_for_send  == 30000){
			u8 current_nodes_nodes = node_num -1;
			count_for_send++;
			TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
			TIM_Cmd(TIM7,DISABLE);
			#ifdef TEST_VERSION
			Uart1_SendString((u8*)"添加多个个节点");
			#endif
			add_extra_nodes_flag = 1;  //重新规划时隙
			curent_search_bandwidth = 10;
			ret = search_node(100);
			sprintf(ret_words,"End_nodes number is %d\r\n",ret);
			if (ret != node_num){
				sprintf(ret_words,"add new extra node %d successfully!!\r\n",node_num - 1 - current_nodes_nodes);
				Uart1_SendString((u8*)ret_words);
				ret_words[0] = 0;
				record_for_extra_node = count_for_send; //记录添加时的
			}
			add_extra_node_set_flag = 0;
			return;
		}
		
		if(count_for_send % 1000 == 0){
//			sprintf(ret_words,"all:%d, 1:%d, 2:%d, 3:%d, 4:%d, 5:%d, 6:%d, record_for_extra:%d, extra:%d\r\n",count_for_send,coutn_for_receive_1,coutn_for_receive_2
//			,coutn_for_receive_3,coutn_for_receive_4,coutn_for_receive_5,coutn_for_receive_6,record_for_extra_node,coutn_for_receive_7);
			sprintf(ret_words,"all:%d, 1:%d, 2:%d, 3:%d, 5:%d, 6:%d, 7:%d, extra:%d\r\n",count_for_send,coutn_for_receive_1,coutn_for_receive_2
			,coutn_for_receive_3,coutn_for_receive_5,coutn_for_receive_6,coutn_for_receive_7,coutn_for_receive_4);
			Uart1_SendString((u8*)ret_words);
			ret_words[0] = 0;
		}

		if(TX_OK==Wireless_Send_Data(buf)){
			count_for_send++;
			sprintf(ret_words,"%d\r\n",node_num + 1);
			Uart1_SendString((u8*)ret_words);
			ret_words[0] = 0;
		}else {
			Uart1_SendString((u8*)" . 射频模块发射失败......\r\n");
		}	
	}		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
}



