#include "sys.h"
#include "usart.h"	


extern u8 add_extra_node_set_flag;
extern u8 add_extra_node_single_flag;



void Uart1_init(u32 bound){                               //串口1初始化
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2; //响应优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化 VIC 寄存器、
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
}

void Uart1_SendString(u8 *str)
{
  while(*str)
  {
  	 USART_SendData(USART1,*str);											   //将*str从USARTx发送出去
	 while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		       //等待发送完毕
	 str++;	                                                                   //str的地址增加1，到下一个字符
  }
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//如果寄存器中有数据
	{
		uint16_t rec_short;
		USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
		rec_short = USART_ReceiveData(USART1);
		if(*((char*)&rec_short) == ADD_SET_NODES)
		{
			add_extra_node_set_flag = 1;
		}else if(*((char*)&rec_short)== ADD_SINGLE_NODES){
			add_extra_node_single_flag = 1;
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	}
}


//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{
	USART_SendData(USART1,c); 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	
} 

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];	//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void MPU9250_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short meox,short meoy,short meoz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=(meox>>8)&0XFF;
	tbuf[13]=meox&0XFF;
	tbuf[14]=(meoy>>8)&0XFF;
	tbuf[15]=meoy&0XFF;
	tbuf[16]=(meoz>>8)&0XFF;
	tbuf[17]=meoz&0XFF;
	usart1_niming_report(0XA1,tbuf,18);//自定义帧,0XA1
}

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void MPU9250_report_imu(MPU9250_RAW_DATD*mup_raw_data,Fliter_Result_Data*result_data)
{
	u8 tbuf[24]; 
	u8 i;
	short roll,pitch,yaw;
	roll = (short)(result_data->euler[0] * 100);
	pitch = (short)(result_data->euler[1] * 100);
	yaw = (short)result_data->euler[2] * 10;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=((mup_raw_data->ax)>>8)&0XFF;
	tbuf[1]=(mup_raw_data->ax)&0XFF;
	tbuf[2]=((mup_raw_data->ay)>>8)&0XFF;
	tbuf[3]=(mup_raw_data->ay)&0XFF;
	tbuf[4]=((mup_raw_data->az)>>8)&0XFF;
	tbuf[5]=(mup_raw_data->az)&0XFF; 
	tbuf[6]=((mup_raw_data->gx)>>8)&0XFF;
	tbuf[7]=(mup_raw_data->gx)&0XFF;
	tbuf[8]=((mup_raw_data->gy)>>8)&0XFF;
	tbuf[9]=(mup_raw_data->gy)&0XFF;
	tbuf[10]=((mup_raw_data->gz)>>8)&0XFF;
	tbuf[11]=(mup_raw_data->gz)&0XFF;	
	tbuf[12]=((mup_raw_data->mx)>>8)&0XFF;
	tbuf[13]=(mup_raw_data->mx)&0XFF;
	tbuf[14]=((mup_raw_data->my)>>8)&0XFF;
	tbuf[15]=(mup_raw_data->my)&0XFF;
	tbuf[16]=((mup_raw_data->mz)>>8)&0XFF;
	tbuf[17]=(mup_raw_data->mz)&0XFF;
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,24);//飞控显示帧,0XAF
} 






































//////////////////////////////////////////////////////////////////////////////////// 	 
////如果使用ucos,则包括下面的头文件即可.
//#if SYSTEM_SUPPORT_OS
//#include "includes.h"					//ucos 使用	  
//#endif
////////////////////////////////////////////////////////////////////////////////////	 
////本程序只供学习使用，未经作者许可，不得用于其它任何用途
////ALIENTEK STM32F4探索者开发板
////串口1初始化		   
////正点原子@ALIENTEK
////技术论坛:www.openedv.com
////修改日期:2014/6/10
////版本：V1.5
////版权所有，盗版必究。
////Copyright(C) 广州市星翼电子科技有限公司 2009-2019
////All rights reserved
////********************************************************************************
////V1.3修改说明 
////支持适应不同频率下的串口波特率设置.
////加入了对printf的支持
////增加了串口接收命令功能.
////修正了printf第一个字符丢失的bug
////V1.4修改说明
////1,修改串口初始化IO的bug
////2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
////3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
////4,修改了EN_USART1_RX的使能方式
////V1.5修改说明
////1,增加了对UCOSII的支持
//////////////////////////////////////////////////////////////////////////////////// 	  
// 

////////////////////////////////////////////////////////////////////
////加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 
//}; 

//FILE __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//_sys_exit(int x) 
//{ 
//	x = x; 
//} 
////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;      
//	return ch;
//}
//#endif
// 
//#if EN_USART1_RX   //如果使能了接收
////串口1中断服务程序
////注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
////接收状态
////bit15，	接收完成标志
////bit14，	接收到0x0d
////bit13~0，	接收到的有效字节数目
//u16 USART_RX_STA=0;       //接收状态标记	

////初始化IO 串口1 
////bound:波特率
//void uart_init(u32 bound){
//   //GPIO端口设置
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
// 
//	//串口1对应引脚复用映射
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
//	
//	//USART1端口配置
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

//   //USART1 初始化设置
//	USART_InitStructure.USART_BaudRate = bound;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//  USART_Init(USART1, &USART_InitStructure); //初始化串口1
//	
//  USART_Cmd(USART1, ENABLE);  //使能串口1 
//	
//	//USART_ClearFlag(USART1, USART_FLAG_TC);
//	
//#if EN_USART1_RX	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

//	//Usart1 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

//#endif
//	
//}


//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	u8 Res;
//#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
//	OSIntEnter();    
//#endif
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
//		
//		if((USART_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
//				else USART_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}   		 
//  } 
//#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
//	OSIntExit();  											 
//#endif
//} 
//#endif	

 



