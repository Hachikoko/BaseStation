#include "sys.h"
#include "NRF24L01.h"
#include "SPI.h"
#include "delay.h"
#include "main.h"
#include "Timer.h"

//地址
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x00}; //发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接收地址	
u8 rev_buf[28];

extern char ret_words[100];
extern u8 node_num;
extern u8 recv_AN_flag;
u8 curent_search_bandwidth = 10;
u8 space_per_step = 5;


extern int coutn_for_receive_1;
extern int coutn_for_receive_2;
extern int coutn_for_receive_3;
extern int coutn_for_receive_4;
extern int coutn_for_receive_5;
extern int coutn_for_receive_6;
extern int coutn_for_receive_7;


void NRF24L01_Init(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           //使能GPIO的时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;            //NRF24L01 CE和CS脚  PC4：CS   PC5：CE
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	Set_NRF24L01_CE;                                          		//初始化时先拉高
    Set_NRF24L01_CSN;                                   			//初始化时先拉高
	
	//配置NRF2401的IRQ
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            //上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);             //外部中断线必须开启SYSCFG 时钟
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);      //映射IO口与中断线
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
	
	
	SPI1_init();                                      //初始化SPI
	Clr_NRF24L01_CE; 	                               //失能24L01
	Set_NRF24L01_CSN;                                  //关闭SPI

	
	//关闭同一组SPI管脚的其他SPI设备
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);          
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;              
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_SetBits(GPIOG,GPIO_Pin_15);
	
	delay_ms(100);
	
	Clr_NRF24L01_CE;	  
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
	  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,100);	     //设置RF通信频率	
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	
	Set_NRF24L01_CE;                                  
	
	return;
}

int search_node(int bandwidth ){
	char ad_node_words[5];
	if(bandwidth > 115){
		sprintf(ret_words,"in search_node,bandwidth %d MHz is too large!!\r\n",bandwidth);
		Uart1_SendString((u8*)ret_words);
		return -1;
	}
	for(;curent_search_bandwidth <= bandwidth;){
		recv_AN_flag = 0;	
#ifdef TEST_VERSION
		sprintf(ret_words,"search frequency %d MHz\r\n",curent_search_bandwidth);
		Uart1_SendString((u8*)ret_words);
#endif
		sprintf(ad_node_words,"AN%02d",node_num);
		Wireless_Send_Data((u8*)ad_node_words);
		delay_ms(10);
		if(recv_AN_flag == 0){                       //当没有接收到节点返回组网字段时，再次发送；
			Uart1_SendString((u8*)ret_words);
			Wireless_Send_Data((u8*)ad_node_words);
			delay_ms(10);
		}
		curent_search_bandwidth = curent_search_bandwidth + space_per_step;
	}
	curent_search_bandwidth = WORK_FREQUENCY;           //正常工作频率在2.4GHz+5MHz频率
	return node_num -1;
}

u8 search_extra_node(void){
	char ad_node_words[5];
	recv_AN_flag = 0;	
	curent_search_bandwidth = ADD_EXTRA_FREQUENCY;
	sprintf(ad_node_words,"AN%02d",node_num);
#ifdef TEST_VERSION
	Uart1_SendString((u8*)ret_words);
#endif
	Wireless_Send_Data((u8*)ad_node_words);
	delay_ms(10);
	if(recv_AN_flag == 0){                       //当没有接收到节点返回组网字段时，再次发送；
#ifdef TEST_VERSION
	Uart1_SendString((u8*)ret_words);
#endif
	Wireless_Send_Data((u8*)ad_node_words);
	delay_ms(10);
	}
	curent_search_bandwidth = WORK_FREQUENCY;           //正常工作频率在2.4GHz+5MHz频率
	return recv_AN_flag;
}

 void EXTI1_IRQHandler(void){
	 
	 u8 RX_Status;
	 u8 recev_node_num = 0;
//	 delay_us(100);
	 if(EXTI_GetITStatus(EXTI_Line1) != RESET){
		 Clr_NRF24L01_CE;
		 RX_Status=NRF24L01_RxPacket(rev_buf);
		 if(RX_Status == 0){
			rev_buf[27] = '\0';
			Uart1_SendString(rev_buf);
			if('A' == rev_buf[0] && 'N' == rev_buf[1]){
				recev_node_num = (rev_buf[2] - '0') * 10 + rev_buf[3] - '0';
				if(recev_node_num == node_num){                                    //发送和接收的节点编号相同，编号加1；
					node_num++;
					recv_AN_flag = 1;
				}else{
					sprintf(ret_words,"节点返回编号%d,基站保存编号%d",recev_node_num,node_num);
					Uart1_SendString((u8*)ret_words);
					ret_words[0] = 0;
				}
			}else{
				int node_num = (rev_buf[0] - '0') * 10 + rev_buf[1] - '0';
//				#ifdef TEST_VERSION
//				sprintf(ret_words,"receive: %02d\r\n",node_num);
//				Uart1_SendString((u8*)ret_words);
//				ret_words[0] = 0;
//				#endif
				switch (node_num){
					case 1:
						coutn_for_receive_1++;
						break;
					case 2:
						coutn_for_receive_2++;
						break;
					case 3:
						coutn_for_receive_3++;
						break;
					case 4:
						coutn_for_receive_4++;
						break;
					case 5:
						coutn_for_receive_5++;
						break;
					case 6:
						coutn_for_receive_6++;
						break;
					case 7:
						coutn_for_receive_7++;
						break;
					default:
						break;
				}

				}
			}else{
				

			}
		 }
		Set_NRF24L01_CE; 
		EXTI_ClearITPendingBit(EXTI_Line1);
	 }
	 
	 
 

//void set_frequency(int frequency){
//	Clr_NRF24L01_CE;	  
////	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
////	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
//		
////	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    //使能通道0的自动应答    
////	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
//	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,frequency);	     //设置RF通信频率		  
////	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
////	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
////	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
//	Set_NRF24L01_CE;
//	
//	return;
//}


u8  Wireless_Send_Data(u8 *txbuf){
	u8 ret = 0;
	TX_Mode();
	delay_us(100);
    ret = NRF24L01_TxPacket(txbuf);
	delay_us(200);
	RX_Mode();
    delay_us(100);
	return ret;
}


void RX_Mode(void)
{
//	Clr_NRF24L01_CE;
//	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
//	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
//	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,100);	     //设置RF通信频率		  	
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
//  	Set_NRF24L01_CE;
	
	Clr_NRF24L01_CE;	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
	  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,curent_search_bandwidth);	     //设置RF通信频率		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	Set_NRF24L01_CE;
}	

void TX_Mode(void)
{
//	Clr_NRF24L01_CE;
//	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
//	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 	
////	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,125);       //设置RF通道为40,AD转换测试，暂时设置为40    
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3e);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式 
//  	Set_NRF24L01_CE;
	
	Clr_NRF24L01_CE;	    
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x00); //使能通道0的接收地址  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x00);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,curent_search_bandwidth);       //设置RF通道为40,AD转换测试，暂时设置为40
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1,0x3e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	Set_NRF24L01_CE; 
}





//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{

	u8 state;   
    u32 timeout = 0; 
	Clr_NRF24L01_CE;
	NRF24L01_Write_Reg(FLUSH_TX,NOP);   //刷新发送缓冲器千万不要去掉，否则数据有时候不更新
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	Set_NRF24L01_CE;                                     //启动发送	   
	while(timeout <2000)
	{
	  timeout++;
	}
	state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x7E);      //清除TX_DS或MAX_RT中断标志
	if(state&TX_OK)                                      //发送完成
	{
		return TX_OK;
	}
	return 0xff;                                         //其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值    	 
        //printf("\n\r Rx state %X", state);
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x7E); //清除TX_DS或MAX_RT中断标志,写1操作为清除
	if(state&RX_OK)                                 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,NOP);          //清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;                                      //没收到任何数据
}


//通过SPI写寄存器
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
    Clr_NRF24L01_CSN;                    //使能SPI传输
  	status =SPI1_ReadWriteByte(regaddr); //发送寄存器号 
  	SPI1_ReadWriteByte(data);            //写入寄存器的值
  	Set_NRF24L01_CSN;                    //禁止SPI传输	   
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
 	Clr_NRF24L01_CSN;                //使能SPI传输		
  	SPI1_ReadWriteByte(regaddr);     //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(NOP);//读取寄存器内容
  	Set_NRF24L01_CSN;                //禁止SPI传输		    
  	return(reg_val);                 //返回状态值
}	


u8  NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;	    
 	Clr_NRF24L01_CSN;                                    //使能SPI传输
  	status = SPI1_ReadWriteByte(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //写入数据	 
  	Set_NRF24L01_CSN;                                    //关闭SPI传输
  	return status;                                       //返回读到的状态值
}


u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //使能SPI传输
  	status=SPI1_ReadWriteByte(regaddr);   //发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(NOP);//读出数据
  	Set_NRF24L01_CSN;                     //关闭SPI传输
  	return status;                        //返回读到的状态值
}

u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i = 0;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);//读出写入的地址  	
	for(i=0;i<5;i++)
	{
	    if(buf1[i]!=0XA5)
		{	//printf("%X ",buf1[i]);
		    break;					   
		}
	}
	if(i!=5)
	{
	
		 Uart1_SendString((u8*)" . 射频模块检测失败......\r\n");
	return 1;                               //NRF24L01不在位
	}
		
		 Uart1_SendString((u8*)" . 射频模块检测通过......\r\n");
		 
	return 0;		                                //NRF24L01在位
}





