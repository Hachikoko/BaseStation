#include "sys.h"
#include "NRF24L01.h"
#include "SPI.h"
#include "delay.h"
#include "main.h"
#include "Timer.h"
#include "string.h"

typedef struct{
	short acc_x;
	short acc_y;
	short acc_z;
	
	short gyr_x;
	short gyr_y;
	short gyr_z;
	
	short mag_x;
	short mag_y;
	short mag_z;
	
	short q0;
	short q1;
	short q2;
	short q3;
}FRAME_DATA;

//��ַ
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x00}; //���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ	
u8 rev_buf[32];
//u8 recev_node_num_for_end_node[7];

extern char ret_words[100];
extern u8 node_num;
extern u8 recv_AN_flag;
u8 curent_search_bandwidth = 10;
u8 space_per_step = 5;  //Ƶ��ɨ����
static FRAME_DATA temp_frame_data;


void NRF24L01_Init(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           //ʹ��GPIO��ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;            //NRF24L01 CE��CS��  PC4��CS   PC5��CE
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	Set_NRF24L01_CE;                                          		//��ʼ��ʱ������
    Set_NRF24L01_CSN;                                   			//��ʼ��ʱ������
	
	//����NRF2401��IRQ
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            //��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);             //�ⲿ�ж��߱��뿪��SYSCFG ʱ��
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);      //ӳ��IO�����ж���
	
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
	
	
	SPI1_init();                                      //��ʼ��SPI
	Clr_NRF24L01_CE; 	                               //ʧ��24L01
	Set_NRF24L01_CSN;                                  //�ر�SPI

	
	//�ر�ͬһ��SPI�ܽŵ�����SPI�豸
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
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 
	  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,100);	     //����RFͨ��Ƶ��	
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	
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
#ifdef TEST_NRF24L01
		sprintf(ret_words,"search frequency %d MHz\r\n",curent_search_bandwidth);
		Uart1_SendString((u8*)ret_words);
#endif
		sprintf(ad_node_words,"AN%02d",node_num);
		Wireless_Send_Data((u8*)ad_node_words);
		delay_ms(10);
		if(recv_AN_flag == 0){                       //��û�н��յ��ڵ㷵�������ֶ�ʱ���ٴη��ͣ�
			Uart1_SendString((u8*)ret_words);
			Wireless_Send_Data((u8*)ad_node_words);
			delay_ms(10);
		}
		curent_search_bandwidth = curent_search_bandwidth + space_per_step;
	}
	curent_search_bandwidth = WORK_FREQUENCY;           //��������Ƶ����2.4GHz+5MHzƵ��
	return node_num -1;
}

u8 search_extra_node(void){
	char ad_node_words[5];
	recv_AN_flag = 0;	
	curent_search_bandwidth = ADD_EXTRA_FREQUENCY;
	sprintf(ad_node_words,"AN%02d",node_num);
#ifdef TEST_NRF24L01
	Uart1_SendString((u8*)ret_words);
#endif
	Wireless_Send_Data((u8*)ad_node_words);
	delay_ms(10);
	if(recv_AN_flag == 0){                       //��û�н��յ��ڵ㷵�������ֶ�ʱ���ٴη��ͣ�
#ifdef TEST_NRF24L01
	Uart1_SendString((u8*)ret_words);
#endif
	Wireless_Send_Data((u8*)ad_node_words);
	delay_ms(10);
	}
	curent_search_bandwidth = WORK_FREQUENCY;           //��������Ƶ����2.4GHz+5MHzƵ��
	return recv_AN_flag;
}

 void EXTI1_IRQHandler(void){
	 
	 u8 RX_Status;
	 int recev_node_num,recev_adsolute_node_num;
	 
	 Fliter_Result_Data fliter_result_data;
	 if(EXTI_GetITStatus(EXTI_Line1) != RESET){
		 Clr_NRF24L01_CE;
		 if(NRF24L01_RxPacket(rev_buf) == 0){
			if('A' == rev_buf[0] && 'N' == rev_buf[1]){
				rev_buf[6] = '\r';
				rev_buf[7] = '\n';
				Uart1_SendString((u8*)rev_buf);
				recev_node_num = (rev_buf[2] - '0') * 10 + rev_buf[3] - '0';
				if(recev_node_num == node_num){                                    //���ͺͽ��յĽڵ�����ͬ����ż�1��
					node_num++;
					recv_AN_flag = 1;
					recev_adsolute_node_num = (rev_buf[4] - '0') * 10 + rev_buf[5] - '0';
//					recev_node_num_for_end_node[recev_adsolute_node_num] = recev_node_num;
				}else{
					sprintf(ret_words,"�ڵ㷵�ر��%d,��վ������%d",recev_node_num,node_num);
					Uart1_SendString((u8*)ret_words);
				}
			}else if('D' == rev_buf[0]){
				int i = 0;
				for(i = 0; i < 32; i++){
					usart1_send_char(rev_buf[i]);
				}
				
//				memset(&temp_frame_data,0,sizeof(temp_frame_data));
//				char recev_adsolute_node_num = rev_buf[1];
//				unsigned int frame_count = *((unsigned int*)(rev_buf+2));
//				temp_frame_data.acc_x = *((short *)(rev_buf+6));
//				temp_frame_data.acc_y = *((short *)(rev_buf+8));
//				temp_frame_data.acc_z = *((short *)(rev_buf+10));
//				temp_frame_data.gyr_x = *((short *)(rev_buf+12));
//				temp_frame_data.gyr_y = *((short *)(rev_buf+14));
//				temp_frame_data.gyr_z = *((short *)(rev_buf+16));
//				temp_frame_data.mag_x = *((short *)(rev_buf+18));
//				temp_frame_data.mag_y = *((short *)(rev_buf+20));
//				temp_frame_data.mag_z = *((short *)(rev_buf+22));
//				
//				temp_frame_data.q0 = *((short *)(rev_buf+24));
//				temp_frame_data.q1 = *((short *)(rev_buf+26));
//				temp_frame_data.q2 = *((short *)(rev_buf+28));
//				temp_frame_data.q3 = *((short *)(rev_buf+30));
//				
//				sprintf(ret_words,"ID:%d,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,mx:%d,my:%d,mz:%d,q0:%d,q1:%d,q2:%d,q3:%d\r\n",recev_adsolute_node_num
//						,temp_frame_data.acc_x,temp_frame_data.acc_y,temp_frame_data.acc_z
//						,temp_frame_data.gyr_x,temp_frame_data.gyr_y,temp_frame_data.gyr_z
//						,temp_frame_data.mag_x,temp_frame_data.mag_y,temp_frame_data.mag_z
//						,temp_frame_data.q0,temp_frame_data.q1,temp_frame_data.q2,temp_frame_data.q3);
//				Uart1_SendString((u8*)ret_words);
//				#ifdef TEST_MPU9250
//				sprintf(ret_words,"ax : %f, ay : %f, az : %f, gx : %f, gy : %f, gz : %f, mx : %f, my : %f, mz : %f\r\n",mpu9250_data.ax,mpu9250_data.ay,mpu9250_data.az
//				,mpu9250_data.gx,mpu9250_data.gy,mpu9250_data.gz,mpu9250_data.mx,mpu9250_data.my,mpu9250_data.mz);
//				Uart1_SendString((u8*)ret_words);
//				#endif
//				complementary_task(&fliter_data,&fliter_result_data,&mpu9250_data,dt);
////				kalman_filter_main(dt,&mup9250_raw_data,&fliter_result_data);

//				#ifdef TEST_MPU_NIMING  //�������
//				MPU9250_report_imu(&mup9250_raw_data,&fliter_result_data);  //�������Խӿ�
//				#endif
//				
//				#ifdef TEST_MPU_PLOT
//				MPU9250_send_data(mup9250_raw_data.ax,mup9250_raw_data.ay,mup9250_raw_data.az,mup9250_raw_data.gx,mup9250_raw_data.gy,mup9250_raw_data.gz,mup9250_raw_data.mx,mup9250_raw_data.my,mup9250_raw_data.mz);
//				#endif
//				
//				#ifdef TEST_MPU_FOR_UP_MORE_THEN_ONE_NODE
//				sprintf(ret_words,"ID:%d,frame_count:%d,%f,%f,%f,%f\r\n",node_index_for_base_station,frame_count,fliter_result_data.q[0],fliter_result_data.q[1],fliter_result_data.q[2],fliter_result_data.q[3]);
//				Uart1_SendString((u8*)ret_words);							//
//				#endif
//				
//				#ifdef TEST_TEST_MPU_FOR_UP_ROOT_NODE
//				sprintf(ret_words,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",fliter_result_data.q[0],fliter_result_data.q[1],fliter_result_data.q[2],fliter_result_data.q[3]
//																	,mpu9250_data.ax,mpu9250_data.ay,mpu9250_data.az,mpu9250_data.gx,mpu9250_data.gy,mpu9250_data.gz);
//				Uart1_SendString((u8*)ret_words);
//				#endif
			}
		}else{
				

		}
	}
	Set_NRF24L01_CE; 
	EXTI_ClearITPendingBit(EXTI_Line1);
}


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
	Clr_NRF24L01_CE;	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 
	  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,curent_search_bandwidth);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1, 0x3f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	Set_NRF24L01_CE;
}	

void TX_Mode(void)
{
	Clr_NRF24L01_CE;	    
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x00); //ʹ��ͨ��0�Ľ��յ�ַ  
 	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x00);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,curent_search_bandwidth);       //����RFͨ��Ϊ40,ADת�����ԣ���ʱ����Ϊ40
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG1,0x3e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	Set_NRF24L01_CE; 
}





//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{

	u8 state;   
    u32 timeout = 0; 
	Clr_NRF24L01_CE;
	NRF24L01_Write_Reg(FLUSH_TX,NOP);   //ˢ�·��ͻ�����ǧ��Ҫȥ��������������ʱ�򲻸���
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	Set_NRF24L01_CE;                                     //��������	   
	while(timeout <2000)
	{
	  timeout++;
	}
	state=NRF24L01_Read_Reg(STATUS);                     //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x7E);      //���TX_DS��MAX_RT�жϱ�־
	if(state&TX_OK)                                      //�������
	{
		return TX_OK;
	}
	return 0xff;                                         //����ԭ����ʧ��
}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ    	 
        //printf("\n\r Rx state %X", state);
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,0x7E); //���TX_DS��MAX_RT�жϱ�־,д1����Ϊ���
	if(state&RX_OK)                                 //���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;                                      //û�յ��κ�����
}


//ͨ��SPIд�Ĵ���
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
    Clr_NRF24L01_CSN;                    //ʹ��SPI����
  	status =SPI1_ReadWriteByte(regaddr); //���ͼĴ����� 
  	SPI1_ReadWriteByte(data);            //д��Ĵ�����ֵ
  	Set_NRF24L01_CSN;                    //��ֹSPI����	   
  	return(status);       		         //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
 	Clr_NRF24L01_CSN;                //ʹ��SPI����		
  	SPI1_ReadWriteByte(regaddr);     //���ͼĴ�����
  	reg_val=SPI1_ReadWriteByte(NOP);//��ȡ�Ĵ�������
  	Set_NRF24L01_CSN;                //��ֹSPI����		    
  	return(reg_val);                 //����״ֵ̬
}	


u8  NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;	    
 	Clr_NRF24L01_CSN;                                    //ʹ��SPI����
  	status = SPI1_ReadWriteByte(regaddr);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //д������	 
  	Set_NRF24L01_CSN;                                    //�ر�SPI����
  	return status;                                       //���ض�����״ֵ̬
}


u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //ʹ��SPI����
  	status=SPI1_ReadWriteByte(regaddr);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(NOP);//��������
  	Set_NRF24L01_CSN;                     //�ر�SPI����
  	return status;                        //���ض�����״ֵ̬
}

u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i = 0;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);//����д��ĵ�ַ  	
	for(i=0;i<5;i++)
	{
	    if(buf1[i]!=0XA5)
		{	//printf("%X ",buf1[i]);
		    break;					   
		}
	}
	if(i!=5)
	{
	
		 Uart1_SendString((u8*)" . ��Ƶģ����ʧ��......\r\n");
	return 1;                               //NRF24L01����λ
	}
		
		 Uart1_SendString((u8*)" . ��Ƶģ����ͨ��......\r\n");
		 
	return 0;		                                //NRF24L01��λ
}

void MPU9250_raw_to_flot(MPU9250_DATD * mpu_data,MPU9250_RAW_DATD* mpu_raw_data){
	u8 ret = 0;
	
	int temp_data;
	int full_data = 32760;

	temp_data = mpu_raw_data->ax;
	mpu_data->ax = (((float)temp_data)/((float)full_data)) * 4.0;
	temp_data = mpu_raw_data->ay;
	mpu_data->ay = (((float)temp_data)/((float)full_data)) * 4.0;
	temp_data = mpu_raw_data->az;
	mpu_data->az = (((float)temp_data)/((float)full_data)) * 4.0;
	
	temp_data = mpu_raw_data->gx;
	mpu_data->gx = (((float)temp_data)/((float)full_data)) * 8.7266465;
	temp_data = mpu_raw_data->gy;
	mpu_data->gy = (((float)temp_data)/((float)full_data)) * 8.7266465;
	temp_data = mpu_raw_data->gz;
	mpu_data->gz = (((float)temp_data)/((float)full_data)) * 8.7266465;
	
	mpu_data->mx = (mpu_raw_data->mx * 0.15) * 0.01;		//ת���Ը�˹Ϊ��λ
	mpu_data->my = (mpu_raw_data->my * 0.15) * 0.01;
	mpu_data->mz = (mpu_raw_data->mz * 0.15) * 0.01;
	
}





