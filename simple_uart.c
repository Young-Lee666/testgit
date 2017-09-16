/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdint.h>
#include "nrf.h"
#include "simple_uart.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "v_timer.h"
#include "parser.h"
#include "variable.h"
#include "api.h"
#include "boards.h"
#include "string.h"
#include "crc16.h"
#include "radio_config.h"
extern uint8_t rcv_ok;
extern uint8_t rcv_idx;
extern uint8_t rcv_uart[RECEIVE_BUFFER_SIZE];
extern uint8_t Snd_Buf[TRANSFER_BUFFER_SIZE];
uint16_t snd_idx = 0;
uint16_t snd_len = 0;
uint16_t snd_int_idx = 0;
extern uint8_t led_delay;
uint8_t xoren = 0;//0-关闭状态，1-开始计算，2-正在计算，3-计算结束。
uint8_t frame2_ok = 0;
void FeedWDT(void);

const uint8_t uart_soh[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0x00};

char *query="\
at+netmode=?\r\n\
at+wifi_conf=?\r\n\
at+dhcpd=?\r\n\
at+dhcpd_ip=?\r\n\
at+dhcpd_dns=?\r\n\
at+dhcpd_time=?\r\n\
at+dhcpc=?\r\n\
at+net_ip=?\r\n\
at+net_dns=?\r\n\
at+net_wanip=?\r\n\
at+remoteip=?\r\n\
at+remoteport=?\r\n\
at+remotepro=?\r\n\
at+timeout=?\r\n\
at+mode=?\r\n\
at+uart=?\r\n\
at+uartpacklen=?\r\n\
at+uartpacktimeout=?\r\n\
at+ver=?\r\n\
";

// at+netmode=1\r\n\
// at+dhcpc=0\r\n\
// at+net_ip=192.168.11.254,255.255.255.0,192.168.11.1\r\n\
// at+net_dns=192.168.11.1,8.8.8.8\r\n\
// at+remoteip=192.168.11.245\r\n\
// at+remoteport=8080\r\n\
// at+remotepro=tcp\r\n\
// at+timeout=0\r\n\
// at+mode=server\r\n\
// at+uart=115200,8,n,1\r\n\
// at+uartpacklen=64\r\n\
// at+uartpacktimeout=10\r\n\
// at+net_commit=1\r\n\
// at+reconn=1\r\n\
uint8_t end_idx = 0;
#define UART0_IRQ_AT			0
#define UART0_IRQ_TRANS		1
uint8_t uart_status = UART0_IRQ_AT;

uint8_t first = 0;
uint8_t tail = 0;

uint8_t t_first,t_tail,t_len;
uint8_t tmpxor;
uint16_t len;

#define END_MAX  11

#define MAX_TIMEOUT_LOOPS 10000000

uint8_t simple_uart_get(void)
{
  while (NRF_UART0->EVENTS_RXDRDY != 1)
  {
    // Wait for RXD data to be received
  }
  
  NRF_UART0->EVENTS_RXDRDY = 0;
  return (uint8_t)NRF_UART0->RXD;
}

bool simple_uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data)
{
  bool ret = true;
  
  while (NRF_UART0->EVENTS_RXDRDY != 1)
  {
    if (timeout_ms-- >= 0)
    {
      // wait in 1ms chunk before checking for status
      nrf_delay_us(1000);
    }
    else
    {
      ret = false;
      break;
    }
  }  // Wait for RXD data to be received

  if (timeout_ms >= 0)
  {
    // clear the event and set rx_data with received byte
      NRF_UART0->EVENTS_RXDRDY = 0;
      *rx_data = (uint8_t)NRF_UART0->RXD;
  }

  return ret;
}

void simple_uart_put(uint8_t cr)
{	
	NRF_UART0->TXD = cr;
  while (NRF_UART0->EVENTS_TXDRDY!=1)
  {
    // Wait for TXD data to be sent
  }
  NRF_UART0->EVENTS_TXDRDY=0;
}

void simple_uart_putstring(const uint8_t *str)
{
  uint_fast8_t i = 0;
  uint8_t ch = str[i++];
  while (ch != '\0')
  {
    simple_uart_put(ch);
    ch = str[i++];
  }
}

void uart_putarray(const uint8_t *str,uint8_t len)
{
  uint_fast8_t i = 0;
	static uint8_t tmpxor;
  uint8_t ch;
	
	switch(xoren)
	{
		case 1://触发校验开始
			xoren = 2;
			tmpxor = 0;
		case 0://发送枕头
		case 2://发送数据内容
			for(i=0;i<len;i++)
			{
				ch = str[i];
				simple_uart_put(ch);
				tmpxor ^= ch;
			}
			break;
		case 3://发送校验值
			simple_uart_put(tmpxor);
			break;
		default:
			break;
	}
}
void uart_send(uint8_t *p,uint16_t len)
{
	snd_len = len;
	snd_int_idx = 0;
	if(snd_len)
		NRF_UART0->TXD = Snd_Buf[snd_int_idx];
}

void uart_send_ver(uint8_t srl)
{
	uint16_t crc;
	uint32_t i;
	for(i= 0;i<MAX_TIMEOUT_LOOPS;i++)
	{
		if(snd_len == 0)
			break;
	}
	
	if(i == MAX_TIMEOUT_LOOPS)
		return;
	Snd_Buf[0] = 0xaa;
	Snd_Buf[1] = 0xaa;
	memcpy(Snd_Buf+2,DeliverID,4);
	Snd_Buf[6] = srl;
	Snd_Buf[7] = 0x8c;
	Snd_Buf[8] = 00;
	Snd_Buf[9] = sizeof(AppVer)+1;
	Snd_Buf[10] = 0;
	memcpy(Snd_Buf+11,AppVer,sizeof(AppVer));
	crc = crc16(Snd_Buf+2,9+sizeof(AppVer));
	Snd_Buf[11+sizeof(AppVer)] = (uint8_t)(crc>>8);
	Snd_Buf[12+sizeof(AppVer)] = (uint8_t)crc;
	uart_send(Snd_Buf,13+sizeof(AppVer));
}

void simple_uart_config(  uint8_t rts_pin_number,
                          uint8_t txd_pin_number,
                          uint8_t cts_pin_number,
                          uint8_t rxd_pin_number,
                          bool hwfc)
{
  nrf_gpio_cfg_output(txd_pin_number);
  nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);  

  NRF_UART0->PSELTXD = txd_pin_number;
  NRF_UART0->PSELRXD = rxd_pin_number;

  if (hwfc)
  {
    nrf_gpio_cfg_output(rts_pin_number);
    nrf_gpio_cfg_input(cts_pin_number, NRF_GPIO_PIN_NOPULL);
    NRF_UART0->PSELCTS = cts_pin_number;
    NRF_UART0->PSELRTS = rts_pin_number;
    NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
  }

  NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud38400 << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0;
	
	//vendor define
	NRF_UART0->INTENSET					= (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos);
	NVIC_SetPriority(UART0_IRQn,UART_Priority);
	NVIC_EnableIRQ( UART0_IRQn );
}


void uart_config(uint8_t txd_pin_number,
								uint8_t rxd_pin_number)
{
	nrf_gpio_cfg_output(txd_pin_number);
  nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);  

  NRF_UART0->PSELTXD = txd_pin_number;
  NRF_UART0->PSELRXD = rxd_pin_number;
	
	NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud9600 << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0;
	
	//vendor define
	NRF_UART0->INTENSET					= (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos)|
																(UART_INTENSET_TXDRDY_Enabled << UART_INTENSET_TXDRDY_Pos);
	
	NVIC_SetPriority(UART0_IRQn,UART_Priority);
	NVIC_EnableIRQ( UART0_IRQn );
}
/*******************************************************
函数名:自定义串口中断函数
参  数:
返  回:
*******************************************************/
void UART0_IRQHandler(void)
{
	uint8_t c;
	while(NRF_UART0->EVENTS_RXDRDY)
	{	
		c = (uint8_t)NRF_UART0->RXD;
		NRF_UART0->EVENTS_RXDRDY = 0;
		Com_Over_timer = COM_OVER_TIME;
		if(rcv_ok == false)
		{
			if(((tail+1) % RECEIVE_BUFFER_SIZE) != first)
				rcv_uart[tail++] = c;
			tail %= RECEIVE_BUFFER_SIZE;
		}
	}
	
	if(NRF_UART0->EVENTS_TXDRDY)
	{
		NRF_UART0->EVENTS_TXDRDY = 0;
		if(snd_len)
		{
			snd_len--;
			snd_int_idx++;
			if(snd_len)
			{
				NRF_UART0->TXD = Snd_Buf[snd_int_idx];
			}
		}
	}
}
/*******************************************************
函数名:更新串口配置
参  数:
返  回:
*******************************************************/
uint8_t update_uart_cfg(uint8_t * p)
{
	return 0;
}

/*******************************************************
函数名:报告丢包率
参  数:
返  回:
*******************************************************/
uint8_t GetUartCmd(uint8_t * p) 
{
	uint8_t i,j;
	uint8_t ret;
	static uint8_t stat;
	
	ret = 0;
	
	t_first = first;
	t_tail = tail;
	t_len = (t_tail + RECEIVE_BUFFER_SIZE - t_first) % RECEIVE_BUFFER_SIZE;
	
	if(t_len < 9)//长度不够
		return ret;
	
	tmpxor = 0;
	len = 0;
	stat = 0;
	frame2_ok = 0;
	for(i = 0,j=0;i < t_len;i++)
	{
		switch(stat)
		{
			case 0://找帧头
				if(rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE] == uart_soh[0])
				{
					for(j = 1;j < 6;j++)
					{
						if(rcv_uart[(t_first+i+j)%RECEIVE_BUFFER_SIZE] != uart_soh[j])
							break;
					}
					if(j == 6)
					{
						stat = 1;
						i += 5;
					}
				}else{
					if(rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE] == 0xaa)
					{
						if(rcv_uart[(t_first+i+1)%RECEIVE_BUFFER_SIZE] == 0xaa)
						{
							len = 0;
							stat = 4;
							i+= 1;
						}
					}
				}
				break;
			case 1://获取长度高字节
				*p = rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE];
				len = (*p) & 0x00FF;
				p++;
				stat = 2;
				break;
			case 2://获取长度低字节
				*p = rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE];
				len = (len << 8) + (*p)&0x00ff;
				p++;
				stat = 3;
				break;
			case 3://获取数据和校验
				if(len)
				{
					*p = rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE];
					p++;
					len --;
					if(len == 0)
						ret = 1;
				}
				else
					ret = 1;
				break;
			case 4://获取另外一套协议
				*p = rcv_uart[(t_first+i)%RECEIVE_BUFFER_SIZE];
				p++;
				len ++;
				if(len >= 10)
				{
					ret = 1;
					frame2_ok = 1;
				}
				break;
		}	
			
		if(ret)
			break;
	}
	return ret;
}
/*******************************************************
函数名:清空接收状态缓冲
参  数:
返  回:
*******************************************************/
void flush_buffer(uint8_t len)
{
	uint8_t i;
	NVIC_DisableIRQ( UART0_IRQn );
	for(i = 0;i<RECEIVE_BUFFER_SIZE;i++)
	{
		rcv_uart[i] = 0;
	}
	first = tail = 0;
	NVIC_EnableIRQ( UART0_IRQn );
}
/*******************************************************
函数名:发送标签缓冲
参  数:                         在这个门限范围内的标签的ID自动上报
返  回:
*******************************************************/
void SendTap(uint8_t clear,uint8_t t,uint8_t time)
{
	uint16_t tapnum;
	static uint8_t i,j,flag;
	uint16_t tmp_idx;
	static uint16_t len;
	
	if(CmdTestOnFlag == TRAILING_END)
			return;
	if(snd_len)
		return;
 	snd_idx = 0;
	memcpy(Snd_Buf,uart_soh,6);snd_idx += 6;//帧头
	tapnum = CountTap(t,time);//输入RSSI过滤，时间过滤
	len = tapnum * ((para_Record[PARA_MSGTYPE_IDX]&PARA_MSGTYPE_Msk)?TAG_LONG_LEN : TAG_SHORT_LEN) + 8;
	Snd_Buf[snd_idx++] = (uint8_t) (len >> 8);//长度
	Snd_Buf[snd_idx++] = (uint8_t) len;
	memcpy(Snd_Buf+snd_idx,DeliverID,4);snd_idx+=4;//目标接收器
	memcpy(Snd_Buf+snd_idx,para_Record+PARA_PRIO_IDX,2);snd_idx+=2;//内部参数
	Snd_Buf[snd_idx++] = 0;//type
	for(i = 0;i<TAG_LONG_NUM;i++)
	{
		tmp_idx =((i&0x00ff) << 4) + ((i&0x00ff) << 3);
		for(j = 0;j<4;j++)
		{
			if(Tag_Buffer[tmp_idx+j] != 0xff)
			{
				break;
			}
		}
		flag = ReportFlag(i,t,time);
		if((j != 4) && flag)//有效卡
		{
			if(tapnum)
			{
				//发送短信息,=0
				if(!(para_Record[PARA_MSGTYPE_IDX] & PARA_MSGTYPE_Msk))
				{
					memcpy(Snd_Buf+snd_idx,Tag_Buffer + tmp_idx,TAG_SHORT_LEN-1);snd_idx +=(TAG_SHORT_LEN-1);//一张卡
					if(Tag_Buffer[TAGMSG_SHORT_SSRTYPE_IDX + tmp_idx]&TAGMSG_SHORT_SSRTYPE_Msk)//上报传感数值
					{
						Snd_Buf[snd_idx++] = Tag_Buffer[tmp_idx + TAGMSG_SHORT_SSRVALUE_IDX];//心率
					}else{//上报位置信息
						Snd_Buf[snd_idx++] = Tag_Buffer[tmp_idx + TAGMSG_SHORT_REF_IDX];//位置
					}
				}
				else//长信息
				{
					memcpy(Snd_Buf+snd_idx,Tag_Buffer + tmp_idx,TAG_LONG_LEN);snd_idx+=TAG_LONG_LEN;//长信息
				}

				tapnum --;
				if(clear)
				{
					memset(Tag_Buffer + tmp_idx,0xff,TAG_LONG_LEN);
					Tag_Status[i] = TAG_STATUS_INVALID;//复位卡状态
				}
			}
		}
		Tag_Status[i] &= TAG_STATUS_TIME_Msk;//清除更新标志
	}
	
	Snd_Buf[snd_idx] = utl_GetSum(Snd_Buf+6,snd_idx-6);//校验和
	
	uart_send(Snd_Buf,snd_idx+1);
}
/*******************************************************
函数名:扩展信息的串口响应
参  数:
返  回:
*******************************************************/
void ExtCmdResponse(uint8_t *id,uint8_t *pext)
{
	uint32_t i;
	for(i= 0;i<MAX_TIMEOUT_LOOPS;i++)
	{
		if(snd_len == 0)
			break;
	}
	
	if(i == MAX_TIMEOUT_LOOPS)
		return;
	memcpy(Snd_Buf,"\xFF\xFF\xFF\xFF\xFF\x00",6);
	Snd_Buf[6] = 0;
	Snd_Buf[7] = 32;              //0x20
	memcpy(Snd_Buf+8,DeliverID,4);
	memcpy(Snd_Buf+12,para_Record+PARA_PRIO_IDX,2);
	Snd_Buf[14] = 0x02;//应答指令
	memcpy(Snd_Buf+15,id,4);
	memcpy(Snd_Buf+19,pext,20);
	Snd_Buf[39] = utl_GetSum(Snd_Buf+6,33);
	uart_send(Snd_Buf,40);
}
/*******************************************************
函数名:串口下发的扩展信息处理
参  数:pext-24B,ID(4B)+命令(4B)+读写内容(16B)
返  回:
*******************************************************/
#define DEAL_EXT_ID				0
#define DEAL_EXT_INFO			4
#define DEAL_EXT_RECORD		8
void ExtInfoDeal(uint8_t * pext)
{
	uint32_t id;
	uint8_t id_type,cmd,page,idx,i;
	//确定配置的目标ID，标签或读写器
	id_type = TransferID(&id,pext);
	
	switch(id_type)
	{	//外部ID需要透传指令，除测试指令外
		case ID_TYPE_RECEIVER:
			//接收器ID处理扩展指令
//			memset(pext+DEAL_EXT_INFO,0xAA,20);
//		break;
		case ID_TYPE_TRANSCEIVER:
		case ID_TYPE_ALLRECEIVER_RP:
		//自身ID需要处理扩展指令
		case ID_TYPE_SELF_RP:
			cmd = pext[EXTINFO_CMD_IDX2+DEAL_EXT_INFO] & EXTINFO_CMD_Msk;
			
			if(id_type == ID_TYPE_SELF_RP)
			{	
				ConfigType = CONFIG_NONE;			
				page = (pext[EXTINFO_FLSPG_IDX2+DEAL_EXT_INFO] & EXTINFO_FLSPG_Msk) >> EXTINFO_FLSPG_Pos;//解析命令中的信息区地址
				if(pext[EXTINFO_RDVLD_IDX2+DEAL_EXT_INFO] & EXTINFO_RDVLD_Msk)
				{
					idx = (pext[EXTINFO_RDOS_IDX2+DEAL_EXT_INFO] & EXTINFO_RDOS_Msk) >> EXTINFO_RDOS_Pos;
				}
				else
					idx = 0xff;//无效索引
				ReportDelay = REPORTDELAY;
				ReportSuspend = 1;																									//上报挂起
				switch(cmd)
				{
					case 8://读RSSI参数
						page = RAMTYPE_RSSI;
					case 0://读
//						if(heart==1)
//							{
//								idx = 0xff;
//						    Write_Para(page,idx,pext + );
//								heart=0;
//							}
						idx = Read_Para(page,idx,pext + DEAL_EXT_RECORD);
						if(idx != 0xff)//获取成功
						{
							pext[DEAL_EXT_INFO + 1] &= 0xC0;
							pext[DEAL_EXT_INFO + 1] |=(0x20 | (idx & 0x1f));

							ExtCmdResponse(pext + DEAL_EXT_ID,pext+DEAL_EXT_INFO);
						}
						break;
					case 9://写RSSI参数
						page = RAMTYPE_RSSI;
					case 1://写
						idx = 0xff;
						Write_Para(page,idx,pext + DEAL_EXT_RECORD);
						idx = Read_Para(page,idx,pext+DEAL_EXT_RECORD);//读最新数据
						if(idx != 0xff)//获取成功
						{
							pext[DEAL_EXT_INFO + 1] &= 0xC0;
							pext[DEAL_EXT_INFO + 1] |=(0x20 | (idx & 0x1f));
							ExtCmdResponse(pext+ DEAL_EXT_ID,pext+DEAL_EXT_INFO);
						}
						break;
					case 10://擦除RSSI参数
						page=RAMTYPE_RSSI;
					case 2://擦除
						if(Erase_Para(page))
						{
							memset(pext+DEAL_EXT_INFO,0xAA,20);
							ExtCmdResponse(pext+ DEAL_EXT_ID,pext+DEAL_EXT_INFO);
							if(page == RAMTYPE_RSSI)
								Get_ValidPara(RAMTYPE_RSSI,Rssi_Record);
						}
						break;
					case 4://读运行参数
						memcpy(pext+DEAL_EXT_RECORD,para_Record,RECORD_LEN);
						ExtCmdResponse(pext+ DEAL_EXT_ID,pext+DEAL_EXT_INFO);
						break;
					case 5://写运行参数
						GetValidPara(pext+DEAL_EXT_RECORD);
						memcpy(para_Record,pext+DEAL_EXT_RECORD,RECORD_LEN);
						ExtCmdResponse(pext+ DEAL_EXT_ID,pext+DEAL_EXT_INFO);
						RunPara();
						break;
				}
			}//自己处理结束
			else//发送到外部读写器
			{
				ConfigType = CONFIG_READER;
				memcpy(ExtInfoBuffer,pext,24);
				RemoteDelay = REMOTEDELAY;
			}
			//设置测试标签，考虑是否返回一个状态
			if(cmd == 7)
			{
				memcpy(ExtInfoBuffer,pext,24);
				
				for(i = 0; i <RECORD_LEN;i++)
				{
					if(pext[DEAL_EXT_RECORD + i] != 0xFF)
						break;
				}
				

				if(i == RECORD_LEN)//记录数据全FF时，退出测似模式
				{
					CmdTestOnFlag = TEST_COLSE;
					Get_ValidPara(RAMTYPE_PARA,para_Record);
					ConfigType = CONFIG_READER;    //树立配置读写器标志位
					RemoteDelay = REMOTEDELAY;
				}else if(i < RECORD_LEN)//记录数据非全FF时，进入测试模式
				{
					CmdTestOnFlag = TRAILING_END;//后端接收器
					CmdTestCounter = CMDTEST_EXITTIME;
					CmdTestOverFlag = 0;
					CmdTestPriodCounter = CMDTEST_TIME;
					CmdTestPriodOverFlag = 0;
					ConfigType = CONFIG_READER;
					RemoteDelay = REMOTEDELAY;
				}
			}
			break;
		case ID_TYPE_TAP:									//目标为标签
		case ID_TYPE_ALLTAP_RP:
		case ID_TYPE_ALLTAP_NRP:

			memcpy(ExtInfoBuffer,pext,24);
			ConfigType = CONFIG_TAG;       //此处树立配置标签标志位
			break;
	}	
}
/*******************************************************
函数名:串口解析
参  数:
返  回:
*******************************************************/
#define U_DELAY			0
#define U_AT				1
#define U_TRANS			2
#define U_ENTERAT		3


void Uart_Deal(void)
{
	#ifdef __NTN206
	static uint8_t step = U_DELAY;
	#else
	static uint8_t step = U_TRANS;
	#endif
	uint32_t tempID;
	uint8_t id_type;
	uint8_t beg_idx = 0,tmp;
	uint16_t idx;
	char atcmd[50];
	uint8_t cmdbuf[200];
	
	switch(step)
	{
		case U_TRANS:									//透传模式
			if(GetUartCmd(cmdbuf) == 1)
			{
				flush_buffer(cmdbuf[1]);
				if(frame2_ok)
				{
					frame2_ok = 0;
					idx = crc16(cmdbuf,8);
					if((uint8_t)idx != cmdbuf[9] || (uint8_t)(idx >> 8) != cmdbuf[8])
						break;
					if(memcmp(cmdbuf,DeliverID,4)!=0)
						break;
					if(cmdbuf[5] == 0x09)
					{
						GotoBootLoader(cmdbuf[4]);
					}
					if(cmdbuf[5] == 0x0d)
					{
						uart_send_ver(cmdbuf[4]);
					}
					break;
				}
				#ifndef _DEBUG
				if(utl_XorCheck(cmdbuf,(((cmdbuf[0]&0x00ff) << 8)|cmdbuf[1] +2)) == false)
				{
					break;
				}
				#endif
				#ifdef __ST03U
				nrf_gpio_pin_set(LED0); //串口下发指令 led跳变
				led_delay = 0;
				#endif
				//命令处理
				id_type = TransferID(&tempID,cmdbuf+UART_ADDR_IDX);//解析ID
				if(id_type != ID_TYPE_ALLRECEIVER_RP)//目标所有接收器
				{
					break;
				}
				if(tempID == 0)//短号寻址
				{
					if(cmdbuf[UART_ATTR_IDX] != para_Record[PARA_BRIEFNUM_IDX])
						break;
				}
				//系统心跳包
				SysHeartTime = cmdbuf[UART_ATTR_IDX +1] << 2;//单位2S,重连需两个心跳间隔时间
				SysHeartCounter = SysHeartTime;		
				//系统心跳
				//查询
				if((cmdbuf[UART_TYPE_IDX] & UART_TYPE_COMMON_Msk) == 0x08)
				{
					tmp = (cmdbuf[UART_TYPE_IDX]&UART_TYPE_TIME_Msk) >> UART_TYPE_TIME_Pos;//查询回溯时间
					if(tmp > 7)
					{
						tmp = 7;
					}
					tmp <<=1;
					ReportSuspend = 0;//启用上报
					SendTap(cmdbuf[UART_TYPE_IDX]&UART_TYPE_CLEAR_Msk,
									para_Record[PARA_FILTERRSSI_IDX] & PARA_FILTERRSSI_Msk,
									tmp);
				}
				//下发扩展信息
				if((cmdbuf[UART_TYPE_IDX] & UART_TYPE_COMMON_Msk)== 0x0A)
				{
					id_type = TransferID(&tempID,cmdbuf+UART_INFO_IDX);
					if(id_type == ID_TYPE_TAP)//如果是标签
					{
						if(cmdbuf[UART_TYPE_IDX] & UART_TYPE_SENDMODE_Msk)//匹配发送
							CmdExtSendMatch = 1;
						else
							CmdExtSendMatch = 0;
						
						//串口下发指令的处理超时
						tmp = ((cmdbuf[UART_TYPE_IDX] & UART_TYPE_OVERTIME_Msk) >> UART_TYPE_OVERTIME_Pos) << 1;
						CmdExtCounter =tmp;
						if(!tmp)
						{
							CmdExtOverFlag |= 0x80;//持续发送
						}
						else
						{
							CmdExtOverFlag &= ~0x80;//关闭持续发送
						}
					}
//					else{
//						if(id_type==  ID_TYPE_RECEIVER )
//						{
//							
//						}
//					}
					CmdOnFlag = 1;										//配置扩展信息
					ConfigIDBuffer = tempID;
					ExtInfoDeal(cmdbuf + UART_INFO_IDX);//24B
					//memcpy(xinlv,cmdbuf+UART_INFO_IDX,24);
				}
			}
			break;
		case U_AT:										//AT指令模式
			idx = 0;
			do
			{
				beg_idx = utl_getATstring((uint8_t*)query + idx,(uint8_t*)atcmd);
				idx += beg_idx;
				atcmd[beg_idx] = '\0';
				if(beg_idx != 0xff)
				{	
					simple_uart_putstring((const uint8_t *)atcmd);
					while((tmp = utl_getATstring(rcv_uart+ first,(uint8_t *)atcmd)) == 0xff)
					{
					}
					flush_buffer(tmp);
				}
			}
			while(beg_idx != 0xff);
			break;
		case U_ENTERAT:									//进入AT指令
			#if defined(__NTN206)||defined(__ST01U)
			nrf_gpio_pin_clear(ES_RST);
			nrf_delay_ms(120);
			nrf_gpio_pin_set(ES_RST);
			step = U_TRANS;
			#endif
			break;   
		case U_DELAY:									//延时
			if(U_OverFlag)
			{
				step = U_TRANS;
				EthDelayFlag = 0;
			}
			break;
	}
	#ifdef __NTN206
	if((!(para_Record[PARA_NOLINKTEST_IDX] & PARA_NOLINKTEST_Msk)) && SysHeartOverFlag)//重启RM04
	{
		SysHeartOverFlag = 0;
		FeedWDT();
		nrf_gpio_pin_clear(ETH_PWREN);
		nrf_delay_ms(1000);
		nrf_gpio_pin_set(ETH_PWREN);
		U_OverFlag = 0;
		step = U_DELAY;
		U_Delay = UARTDELAY_TIME;
		SysHeartCounter = SysHeartTime;//0-无心跳
	}
	#endif
}
//FILE END
