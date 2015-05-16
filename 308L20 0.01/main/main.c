#include "lpc11xx.h"
#include "system_lpc11xx.h"
#include "dali_master.h"
#include "dali_master_2nd.h"
#include "dali_master_3rd.h"
#include "gpio.h"
#include "i2c1.h"
#include "timer32.h"
#include "timer16.h"
#include "uart.h" 
#include "wdt.h"
#include "main.h"
#include "PanDali_cmd.h"
#include "sft_tmr.h"
#include "Apps.h"
/********************************************************/
#define K1_OFF()		{LPC_GPIO3->DATA &= ~(1<<5);}
#define K2_OFF()		{LPC_GPIO2->DATA &= ~(1<<1);}
#define K3_OFF()		{LPC_GPIO3->DATA &= ~(1<<4);}
#define K1_ON()			{LPC_GPIO2->DATA &= ~(1<<5);}
#define K2_ON()			{LPC_GPIO0->DATA &= ~(1<<7);}
#define K3_ON()			{LPC_GPIO1->DATA &= ~(1<<9);}
#define EN_TIMER16_0	(1ul<<7)
#define EN_TIMER16_1	(1ul<<8)
#define	EN_TIMER32_0	(1ul<<9)
#define	EN_TIMER32_1	(1ul<<10)
#define	EN_IOCON		(1ul<<16)
#define DeviceCode 		0xEC
uint8_t xcode 		=   0x10;

#define Preset_addr				0xf29
#define DALI_Data_base_addr		0x6629
#define Area_addr				0x6929
#define Area_append_addr 		0x69E9
#define Logic_Channel_addr 		0x6AA9
#define Max_level_addr			0x6B69
#define Switch_level_addr 		0x6E69
#define Switch_flag_addr		0x7229
#define Duplicate_addr			0x7241
#define AreaLink_addr			0x6f29	

#define release
/********************************************************/
//static volatile uint8_t Box;
//static volatile uint8_t Channel;

volatile uint8_t bp_flag=0;
//volatile uint32_t pressed_time=0;
uint8_t aux_press,aux_release,Tpud=0;				  //AUX按下标志（同时计时），释放标志（同时计时），上电延迟时间

extern volatile uint8_t 	GucRcvNew;			  //用来标志是否从485网络收到了一条命令
volatile uint16_t	dali_counter_1st = 0; 		  //用来标志需要发送的通道数
volatile uint16_t	dali_counter_2nd = 0;
volatile uint16_t	dali_counter_3rd = 0;
//volatile uint8_t CH_temp_data[192];
extern channelDataType	ChannelData[CHANNELNUMS];
extern uint8_t 		UARTBuffer[20];
CirBufType DaliCirBuffer;
CirBufType DaliCirBuffer_2nd;
CirBufType DaliCirBuffer_3rd;
volatile uint32_t	flag20ms,Check_Change_Flag;
volatile uint8_t 	CheckFlag_1,relay_off;
volatile uint8_t 	aux_flag=0,aux_check=0;

uint8_t Box,AppendArea;
volatile uint16_t BlockWriteAddr; 
extern uint8_t BlockOption;

extern volatile uint8_t power_on_flag;

		  


/*****************************************************************************
** Function name:		WDT_CLK_Setup
**
** Descriptions:		Configure WDT clock.
** parameters:			clock source: irc_osc(0), main_clk(1), wdt_osc(2).			 
** 						
** Returned value:		None
** 
*****************************************************************************/
void WDT_CLK_Setup ( uint32_t clksrc )
{
  /* Watchdog configuration. */
  /* Freq = 0.5Mhz, div_sel is 0, divided by 2. WDT_OSC should be 250khz */
  LPC_SYSCON->WDTOSCCTRL = (0x5<<5)|0x00;
  LPC_SYSCON->WDTCLKSEL = clksrc;        /* Select clock source */
  LPC_SYSCON->WDTCLKUEN = 0x01;          /* Update clock */
  LPC_SYSCON->WDTCLKUEN = 0x00;          /* Toggle update register once */
  LPC_SYSCON->WDTCLKUEN = 0x01;
  while ( !(LPC_SYSCON->WDTCLKUEN & 0x01) );  /* Wait until updated */
  LPC_SYSCON->WDTCLKDIV = 1;            /* Divided by 1 */  
  LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);    /* Let WDT clock run */
  return;
}



/**************************************************************/
/*function DaliCirBufferInit is used to reset the dali circle */
/* 			buffer 											  */
/**************************************************************/

void DaliCirBufInit(CirBufType *DaliCirBuffer)
{
    unsigned int i;
    
    DaliCirBuffer->read_index    = 0;                                   /* Buffer[]写下标清零           */
    DaliCirBuffer->save_index    = 0;                                   /* Buffer[]读下标清零           */
    for (i = 0; i < BufferSize; i++) {
        DaliCirBuffer->buffer[i] = 0;                  /* 初始化为空帧                 */
    }
}
/*************************************************************/
/*Function init_channel_data is just for test,do not use it	 */
/*		   in the release version							 */
/*************************************************************/
void init_channel_data(void)
{	
	uint8_t	i;
	for(i=CHANNELNUMS;i!=0;i--)
	{
		ChannelData[i-1].GoalLevel=0xff;
		ChannelData[i-1].OringalLevel=0xff;
		ChannelData[i-1].PrevDALILevel=0xff;
		ChannelData[i-1].PastedTime=1;
		ChannelData[i-1].timeToGoal=1;
		ChannelData[i-1].MaxLevel=0x01;
//		ChannelData[i-1].MinLevel=0xff;
		ChannelData[i-1].SwitchLevel=0;
		ChannelData[i-1].ShortAddr=0xff;
		ChannelData[i-1].Area=2;
		ChannelData[i-1].AppendArea=0xff;

	}			
}
/****************************************************/
/*	Function Name:Get_Channel_Param					*/
/****************************************************/
void Get_Channel_Param(void)
{						 

	uint8_t idx;
	uint16_t Num;
	uint8_t CH_temp_data[CHANNELNUMS];
//	CH_temp_data[0]=0;
	m24xx_read(EEPROM_24XX256,0xF19,0,&aux_press,1);
	m24xx_read(EEPROM_24XX256,0xF21,0,&aux_release,1);
	for(Num=0;Num<CHANNELNUMS;Num++)
	{
		if(m24xx_read(EEPROM_24XX256,DALI_Data_base_addr+Num*4,0,CH_temp_data,1)==I2C_NO_ERR)	 //此处if语句更新了每个通道的DALI短地址
		{
			ChannelData[Num].ShortAddr    = (CH_temp_data[0]&0x3f);
			ChannelData[Num].DALI_Channel = ((CH_temp_data[0]>>6)&0x03)+1; 	
		} 
	}

	if(m24xx_read(EEPROM_24XX256,Area_addr,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)	 //更新了每个通道所属的区
	{
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].Area=CH_temp_data[Num];	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Area_append_addr,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)	 //更新了每个通道所属的附加区
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].AppendArea=CH_temp_data[Num];	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Logic_Channel_addr,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)	//更新逻辑通道号
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].LogicChannel=CH_temp_data[Num];	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Max_level_addr,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)		//更新最大电平
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].MaxLevel=((~CH_temp_data[Num])+1);	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Switch_level_addr,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)	   //更新开关电平
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].SwitchLevel=CH_temp_data[Num];	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Duplicate_addr,0,CH_temp_data,24)==I2C_NO_ERR)		  //更新重复标志位
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].flag_bit |= ((CH_temp_data[(Num/8)]>>(Num%8))&0x01);	
		} 
	}
	if(m24xx_read(EEPROM_24XX256,Switch_flag_addr,0,CH_temp_data,24)==I2C_NO_ERR)	  //更新开关电平标志位
	{																				 
		for(Num=0;Num<CHANNELNUMS;Num++)
		{
			ChannelData[Num].flag_bit |= (((CH_temp_data[(Num/8)]>>(Num%8))&0x01)<<1);	
		} 
	}
	 for(idx=1;idx<4;idx++)
	 {
	 	if(m24xx_read(EEPROM_24XX256,AreaLink_addr+CHANNELNUMS*idx,0,CH_temp_data,CHANNELNUMS)==I2C_NO_ERR)
		{
			for(Num=0;Num<CHANNELNUMS;)
			{
				ChannelData[idx+(Num/4)].AreaLink = CH_temp_data[Num/4];
				Num+=4;
			}
		}
	 }
	 m24xx_read(EEPROM_24XX256,0X6,0,&xcode,1);	//读取最大场景数
			
}
/****************************************************************/
/*Function power_supply_dection_init is to enable the detection */
/****************************************************************/
void power_supply_dection_init(void)
{
	GPIOSetDir(0,6,0);
	GPIOSetInterrupt(0,6,0,1,1);
	GPIOIntEnable(0,6);
	GPIOSetDir(2,4,0);
	GPIOSetInterrupt(2,4,0,1,1);
	GPIOIntEnable(2,4);
	GPIOSetDir(0,3,0);
	GPIOSetInterrupt(0,3,0,1,1);
	GPIOIntEnable(0,3);
	
//	LPC_GPIO0->IS=0X0F;	  //here i need to see the schdoc to confirm the value
						  //配置io口为边沿触发中断,bit11~0中某个bit为1则代表这个io口为边沿触发中断，否者为电平触发中断
//	LPC_GPIO3->IBE=0X0f;  //配置io口边沿触发类型,bit11~0中某个bit为1则代表这个io口为双边沿触发，否者为GPIO->IEV控制
//	LPC_GPIO3->IE=0X0F;	  //使能和屏蔽io口中断，bit11~0中某个bit为1则代表这个io口使能了中断，否者禁止了中断
						  //自加标志位在PIO3INT_IRQHandle中
} 

/*****************************************************/
/* Function SysTickInit is used to init SysTick timer*/
/*****************************************************/

void SysTickInit(uint8_t msTime)
{
	SysTick->CTRL |=((1<<0)|(1<<1));
	SysTick->LOAD = SystemFrequency / 2000* msTime - 1;              /*  定时时间设置                */
    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 1);
}
/***********************************************************/
/*	Function SysTick_Handler is used to set the 20ms flag  */
/*	and check flag										   */
/***********************************************************/
#if 0
void SysTick_Handler(void)
{
	flag20ms++;
	Check_Change_Flag++;
	if(bp_flag)
	{
		pressed_time++;
	}
	if(aux_flag&0x7f)
	{
		aux_check++;
	}
	if(relay_off)
	{
		relay_off++;
	}
		
}
#endif


void BlockRead(void)
{
	if(m24xx_read(EEPROM_24XX256,((UARTBuffer[4]<<8)|UARTBuffer[5]),0,&UARTBuffer[1],6/*TriggerNum-2*/)==I2C_NO_ERR);
	{
		UARTBuffer[0]=0xfc;
		UARTBuffer[7]=CheckSum(UARTBuffer,7); 
		UARTSend(UARTBuffer,8);
	}
}
void BlockWrite(void)
{
	if(m24xx_write(EEPROM_24XX256,BlockWriteAddr,0,&UARTBuffer[1],6/*TriggerNum-2*/)==I2C_NO_ERR);
	{
		m24xx_read(EEPROM_24XX256,BlockWriteAddr,0,&UARTBuffer[8],6/*TriggerNum-2*/);
		if((BlockWriteAddr+6/*TriggerNum-2*/)<=0x7fff)
		{
			UARTBuffer[0]=0xfa;
			UARTBuffer[1]=DeviceCode;
			UARTBuffer[2]=Box;
			UARTBuffer[3]=0xdc;
			UARTBuffer[4]=(BlockWriteAddr>>8);
			UARTBuffer[5]=BlockWriteAddr;
			UARTBuffer[6]=0;
			UARTBuffer[7]=CheckSum(UARTBuffer,7);
			UARTSend(UARTBuffer,8);
			BlockWriteAddr+=6/*TriggerNum-2*/;
		}
		else
			BlockOption=0;
	}
}

void AUX_Init(void)
{
	GPIOInit();
	GPIOSetDir(3,2,0);
	GPIOSetInterrupt(3,2,0,1,1);
	GPIOIntEnable(3,2);
}

void Relay_Init(void)
{
	LPC_IOCON->PIO2_1 = 0X10;		//INT1	 	
	LPC_IOCON->PIO1_9 = 0X10;
	LPC_IOCON->PIO3_4 = 0X10;
    LPC_IOCON->PIO2_5 &= ~0x07;    
	LPC_IOCON->PIO3_5 = 0X10;
	LPC_IOCON->PIO0_7 = 0X10;		//INT6
    
    GPIOSetDir(2,1,1);
	GPIOSetDir(1,9,1);
    GPIOSetDir(3,4,1);
    GPIOSetDir(2,5,1);
	GPIOSetDir(3,5,1);
	GPIOSetDir(0,7,1);
    
    GPIOSetValue(2,1,1);
	GPIOSetValue(1,9,1);
	GPIOSetValue(3,4,1);
	GPIOSetValue(2,5,1);
	GPIOSetValue(3,5,1);
	GPIOSetValue(0,7,1);
}


void DeviceInit(void)
{
	SystemInit();
	LPC_SYSCON->SYSAHBCLKCTRL |= (EN_TIMER32_0 | EN_TIMER32_1 |EN_TIMER16_0 | EN_TIMER16_1 | EN_IOCON | (1<<6));
	UARTInit(9600);
	i2c_lpc_init(I2C_SPEED_400); //i2c初始化，400k
 //  	dali_counter_1st = 0;
	init_channel_data();		 //所有通道都先给灭的状态
	RcvNew = 0;					 //没有收到数据

	Get_Channel_Param();		 //获取通道的参数，如区号，附加区等等
	AUX_Init();					 //aux使能
	Relay_Init();				 //继电器引脚初始化

	GPIOSetDir(3,1,1);			 //LED初始化
	GPIOSetDir(3,3,1);
	LPC_IOCON->PIO3_1 &= ~0x07;	 //led的IO口方向初始化
	LPC_IOCON->PIO3_3 &= ~0x07;
	GPIOSetValue(3,3,1);
//	SysTickInit(20);			 //初始化SysTick,为20ms一次中断
	init_timer32(1,48000000);	 //定时器初始化，设置中断间隔为1s
	NVIC_SetPriority(TIMER_32_0_IRQn, 1);
	NVIC_SetPriority(TIMER_32_1_IRQn, 2);
	enable_timer32(1);			 //使能定时器1

	//初始化设备
	if(m24xx_read(EEPROM_24XX256,1,0,UARTBuffer,1)==I2C_NO_ERR)
	{
		Box = UARTBuffer[0];
	}
	UARTBuffer[0]=0xfa;
	UARTBuffer[1]=DeviceCode;
	UARTBuffer[2]=Box;
	UARTBuffer[3]=0xfe;
	UARTBuffer[4]=VersionHi;
	UARTBuffer[5]=VersionLo;
	UARTBuffer[6]=0X80;
	UARTBuffer[7]=CheckSum(UARTBuffer,7);
	UARTSend(UARTBuffer,8);			  //发送启动命令
	power_supply_dection_init(); //供电检测初始化

	WDT_CLK_Setup(0x10);
	#ifdef release
	WDTInit();			 //调试时暂时未使用看门狗，喂狗程序放在LED闪烁中了，现在已注释掉
	#endif
}

void Get_Offset(void)
{
	uint8_t i;
	uint8_t OffsetValue[CHANNELNUMS];
	if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+3)*CHANNELNUMS,0,OffsetValue,CHANNELNUMS)==I2C_NO_ERR)
	{
		for(i=0;i<CHANNELNUMS;i++)
		{
			ChannelData[i].PresetOffset = OffsetValue[i];
			if(ChannelData[i].PresetOffset == 0xff)
			{
				ChannelData[i].PresetOffset = 0;
			}
		}	
	}	
}

void Set_Offset(void)
{
	uint8_t i;
	uint8_t OffsetValue[CHANNELNUMS];
	for(i=0;i<CHANNELNUMS;i++)
	{
		OffsetValue[i] = ChannelData[i].PresetOffset;
	}
	m24xx_write(EEPROM_24XX256,Preset_addr+(xcode+3)*CHANNELNUMS,0,OffsetValue,192);
}

void PresetRecovery(void)
{
	uint8_t i;
	uint8_t TD[2];
	uint8_t PresetValue[CHANNELNUMS];
	if(m24xx_read(EEPROM_24XX256,0xf17,0,TD,2)==I2C_NO_ERR)
	{
		if(TD[1]==0xff)
		{
			if(m24xx_read(EEPROM_24XX256,0X6C29,0,PresetValue,CHANNELNUMS)==I2C_NO_ERR)			 //读取先前场景号
			{
				for(i=0;i<CHANNELNUMS;i++)
				{
					ChannelData[i].PrePreset = PresetValue[i];	
				} 
			}
			if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+2)*CHANNELNUMS,0,PresetValue,CHANNELNUMS)==I2C_NO_ERR)				//读取掉电前场景的亮度
			{
				for(i=0;i<CHANNELNUMS;i++)
				{
					ChannelData[i].GoalLevel = PresetValue[i];
					ChannelData[i].timeToGoal = TD[0]*50;
				}
			}


		}
		else if(TD[1]==0x80)
		{
			if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+1)*192,0,PresetValue,192))
			{	
				for(i=0;i<CHANNELNUMS;i++)
				{
					ChannelData[i].GoalLevel = PresetValue[i];
					ChannelData[i].timeToGoal = TD[0]*50;
				}//panic场景		
			}
		}
			else if(TD[1]==0x81)
			{
				Tpud = TD[0];//上电延时
				if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+2)*CHANNELNUMS,0,PresetValue,CHANNELNUMS))
				{	
					for(i=0;i<CHANNELNUMS;i++)
					{
						ChannelData[i].GoalLevel = PresetValue[i];
						ChannelData[i].timeToGoal = 2;
					}		//先前场景		
				}
			}
				else 
				{
					if(TD[1]<xcode)
					{
						if(m24xx_read(EEPROM_24XX256,0xf29+TD[1]*CHANNELNUMS,0,PresetValue,CHANNELNUMS)==I2C_NO_ERR);
						for(i=0;i<CHANNELNUMS;i++)
						{
							ChannelData[i].GoalLevel = PresetValue[i+1];
							ChannelData[i].timeToGoal = TD[0]*50;
						}
					}
				}											  
	}
	Get_Offset();				 //获取存储的OFFSET值
}

void RelayPinSetLow(void)
{
	if(relay_off>1)
	{
		GPIOSetValue(2,1,1);
		GPIOSetValue(1,9,1);
		GPIOSetValue(3,4,1);
		GPIOSetValue(2,5,1);
		GPIOSetValue(3,5,1);
		GPIOSetValue(0,7,1);	
	}
	if(	((LPC_GPIO3->DATA&(1<<5))==0)
	  ||((LPC_GPIO2->DATA&(1<<1))==0)
	  ||((LPC_GPIO3->DATA&(1<<4))==0)  
	  ||((LPC_GPIO2->DATA&(1<<5))==0)
	  ||((LPC_GPIO0->DATA&(1<<7))==0)
	  ||((LPC_GPIO1->DATA&(1ul<<9))==0))
	{
		relay_off=1;
	}			
}

void Check_REL(void)
{
	uint8_t i;
	int sum=0;
	for(i=0;i<64;i++)
	{
		sum=sum+(((~ChannelData[i].OringalLevel)) +(~ChannelData[i].GoalLevel));
	}
	if(sum==0xffff8000)
	{
		if((power_on_flag&0x02)==2)
		K1_OFF();
	}
	else 
	{
		if((power_on_flag&0x02)==0)
		K1_ON();
	}
	sum=0;
	for(i=64;i<128;i++)
	{
		sum=sum+(((~ChannelData[i].OringalLevel)) +(~ChannelData[i].GoalLevel));
	}
	if(sum==0xffff8000)
	{
		if((power_on_flag&0x04)==0x04)
		K2_OFF();
	}
	else 
	{
		if((power_on_flag&0x04)==0)
		K2_ON();
	}
	sum=0;
	for(i=128;i<192;i++)
	{
		sum=sum+(((~ChannelData[i].OringalLevel)) +(~ChannelData[i].GoalLevel));
	}
	if(sum==0xffff8000)
	{
		if((power_on_flag&0x08)==0x08)
		K3_OFF();
	}
	else 
	{
		if((power_on_flag&0x08)==0)		
		K3_ON();
	}
	sum=0; 
}



int main (void)
{
	DeviceInit();
	PresetRecovery();
    /* 配置软件定时器 */
    mtimer_conf();
	//DALI_Thread中包含有while（1）循环
	DALI_Thread();

	return 0;
}
