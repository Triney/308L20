#include "gpio.h"

#include "keyscan.h"

uint16_t key_released = 0 ,\
				 key_posedge  = 0 ,\
				 key_pressed  = 0 ,\
				 key_negedge  = 0;

uint32_t long_press_time[MAX_KEYS];


uint32_t KeyValue(uint8_t ucGPIO)
{
    uint32_t ulTmp;
    switch(ucGPIO)
    {
    case 0:
        return GPIOStatusValue(0,6);
    case 1:
        return GPIOStatusValue(2,4);
    case 2:
        return GPIOStatusValue(0,3);
    case 3:
        return GPIOStatusValue(2,6);
    case 4:
    {
        /* 检查led与按键共用的口 */
        GPIOSetDir(3,1,0);
        ulTmp = GPIOStatusValue(3,1);
        GPIOSetDir(3,1,1);
        return ulTmp;
    }
    default:
        return 0;
    }
}


void Panel_KeyInit(void)
{
    /* 使能GPIO时钟 */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
    
    /* 配置GPIO P0.6 为输入 */
    LPC_IOCON->PIO0_6   = 0;
    LPC_GPIO0->DIR     &= ~(1<<6); 

    /* 配置GPIO P2.4 为输入 */
    LPC_IOCON->PIO2_4   = 0;
    LPC_GPIO2->DIR     &= ~(1<<4);    

    /* 配置GPIO P0.3 为输入 */
    LPC_IOCON->PIO0_3   = 0;
    LPC_GPIO2->DIR     &= ~(1<<3);     
}


void KeyScan(void)
{
	static uint16_t scan1,scan2,scan3;
    uint8_t i;
    
    unsigned int temp=0;
   
#if 1
    
	/* 读取IO */
    for(i = 0; i< MAX_KEYS; i++)
    {
        temp |= KeyValue(i);
        temp = temp << i;
    }
	
#endif

    scan3 = scan2;
    scan2 = scan1;
    scan1 = temp;  
    scan1 &= 0x03FF;  // 
    for(temp=0;temp<16;temp++)
    {
        if(((scan2&(1ul<<temp)) != (scan3&(1ul<<temp)))&&((scan1&(1ul<<temp)) == (scan2&(1ul<<temp))))
        {
          {
            if((scan1&(1ul<<temp))==(1ul<<temp))
            {
              key_released |= (1ul<<temp);
              key_posedge |= (1ul<<temp);
            }      
            else
            {
              key_pressed  |= (1ul<<temp);
              key_negedge  |= (1ul<<temp);
              long_press_time[temp] = 0;
            }
          }
        }
        else
        {
          if(((scan2&(1ul<<temp)) == (scan3&(1ul<<temp)))&&((scan1&(1ul<<temp)) == (scan2&(1ul<<temp))))
          {
            {
            if(((key_pressed&(1ul<<temp))==(1ul<<temp))&&((scan3&(1ul<<temp))==0))
                long_press_time[temp]++;  
            }
          }
        }
    }
}
