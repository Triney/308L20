#include "keyscan.h"

uint16_t key_released = 0 ,\
				 key_posedge  = 0 ,\
				 key_pressed  = 0 ,\
				 key_negedge  = 0;

uint32_t long_press_time[MAX_KEYS];




void Panel_KeyInit(void)
{
    /* 使能IO口时钟 */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
    
    /* 初始化GPIO的P0.6口为普通功能口，输入状态 */
    LPC_IOCON->PIO0_6   = 0;
    LPC_GPIO0->DIR     &= ~(1<<6); 

    /* 初始化GPIO的P2.4口为普通功能口，输入状态 */
    LPC_IOCON->PIO2_4   = 0;
    LPC_GPIO2->DIR     &= ~(1<<4);    

    /* 初始化GPIO的P0.3口为普通功能口，输入状态 */
    LPC_IOCON->PIO0_3   = 0;
    LPC_GPIO2->DIR     &= ~(1<<3);     
}


void KeyScan(void)
{
	static uint16_t scan1,scan2,scan3;
    uint8_t i;
  //scan1 最近一次按键值，scan2 最后第二次按键值，scan3 最后第三次按键值
  unsigned int temp=0;
   
#if 1
    
	/* 此处填写IO口读取操作 */
    for(i = 0; i< MAX_KEYS; i++)
    {
        temp |= KeyValue(i);
        temp = temp << i;
    }
	
#endif

    scan3 = scan2;
    scan2 = scan1;
    scan1 = temp;  
    scan1 &= 0x03FF;  // 此处如果修改io口，则需修改
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
