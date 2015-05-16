#include "keyscan.h"

uint16_t key_released = 0 ,\
				 key_posedge  = 0 ,\
				 key_pressed  = 0 ,\
				 key_negedge  = 0;

uint32_t long_press_time[MAX_KEYS];




void Panel_KeyInit(void)
{
    /* ʹ��IO��ʱ�� */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
    
    /* ��ʼ��GPIO��P0.6��Ϊ��ͨ���ܿڣ�����״̬ */
    LPC_IOCON->PIO0_6   = 0;
    LPC_GPIO0->DIR     &= ~(1<<6); 

    /* ��ʼ��GPIO��P2.4��Ϊ��ͨ���ܿڣ�����״̬ */
    LPC_IOCON->PIO2_4   = 0;
    LPC_GPIO2->DIR     &= ~(1<<4);    

    /* ��ʼ��GPIO��P0.3��Ϊ��ͨ���ܿڣ�����״̬ */
    LPC_IOCON->PIO0_3   = 0;
    LPC_GPIO2->DIR     &= ~(1<<3);     
}


void KeyScan(void)
{
	static uint16_t scan1,scan2,scan3;
    uint8_t i;
  //scan1 ���һ�ΰ���ֵ��scan2 ���ڶ��ΰ���ֵ��scan3 �������ΰ���ֵ
  unsigned int temp=0;
   
#if 1
    
	/* �˴���дIO�ڶ�ȡ���� */
    for(i = 0; i< MAX_KEYS; i++)
    {
        temp |= KeyValue(i);
        temp = temp << i;
    }
	
#endif

    scan3 = scan2;
    scan2 = scan1;
    scan1 = temp;  
    scan1 &= 0x03FF;  // �˴�����޸�io�ڣ������޸�
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
