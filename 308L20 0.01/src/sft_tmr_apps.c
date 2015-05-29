#include "sft_tmr.h"
#include "main.h"
#include "dali_master.h"
#include "uart.h"

extern volatile uint8_t bp_flag, aux_flag, aux_check, relay_off;
extern volatile uint32_t flag20ms ,Check_Change_Flag ;//, pressed_time;


mtimer_app_t stTimer20ms    = MT_FULL;
mtimer_app_t stKeyscan      = MT_FULL;
mtimer_app_t stLedBlink     = MT_FULL;
mtimer_app_t stCheckSend    = MT_FULL;
mtimer_app_t stRS485Send    = MT_FULL;


void Instead_of_SysTick_Handler_at_v1_02(void * parameter)
{
	flag20ms++;
	Check_Change_Flag++;
    /*
	if(bp_flag)
	{
		pressed_time++;
	}
    */
	if(aux_flag&0x7f)
	{
		aux_check++;
	}
	if(relay_off)
	{
		relay_off++;
	}
		
}
/*****************************************************************************
 �� �� ��  : module_LedBlink
 ��������  : Led��˸�߳�
 �������  : void *parameter  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��28��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void module_LedBlink(void *parameter)
{
    LPC_GPIO3->DATA ^= (1<<3);
}

/*****************************************************************************
 �� �� ��  : RS485_BeginSend
 ��������  : RS485���ͺ������ڽ���ʱ��ֹ����
 �������  : void *parameter  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��28��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void RS485_BeginSend(void *parameter)
{
    uint8_t i;
    /* ����Ƿ��ڽ���״̬ */
    if (0 != UARTCount)
    {
        /* �ڽ���״̬�����¿�����ʱ�����ȴ���һ�μ�� */
        i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
        if (MT_NULL != i)
            mtimer_start(i);
    }
    else
    {
        /* ���������û�з��͹������û�з��͹����ݣ����Ƚ�RS485״̬����Ϊ����״̬�����ӳ�3ms */
        if (RS485_START_SEND == (g_SendNums & RS485_START_SEND))
        {   
            /* ���û�з��͹��������״̬ */            
            g_SendNums &= ~RS485_START_SEND;
            
            /* ����RS485����Ϊ����״̬ */
            LPC_GPIO3->DATA &= ~(1<<3);
            
            /* ����3ms��ʱ�� */
            i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
            if (MT_NULL != i)
                mtimer_start(i);            
        }
        else
        {        
            /* RS485����״̬�Ѿ�����3ms����ʼ�������� */
            pBuffer = UARTSendBuf;
            LPC_UART->THR = *pBuffer;
        }

    }
}

/*****************************************************************************
 �� �� ��  : Sft_Init_all_timers
 ��������  : �������е����ڼ��Ķ�ʱ��
 �������  : void *parameter  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��28��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void Sft_Init_all_timers(void *parameter)
{
    stTimer20ms =  mtimer_create("T_20ms",
                                    Instead_of_SysTick_Handler_at_v1_02, (void*)0,
                                    20,
                                    SFT_TIMER_FLAG_PERIODIC);
    if (stTimer20ms != MT_FULL)
            mtimer_start(stTimer20ms); 
    
    stKeyscan   = mtimer_create("keyscan",
                                    KeyScan ,(void *)0,
                                    20,
                                    SFT_TIMER_FLAG_PERIODIC);
    if (stKeyscan != MT_FULL)
       mtimer_start(stKeyscan);     
    
    stLedBlink  = mtimer_create("LedBlink",
                                    module_LedBlink ,(void *)0,
                                    500,
                                    SFT_TIMER_FLAG_PERIODIC);
    if (stLedBlink != MT_FULL)
        mtimer_start(stLedBlink);

}

/*****************************************************************************
 �� �� ��  : app_DelayStart
 ��������  : �ӳ�ʱ�䵽��������ʱ��
 �������  : void *parameter  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��29��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void app_DelayStart(void *parameter)
{
    /* �ж����ڼ��Ķ�ʱ���Ƿ��Ѿ����� */
    if (MT_FULL ==  stLedBlink )
    {
        Sft_Init_all_timers( (void *) 0);
    }
    /* ������鷢�ͺͼ̵���������ʱ�� */
    mtimer_start(stCheckSend);
    return;
}
/*****************************************************************************
 �� �� ��  : SoftDelayStart
 ��������  : �ӳ�������ʱ������ֹͣ��鷢�Ͷ����ͼ̵�������
 �������  : uint32_t ulDelayMs  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��29��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void SoftDelayStart(uint32_t ulDelayMs)
{
    mtimer_app_t stDelayStart;
    if (stTimer20ms != MT_FULL)
        mtimer_stop(stCheckSend);
    stDelayStart  = mtimer_create("DelayStart",
                                    app_DelayStart ,(void *)0,
                                    (uint64_t)ulDelayMs,
                                    SFT_TIMER_FLAG_ONE_SHOT);
    if (stDelayStart != MT_FULL)
        mtimer_start(stDelayStart);    
    
}


