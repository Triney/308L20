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

void module_LedBlink(void *parameter)
{
    LPC_GPIO3->DATA ^= (1<<3);
}

void RS485_BeginSend(void *parameter)
{
    uint8_t i;
    if (0 != UARTCount)
    {
            i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
    }
    else
    {
        pBuffer = UARTSendBuf;
        LPC_UART->THR = *pBuffer;
    }
}


void Sft_Init_all_timers(void)
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

void app_DelayStart(void *parameter)
{
    if (MT_FULL ==  stLedBlink )
    {
        Sft_Init_all_timers();
    }
    mtimer_start(stCheckSend);
    return;
}

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


