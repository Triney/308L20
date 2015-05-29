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
 函 数 名  : module_LedBlink
 功能描述  : Led闪烁线程
 输入参数  : void *parameter  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月28日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void module_LedBlink(void *parameter)
{
    LPC_GPIO3->DATA ^= (1<<3);
}

/*****************************************************************************
 函 数 名  : RS485_BeginSend
 功能描述  : RS485发送函数，在接收时禁止发送
 输入参数  : void *parameter  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月28日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void RS485_BeginSend(void *parameter)
{
    uint8_t i;
    /* 检查是否在接收状态 */
    if (0 != UARTCount)
    {
        /* 在接收状态，重新开启定时器。等待下一次检查 */
        i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
        if (MT_NULL != i)
            mtimer_start(i);
    }
    else
    {
        /* 检查数据有没有发送过，如果没有发送过数据，则先将RS485状态设置为发送状态，在延迟3ms */
        if (RS485_START_SEND == (g_SendNums & RS485_START_SEND))
        {   
            /* 清除没有发送过数据这个状态 */            
            g_SendNums &= ~RS485_START_SEND;
            
            /* 设置RS485总线为发送状态 */
            LPC_GPIO3->DATA &= ~(1<<3);
            
            /* 开启3ms定时器 */
            i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
            if (MT_NULL != i)
                mtimer_start(i);            
        }
        else
        {        
            /* RS485发送状态已经保持3ms，开始发送数据 */
            pBuffer = UARTSendBuf;
            LPC_UART->THR = *pBuffer;
        }

    }
}

/*****************************************************************************
 函 数 名  : Sft_Init_all_timers
 功能描述  : 开启所有的周期检查的定时器
 输入参数  : void *parameter  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月28日
    作    者   : Enix
    修改内容   : 新生成函数

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
 函 数 名  : app_DelayStart
 功能描述  : 延迟时间到，开启定时器
 输入参数  : void *parameter  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月29日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void app_DelayStart(void *parameter)
{
    /* 判读周期检查的定时器是否已经启动 */
    if (MT_FULL ==  stLedBlink )
    {
        Sft_Init_all_timers( (void *) 0);
    }
    /* 开启检查发送和继电器动作定时器 */
    mtimer_start(stCheckSend);
    return;
}
/*****************************************************************************
 函 数 名  : SoftDelayStart
 功能描述  : 延迟启动定时器，即停止检查发送动作和继电器动作
 输入参数  : uint32_t ulDelayMs  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月29日
    作    者   : Enix
    修改内容   : 新生成函数

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


