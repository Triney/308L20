
#include "sft_tmr.h"

volatile unsigned long mtimer_tick;
struct mtimer mt_array[MAX_MTIMER];        //由于只用于少量定时器的场合，不使用链表

//不会检测name长度,所以没用name操作，直接用index操作
mtimer_app_t  mtimer_create(const char* name,          //暂时无实际作用
                                                   void (*timeout)(void* parameter), void* parameter,
                                                   unsigned long time, char flag)
{
        char index=0;
        mtimer_t timer;
        for(index=0;index<MAX_MTIMER;index++)
        {
                if(mt_array[index].flag==SFT_TIMER_FLAG_DEACTIVATED)
                {
                        break;
                }
        }
        
        if(index==MAX_MTIMER)
        {
            return MT_FULL;//(void*)0!=0
        }
        else
        {
            if(timeout==MT_NULL)
            {
                return MT_FULL;
            }
        }
        timer                   = (mtimer_t)&mt_array[index];
        timer->timeout_func     = timeout;
        timer->parameter        = parameter;
        timer->flag             = flag;
        timer->timeout_tick     = 0;
        timer->init_tick        = time;
        
        return index;
}
void mtimer_start(mtimer_app_t index)
{
        mtimer_t timer;
        if(index>=MAX_MTIMER)
                return;
        timer = (mtimer_t)&mt_array[index];
        timer->flag |= SFT_TIMER_FLAG_ACTIVATED;
        timer->timeout_tick = mtimer_tick + timer->init_tick;
}

void mtimer_stop(mtimer_app_t index)
{
        mtimer_t timer;
        if(index>=MAX_MTIMER)
                return;
        timer        = (mtimer_t)&mt_array[index];
        timer->flag &= ~SFT_TIMER_FLAG_ACTIVATED;
}

void mtimer_detach(mtimer_app_t index)
{
        mtimer_t timer;
        if(index>=MAX_MTIMER)
                return;
        timer = (mtimer_t)&mt_array[index];
        timer->flag         = SFT_TIMER_FLAG_DEACTIVATED;
        timer->timeout_func = MT_NULL;
}

static void mtimer_isr(void)
{
    char index;
    mtimer_t p;
    mtimer_tick++;
    for(index=0;index<MAX_MTIMER;index++)
    {
        p = (mtimer_t)&mt_array[index];
        if( (p->flag&SFT_TIMER_FLAG_ACTIVATED) && ((mtimer_tick == p->timeout_tick)) )
        {
            p->flag |= SFT_TIMER_FLAG_TIMEOUT;
            if(p->flag&SFT_TIMER_FLAG_PERIODIC)
            {
                    p->timeout_tick = mtimer_tick + p->init_tick;

            }
//          else
//                  p->flag &= ~ SFT_TIMER_FLAG_ACTIVATED;
//          p->timeout_func(p->parameter);
        }
    }
}

unsigned long mtimer_get_run_tick(mtimer_app_t index)
{
    unsigned long runtime=0;
    mtimer_t p;
    p = (mtimer_t)&mt_array[index];
    if(mtimer_tick<p->timeout_tick)
            runtime = p->init_tick - (p->timeout_tick - mtimer_tick);
    //else
    return runtime;                
}

void sft_timer_thread(void)
{
    uint8_t index;
    mtimer_t p;
    for(index=0;index<MAX_MTIMER;index++)
    {
        p = (mtimer_t)&mt_array[index];
        if( (p->flag&SFT_TIMER_FLAG_TIMEOUT) )
        {
            if((p->flag & SFT_TIMER_FLAG_PERIODIC) != SFT_TIMER_FLAG_PERIODIC)
            {
                p->flag &= ~ SFT_TIMER_FLAG_ACTIVATED;
            
            }
            p->timeout_func(p->parameter);
        }
    }
}

//硬件相关函数

extern uint8_t CheckFlag_1,BlockOption;
extern uint32_t exit_block_option;
void MT_TIMERX_IRQHandler(void)
{ 
    mtimer_isr();  
}

#if 0
void Instead_of_SysTick_Handler_at_v1_02(void)
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

//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
void mtimer_conf(void)         //硬件定时器配置
{
    Init_Timer_Ms() ;
    Start_Timer_Ms() ;
}
//测试

#ifdef MTIMER_TEST
mtimer_app_t mt1;
void mt_test_handle(void)
{
        volatile static char i;
        i++;
        if(i%2)
                GPIOC->BSRR = 1;
        else
                GPIOC->BRR = 1;
}
void microtimer_test(void)
{
        mt1 =  mtimer_create("mt1",
                                        mt_test_handle, (void*)0,
                                        10000,
                                        SFT_TIMER_FLAG_PERIODIC);
        if(mt1!=MT_FULL)
                mtimer_start(mt1);    
}
#endif //MTIMER_TEST
