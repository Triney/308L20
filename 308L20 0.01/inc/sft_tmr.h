#ifndef SFT_TMR_H
#define SFT_TMR_H

#include "keyscan.h"
#include "timer16.h"

#define USE_SFT_TIMER

#define SFT_TIMER_FLAG_DEACTIVATED         0x0                                /* timer is deactive.                                                 */
#define SFT_TIMER_FLAG_ACTIVATED           0x1                                /* timer is active.                                                 */
#define SFT_TIMER_FLAG_ONE_SHOT            0x0                                /* one shot timer.                                                         */
#define SFT_TIMER_FLAG_PERIODIC            0x2                                /* periodic timer.                                                         */
#define SFT_TIMER_FLAG_TIMEOUT             0x4                                /* timeout flag */

#define MAX_MTIMER                      10
#define MAX_MTIMER_NAME                 8
#define MT_FULL                         0xff
#define MT_NULL                         ((void *)0)

#define MT_US                           1000

typedef struct mtimer* mtimer_t;
struct mtimer
{
        //char  name[MAX_MTIMER_NAME];
        char flag; 
        void (*timeout_func)(void* parameter);                                /* timeout function.                                                 */
        void *parameter;                                                                        /* timeout function's parameter.                         */

        unsigned long init_tick;                                                                /* timer timeout tick.                                                 */
        unsigned long timeout_tick;                                                                /* timeout tick.                                                         */
};
typedef unsigned long mtimer_app_t;


//不会检测name长度,所以没用name操作，直接用index操作
mtimer_app_t  mtimer_create(const char* name,          //暂时无实际作用
                            void (*timeout)(void* parameter), void* parameter,
                            unsigned long time, char flag);
void mtimer_start(mtimer_app_t index);
void mtimer_stop(mtimer_app_t index);
void mtimer_detach(mtimer_app_t index);
unsigned long mtimer_get_run_tick(mtimer_app_t index);
void mtimer_conf(void);         //硬件定时器配置
void sft_timer_thread(void);    //软件定时器进程
// <<< Use Configuration Wizard in Context Menu >>>
// <h>Timer Choose

// <o> Default Timerx <0=>TIM2 <1=> TIM3 <2=>TIM4
//        <i>Default: 0
#define MT_TIMERX_VALUE  0
#if MT_TIMERX_VALUE==0
#define MT_TIMERX              LPC_TMR16B0
#define MT_TIMERX_IRQHandler   SysTick_Handler
#define Init_Timer_Ms()        SysTickInit(1)
#define Start_Timer_Ms()       {SysTick->CTRL |= 1;}
#elif MT_TIMERX_VALUE==1
#define MT_TIMERX              TIM3
#define MT_TIMERX_IRQN         TIM3_IRQn
#define MT_TIMERX_IRQHandler   TIM3_IRQHandler
#elif MT_TIMERX_VALUE==2
#define MT_TIMERX              TIM4
#define MT_TIMERX_IRQN         TIM4_IRQn
#define MT_TIMERX_IRQHandler   TIM4_IRQHandler
#endif 

// </h>
// <<< Use Configuration Wizard in Context Menu >>>

#endif

