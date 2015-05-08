#ifndef _KEY_SCAN_H_
#define _KEY_SCAN_H_

#include "lpc11xx.h"
#include "system_lpc11xx.h"
#include "gpio.h"

#define MAX_KEYS   7

#define Pin_0 GPIOStatusValue(0,6)
#define Pin_1 GPIOStatusValue(2,4)
#define Pin_2 GPIOStatusValue(0,3)
#define Pin_3 GPIOStatusValue(2,6)


//#define KeyValue(x) {switch((x))\
//{\
//    case 0:\
//        return GPIOStatusValue(0,6);\
//    case 1:\
//        return GPIOStatusValue(2,4);\
//    case 2:\
//        return GPIOStatusValue(0,3);\
//    case 3:\
//        return GPIOStatusValue(2,6);\
//    default:\
//        return 0;\
//}}

#define KeyInit  Panel_KeyInit
extern uint16_t key_released  ,\
				 key_posedge  ,\
				 key_pressed  ,\
				 key_negedge ;

extern uint32_t long_press_time[MAX_KEYS];


void Panel_KeyInit(void);
extern void KeyScan(void);
#endif
