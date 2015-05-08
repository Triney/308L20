/*****************************************************************************
 *   wdt.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.09.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __WDT_H 
#define __WDT_H

#define WDEN		0x00000001
#define WDRESET		0x00000002
#define WDTOF		0x00000004
#define WDINT		0x00000008

#define WDT_FEED_VALUE		0x003d09000
/* Fsys_ahb is 48Mhz. */ 
#define TIMER0_FEED_FREQ	((SystemSHBFrequency/4800 - 1))	

extern void WDT_IRQHandler(void);
extern void WDTInit( void );
extern void WDTFeed( void );

#endif /* end __WDT_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
