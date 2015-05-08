/*****************************************************************************
 *   wdt.c:  Watchdog C file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.07.20  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/
#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "wdt.h"

volatile uint32_t wdt_counter;

/*****************************************************************************
** Function name:		WDTHandler
**
** Descriptions:		Watchdog timer interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
/*void WDT_IRQHandler(void)
{
  LPC_WDT->MOD &= ~WDTOF;	*/	/* clear the time-out flag */
								/* Please note, the sequence here is crucial. */
/*  LPC_WDT->MOD &= ~WDINT;	*/	/* Then, clear the interrupt flag */
/*  wdt_counter++;
}
*/
/*****************************************************************************
** Function name:		WDTInit
**
** Descriptions:		Initialize watchdog timer, install the
**						watchdog timer interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDTInit( void )
{
  /* Enable clock to WDT */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);

  wdt_counter = 0;

  /* Enable the WDT Interrupt */
  NVIC_EnableIRQ(WDT_IRQn);

  LPC_WDT->TC = WDT_FEED_VALUE;	/* once WDEN is set, the WDT will start after feeding */
  LPC_WDT->MOD = WDEN | WDRESET;
//  LPC_WDT->MOD = WDEN;

  LPC_WDT->FEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->FEED = 0x55;    
  return;
}

/*****************************************************************************
** Function name:		WDTFeed
**
** Descriptions:		Feed watchdog timer to prevent it from timeout
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDTFeed( void )
{
  LPC_WDT->FEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->FEED = 0x55;
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
