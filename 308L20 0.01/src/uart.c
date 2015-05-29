/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.21  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "LPC11xx.h"
#include "uart.h"
#include "gpio.h"
#include "dali_master.h"

#include "sft_tmr.h"
#include "sft_time_apps.h"

volatile uint32_t UARTStatus;
volatile uint32_t  UARTTxEmpty = 1;
uint8_t     UARTBuffer[UART_BUFFER_LEN];
uint8_t	    UARTBuf[BUFSIZE]; 	
uint8_t     UARTSendBuf[BUFSIZE];
uint8_t   *pBuffer;
uint32_t    g_SendNums;
volatile uint32_t UARTCount = 0;
volatile uint8_t	RcvNew;
volatile uint8_t status=0;
volatile uint8_t TriggerNum=8;
/*****************************************************************************
** Function name:		UART_IRQHandler
**
** Descriptions:		UART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/


void UART_IRQHandler(void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
  uint8_t TempData;

  IIRValue = LPC_UART->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if (IIRValue == IIR_RLS)		/* Receive Line Status */
  {
    	LSRValue = LPC_UART->LSR;
	    /* Receive Line Status */
	if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
    {
	      /* There are errors or break interrupt */
	      /* Read LSR will clear the interrupt */
		UARTStatus = LSRValue;
		Dummy = LPC_UART->RBR;	/* Dummy read on RX to clear 
						interrupt, then bail out */
		return;
   	}

	if (LSRValue & LSR_RDR)	/* Receive Data Ready */			
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
    	  UARTBuffer[UARTCount++] = LPC_UART->RBR;	  //2012-6-14修改，希望判断接收头
		  /*************************************************/

		  /**********************************************************/

    	  if (UARTCount == BUFSIZE)
	      {
	        UARTCount = 0;		/* buffer overflow */
	      }	
    }
  }
  else if (IIRValue == IIR_RDA)	/* Receive Data Available */
  {
    /* Receive Data Available */
  //  UARTBuffer[UARTCount++] = LPC_UART->RBR;
	/**************/

		if(status==0)
		  {
		  	TempData = LPC_UART->RBR;
			if((TempData==0xfa)||(TempData==0xf5)||(TempData==0xfc))
			{
				UARTBuf[UARTCount++]=TempData;
				status=1;
			}
		  }
		  else
		  {
              UARTBuf[UARTCount++] = LPC_UART->RBR;
			  if(UARTCount==TriggerNum)
			  {
		 			RcvNew=1;
					status = 0;
			  }
		}
	  /***************/
    if (UARTCount == BUFSIZE)
    {
      UARTCount = 0;		/* buffer overflow */
    }
  }
  else if (IIRValue == IIR_CTI)	/* Character timeout indicator */
  {
    /* Character Time-out indicator */
    UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
	status = 0;
  }
  else if (IIRValue == IIR_THRE)	/* THRE, transmit holding register empty */
  {
    /* THRE interrupt */
    LSRValue = LPC_UART->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
    if (LSRValue & LSR_THRE)
    {
        UARTTxEmpty = 1;
        if(0 != g_SendNums)
        {
            /* 发送下一个字节的数据 */
            pBuffer++;
            g_SendNums--;
        }
        else
        {
            /* RS485线接收状态 */
            LPC_GPIO3->DATA |=  (1<<3);
        }
    }
    else
    {
      UARTTxEmpty = 0;
	  LPC_GPIO3->DATA |=  (1<<3);
    }
  }
  return;
}

#if MODEM_TEST
/*****************************************************************************
** Function name:		ModemInit
**
** Descriptions:		Initialize UART0 port as modem, setup pin select.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void ModemInit( void )
{
  LPC_IOCON->PIO2_0 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_0 |= 0x01;     /* UART DTR */
  LPC_IOCON->PIO0_7 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO0_7 |= 0x01;     /* UART CTS */
  LPC_IOCON->PIO1_5 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_5 |= 0x01;     /* UART RTS */
#if 1 
  LPC_IOCON->DSR_LOC	= 0;
  LPC_IOCON->PIO2_1 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_1 |= 0x01;     /* UART DSR */

  LPC_IOCON->DCD_LOC	= 0;
  LPC_IOCON->PIO2_2 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_2 |= 0x01;     /* UART DCD */

  LPC_IOCON->RI_LOC	= 0;
  LPC_IOCON->PIO2_3 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_3 |= 0x01;     /* UART RI */

#else
  LPC_IOCON->DSR_LOC = 1;
  LPC_IOCON->PIO3_1 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_1 |= 0x01;     /* UART DSR */

  LPC_IOCON->DCD_LOC = 1;
  LPC_IOCON->PIO3_2 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_2 |= 0x01;     /* UART DCD */

  LPC_IOCON->RI_LOC = 1;
  LPC_IOCON->PIO3_3 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_3 |= 0x01;     /* UART RI */
#endif
  LPC_UART->MCR = 0xC0;          /* Enable Auto RTS and Auto CTS. */			
  return;
}
#endif

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART0 port, setup pin select,
**				clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
** 
*****************************************************************************/
void UARTInit(uint32_t baudrate)
{
  uint32_t Fdiv;
  uint32_t regVal;

  UARTTxEmpty = 1;
  UARTCount = 0;
  
  NVIC_DisableIRQ(UART_IRQn);

  LPC_IOCON->PIO1_6 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_6 |= 0x01;     /* UART RXD */
  LPC_IOCON->PIO1_7 &= ~0x07;	
  LPC_IOCON->PIO1_7 |= 0x01;     /* UART TXD */

  /* Enable UART clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
  LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

  LPC_UART->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
  regVal = LPC_SYSCON->UARTCLKDIV;
  Fdiv = ((SystemAHBFrequency/regVal)/16)/baudrate ;	/*baud rate */

  LPC_UART->DLM = Fdiv / 256;							
  LPC_UART->DLL = Fdiv % 256;
  LPC_UART->LCR = 0x03;		/* DLAB = 0 */
  LPC_UART->FCR  = 0x87;    /* 使能FIFO，设置8个字节触发点  */
  LPC_UART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

  /* Read to clear the line status. */
  regVal = LPC_UART->LSR;

  /* Ensure a clean start, no data in either TX or RX FIFO. */
  while ( (LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
  while ( LPC_UART->LSR & LSR_RDR )
  {
	regVal = LPC_UART->RBR;	/* Dump data from RX FIFO */
  }
 
  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(UART_IRQn);

#if TX_INTERRUPT
  LPC_UART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
#else
  LPC_UART->IER = IER_RBR | IER_RLS;	/* Enable UART interrupt */
#endif
  return;
}

/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**				on the data length
**
** parameters:		buffer pointer, and data length
** Returned value:	None
** 
*****************************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
    uint8_t i = 0;
    /* 需要发送的字节数 */
    g_SendNums = Length;

    /* 设置标志位，表明没有开始发送 */
    g_SendNums |= RS485_START_SEND;
    
    while(Length--)
    {
        if(i < BUFSIZE)
        {
            UARTSendBuf[i] = BufferPtr[i];
            i++;
        }
    }
    
    i = mtimer_create("485_check_send",
                       RS485_BeginSend, (void *)0,
                       3,SFT_TIMER_FLAG_ONE_SHOT
                    );
    if (MT_FULL != i)
        mtimer_start(i);
#if 0    
	LPC_GPIO3->DATA &= ~(1<<3);
    DelayMS(3);				 //不等的会漏数据
    while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
    #if !TX_INTERRUPT
//	  while ( !(LPC_UART->LSR & LSR_TEMT) );
        while ( !(LPC_UART->LSR & LSR_THRE) );
        LPC_UART->THR = *BufferPtr;
    #else
	  /* Below flag is set inside the interrupt handler when THRE occurs. */
        while ( !(UARTTxEmpty & 0x01) );
        LPC_UART->THR = *BufferPtr;

        UARTTxEmpty = 0;	/* not empty in the THR until it shifts out */
    #endif
        BufferPtr++;
        Length--;
    }
    while ((LPC_UART->LSR & 0x40) == 0);
    LPC_GPIO3->DATA |=  (1<<3);
//   LPC_GPIO3->DATA &= ~(1<<3);
#endif
	return;
}
#if 0
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
	LPC_GPIO3->DATA &= ~(1<<3);
    DelayMS(3);				 //不等的会漏数据
    while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
    #if !TX_INTERRUPT
//	  while ( !(LPC_UART->LSR & LSR_TEMT) );
        while ( !(LPC_UART->LSR & LSR_THRE) );
        LPC_UART->THR = *BufferPtr;
    #else
	  /* Below flag is set inside the interrupt handler when THRE occurs. */
        while ( !(UARTTxEmpty & 0x01) );
        LPC_UART->THR = *BufferPtr;

        UARTTxEmpty = 0;	/* not empty in the THR until it shifts out */
    #endif
        BufferPtr++;
        Length--;
    }
    while ((LPC_UART->LSR & 0x40) == 0);
    LPC_GPIO3->DATA |=  (1<<3);
//   LPC_GPIO3->DATA &= ~(1<<3);
	return;
}
#endif
void uartSendByte (uint8_t ucDat)
{
//	GPIOSetValue(3,3,0);//485 OE ENABLE
	LPC_GPIO3->DATA &= ~(1<<3);
    LPC_UART->THR = ucDat;                                              /*  写入数据                    */
//	GPIOSetValue(3,3,1);//485 OE DISABLE
    while ((LPC_UART->LSR & 0x40) == 0);                                /*  等待数据发送完毕            */
	LPC_GPIO3->DATA |=  (1<<3);
}


/******************************************************************************
**                            End Of File
******************************************************************************/
