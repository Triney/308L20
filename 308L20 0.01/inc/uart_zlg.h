#include "LPC11xx.h"

extern void uartInit (void);
extern void uartSendByte (uint8_t ucDat);
extern void uartSendStr (uint8_t const *pucStr, uint32_t ulNum);
void UART_IRQHandler (void)	;