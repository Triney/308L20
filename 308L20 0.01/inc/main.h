#ifndef _MAIN_H_
#define _MAIN_H_

#include "lpc11xx.h"

extern volatile uint16_t BlockWriteAddr;

void SysTickInit(uint8_t msTime);
void BlockRead(void);
void BlockWrite(void);
void Get_Channel_Param(void);
void RelayPinSetLow(void);
void Check_REL(void);
void Get_Offset(void);
void Set_Offset(void);

#endif
