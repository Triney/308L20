#ifndef _DALI_MASTER_3RD_H_
#define _DALI_MASTER_3RD_H_
#if 0
#include "lpc11xx.h"
#include "dali_master.h"

extern volatile uint8_t      BackwardFrame_3rd; // DALI slave answer
extern volatile answer_t     usbBackwardFrameAnswer_3rd;
extern volatile MASTER_STATE masterState_3rd;
extern volatile bool         waitForAnswer_3rd;

void DALI_3rd_Init(void);
void DALI_3rd_Send(uint16_t forwardFrame);
void For_3rd_Decode(void);
extern bool DALI_3rd_Decode(void);
void Enumerate_DALI_Ballast_3rd(void);
#endif
#endif
