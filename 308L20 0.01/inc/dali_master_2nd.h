#ifndef _DALI_MASTER_2ND_H_
#define _DALI_MASTER_2ND_H_

#include "lpc11xx.h"
#include "dali_master.h"

extern volatile uint8_t      BackwardFrame_2nd; // DALI slave answer
extern volatile answer_t     usbBackwardFrameAnswer_2nd;
extern volatile MASTER_STATE masterState_2nd;
extern volatile bool         waitForAnswer_2nd;

void DALI_2nd_Init(void);
void DALI_2nd_Send(uint16_t forwardFrame);
void For_2nd_Decode(void);
bool DALI_2nd_Decode(void);
void Enumerate_DALI_Ballast_2nd(void);

#endif
