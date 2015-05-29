#if 0
#include <stdbool.h>
#include "lpc11xx.h"
#include "system_LPC11xx.h"
#include "dali_master_2nd.h"
#include "uart.h"
#include "wdt.h"
#include "timer16.h"
#include "i2c1.h"

/* in case of inverted RX path define INVERTED_RX */
#define INVERTED_RX

/***********************************************************/
/* Microcontroller and Board specific defines              */
/***********************************************************/


/* PIO pin P2.7 is used as 2th DALI send (tx) pin */
#define DALI_2nd_SetOutputHigh() { LPC_GPIO2->DATA |=  (1<<7); } 
#define DALI_2nd_SetOutputLow()  { LPC_GPIO2->DATA &= ~(1<<7); }
#define DALI_2nd_ConfigOutput()  { LPC_GPIO2->DIR  |=  (1<<7); }

/* PIO pin P0.8 is used as 2th DALI receive (rx) pin */
#ifdef INVERTED_RX
#define DALI_2nd_GetInput(x)     { x = LPC_GPIO0->MASKED_ACCESS[(1<<8)] ? 0 : 1; }
#else
#define DALI_2nd_GetInput(x)     { x = LPC_GPIO0->MASKED_ACCESS[(1<<8)] ? 1 : 0; }
#endif

/* For receive, this module uses TMR16B0-CAP0 input (capture and interrupt on both edges) */
/* TMR16B0-CAP0 input (P0.2) is connected to P1.8 (to check high / low level by software) */
/* So set P1.5 as CT32B0.CAP0 (= DALI receive pin). Bit 7:6 (reserved) are also set to 1  */
#define DALI_2nd_ConfigInput()   { LPC_IOCON->PIO0_2 = 0xD2; }

/* TMR32B0 is used for DALI timing and capturing of DALI input */
#define TIMER_2nd_IRQ            TIMER_16_0_IRQn
#define GET_2nd_TIMER_REG_CR0(x) { x = LPC_TMR16B0->CR0; }
#define GET_2nd_TIMER_REG_IR(x)  { x = LPC_TMR16B0->IR;  }
#define SET_2nd_TIMER_REG_IR(x)  { LPC_TMR16B0->IR  = x; }
#define SET_2nd_TIMER_REG_PR(x)  { LPC_TMR16B0->PR  = x; }
#define SET_2nd_TIMER_REG_TC(x)  { LPC_TMR16B0->TC  = x; }
#define SET_2nd_TIMER_REG_CCR(x) { LPC_TMR16B0->CCR = x; }
#define SET_2nd_TIMER_REG_TCR(x) { LPC_TMR16B0->TCR = x; }
#define SET_2nd_TIMER_REG_MCR(x) { LPC_TMR16B0->MCR = x; }
#define SET_2nd_TIMER_REG_MR0(x) { LPC_TMR16B0->MR0 = x; }


/***************************************************/

extern volatile uint8_t 		ShortAddress;

/***********************************************************/
/* Global variables                                        */
/***********************************************************/

static volatile bool         usbForwardFrameReceived_2nd;
static volatile uint16_t     usbForwardFrame_2nd;
volatile uint8_t      BackwardFrame_2nd; // DALI slave answer
volatile answer_t     usbBackwardFrameAnswer_2nd;
volatile MASTER_STATE masterState_2nd;
volatile bool         waitForAnswer_2nd;
static volatile uint32_t     daliForwardFrame_2nd ;// converted DALI master command
static volatile capturedFrameType     capturedFrame_2nd;    // data structure for the capture


static __inline bool DALI_2nd_CheckLogicalError(void)
{
    uint8_t  bitLevel;
    uint16_t receivedFrame;
    uint32_t bitStream, i, item, pattern, bitPair;

    // build frame from captured bit levels in bitStream
    bitStream = 0;
    for (i=0, item=0;  ((i < 18) && (item < capturedFrame_2nd.capturedItems+1)); item++)
    {
        bitLevel = capturedFrame_2nd.capturedData[item].bitLevel;
        bitStream |= (bitLevel << (17 - i));
        i++;
        // shift another bit in case of long symbol
        if (capturedFrame_2nd.capturedData[item].levelType == 'l')
        {
            bitStream |= (bitLevel << (17 - i));
            i++;
        }
    }
    // check if there are 3 zeros or 3 ones in a row
    for (i=0; i < 16; i++)
    {
        pattern = 7 << i;
        if (((bitStream & pattern) == 0) ||
            ((bitStream & pattern) == pattern))
        {
            return true; // error, invalid data, so return immediately
        }
    }
    // compose answer byte in receivedFrame
    receivedFrame = 0;
    for (i=0; i < 18; i += 2)
    {
        receivedFrame <<= 1;
        bitPair = (bitStream >> (16 - i)) & 3;
        if ((bitPair == 0) || bitPair == 3)
        {

            return true; // error '00' or '11' is not a valid bit
        }
        if (bitPair == 1) receivedFrame |= 1;
    }
    // need to have the start bit in position 9 for a valid frame
    if (!(receivedFrame & 0x100)) return true;
    // cast out the start bit for the answer byte
    BackwardFrame_2nd = (uint8_t) receivedFrame;
    return false;
}

static __inline bool DALI_2nd_CheckTimingError(void)
{
    uint32_t i, capT1, capT2, interval;

    for (i=0; i < (capturedFrame_2nd.capturedItems - 1); i++)
    {
        capT1 = capturedFrame_2nd.capturedData[i].capturedTime;
        capT2 = capturedFrame_2nd.capturedData[i+1].capturedTime;
        interval = capT2 - capT1;
        if ((interval >= MIN_TE) && (interval <= MAX_TE))
        {
            capturedFrame_2nd.capturedData[i].levelType = 's';
        }
        else if ((interval >= MIN_2TE) && (interval <= MAX_2TE))
        {
            capturedFrame_2nd.capturedData[i].levelType = 'l';
        }
        else
        {
            return true; // timing error, so stop check immediately
        }
    }
    capturedFrame_2nd.capturedData[i].levelType = 'x'; // terminate the frame
    return false;
}

bool DALI_2nd_Decode(void)
{
    if (DALI_2nd_CheckTimingError()) return false;
    if (DALI_2nd_CheckLogicalError()) return false;
//	uartSendByte();
    return true;
}
/*
static __inline uint32_t DALI_ConvertForwardFrame_2nd(uint16_t forwardFrame)
{
    uint32_t convertedForwardFrame = 0;
    int8_t   i;
    
    for (i=15; i>=0; i--)
    {
        if (forwardFrame & (1 << i))
        {   // shift in bits values '0' and '1'
            convertedForwardFrame <<= 1;
            convertedForwardFrame <<= 1;
            convertedForwardFrame  |= 1;
        }
        else
        {   // shift in bits values '1' and '0'
            convertedForwardFrame <<= 1;
            convertedForwardFrame  |= 1;
            convertedForwardFrame <<= 1;
        }
    }
    return convertedForwardFrame;
}

static __inline bool DALI_CheckWaitForAnswer(uint16_t forwardFrame)
{
    bool waitFlag = false;

    if (((forwardFrame & 0x01FF) >= CMD144 && (forwardFrame & 0x01FF) <= CMD157) ||
        ((forwardFrame & 0x01FF) >= CMD160 && (forwardFrame & 0x01FF) <= CMD165) ||
        ((forwardFrame & 0x01FF) >= CMD176 && (forwardFrame & 0x01FF) <= CMD197) ||
        ((forwardFrame & 0x01FF) == CMD255) ||
        (forwardFrame == CMD260) ||
		(forwardFrame == CMD268) ||
        (forwardFrame == CMD269))
    {
        waitFlag = true;
    }
    return waitFlag;
}

static __inline bool DALI_CheckRepeatCmd(uint16_t forwardFrame)
{
    bool repeatCmd = false;
    
    // configuration command shall all be repeated within 100 ms
    if (((forwardFrame & 0x01FF) >= CMD32) &&
        ((forwardFrame & 0x01FF) <= CMD129))
    {
        repeatCmd = true;
    }
    // randomize and initialize shall be repeated within 100 ms
    if (((forwardFrame & 0xFF00) == INITIALISE) ||
        (forwardFrame == RANDOMISE))
    {
        repeatCmd = true;										 
    }
    return repeatCmd;
}
*/
static __inline void DALI_2nd_DoTransmission(uint32_t convertedForwardFrame, bool waitFlag)
{
    // Claim the bus and setup global variables
    masterState_2nd      = MS_TX_SECOND_HALF_START_BIT;
    waitForAnswer_2nd    = waitFlag;
    daliForwardFrame_2nd = convertedForwardFrame;



    DALI_2nd_SetOutputLow();
    // Activate the timer module to output the forward frame
    SET_2nd_TIMER_REG_TC(0);       // clear timer
    SET_2nd_TIMER_REG_MR0(TE);     // ~ 2400 Hz (half bit time)
    SET_2nd_TIMER_REG_CCR(0);      // disable capture
    SET_2nd_TIMER_REG_MCR(3);      // interrupt on MR0, reset timer on match 0
    SET_2nd_TIMER_REG_TCR(1);      // enable the timer
 /*   while (masterState_2nd != MS_IDLE)
    {
        // wait till transmission is completed
        // __WFI();
    }
    if (waitForAnswer_2nd)
    {
        if (capturedFrame_2nd.capturedItems == 0)
        {
            usbBackwardFrameAnswer_2nd = ANSWER_NOTHING_RECEIVED;
        }
        else if (capturedFrame_2nd.capturedItems == 1)
        {
            usbBackwardFrameAnswer_2nd = ANSWER_INVALID_DATA;
//			answerflag=1;
        }
        else
        {
            if (DALI_2nd_Decode())
            {
				uartSendByte(BackwardFrame_2nd);
                usbBackwardFrameAnswer_2nd = ANSWER_GOT_DATA;
            }
            else
            {
                usbBackwardFrameAnswer_2nd = ANSWER_INVALID_DATA;
            }
        }


        //while (usbBackwardFrameAnswer != ANSWER_NOT_AVAILABLE)   //没有使用上位机程序且使用的是串口，为了防止程序停止在这边先注释掉
        {
            // wait till answer is send to USB host (PC)
            // __WFI();
        }
    }	*/
}

void DALI_2nd_Send(uint16_t forwardFrame)
{
    uint8_t  i = 0;
    uint8_t  n = 1;
	
    uint32_t convertedForwardFrame = DALI_ConvertForwardFrame(forwardFrame);
    bool     waitFlag = DALI_CheckWaitForAnswer(forwardFrame);
	
    if (DALI_CheckRepeatCmd(forwardFrame)) n = 2;
    while (i < n)
    {
		while(masterState_2nd!=MS_IDLE){};
        DALI_2nd_DoTransmission(convertedForwardFrame, waitFlag);
        i++;
    }
}

void DALI_2nd_Init(void)
{
    // First init ALL the global variables
    usbForwardFrameReceived_2nd = false;
    usbForwardFrame_2nd         = 0;
    BackwardFrame_2nd           = 0;
    usbBackwardFrameAnswer_2nd  = ANSWER_NOT_AVAILABLE;
    masterState_2nd             = MS_IDLE;
    waitForAnswer_2nd           = false;
    daliForwardFrame_2nd        = 0;
    capturedFrame_2nd.capturedItems = 0;
    // Initialize the phisical layer of the dali master
    DALI_2nd_SetOutputHigh();
    DALI_2nd_ConfigOutput();
    DALI_2nd_ConfigInput();
    SET_2nd_TIMER_REG_PR((SystemFrequency/1000000) - 1); // timer runs at 1 MHz - 1usec per tick
    NVIC_ClearPendingIRQ(TIMER_2nd_IRQ);
    NVIC_EnableIRQ(TIMER_2nd_IRQ);
}

void For_2nd_Decode()
{
     	   if (capturedFrame_2nd.capturedItems == 0)
     	   {
     	       usbBackwardFrameAnswer_2nd = ANSWER_NOTHING_RECEIVED;
     	   }
     	   else if (capturedFrame_2nd.capturedItems == 1)
     	   {
     	       usbBackwardFrameAnswer_2nd = ANSWER_INVALID_DATA;
//				answerflag=1;
    	   }
        	else
        	{
        	    if (DALI_2nd_Decode())
        	    {
					waitForAnswer_2nd=FALSE;
        	        usbBackwardFrameAnswer_2nd = ANSWER_GOT_DATA;
	//				answerflag = 1;
            	}
            	else
            	{
            	    usbBackwardFrameAnswer_2nd = ANSWER_INVALID_DATA;
	//				answerflag=1;
            	}
  			}
		
}


/***********************************************************/
/* Exported Counter/Timer IRQ handler                      */
/***********************************************************/

/* the handling of the protocol is done in the IRQ */
void TIMER16_0_IRQHandler(void)
{
static uint8_t bitcount;
       uint8_t irq_stat;

	//本段是发送数据的部分
    GET_2nd_TIMER_REG_IR(irq_stat);
    if (irq_stat & 1)
    {   // match 0 interrupt
        SET_2nd_TIMER_REG_IR(1);   // clear MR0 interrupt flag
        if (masterState_2nd == MS_TX_SECOND_HALF_START_BIT)
        {
            DALI_2nd_SetOutputHigh();
            bitcount = 0;
            masterState_2nd = MS_TX_DALI_FORWARD_FRAME;
        }
        else if (masterState_2nd == MS_TX_DALI_FORWARD_FRAME)
        {
            if (daliForwardFrame_2nd & 0x80000000)
            {
                DALI_2nd_SetOutputHigh();
            }				   
            else
            {
                DALI_2nd_SetOutputLow();
            }
            daliForwardFrame_2nd <<= 1;
            bitcount++;
            if (bitcount == 32) masterState_2nd = MS_TX_STOP_BITS;
        }
        else if (masterState_2nd == MS_TX_STOP_BITS)
        {
            DALI_2nd_SetOutputHigh();
            // the first half of the first stop bit has just been output.
            // do we have to wait for an answer?
            if (waitForAnswer_2nd)
            {   // elapse until the end of the last half of the second stop bit
                SET_2nd_TIMER_REG_MR0(4*TE);
                BackwardFrame_2nd = 0;
                capturedFrame_2nd.capturedItems = 0;
                masterState_2nd = MS_SETTLING_BEFORE_BACKWARD;
            }
            else
            {   // no answer from slave expected, need to wait for the remaining
                // bus idle time before next forward frame
            	// add additional 3 TE to minimum specification to be not at the edge of the timing specification
                SET_2nd_TIMER_REG_MR0((4*TE) + (22*TE) + (3*TE));
                masterState_2nd = MS_SETTLING_BEFORE_IDLE;
            }
        }
        else if (masterState_2nd == MS_SETTLING_BEFORE_BACKWARD)
        {   // setup the first window limit for the slave answer
            // slave should not respond before 7TE (-10%)
            SET_2nd_TIMER_REG_MR0((5*TE*9)/10);			//放宽时间限制，测试欧切斯镇流器时出现问题
            SET_2nd_TIMER_REG_CCR(7);   // enable receive, capture on both edges
            masterState_2nd = MS_WAITING_FOR_SLAVE_START_WINDOW;
        }
        else if (masterState_2nd == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // setup the second window limit for the slave answer,
            // slave must start transmit within the next 22 TE (+10%) window
            SET_2nd_TIMER_REG_MR0(((7*TE*2)/10) + ((22*TE*11)/10));
            masterState_2nd = MS_WAITING_FOR_SLAVE_START;
        }
        else if (masterState_2nd == MS_WAITING_FOR_SLAVE_START)
        {   // if we still get here, got 'no' or too early answer from slave
            // idle time of 22TE (+10%) was already elapsed while waiting, so
            // immediately release the bus
            SET_2nd_TIMER_REG_TCR(2);   // reset and stop the timer
            SET_2nd_TIMER_REG_CCR(0);   // disable capture
            SET_2nd_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
            masterState_2nd = MS_IDLE;
        }
        else if (masterState_2nd == MS_RECEIVING_ANSWER)
        {   // stop receiving
            // now idle the bus between backward and next forward frame
            // since we don't track the last edge of received frame,
            // conservatively we wait for 23 TE (>= 22 TE as for specification)
            // Receive interval considered anyway the max tolerance for
            // backward frame duration so >22TE should already be asserted
            SET_2nd_TIMER_REG_MR0(23*TE);
            SET_2nd_TIMER_REG_CCR(0);   // disable capture
            SET_2nd_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
            masterState_2nd = MS_SETTLING_BEFORE_IDLE;
        }
        else if (masterState_2nd == MS_SETTLING_BEFORE_IDLE)
        {
            SET_2nd_TIMER_REG_TCR(2);   // reset and stop the timer
		    masterState_2nd = MS_IDLE;
        }
    }
	//接收中断
    else if (irq_stat & 0x10)
    {   // capture interrupt
        uint32_t index = capturedFrame_2nd.capturedItems;
        SET_2nd_TIMER_REG_IR(0x10);     // clear capture interrupt flag
        if (masterState_2nd == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // slave should not answer yet, it came too early!!!!
            SET_2nd_TIMER_REG_CCR(0);   // disable capture
        }
        else if (masterState_2nd == MS_WAITING_FOR_SLAVE_START)
        {   // we got an edge, so the slave is transmitting now
            // allowed remaining answering time is 22TE (+10%)
            SET_2nd_TIMER_REG_MR0(((22*TE*11)/10));
            SET_2nd_TIMER_REG_TC(0);
            SET_2nd_TIMER_REG_IR(1);    // clear possible MR0 interrupt flag
            // first pulse is begin of the start bit
			DALI_2nd_GetInput(capturedFrame_2nd.capturedData[index].bitLevel);
			capturedFrame_2nd.capturedData[index].capturedTime = 0;
            masterState_2nd = MS_RECEIVING_ANSWER;
        }
       		else if (masterState_2nd == MS_RECEIVING_ANSWER)
        	{   // this part just captures the frame data, evaluation is done
            	// at the end of max backward frame duration
				DALI_2nd_GetInput(capturedFrame_2nd.capturedData[index].bitLevel);
            	GET_2nd_TIMER_REG_CR0(capturedFrame_2nd.capturedData[index].capturedTime);
			
        	}

		if (capturedFrame_2nd.capturedItems < 17)
		{	
			capturedFrame_2nd.capturedItems++;
    	}
	}
}







static __inline uint8_t CompareAndReply_2nd(void)
{
	DALI_2nd_Send (0xa900);
	while(masterState_2nd!=MS_IDLE)
	{};
	For_2nd_Decode();
		if (usbBackwardFrameAnswer_2nd == ANSWER_INVALID_DATA)
	{
		return 0xff;
	}
	else
		if(usbBackwardFrameAnswer_2nd == ANSWER_GOT_DATA )
		{
			return BackwardFrame_2nd;
		}
		else if(usbBackwardFrameAnswer_2nd == ANSWER_NOTHING_RECEIVED)
		{
			return 0;
		}
	return BackwardFrame_2nd;
} 

static __inline void InitialAll_2nd(void)
{
//	DALI_Send(0xA100);

//	DALI_2nd_Send(0xFE80);
//	while(masterState_2nd!=MS_IDLE){};
	DALI_2nd_Send(0XA3FF);
	while(masterState_2nd!=MS_IDLE){};
	DALI_2nd_Send(0XFF80);
	while(masterState_2nd!=MS_IDLE){};

	DelayMS(10);
	DALI_2nd_Send(INITIALISE);
	while(masterState_2nd!=MS_IDLE){};
}

static __inline void ProgramShortAddress_2nd(uint8_t ShortAddress)
{
	DALI_2nd_Send(0xBB00);
	while(masterState_2nd!=MS_IDLE){};
	DelayMS(50);
	For_2nd_Decode();
	if(BackwardFrame_2nd)					 //if no answer,what should i do?
	{
		BackwardFrame_2nd=0;
		DALI_2nd_Send((0xb7ul<<8)|(ShortAddress<<1)|1);
		while(masterState_2nd!=MS_IDLE){};
		DelayMS(20);
		DALI_2nd_Send(0XBB00);
		while(masterState_2nd!=MS_IDLE){};
		DelayMS(20);
		For_2nd_Decode();
		if(BackwardFrame_2nd==((ShortAddress<<1)|0x01))	 //if no answer ,what should i do?
		{
			BackwardFrame_2nd = 0;
			DALI_2nd_Send(0XAB00);
			while(masterState_2nd!=MS_IDLE){};
			DelayMS(20);
			DALI_2nd_Send((ShortAddress<<9)|0x100);
			while(masterState_2nd!=MS_IDLE){};
		}
	}	
		
}










/***************************************
Fuction Name:Enumerate_DALI_Ballast()
Fuction :枚举镇流器
***************************************/
														   
void Enumerate_DALI_Ballast_2nd(void)
{
	uint32_t				i;
	uint32_t				LongAddress;
	uint8_t 				databuf[10];		
	
	ShortAddress =0;

	InitialAll_2nd();	   
		
    LED_Toggle();
	
	for(i=0;i<64;i++)
	{
		if(I2C_NO_ERR != m24xx_read(EEPROM_24XX256,0x6729+i*4,0,databuf,4))
		{
            continue;
        }
        LongAddress=(databuf[1]<<16)|(databuf[2]<<8)|databuf[3];	
        if(((databuf[0]>>6)&0x03)==1)
        {
            DALI_2nd_Send((0xb5<<8)|databuf[3]);			//发送寻找短地址低八位
            while(masterState_2nd==MS_IDLE){};
            DALI_2nd_Send((0xb3<<8)|databuf[2]);			//发送寻找短地址中八位
            while(masterState_2nd==MS_IDLE);
            DALI_2nd_Send((0xb1<<8)|databuf[1]);			//发送寻找短地址高八位
            while(masterState_2nd==MS_IDLE){};

            LED_Toggle();

            if(CompareAndReply_2nd()==0xff)
            {
                ShortAddress=databuf[0]&0x3f;
                ProgramShortAddress_2nd(ShortAddress);
                ShortAddress++;	
            }
        }
		
	}

    LED_Toggle();

    DelayMS(500);
    DALI_2nd_Send(0xa100);

    LED_Toggle();

	DelayMS(500);
	DALI_2nd_Send(0XA5FF);
	while(masterState_2nd!=MS_IDLE){};		

    LED_Toggle();

	DALI_2nd_Send(0xa700);	
	while(masterState_2nd!=MS_IDLE){};

    LED_Toggle();

	for(i=64;i!=0;i--)
	{
		uint32_t a=0,b=0xffffff;
		static volatile uint32_t PrevAddress=0;
		while(a!=b)
		{
			
			LongAddress = ((a+b+1)/2);
			if(LongAddress==a||LongAddress==b)
			{
				DALI_2nd_Send(AddressingLow8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};
				DALI_2nd_Send(AddressingMiddle8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};
				DALI_2nd_Send(AddressingHigh8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};

                LED_Toggle();

				break;
			}
			
			if(LongAddress==0x800000)
			{
				DALI_2nd_Send(AddressingLow8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};
				DALI_2nd_Send(AddressingMiddle8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};
				DALI_2nd_Send(AddressingHigh8bit(LongAddress));
				while(masterState_2nd!=MS_IDLE){};
				PrevAddress = LongAddress;


                LED_Toggle();

				if(CompareAndReply_2nd()==0xff)
				{
					b = LongAddress;
					BackwardFrame_2nd=0;
				}
				else a =LongAddress;
			}
			else
			{
				DelayMS(20);
				if((LongAddress&0xff) != (PrevAddress&0xff))
					DALI_2nd_Send(AddressingLow8bit(LongAddress));
					while(masterState_2nd!=MS_IDLE){};
				if((LongAddress&0xff00) != (PrevAddress&0xff00))
					DALI_2nd_Send(AddressingMiddle8bit(LongAddress));
					while(masterState_2nd!=MS_IDLE){};
				if((LongAddress&0xff0000) != (PrevAddress&0xff0000))
					DALI_2nd_Send(AddressingHigh8bit(LongAddress));
					while(masterState_2nd!=MS_IDLE){};

                LED_Toggle();

				PrevAddress = LongAddress;

				if(CompareAndReply_2nd()==0xff)
				{
					b = LongAddress;
				}
				else a = LongAddress;
			}

			
		}
			
		if(LongAddress==0xFFFFFF)
		{
			DALI_2nd_Send(0xA100);
			while(masterState_2nd!=MS_IDLE){};
			break;

		}
		else 
		{
			ProgramShortAddress_2nd(ShortAddress);
			databuf[0]= ShortAddress|(1ul<<6) ;
			databuf[1]=(LongAddress>>16)&0xff;
			databuf[2]=(LongAddress>>8)&0xff;
			databuf[3]=LongAddress&0xff;
			m24xx_write(EEPROM_24XX256,0x6729+ShortAddress*4,0,databuf,4);
			ShortAddress++;

            LED_Toggle();

			if(ShortAddress>63)
			{	
				DALI_2nd_Send(0xa100);
				break;
			}
		}
	  }
	}
#endif
	

