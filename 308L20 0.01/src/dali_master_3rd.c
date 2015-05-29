#if 0
#include <stdbool.h>
#include "lpc11xx.h"
#include "system_LPC11xx.h"
#include "dali_master_3rd.h"
#include "uart.h"
#include "wdt.h"
#include "timer16.h"
#include "i2c1.h"

/* in case of inverted RX path define INVERTED_RX */
#define INVERTED_RX

/***********************************************************/
/* Microcontroller and Board specific defines              */
/***********************************************************/


/* PIO pin P2.8 is used as 3rd DALI send (tx) pin */
#define DALI_3rd_SetOutputHigh() { LPC_GPIO2->DATA |=  (1<<0); } 
#define DALI_3rd_SetOutputLow()  { LPC_GPIO2->DATA &= ~(1<<0); }
#define DALI_3rd_ConfigOutput()  { LPC_GPIO2->DIR  |=  (1<<0); }

/* PIO pin P0.9 is used as 3rd DALI receive (rx) pin */
#ifdef INVERTED_RX
#define DALI_3rd_GetInput(x)     { x = LPC_GPIO2->MASKED_ACCESS[(1ul<<11)] ? 0 : 1; }
#else
#define DALI_3rd_GetInput(x)     { x = LPC_GPIO2->MASKED_ACCESS[(1ul<<11)] ? 1 : 0; }
#endif

/* For receive, this module uses TMR16B0-CAP0 input (capture and interrupt on both edges) */
/* TMR16B0-CAP0 input (P1.8) is connected to P0.9 (to check high / low level by software) */
/* So set P1.5 as CT16B1.CAP0 (= DALI receive pin). Bit 7:6 (reserved) are also set to 1  */
#define DALI_3rd_ConfigInput()   { LPC_IOCON->PIO1_8 = 0xD1; }

/* TMR32B0 is used for DALI timing and capturing of DALI input */
#define TIMER_3rd_IRQ            TIMER_16_1_IRQn
#define GET_3rd_TIMER_REG_CR0(x) { x = LPC_TMR16B1->CR0; }
#define GET_3rd_TIMER_REG_IR(x)  { x = LPC_TMR16B1->IR;  }
#define SET_3rd_TIMER_REG_IR(x)  { LPC_TMR16B1->IR  = x; }
#define SET_3rd_TIMER_REG_PR(x)  { LPC_TMR16B1->PR  = x; }
#define SET_3rd_TIMER_REG_TC(x)  { LPC_TMR16B1->TC  = x; }
#define SET_3rd_TIMER_REG_CCR(x) { LPC_TMR16B1->CCR = x; }
#define SET_3rd_TIMER_REG_TCR(x) { LPC_TMR16B1->TCR = x; }
#define SET_3rd_TIMER_REG_MCR(x) { LPC_TMR16B1->MCR = x; }
#define SET_3rd_TIMER_REG_MR0(x) { LPC_TMR16B1->MR0 = x; }


/***********************************************************/
/* Type definitions and defines                            */
/***********************************************************/


#if 0
/* protocol timing definitions */
#define TE          (417)        // half bit time = 417 usec
#if 0 /* strict receive timing according to specification  */
#define MIN_TE      (TE - 42)            // minimum half bit time
#define MAX_TE      (TE + 42)            // maximum half bit time
#define MIN_2TE     (2*TE - 83)          // minimum full bit time
#define MAX_2TE     (2*TE + 83)          // maximum full bit time
#else /* More relaxed receive timing */
#define MIN_TE      (300)            		    // minimum half bit time
#define MAX_TE      (550)            		    // maximum half bit time
#define MIN_2TE     (2*TE - (2*(TE/5)))		    // minimum full bit time
#define MAX_2TE     (2*TE + (2*(TE/5)))         // maximum full bit time
#endif

#define CMD32       (0x0120) // these configuration commands need to be repeated
#define CMD129      (0x0180)
#define CMD144      (0x0190)
#define CMD157      (0x019D)
#define CMD160      (0x01A0)
#define CMD165      (0x01A5)
#define CMD176      (0x01B0)
#define CMD197      (0x01C5)
#define CMD255      (0x01FF)
#define CMD260		(0xA900)
#define CMD268      (0xB901)
#define CMD269      (0xBB00)
#define INITIALISE  (0xA500) // command for starting initialization mode
#define RANDOMISE   (0xA700) // command for generating a random address

#endif

/***************************************************************/
/*	Struct for channel property								   */
/***************************************************************/


extern volatile uint8_t 		ShortAddress;


/***********************************************************/
/* Global variables                                        */
/***********************************************************/

static volatile bool         usbForwardFrameReceived_3rd;
static volatile uint16_t     usbForwardFrame_3rd;
volatile uint8_t      BackwardFrame_3rd; // DALI slave answer
volatile answer_t     usbBackwardFrameAnswer_3rd;
volatile MASTER_STATE masterState_3rd;
volatile bool         waitForAnswer_3rd;
static volatile uint32_t     daliForwardFrame_3rd ;// converted DALI master command
static volatile capturedFrameType     capturedFrame_3rd;    // data structure for the capture


static __inline bool DALI_3rd_CheckLogicalError(void)
{
    uint8_t  bitLevel;
    uint16_t receivedFrame;
    uint32_t bitStream, i, item, pattern, bitPair;

    // build frame from captured bit levels in bitStream
    bitStream = 0;
    for (i=0, item=0;  ((i < 18) && (item < capturedFrame_3rd.capturedItems+1)); item++)
    {
        bitLevel = capturedFrame_3rd.capturedData[item].bitLevel;
        bitStream |= (bitLevel << (17 - i));
        i++;
        // shift another bit in case of long symbol
        if (capturedFrame_3rd.capturedData[item].levelType == 'l')
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
    BackwardFrame_3rd = (uint8_t) receivedFrame;
    return false;
}

static __inline bool DALI_3rd_CheckTimingError(void)
{
    uint32_t i, capT1, capT2, interval;

    for (i=0; i < (capturedFrame_3rd.capturedItems - 1); i++)
    {
        capT1 = capturedFrame_3rd.capturedData[i].capturedTime;
        capT2 = capturedFrame_3rd.capturedData[i+1].capturedTime;
        interval = capT2 - capT1;
        if ((interval >= MIN_TE) && (interval <= MAX_TE))
        {
            capturedFrame_3rd.capturedData[i].levelType = 's';
        }
        else if ((interval >= MIN_2TE) && (interval <= MAX_2TE))
        {
            capturedFrame_3rd.capturedData[i].levelType = 'l';
        }
        else
        {
            return true; // timing error, so stop check immediately
        }
    }
    capturedFrame_3rd.capturedData[i].levelType = 'x'; // terminate the frame
    return false;
}

bool DALI_3rd_Decode(void)
{
    if (DALI_3rd_CheckTimingError()) return false;
    if (DALI_3rd_CheckLogicalError()) return false;
    return true;
}

static __inline void DALI_3rd_DoTransmission(uint32_t convertedForwardFrame, bool waitFlag)
{
    // Claim the bus and setup global variables
    masterState_3rd      = MS_TX_SECOND_HALF_START_BIT;
    waitForAnswer_3rd    = waitFlag;
    daliForwardFrame_3rd = convertedForwardFrame;



    DALI_3rd_SetOutputLow();
    // Activate the timer module to output the forward frame
    SET_3rd_TIMER_REG_TC(0);       // clear timer
    SET_3rd_TIMER_REG_MR0(TE);     // ~ 2400 Hz (half bit time)
    SET_3rd_TIMER_REG_CCR(0);      // disable capture
    SET_3rd_TIMER_REG_MCR(3);      // interrupt on MR0, reset timer on match 0
    SET_3rd_TIMER_REG_TCR(1);      // enable the timer
 
}

void DALI_3rd_Send(uint16_t forwardFrame)
{
    uint8_t  i = 0;
    uint8_t  n = 1;
	
    uint32_t convertedForwardFrame = DALI_ConvertForwardFrame(forwardFrame);
    bool     waitFlag = DALI_CheckWaitForAnswer(forwardFrame);
	
    if (DALI_CheckRepeatCmd(forwardFrame)) n = 2;
    while (i < n)
    {
		while(masterState_3rd!=MS_IDLE){};
        DALI_3rd_DoTransmission(convertedForwardFrame, waitFlag);
        i++;
    }
}

void DALI_3rd_Init(void)
{
    // First init ALL the global variables
    usbForwardFrameReceived_3rd = false;
    usbForwardFrame_3rd         = 0;
    BackwardFrame_3rd           = 0;
    usbBackwardFrameAnswer_3rd  = ANSWER_NOT_AVAILABLE;
    masterState_3rd             = MS_IDLE;
    waitForAnswer_3rd           = false;
    daliForwardFrame_3rd        = 0;
    capturedFrame_3rd.capturedItems = 0;
    // Initialize the phisical layer of the dali master
    DALI_3rd_SetOutputHigh();
    DALI_3rd_ConfigOutput();
    DALI_3rd_ConfigInput();
    SET_3rd_TIMER_REG_PR((SystemFrequency/1000000) - 1); // timer runs at 1 MHz - 1usec per tick
    NVIC_ClearPendingIRQ(TIMER_3rd_IRQ);
    NVIC_EnableIRQ(TIMER_3rd_IRQ);
}

void For_3rd_Decode()
{
     	   if (capturedFrame_3rd.capturedItems == 0)
     	   {
     	       usbBackwardFrameAnswer_3rd = ANSWER_NOTHING_RECEIVED;
     	   }
     	   else if (capturedFrame_3rd.capturedItems == 1)
     	   {
     	       usbBackwardFrameAnswer_3rd = ANSWER_INVALID_DATA;
//				answerflag=1;
    	   }
        	else
        	{
        	    if (DALI_3rd_Decode())
        	    {
	//				uartSendByte(BackwardFrame_3rd);
					waitForAnswer_3rd=FALSE;
        	        usbBackwardFrameAnswer_3rd = ANSWER_GOT_DATA;
	//				answerflag = 1;
            	}
            	else
            	{
            	    usbBackwardFrameAnswer_3rd = ANSWER_INVALID_DATA;
	//				answerflag=1;
            	}
  			}
		
}


/***********************************************************/
/* Exported Counter/Timer IRQ handler                      */
/***********************************************************/

/* the handling of the protocol is done in the IRQ */
void TIMER16_1_IRQHandler(void)
{
static uint8_t bitcount;
       uint8_t irq_stat;

	//本段是发送数据的部分
    GET_3rd_TIMER_REG_IR(irq_stat);
    if (irq_stat & 1)
    {   // match 0 interrupt
        SET_3rd_TIMER_REG_IR(1);   // clear MR0 interrupt flag
        if (masterState_3rd == MS_TX_SECOND_HALF_START_BIT)
        {
            DALI_3rd_SetOutputHigh();
            bitcount = 0;
            masterState_3rd = MS_TX_DALI_FORWARD_FRAME;
        }
        else if (masterState_3rd == MS_TX_DALI_FORWARD_FRAME)
        {
            if (daliForwardFrame_3rd & 0x80000000)
            {
                DALI_3rd_SetOutputHigh();
            }				   
            else
            {
                DALI_3rd_SetOutputLow();
            }
            daliForwardFrame_3rd <<= 1;
            bitcount++;
            if (bitcount == 32) masterState_3rd = MS_TX_STOP_BITS;
        }
        else if (masterState_3rd == MS_TX_STOP_BITS)
        {
            DALI_3rd_SetOutputHigh();
            // the first half of the first stop bit has just been output.
            // do we have to wait for an answer?
            if (waitForAnswer_3rd)
            {   // elapse until the end of the last half of the second stop bit
                SET_3rd_TIMER_REG_MR0(4*TE);
                BackwardFrame_3rd = 0;
                capturedFrame_3rd.capturedItems = 0;
                masterState_3rd = MS_SETTLING_BEFORE_BACKWARD;
            }
            else
            {   // no answer from slave expected, need to wait for the remaining
                // bus idle time before next forward frame
            	// add additional 3 TE to minimum specification to be not at the edge of the timing specification
                SET_3rd_TIMER_REG_MR0((4*TE) + (22*TE) + (3*TE));
                masterState_3rd = MS_SETTLING_BEFORE_IDLE;
            }
        }
        else if (masterState_3rd == MS_SETTLING_BEFORE_BACKWARD)
        {   // setup the first window limit for the slave answer
            // slave should not respond before 7TE (-10%)
            SET_3rd_TIMER_REG_MR0((5*TE*9)/10);      //放宽时间限制，测试欧切斯镇流器时出现问题
            SET_3rd_TIMER_REG_CCR(7);   // enable receive, capture on both edges
            masterState_3rd = MS_WAITING_FOR_SLAVE_START_WINDOW;
        }
        else if (masterState_3rd == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // setup the second window limit for the slave answer,
            // slave must start transmit within the next 22 TE (+10%) window
            SET_3rd_TIMER_REG_MR0(((7*TE*2)/10) + ((22*TE*11)/10));
            masterState_3rd = MS_WAITING_FOR_SLAVE_START;
        }
        else if (masterState_3rd == MS_WAITING_FOR_SLAVE_START)
        {   // if we still get here, got 'no' or too early answer from slave
            // idle time of 22TE (+10%) was already elapsed while waiting, so
            // immediately release the bus
            SET_3rd_TIMER_REG_TCR(2);   // reset and stop the timer
            SET_3rd_TIMER_REG_CCR(0);   // disable capture
            SET_3rd_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
            masterState_3rd = MS_IDLE;
        }
        else if (masterState_3rd == MS_RECEIVING_ANSWER)
        {   // stop receiving
            // now idle the bus between backward and next forward frame
            // since we don't track the last edge of received frame,
            // conservatively we wait for 23 TE (>= 22 TE as for specification)
            // Receive interval considered anyway the max tolerance for
            // backward frame duration so >22TE should already be asserted
            SET_3rd_TIMER_REG_MR0(23*TE);
            SET_3rd_TIMER_REG_CCR(0);   // disable capture
            SET_3rd_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
            masterState_3rd = MS_SETTLING_BEFORE_IDLE;
        }
        else if (masterState_3rd == MS_SETTLING_BEFORE_IDLE)
        {
            SET_3rd_TIMER_REG_TCR(2);   // reset and stop the timer
		    masterState_3rd = MS_IDLE;
        }
    }
	//接收中断
    else if (irq_stat & 0x10)
    {   // capture interrupt
        uint32_t index = capturedFrame_3rd.capturedItems;
        SET_3rd_TIMER_REG_IR(0x10);     // clear capture interrupt flag
        if (masterState_3rd == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // slave should not answer yet, it came too early!!!!
            SET_3rd_TIMER_REG_CCR(0);   // disable capture
        }
        else if (masterState_3rd == MS_WAITING_FOR_SLAVE_START)
        {   // we got an edge, so the slave is transmitting now
            // allowed remaining answering time is 22TE (+10%)
            SET_3rd_TIMER_REG_MR0(((22*TE*11)/10));
            SET_3rd_TIMER_REG_TC(0);
            SET_3rd_TIMER_REG_IR(1);    // clear possible MR0 interrupt flag
            // first pulse is begin of the start bit
			DALI_3rd_GetInput(capturedFrame_3rd.capturedData[index].bitLevel);
			capturedFrame_3rd.capturedData[index].capturedTime = 0;
            masterState_3rd = MS_RECEIVING_ANSWER;
        }
       		else if (masterState_3rd == MS_RECEIVING_ANSWER)
        	{   // this part just captures the frame data, evaluation is done
            	// at the end of max backward frame duration
				DALI_3rd_GetInput(capturedFrame_3rd.capturedData[index].bitLevel);
            	GET_3rd_TIMER_REG_CR0(capturedFrame_3rd.capturedData[index].capturedTime);
			
        	}

		if (capturedFrame_3rd.capturedItems < 17)
		{	
			capturedFrame_3rd.capturedItems++;
    	}
	}
}




static __inline uint8_t CompareAndReply_3rd(void)
{
	DALI_3rd_Send (0xa900);
	while(masterState_3rd!=MS_IDLE)
	{};
	For_3rd_Decode();
		if (usbBackwardFrameAnswer_3rd == ANSWER_INVALID_DATA)
	{
		return 0xff;
	}
	else
		if(usbBackwardFrameAnswer_3rd == ANSWER_GOT_DATA )
		{
			return BackwardFrame_3rd;
		}
		else if(usbBackwardFrameAnswer_3rd == ANSWER_NOTHING_RECEIVED)
		{
			return 0;
		}
	return BackwardFrame_3rd;
} 

static __inline void InitialAll_3rd(void)
{
//	DALI_Send(0xA100);

//	DALI_3rd_Send(0xFE80);
//	while(masterState_3rd!=MS_IDLE){};
	DALI_3rd_Send(0XA3FF);
	while(masterState_3rd!=MS_IDLE){};
	DALI_3rd_Send(0XFF80);
	while(masterState_3rd!=MS_IDLE){};

	DelayMS(10);
	DALI_3rd_Send(INITIALISE);
	while(masterState_3rd!=MS_IDLE){};
}

static __inline void ProgramShortAddress_3rd(uint8_t ShortAddress)
{
	DALI_3rd_Send(0xBB00);
	while(masterState_3rd!=MS_IDLE){};
	DelayMS(50);
	For_3rd_Decode();
	if(BackwardFrame_3rd)					 //if no answer,what should i do?
	{
		BackwardFrame_3rd=0;
		DALI_3rd_Send((0xb7ul<<8)|(ShortAddress<<1)|1);
		while(masterState_3rd!=MS_IDLE){};
		DelayMS(20);
		DALI_3rd_Send(0XBB00);
		while(masterState_3rd!=MS_IDLE){};
		DelayMS(20);
		For_3rd_Decode();
		if(BackwardFrame_3rd==((ShortAddress<<1)|0x01))	 //if no answer ,what should i do?
		{
			BackwardFrame_3rd = 0;
			DALI_3rd_Send(0XAB00);
			while(masterState_3rd!=MS_IDLE){};
			DelayMS(20);
			DALI_3rd_Send((ShortAddress<<9)|0x100);
			while(masterState_3rd!=MS_IDLE){};
		}
	}	
		
}










/***************************************
Fuction Name:Enumerate_DALI_Ballast()
Fuction :枚举镇流器
***************************************/
														   
void Enumerate_DALI_Ballast_3rd(void)
{
	uint32_t				i;
	uint32_t				LongAddress;
	uint8_t 				databuf[10];		
	
	ShortAddress =0;

	InitialAll_3rd();	   

	LED_Toggle();
	
	for(i=0;i<64;i++)
	{
		if(m24xx_read(EEPROM_24XX256,0x6829+i*4,0,databuf,4)==I2C_NO_ERR)
		{
			LongAddress=(databuf[1]<<16)|(databuf[2]<<8)|databuf[3];	
			if(((databuf[0]>>6)&0x03)==2)
			{
				DALI_3rd_Send((0xb5<<8)|databuf[3]);			//发送寻找短地址低八位
				while(masterState_3rd==MS_IDLE);
				DALI_3rd_Send((0xb3<<8)|databuf[2]);			//发送寻找短地址中八位
				while(masterState_3rd==MS_IDLE);
				DALI_3rd_Send((0xb1<<8)|databuf[1]);			//发送寻找短地址高八位
				while(masterState_3rd==MS_IDLE);
	
				if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
				{
					LED_on();
				}				
				else 	LED_off();

				if(CompareAndReply_3rd()==0xff)
				{
					ShortAddress=databuf[0]&0x3f;
					ProgramShortAddress_3rd(ShortAddress);
					ShortAddress++;	
				}
			}
		}
	}

	LED_Toggle();

	DelayMS(500);

	LED_Toggle();

	DALI_3rd_Send(0xa100);
	DelayMS(500);

	LED_Toggle();

	DALI_3rd_Send(0XA5FF);
	while(masterState_3rd!=MS_IDLE){};		
	DALI_3rd_Send(0xa700);	
	while(masterState_3rd!=MS_IDLE){};

	LED_Toggle();

	for(i=64;i!=0;i--)
	{
		uint32_t a=0,b=0xffffff;
//		uint32_t FindAddrFlag;
		static volatile uint32_t PrevAddress=0;
		while(a!=b)
		{
			
			LongAddress = ((a+b+1)/2);
			if(LongAddress==a||LongAddress==b)
			{
				DALI_3rd_Send(AddressingLow8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};
				DALI_3rd_Send(AddressingMiddle8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};
				DALI_3rd_Send(AddressingHigh8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};

				LED_Toggle();

				break;
			}
			
			if(LongAddress==0x800000)
			{
				DALI_3rd_Send(AddressingLow8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};
				DALI_3rd_Send(AddressingMiddle8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};
				DALI_3rd_Send(AddressingHigh8bit(LongAddress));
				while(masterState_3rd!=MS_IDLE){};
				PrevAddress = LongAddress;

				LED_Toggle();

				if(CompareAndReply_3rd()==0xff)
				{
					b = LongAddress;
					BackwardFrame_3rd=0;
				}
				else a =LongAddress;
			}
			else
			{
				DelayMS(20);
				if((LongAddress&0xff) != (PrevAddress&0xff))
					DALI_3rd_Send(AddressingLow8bit(LongAddress));
					while(masterState_3rd!=MS_IDLE){};
				if((LongAddress&0xff00) != (PrevAddress&0xff00))
					DALI_3rd_Send(AddressingMiddle8bit(LongAddress));
					while(masterState_3rd!=MS_IDLE){};
				if((LongAddress&0xff0000) != (PrevAddress&0xff0000))
					DALI_3rd_Send(AddressingHigh8bit(LongAddress));
					while(masterState_3rd!=MS_IDLE){};

				LED_Toggle();

				PrevAddress = LongAddress;

				if(CompareAndReply_3rd()==0xff)
				{
					b = LongAddress;
				}
				else a = LongAddress;
			}

			
		}
			
		if(LongAddress==0xFFFFFF)
		{
			DALI_3rd_Send(0xA100);
			while(masterState_3rd!=MS_IDLE){};
			break;

		}
		else 
		{
			ProgramShortAddress_3rd(ShortAddress);
			databuf[0]=ShortAddress|(1<<7);
			databuf[1]=(LongAddress>>16)&0xff;
			databuf[2]=(LongAddress>>8)&0xff;
			databuf[3]=LongAddress&0xff;
			m24xx_write(EEPROM_24XX256,0x6829+ShortAddress*4,0,databuf,4);
			ShortAddress++;

			LED_Toggle();

			if(ShortAddress>63)
			{	
				DALI_3rd_Send(0xa100);
				break;
			}
		}
	  }
	}
#endif


