#include <stdbool.h>
#include "lpc11xx.h"
#include "system_LPC11xx.h"
#include "dali_master.h"
#include "dali_master_2nd.h"
#include "dali_master_3rd.h"
#include "uart.h"
#include "wdt.h"
#include "timer32.h"
#include "i2c1.h"
#include "gpio.h"
#include "main.h"
#include "PanDali_cmd.h"
#include "sft_tmr.h"
#include "sft_time_apps.h"
#include "keyscan.h"
#include "Apps.h"
/***********************************************************/
/* Configuration flags                                     */
/***********************************************************/

/* in case of inverted RX path define INVERTED_RX */
#define INVERTED_RX
				


/***********************************************************/
/* Microcontroller and Board specific defines              */
/***********************************************************/

#define LED_config()         { LPC_GPIO3->DIR  |=  (1<<1); }
#define EnableSend485()		 { LPC_GPIO3->DATA &= ~(1<<3);}
#define DisableSend485()	 { LPC_GPIO3->DATA |=  (1<<3);}

/* PIO pin P2.6 is used as DALI send (tx) pin */
#define DALI_SetOutputHigh() { LPC_GPIO2->DATA |=  (1<<6); }
#define DALI_SetOutputLow()  { LPC_GPIO2->DATA &= ~(1<<6); }
#define DALI_ConfigOutput()  { LPC_GPIO2->DIR  |=  (1<<6); }

#if 0
#define VersionHi 		0x01
#define K1_OFF()		{LPC_GPIO3->DATA &= ~(1<<5);}
#define K2_OFF()		{LPC_GPIO2->DATA &= ~(1<<1);}
#define K3_OFF()		{LPC_GPIO3->DATA &= ~(1<<4);}
#define K1_ON()			{LPC_GPIO2->DATA &= ~(1<<5);}
#define K2_ON()			{LPC_GPIO0->DATA &= ~(1<<7);}
#define K3_ON()			{LPC_GPIO1->DATA &= ~(1<<9);}
#endif

/* PIO pin P1.1 is used as DALI receive (rx) pin */
#ifdef INVERTED_RX
#define DALI_GetInput(x)     { x = LPC_GPIO1->MASKED_ACCESS[(1<<1)] ? 0 : 1; }
#else
#define DALI_GetInput(x)     { x = LPC_GPIO1->MASKED_ACCESS[(1<<1)] ? 1 : 0; }
#endif

/* For receive, this module uses TMR32B0-CAP0 input (capture and interrupt on both edges) */
/* TMR32B0-CAP0 input (P1.5) is connected to P1.1 (to check high / low level by software) */
/* So set P1.5 as CT32B0.CAP0 (= DALI receive pin). Bit 7:6 (reserved) are also set to 1  */
#define DALI_ConfigInput()   { LPC_IOCON->PIO1_5 = 0xD2; }

/* TMR32B0 is used for DALI timing and capturing of DALI input */
#define TIMER_IRQ            TIMER_32_0_IRQn
#define GET_TIMER_REG_CR0(x) { x = LPC_TMR32B0->CR0; }
#define GET_TIMER_REG_IR(x)  { x = LPC_TMR32B0->IR;  }
#define SET_TIMER_REG_IR(x)  { LPC_TMR32B0->IR  = x; }
#define SET_TIMER_REG_PR(x)  { LPC_TMR32B0->PR  = x; }
#define SET_TIMER_REG_TC(x)  { LPC_TMR32B0->TC  = x; }
#define SET_TIMER_REG_CCR(x) { LPC_TMR32B0->CCR = x; }
#define SET_TIMER_REG_TCR(x) { LPC_TMR32B0->TCR = x; }
#define SET_TIMER_REG_MCR(x) { LPC_TMR32B0->MCR = x; }
#define SET_TIMER_REG_MR0(x) { LPC_TMR32B0->MR0 = x; }

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

#define Preset_addr             0xf29
#define AreaLink_addr			0x6f29
//#define Preset`_addr				0xf29






/***********************************************************/
/* Global variables                                        */
/***********************************************************/

#if 0
static volatile bool         usbForwardFrameReceived;
static volatile uint16_t     usbForwardFrame;
static volatile uint8_t      usbBackwardFrame; // DALI slave answer
static volatile answer_t     usbBackwardFrameAnswer;
static volatile MASTER_STATE masterState;
static volatile bool         waitForAnswer;
static volatile uint32_t     daliForwardFrame; // converted DALI master command
static volatile capturedFrameType     capturedFrame;    // data structure for the capture
#endif

volatile bool         usbForwardFrameReceived;
volatile uint16_t     usbForwardFrame;
volatile uint8_t      usbBackwardFrame; // DALI slave answer
volatile answer_t     usbBackwardFrameAnswer;
volatile MASTER_STATE masterState;
volatile bool         waitForAnswer;
volatile uint32_t     daliForwardFrame; // converted DALI master command
volatile capturedFrameType     capturedFrame;    // data structure for the capture

/****************************************************************/
/*	the following variables are defined by me				    */
/****************************************************************/

const uint8_t ArcPower[]=
{
	0, 	51,	  76,	91,	102, 110, 117, 122, 127, 132, 135, 139, 142, 145, 148,
	150, 153, 155, 157, 159, 161, 163, 164, 166, 167, 169, 170, 172, 173, 174,
	176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
	190, 191, 192, 193, 194, 194, 195, 196,	196, 197, 198, 198, 199, 200, 200,
	201, 202, 202, 203, 203, 204, 204, 205, 206, 206, 207, 207, 208, 208, 209,
	209, 210, 210, 211, 211, 212, 212, 212, 213, 213, 214, 214, 215, 215, 215,
	216, 216, 217, 217, 217, 218, 218, 219, 219, 219, 220, 220, 220, 221, 221,
	222, 222, 222, 223, 223, 223, 224, 224, 224, 225, 225, 225, 225, 226, 226,
	226, 227, 227, 227, 228, 228, 228, 228, 229, 229, 229, 230, 230, 230, 230,
	231, 231, 231, 232, 232, 232, 232, 233, 233, 233, 233, 234, 234, 234, 234,
	235, 235, 235, 235, 236, 236, 236, 236, 236, 237, 237, 237, 237, 238, 238,
	238, 238, 238, 239, 239, 239, 239, 240, 240, 240, 240, 240, 241, 241, 241,
	241, 241, 242, 242, 242, 242, 242, 243, 243, 243, 243, 243, 244, 244, 244,
	244, 244, 245, 245, 245, 245, 245, 245, 246, 246, 246, 246, 246, 247, 247,
	247, 247, 247, 247, 248, 248, 248, 248, 248, 248, 249, 249, 249, 249, 249,
	249, 250, 250, 250, 250, 250, 250, 251, 251, 251, 251, 251, 251, 251, 252,
	252, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 253, 254, 254, 254,
	254
} ;

const uint8_t LDS_Power[]=
{
	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,
	2,	2,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	4,	4,	4,	4,	4,
	4,	4,	4,	4,	4,	5,	5,	5,	5,	5,	5,	5,	5,	6,	6,	6,	6,	6,	6,	7,
	7,	7,	7,	7,	7,	8,	8,	8,	8,	8,	9,	9,	9,	9,	10,	10,	10,	10,	11,	11,
	11,	12,	12,	12,	13,	13,	14,	14,	14,	15,	15,	16,	16,	16,	17,	17,	18,	18,	19,	19,
	20,	20,	21,	21,	22,	23,	23,	24,	24,	25,	26,	27,	27,	28,	29,	30,	30,	31,	32,	33,
	34,	35,	36,	37,	38,	39,	40,	41,	42,	43,	45,	46,	47,	48,	50,	51,	53,	54,	55,	57,
	58,	60,	62,	63,	65,	67,	69,	71,	73,	75,	77,	79,	81,	83,	86,	88,	91,	93,	96,	98,
	101,104,107,109,112,116,119,122,125,129,133,136,140,144,148,152,156,160,165,169,
	174,179,184,189,194,200,205,211,216,223,229,235,241,248,254	
} ;

channelDataType	ChannelData[CHANNELNUMS];

static volatile bool         uartForwardFrameReceived;
static volatile uint16_t     uartForwardFrame;
static volatile uint8_t      uartBackwardFrame; // DALI slave answer
static volatile answer_t     uartBackwardFrameAnswer;
extern volatile uint16_t	 dali_counter_1st;
extern volatile uint16_t	 dali_counter_2nd;
extern volatile uint16_t	 dali_counter_3rd;
extern volatile uint8_t		 bp_flag;
//extern volatile uint8_t CH_temp_data[192];
//extern volatile uint32_t 	 pressed_time;
uint8_t 					 broadcast_flag;

extern volatile uint8_t 	 aux_flag;
extern uint8_t	UARTBuf[BUFSIZE]; 
uint8_t BlockOption=0;
volatile uint8_t supply_1st,supply_2nd,supply_3rd;
volatile uint8_t power_on_flag;


uint32_t					exit_block_option;
volatile uint8_t		 	ShortAddress;
static volatile uint8_t		 answerflag;
extern volatile uint16_t	 time_flag;
extern uint8_t  	 UARTBuffer[UART_BUFFER_LEN];
extern volatile uint32_t 	 UARTCount;
extern volatile uint8_t 	 TimeCir_1st;
extern volatile uint8_t 	 TimeCir_2nd;
extern volatile uint8_t 	 TimeCir_3rd;
extern uint16_t	 DALI_Data ; 
#if 0
extern volatile bool		 waitForAnswer_2nd;
extern volatile MASTER_STATE masterState_2nd;
extern volatile bool		 waitForAnswer_3rd;
extern volatile MASTER_STATE masterState_3rd;
#endif
extern volatile uint32_t	 flag20ms,Check_Change_Flag;
extern volatile uint16_t	 BlockWriteAddr;
extern volatile uint32_t	 UARTTxEmpty;
extern volatile uint8_t		 GucRcvNew;
extern volatile uint8_t 	 TriggerNum;
extern          uint8_t 	 Box,/*VersionH,VersionL,*/AppendArea;
extern volatile uint8_t 	 CheckFlag_1,CheckFlag_2,CheckFlag_3;
extern volatile uint16_t     usbForwardFrame_2nd;
extern volatile answer_t     usbBackwardFrameAnswer_2nd,usbBackwardFrameAnswer_3rd;
extern volatile uint8_t      BackwardFrame_2nd,BackwardFrame_3rd;
extern volatile uint8_t 	 aux_check;
extern uint8_t aux_press,aux_release,xcode,Tpud;



/***********************************************************/
/* Local functions                                         */
/***********************************************************/
#if 0
void DelayMS(uint32_t DelayTime)
{
	uint32_t time,i;

//	masterState =MS_RECEIVING_ANSWER ; 
	for(time=DelayTime;time!=0;time--)
	{
			for(i=4800;i!=0;i--)
			{
				;
			}
	}
//	masterState =MS_IDLE ;
}


static __inline bool DALI_CheckLogicalError(void)
{
    uint8_t  bitLevel;
    uint16_t receivedFrame;
    uint32_t bitStream, i, item, pattern, bitPair;

    // build frame from captured bit levels in bitStream
    bitStream = 0;
    for (i=0, item=0;  ((i < 18) && (item < capturedFrame.capturedItems+1)); item++)
    {
        bitLevel = capturedFrame.capturedData[item].bitLevel;
        bitStream |= (bitLevel << (17 - i));
        i++;
        // shift another bit in case of long symbol
        if (capturedFrame.capturedData[item].levelType == 'l')
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
    usbBackwardFrame = (uint8_t) receivedFrame;
    return false;
}

static __inline bool DALI_CheckTimingError(void)
{
    uint32_t i, capT1, capT2, interval;

    for (i=0; i < (capturedFrame.capturedItems - 1); i++)
    {
        capT1 = capturedFrame.capturedData[i].capturedTime;
        capT2 = capturedFrame.capturedData[i+1].capturedTime;
        interval = capT2 - capT1;
        if ((interval >= MIN_TE) && (interval <= MAX_TE))
        {
            capturedFrame.capturedData[i].levelType = 's';
        }
        else if ((interval >= MIN_2TE) && (interval <= MAX_2TE))
        {
            capturedFrame.capturedData[i].levelType = 'l';
        }
        else
        {
            return true; // timing error, so stop check immediately
        }
    }
    capturedFrame.capturedData[i].levelType = 'x'; // terminate the frame
    return false;
}

static __inline bool DALI_Decode(void)
{
    if (DALI_CheckTimingError()) return false;
    if (DALI_CheckLogicalError()) return false;
//	uartSendByte();
    return true;
}

uint32_t DALI_ConvertForwardFrame(uint16_t forwardFrame)
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

bool DALI_CheckWaitForAnswer(uint16_t forwardFrame)
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

bool DALI_CheckRepeatCmd(uint16_t forwardFrame)
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

static __inline void DALI_DoTransmission(uint32_t convertedForwardFrame, bool waitFlag)
{
    // Claim the bus and setup global variables
    masterState      = MS_TX_SECOND_HALF_START_BIT;
    waitForAnswer    = waitFlag;
    daliForwardFrame = convertedForwardFrame;



    DALI_SetOutputLow();
    // Activate the timer module to output the forward frame
    SET_TIMER_REG_TC(0);       // clear timer
    SET_TIMER_REG_MR0(TE);     // ~ 2400 Hz (half bit time)
    SET_TIMER_REG_CCR(0);      // disable capture
    SET_TIMER_REG_MCR(3);      // interrupt on MR0, reset timer on match 0
    SET_TIMER_REG_TCR(1);      // enable the timer

}
void For_Decode(void)
{
     	   if (capturedFrame.capturedItems == 0)
     	   {
     	       usbBackwardFrameAnswer = ANSWER_NOTHING_RECEIVED;
     	   }
     	   else if (capturedFrame.capturedItems == 1)
     	   {
     	       usbBackwardFrameAnswer = ANSWER_INVALID_DATA;
    	   }
        	else
        	{
        	    if (DALI_Decode())
        	    {
					waitForAnswer=FALSE;
        	        usbBackwardFrameAnswer = ANSWER_GOT_DATA;
            	}
            	else
            	{
            	    usbBackwardFrameAnswer = ANSWER_INVALID_DATA;
            	}
  			}
		
}


void DALI_Send(uint16_t forwardFrame)
{
    uint8_t  i = 0;
    uint8_t  n = 1;
	
    uint32_t convertedForwardFrame = DALI_ConvertForwardFrame(forwardFrame);
    bool     waitFlag = DALI_CheckWaitForAnswer(forwardFrame);
	
    if (DALI_CheckRepeatCmd(forwardFrame)) n = 2;
    while (i < n)
    {
		while(masterState!=MS_IDLE){;};
        DALI_DoTransmission(convertedForwardFrame, waitFlag);
        i++;
    }
}

static __inline void DALI_Init(void)
{
    // First init ALL the global variables
    usbForwardFrameReceived = false;
    usbForwardFrame         = 0;
    usbBackwardFrame        = 0;
    usbBackwardFrameAnswer  = ANSWER_NOT_AVAILABLE;
    masterState             = MS_IDLE;
    waitForAnswer           = false;
    daliForwardFrame        = 0;
    capturedFrame.capturedItems = 0;
    // Initialize the phisical layer of the dali master
    DALI_SetOutputHigh();
    DALI_ConfigOutput();
    DALI_ConfigInput();
    SET_TIMER_REG_PR((SystemFrequency/1000000) - 1); // timer runs at 1 MHz - 1usec per tick
    NVIC_ClearPendingIRQ(TIMER_IRQ);
    NVIC_EnableIRQ(TIMER_IRQ);
}


/***********************************************************/
/* Exported Counter/Timer IRQ handler                      */
/***********************************************************/

/* the handling of the protocol is done in the IRQ */
void TIMER32_0_IRQHandler(void)
{
static uint8_t bitcount;
       uint8_t irq_stat;

	//本段是发送数据的部分
    GET_TIMER_REG_IR(irq_stat);
    if (irq_stat & 1)
    {   // match 0 interrupt
        SET_TIMER_REG_IR(1);   // clear MR0 interrupt flag
        if (masterState == MS_TX_SECOND_HALF_START_BIT)
        {
            DALI_SetOutputHigh();
            bitcount = 0;
            masterState = MS_TX_DALI_FORWARD_FRAME;
        }
        else if (masterState == MS_TX_DALI_FORWARD_FRAME)
        {
            if (daliForwardFrame & 0x80000000)
            {
                DALI_SetOutputHigh();
            }				   
            else
            {
                DALI_SetOutputLow();
            }
            daliForwardFrame <<= 1;
            bitcount++;
            if (bitcount == 32) masterState = MS_TX_STOP_BITS;
        }
        else if (masterState == MS_TX_STOP_BITS)
        {
            DALI_SetOutputHigh();
            // the first half of the first stop bit has just been output.
            // do we have to wait for an answer?
            if (waitForAnswer)
            {   // elapse until the end of the last half of the second stop bit
                SET_TIMER_REG_MR0(4*TE);
                usbBackwardFrame = 0;
                capturedFrame.capturedItems = 0;
                masterState = MS_SETTLING_BEFORE_BACKWARD;
            }
            else
            {   // no answer from slave expected, need to wait for the remaining
                // bus idle time before next forward frame
            	// add additional 3 TE to minimum specification to be not at the edge of the timing specification
                SET_TIMER_REG_MR0((4*TE) + (22*TE) + (3*TE));
                masterState = MS_SETTLING_BEFORE_IDLE;
            }
        }
        else if (masterState == MS_SETTLING_BEFORE_BACKWARD)
        {   // setup the first window limit for the slave answer
            // slave should not respond before 7TE (-10%)
            SET_TIMER_REG_MR0((5*TE*9)/10);     //放宽时间限制，测试欧切斯镇流器时出现问题
            SET_TIMER_REG_CCR(7);   // enable receive, capture on both edges
            masterState = MS_WAITING_FOR_SLAVE_START_WINDOW;
        }
        else if (masterState == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // setup the second window limit for the slave answer,
            // slave must start transmit within the next 22 TE (+10%) window
            SET_TIMER_REG_MR0(((7*TE*2)/10) + ((22*TE*11)/10));
            masterState = MS_WAITING_FOR_SLAVE_START;
        }
        else if (masterState == MS_WAITING_FOR_SLAVE_START)
        {   // if we still get here, got 'no' or too early answer from slave
            // idle time of 22TE (+10%) was already elapsed while waiting, so
            // immediately release the bus
            SET_TIMER_REG_TCR(2);   // reset and stop the timer
            SET_TIMER_REG_CCR(0);   // disable capture
            SET_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
						waitForAnswer=0;		//没有接收到或者太早收到的，不需要等待应答了
            masterState = MS_IDLE;
        }
        else if (masterState == MS_RECEIVING_ANSWER)
        {   // stop receiving
            // now idle the bus between backward and next forward frame
            // since we don't track the last edge of received frame,
            // conservatively we wait for 23 TE (>= 22 TE as for specification)
            // Receive interval considered anyway the max tolerance for
            // backward frame duration so >22TE should already be asserted
            SET_TIMER_REG_MR0(23*TE);
            SET_TIMER_REG_CCR(0);   // disable capture
            SET_TIMER_REG_IR(0x10); // clear possible capture interrupt flag
            masterState = MS_SETTLING_BEFORE_IDLE;
        }
        else if (masterState == MS_SETTLING_BEFORE_IDLE)
        {
            SET_TIMER_REG_TCR(2);   // reset and stop the timer
		    masterState = MS_IDLE;
        }
    }
	//接收中断
    else if (irq_stat & 0x10)
    {   // capture interrupt
        uint32_t index = capturedFrame.capturedItems;
        SET_TIMER_REG_IR(0x10);     // clear capture interrupt flag
        if (masterState == MS_WAITING_FOR_SLAVE_START_WINDOW)
        {   // slave should not answer yet, it came too early!!!!
            SET_TIMER_REG_CCR(0);   // disable capture
        }
        else if (masterState == MS_WAITING_FOR_SLAVE_START)
        {   // we got an edge, so the slave is transmitting now
            // allowed remaining answering time is 22TE (+10%)
            SET_TIMER_REG_MR0(((22*TE*11)/10));
            SET_TIMER_REG_TC(0);
            SET_TIMER_REG_IR(1);    // clear possible MR0 interrupt flag
            // first pulse is begin of the start bit
			DALI_GetInput(capturedFrame.capturedData[index].bitLevel);
			capturedFrame.capturedData[index].capturedTime = 0;
            masterState = MS_RECEIVING_ANSWER;
        }
       		else if (masterState == MS_RECEIVING_ANSWER)
        	{   // this part just captures the frame data, evaluation is done
            	// at the end of max backward frame duration
				DALI_GetInput(capturedFrame.capturedData[index].bitLevel);
            	GET_TIMER_REG_CR0(capturedFrame.capturedData[index].capturedTime);
			
        	}

		if (capturedFrame.capturedItems < 17)
		{	
			capturedFrame.capturedItems++;
    	}
	}
}
#endif
/************************************************************************************************/
/*	the following code is used to enumerate the dali ballast									*/
/*	caculation:these are my codes and may have some bugs									*/

bool WriteBuffer(CirBufType *DaliCirBuffer,uint16_t data)
{
	if (DaliCirBuffer->read_index==((DaliCirBuffer->save_index+1)%BufferSize))
		return FALSE;
	else
	{ 
		DaliCirBuffer->buffer[DaliCirBuffer->save_index++]=data;
		DaliCirBuffer->save_index=DaliCirBuffer->save_index%BufferSize;
		return TURE; 
	}
}


bool ReadBuffer(CirBufType *DaliCirBuffer,uint16_t *dali_data)
{
	if (DaliCirBuffer->read_index==DaliCirBuffer->save_index)

		return FALSE;
	else
	{ 
		*dali_data = DaliCirBuffer->buffer[DaliCirBuffer->read_index++];
		DaliCirBuffer->buffer[DaliCirBuffer->read_index-1] = 0;
		DaliCirBuffer->read_index=DaliCirBuffer->read_index%BufferSize;
		return TURE;
	}
}


void ClearBuffer(CirBufType *DaliCirBuffer)
{
	DaliCirBuffer->read_index = DaliCirBuffer->save_index;
}
#if 0
uint16_t AddressingLow8bit(uint32_t LongAddress)
{
	
	uint16_t	AddrLowCMD = 0;
	AddrLowCMD |= ((0XB5<<8)|(uint8_t)(LongAddress&0xff));
	return AddrLowCMD;
}
uint16_t AddressingMiddle8bit(uint32_t LongAddress)
{
	
	uint16_t	AddrMiddleCMD = 0;
	AddrMiddleCMD |= ((0XB3<<8)|((LongAddress&0xff00)>>8));
	return AddrMiddleCMD;
}

uint16_t AddressingHigh8bit(uint32_t LongAddress)
{
	
	uint16_t	AddrHighCMD = 0;
	AddrHighCMD |= ((0XB1<<8)|((LongAddress&0xff0000)>>16));
	return AddrHighCMD;
}
static __inline uint8_t CompareAndReply(void)
{
	DALI_Send (0xa900);
	while(masterState!=MS_IDLE)
	{};
	For_Decode();
	if (usbBackwardFrameAnswer == ANSWER_INVALID_DATA)
	{
		return 0xff;
	}
	else
		if(usbBackwardFrameAnswer == ANSWER_GOT_DATA )
		{
			return usbBackwardFrame;
		}
		else if(usbBackwardFrameAnswer == ANSWER_NOTHING_RECEIVED)
		{
			return 0;
		}
	return 0x01;
} 

static __inline void InitialAll(void)
{
//	DALI_Send(0xA100);

//	DALI_Send(0xFE80);
//	while(masterState!=MS_IDLE){};
	DALI_Send(0XA3FF);
	while(masterState!=MS_IDLE){};
	DALI_Send(0XFF80);
	while(masterState!=MS_IDLE){};

	DelayMS(10);
	DALI_Send(INITIALISE);
	while(masterState!=MS_IDLE){};
}

static __inline void ProgramShortAddress(uint8_t ShortAddress)
{
	DALI_Send(0xBB00);
	while(masterState!=MS_IDLE){};
	DelayMS(50);
	For_Decode();
	if(usbBackwardFrame)					 //if no answer,what should i do?
	{
		usbBackwardFrame=0;
		DALI_Send((0xb7ul<<8)|(ShortAddress<<1)|1);
		while(masterState!=MS_IDLE){};
		DelayMS(20);
		DALI_Send(0XBB00);
		while(masterState!=MS_IDLE){};
		DelayMS(20);
		For_Decode();
		if(usbBackwardFrame==((ShortAddress<<1)|0x01))	 //if no answer ,what should i do?
		{
			usbBackwardFrame = 0;
			DALI_Send(0XAB00);
			while(masterState!=MS_IDLE){};
			DelayMS(20);
			DALI_Send((ShortAddress<<9)|0x100);
			while(masterState!=MS_IDLE){};
		}
	}	
		
}
#endif
void EE_Block_Option(void)
{
	if(UARTBuffer[0]==0xfa)
	{
		if(UARTBuffer[1]==DeviceCode)
		{
			if(UARTBuffer[2]==Box)
			{
				switch(UARTBuffer[3])
				{
					case 0xfd:
						if(UARTBuffer[5]==0)
						{
							BlockOption=0;
							TriggerNum=8;
						}
						break;
					case 0xfc:
						{
//							((void(code *)(void))0x00)();
							Get_Channel_Param();
							if(m24xx_read(EEPROM_24XX256,1,0,&UARTBuffer[2],1)==I2C_NO_ERR)
							{
								Box = UARTBuffer[2];
								UARTBuffer[3] = 0xfe;
								UARTBuffer[4] = VersionHi;
								UARTBuffer[5] = VersionLo;
								UARTBuffer[6] = 0x80;
								UARTBuffer[7] = CheckSum(UARTBuffer,7);
								UARTSend(UARTBuffer,8);
								BlockOption=0;
								TriggerNum=8;
								BlockWriteAddr=0;
							}
						}
						break;	 
					case 0xde:
						BlockRead();
						break;
//					case 0xdc:
//						BlockWriteAddr = (UARTBuffer[4]<<8)|UARTBuffer[5];
//						break;
					case 0xdd:
						UARTBuffer[3] = 0xdc;
						UARTBuffer[7] = UARTBuffer[7]+1;
						UARTSend(UARTBuffer,8);
						BlockWriteAddr=((UARTBuffer[4]<<8)|UARTBuffer[5]);
						break;
					case 0xba:
						UARTBuffer[3] =	0xfe;
						UARTBuffer[4] = VersionHi;
						UARTBuffer[5] = VersionLo;
						UARTBuffer[7] = CheckSum(UARTBuffer,7);
						UARTSend(UARTBuffer,8);
						break;
					default: break;
				}
			}

		}
	}
	else if(UARTBuffer[0]==0xfc)
	{
		BlockWrite();
	}
}

/*****************************
Fuction Name: Check_Channel
fuction:检查通道亮度是否需要变化
*****************************/
void Check_Channel(void)
{
	uint16_t index,tempdata_1,tempdata_2;
	for(index=0;index<192;index++)
	{
        /* 检查最大亮度 */
	 	if((ChannelData[index].MaxLevel>ChannelData[index].GoalLevel)&&(ChannelData[index].GoalLevel!=0))
		{
			ChannelData[index].GoalLevel = ChannelData[index].MaxLevel;
		}

		if(ChannelData[index].timeToGoal!=ChannelData[index].PastedTime)
		{
			if((ChannelData[index].GoalLevel&0xff)>(ChannelData[index].OringalLevel&0xff))
			{
				tempdata_2=~(ChannelData[index].OringalLevel+(((ChannelData[index].GoalLevel&0xff)-(ChannelData[index].OringalLevel))*(ChannelData[index].PastedTime+1))/ChannelData[index].timeToGoal);

				tempdata_1 = ArcPower[(uint8_t) tempdata_2];
				if(tempdata_1 != ChannelData[index].PrevDALILevel)
				{
					 ChannelData[index].dali_data= ((ChannelData[index].ShortAddr)<<9)|tempdata_1;
					 ChannelData[index].dali_TX_flag=1;
					 ChannelData[index].PastedTime++;
				}
			}
			else if((ChannelData[index].GoalLevel&0xff)<(ChannelData[index].OringalLevel&0xff))
			{
				tempdata_2 = ~(ChannelData[index].OringalLevel-(((ChannelData[index].OringalLevel&0xff)-(ChannelData[index].GoalLevel))*(ChannelData[index].PastedTime+1))/ChannelData[index].timeToGoal);
				tempdata_1 = ArcPower[(uint8_t) tempdata_2];
				if(tempdata_1 != ChannelData[index].PrevDALILevel)
				{
					tempdata_1 = ((ChannelData[index].ShortAddr)<<9)|tempdata_1;
					ChannelData[index].dali_data= ((ChannelData[index].ShortAddr)<<9)|tempdata_1;
					ChannelData[index].dali_TX_flag=1;
					ChannelData[index].PastedTime++;	
				}
			}
		}
		else
		{
			ChannelData[index].OringalLevel=ChannelData[index].GoalLevel;
			ChannelData[index].PastedTime=1;
			ChannelData[index].timeToGoal=1;
		}
	}
}
#if 0
void Check_DALI_Send(void)
{
	uint16_t i;
	if(broadcast_flag==1)
	{
		if(ChannelData[0].dali_TX_flag)
		{
			WriteBuffer(&DaliCirBuffer ,(((ChannelData[0].dali_data&0x00ff))|0xfe00));
			TimeCir_1st=1;
		}
		if(ChannelData[64].dali_TX_flag)
		{
			WriteBuffer(&DaliCirBuffer_2nd,((ChannelData[64].dali_data&0x00ff)|0xfe00));
			TimeCir_2nd=1;
		}
		if(ChannelData[128].dali_TX_flag)
		{
			WriteBuffer(&DaliCirBuffer_3rd,((ChannelData[128].dali_data&0x00ff)|0xfe00));
			TimeCir_3rd=1;
		}
	}
	for(i=0;i<192;i++)
	{
		if(ChannelData[i].dali_TX_flag)
		{
			switch(ChannelData[i].DALI_Channel&0x03)
			{
				case 1 :
					if((masterState==MS_IDLE))
					{
						if(broadcast_flag==0)
							WriteBuffer (&DaliCirBuffer,ChannelData[i].dali_data);
						TimeCir_1st=1;
						ChannelData[i].dali_TX_flag=0;

					}
					break;
				case 2:
					if((masterState_2nd==MS_IDLE))
					{
						if(broadcast_flag==0)
							WriteBuffer(&DaliCirBuffer_2nd,ChannelData[i].dali_data);
						ChannelData[i].dali_TX_flag=0;
						TimeCir_2nd=1;
						
					}
					break;	
				case 3:
					if((masterState_3rd == MS_IDLE)/*&&((power_on_flag&(1<<3))==(1<<3))*/	)
					{
						if(broadcast_flag==0)
							WriteBuffer(&DaliCirBuffer_3rd,ChannelData[i].dali_data);
						ChannelData[i].dali_TX_flag=0;
						TimeCir_3rd=1;
					}
					break;
				default:
					if(broadcast_flag == 0)
					{
						ChannelData[i].dali_TX_flag=0;
						ChannelData[i].GoalLevel=0xff;
						ChannelData[i].OringalLevel=0xff;
						ChannelData[i].PastedTime=1;
						ChannelData[i].timeToGoal=1;
					}
					else
					{
						ChannelData[i].dali_TX_flag=0;
					}
			}	
		}					   
	}
/*			if(((DaliCirBuffer.save_index-DaliCirBuffer.read_index)!=0)
			||((DaliCirBuffer_2nd.save_index-DaliCirBuffer_2nd.read_index)!=0)
			||((DaliCirBuffer_3rd.save_index-DaliCirBuffer_3rd.read_index)!=0))
			{
				Check_REL();
				Tpud=1;	
			}	*/
//			if(((power_on_flag&(1<<1))==0)||((power_on_flag&(1<<2))==0)||(((power_on_flag&(1<<1))==0)))
			{
//				Tpud=1;				//如果继电器是关的，延迟1s
			}


}

/***************************************
Fuction Name:Enumerate_DALI_Ballast()
Fuction :枚举镇流器
***************************************/														   
void Enumerate_DALI_Ballast(void)
{
	uint32_t				i;
	uint32_t				LongAddress;
	uint8_t 				databuf[10];		
	
	ShortAddress =0;

	InitialAll();	   
	
	LED_on();
	for(i=0;i<64;i++)
	{
		if(m24xx_read(EEPROM_24XX256,0x6629+i*4,0,databuf,4)==I2C_NO_ERR)
		{
			LongAddress=(databuf[1]<<16)|(databuf[2]<<8)|databuf[3];	
			if(((databuf[0]>>6)&0x03)==0)
			{
				DALI_Send((0xb5<<8)|databuf[3]);			//发送寻找短地址低八位
				while(masterState==MS_IDLE){};
				DALI_Send((0xb3<<8)|databuf[2]);			//发送寻找短地址中八位
				while(masterState==MS_IDLE);
				DALI_Send((0xb1<<8)|databuf[1]);			//发送寻找短地址高八位
				while(masterState==MS_IDLE){};

				LED_Toggle();

				if(CompareAndReply()==0xff)
				{
					ShortAddress=databuf[0]&0x3f;
					ProgramShortAddress(ShortAddress);
					ShortAddress++;	
				}
			}
		}
	}

	LED_Toggle();

	DelayMS(500);
	DALI_Send(0xa100);

	LED_Toggle();

	DelayMS(500);
	DALI_Send(0XA5FF);
	while(masterState!=MS_IDLE){};		
	DALI_Send(0xa700);	
	while(masterState!=MS_IDLE){};

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
				DALI_Send(AddressingLow8bit(LongAddress));
				while(masterState!=MS_IDLE){};
				DALI_Send(AddressingMiddle8bit(LongAddress));
				while(masterState!=MS_IDLE){};
				DALI_Send(AddressingHigh8bit(LongAddress));
				while(masterState!=MS_IDLE){};

				LED_Toggle();

				break;
			}
			
			if(LongAddress==0x800000)
			{
				DALI_Send(AddressingLow8bit(LongAddress));
				while(masterState!=MS_IDLE){};
				DALI_Send(AddressingMiddle8bit(LongAddress));
				while(masterState!=MS_IDLE){};
				DALI_Send(AddressingHigh8bit(LongAddress));
				while(masterState!=MS_IDLE){};
				PrevAddress = LongAddress;
				
				LED_Toggle();
				
				if(CompareAndReply()==0xff)
				{
					b = LongAddress;
					usbBackwardFrame=0;
				}
				else a =LongAddress;
			}
			else
			{
				DelayMS(20);
				if((LongAddress&0x0000ff) != (PrevAddress&0x0000ff))
					DALI_Send(AddressingLow8bit(LongAddress));
					while(masterState!=MS_IDLE){};
				if((LongAddress&0x00ff00) != (PrevAddress&0x00ff00))
					DALI_Send(AddressingMiddle8bit(LongAddress));
					while(masterState!=MS_IDLE){};
				if((LongAddress&0xff0000) != (PrevAddress&0xff0000))
					DALI_Send(AddressingHigh8bit(LongAddress));
					while(masterState!=MS_IDLE){};

				LED_Toggle();

				PrevAddress = LongAddress;

				if(CompareAndReply()==0xff)
				{
					b = LongAddress;
				}
				else a = LongAddress;
			}

			
		}
			
		if(LongAddress==0xFFFFFF)
		{
			DALI_Send(0xA100);
			while(masterState!=MS_IDLE){};
			break;

		}
		else 
		{
			ProgramShortAddress(ShortAddress);
			databuf[0]=ShortAddress;
			databuf[1]=(LongAddress>>16)&0xff;
			databuf[2]=(LongAddress>>8)&0xff;
			databuf[3]=LongAddress&0xff;
			m24xx_write(EEPROM_24XX256,0x6629+ShortAddress*4,0,databuf,4);
			DelayMS(10);
			ShortAddress++;
			if(ShortAddress>63)
			{	
				DALI_Send(0xa100);
				break;
			}
            
			LED_Toggle();
		}
	  }
	}

void Set_Ballast_Param(uint8_t DALI_Channel)
{
    uint8_t cir_i,delay_i;
    volatile uint8_t      *pucBackwardFrame;
    volatile answer_t     *pbUsbBackwardFrameAnswer;
    volatile MASTER_STATE *penMasterState;
    volatile bool         *pbWaitForAnswer;
    
    void (*pDaliSend)(uint16_t forwardFrame);
    void (*pDecodeDaliData)(void);
    
    UARTBuffer[3] = 0xca;  
    switch (ChannelData[UARTBuffer[5]].DALI_Channel & 0x03)
    {
        case 1:
        {
            penMasterState           = &masterState;
            pucBackwardFrame         = &usbBackwardFrame;
            pbWaitForAnswer          = &waitForAnswer;
            pbUsbBackwardFrameAnswer = &usbBackwardFrameAnswer;
            pDaliSend                = DALI_Send;
            pDecodeDaliData          = For_Decode;
            break;
        } 
        case 2:
        {
            penMasterState           = &masterState_2nd;
            pbWaitForAnswer          = &waitForAnswer_2nd;
            pucBackwardFrame         = &BackwardFrame_2nd;
            pbUsbBackwardFrameAnswer = &usbBackwardFrameAnswer_2nd;
            pDaliSend                = DALI_2nd_Send;
            pDecodeDaliData          = For_2nd_Decode;
            break;
        }
        case 3:
        {
            penMasterState           = &masterState_3rd;
            pbWaitForAnswer          = &waitForAnswer_3rd;
            pucBackwardFrame         = &BackwardFrame_3rd;
            pbUsbBackwardFrameAnswer = &usbBackwardFrameAnswer_3rd;
            pDaliSend                = DALI_3rd_Send;
            pDecodeDaliData          = For_3rd_Decode;
            break;
        }
        default:
            return;
    }
    pDaliSend(0XA300);
    while(*penMasterState != MS_IDLE){;};
    pDaliSend((cir_i<<9)|0x198);
    while(*penMasterState != MS_IDLE){;};
    pDecodeDaliData();
    if(0 == *pucBackwardFrame)
    {   
        /* 闪烁LED灯 */
        DelayMS(100);
        LPC_GPIO3->DATA ^= (1<<1);

        pDaliSend(0XA300);
        while(*penMasterState != MS_IDLE){;};
        for(delay_i=3;delay_i!=0;delay_i--)
        {
            DelayMS(100);
            LED_Toggle();
        }
        pDaliSend((cir_i<<9)|0x198);
        while(*penMasterState != MS_IDLE){;};
        pDecodeDaliData();
        if (0 == *pucBackwardFrame)	
        {
            for(delay_i=7;delay_i!=0;delay_i--)
            {
                DelayMS(100);
                LED_Toggle();
            }
            pDaliSend(0XA300);
            while(*penMasterState != MS_IDLE){;};
            for(delay_i=3;delay_i!=0;delay_i--)
            {
                DelayMS(100);
                LED_Toggle();
            }
            pDaliSend((cir_i<<9)|0x198);
            while(*penMasterState != MS_IDLE){;};
            pDecodeDaliData();
            if (0 == *pucBackwardFrame)
            {
                pDaliSend(0xa304);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xff2e);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xa300);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xff2b);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xa3fe);
                
                LED_Toggle();
                    
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xff2a);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xa300);
                while(*penMasterState != MS_IDLE){;};
                pDaliSend(0xff2d);
                while(*penMasterState != MS_IDLE){;};
                LED_Toggle();												
            }

        }
    }
}
#endif

uint8_t CheckSum(uint8_t *buffer,uint8_t Num)
{
	uint8_t i,sum=0;
	for(i=Num;i!=0;i--)
	{
		sum+=*buffer++;
	}
	sum = 256-sum;
	return sum;	
}



/*****************************************************************************************/
/***********************************************************/
/* Public (exported) functions                             */
/***********************************************************/

#if 0
void DALI_Thread(void)
{
	uint8_t nop_i ;
	uint32_t aux_value;   

	DALI_Init();
	DALI_2nd_Init();
	DALI_3rd_Init();
	ClearBuffer(&DaliCirBuffer);
    LED_config();
	DisableSend485();
    
    Sft_Init_all_timers((void *) 0);
    while (1)
    {
        __WFI();
//		WDTFeed();

        sft_timer_thread();
        
        if(RcvNew)
        { 	
			dali_counter_1st=0;
			if(CheckSum(UARTBuf,TriggerNum-1)==UARTBuf[TriggerNum-1])
			{
				for(nop_i=0;nop_i<TriggerNum;nop_i++)
				{
					UARTBuffer[nop_i]=UARTBuf[nop_i]; 
				}
				if(BlockOption==0)
				{
					ProcessCMD();
				}
				else
				{
					EE_Block_Option();
					exit_block_option=0;
				}
			}
			RcvNew--;
			UARTCount=0;	
		}
/*
		if(BlockOption==1)
		{
			if(exit_block_option>50)
				BlockOption=0;
		}
		*/
       
        if(key_pressed & (1<<4))
        {
            if(long_press_time[4] > 150)
            {
                LED_on();
                NVIC_SystemReset();
            }
            else
            {
                if(key_released & (1<<4))
                {
                    UARTBuffer[0]=0xfa;
                    UARTBuffer[1]=DeviceCode;
                    UARTBuffer[2] = Box;
                    UARTBuffer[3] = 0xfe;
                    UARTBuffer[4] = VersionHi;
                    UARTBuffer[5] = VersionLo;
                    UARTBuffer[6] = 0x00;
                    UARTBuffer[7] = CheckSum(UARTBuffer,7);
                    UARTSend(UARTBuffer,8);
                }
            }
        }

		if((waitForAnswer==TURE)&&(masterState==MS_IDLE))
		{
			For_Decode();								 //返回的数据在usbBackwardFrame中
			UARTBuffer[0]=0xfa;
			UARTBuffer[1]=0xec;
			UARTBuffer[2]=Box;
			UARTBuffer[3]=0x6e;
			UARTBuffer[4]=3;
			UARTBuffer[5]=usbBackwardFrame;
			UARTBuffer[6]=0;
			UARTBuffer[7]=CheckSum(UARTBuffer,7);
			waitForAnswer = FALSE;
			UARTSend(UARTBuffer,8);
		}
		if((waitForAnswer_2nd==TURE)&&(masterState_2nd==MS_IDLE))
		{
			For_2nd_Decode();							//返回的数据在BackwardFrame_2nd中
			UARTBuffer[0]=0xfa;
			UARTBuffer[1]=0xec;
			UARTBuffer[2]=Box;
			UARTBuffer[3]=0x6e;
			UARTBuffer[4]=3;
			UARTBuffer[5]=BackwardFrame_2nd;
			UARTBuffer[6]=0;
			UARTBuffer[7]=CheckSum(UARTBuffer,7);
			waitForAnswer_2nd = FALSE;
			UARTSend(UARTBuffer,8);
		}
		if((waitForAnswer_3rd==TURE)&&(masterState_3rd==MS_IDLE))
		{
			For_3rd_Decode();						    //返回的数据在BackwardFrame_2nd中
			UARTBuffer[0]=0xfa;
			UARTBuffer[1]=0xec;
			UARTBuffer[2]=Box;
			UARTBuffer[3]=0x6e;
			UARTBuffer[4]=3;
			UARTBuffer[5]=BackwardFrame_2nd;
			UARTBuffer[6]=0;
			UARTBuffer[7]=CheckSum(UARTBuffer,7);
			waitForAnswer_3rd = FALSE;
			UARTSend(UARTBuffer,8);
		}

				
		if(TimeCir_1st)
		{
			if(Tpud==0)
//			if((power_on_flag&0x02)==0x02)						   //检测DALI回路是否供电了，0x02表示供电了，如果没有电，打开继电器
			{
				if(masterState==MS_IDLE)			 			  //检测总线是否有数据在发送
				{
					if(ReadBuffer(&DaliCirBuffer,&DALI_Data))	  //从DALI第一通道（印刷的是第三通道）的发送缓冲区里读取数据
					{
						DALI_Send(DALI_Data);					  //从缓冲区读取数据成功，发送到对应DALI回路
					}
					else
					{	
						dali_counter_1st = 0  ;					  //从缓冲区读取数据失败，缓冲区为空，表示发送完成
						TimeCir_1st=0;
//						Check_REL();
					}											  //检查本DALI回路是否所有灯都灭了，如果是，关掉继电器
				}
			}
//			else Check_REL();									  //如果没有电，打开继电器
		}


		if(TimeCir_2nd)
		{
			if(Tpud==0)
//			if((power_on_flag&0x04)==0x04)
			{
				if(masterState_2nd==MS_IDLE)
				{
					if(ReadBuffer(&DaliCirBuffer_2nd,&DALI_Data))
					{
						DALI_2nd_Send(DALI_Data);
					}
					else
					{					
						dali_counter_2nd = 0  ;
						TimeCir_2nd	=0;
//						Check_REL();
					}						
				}
			}
//			else Check_REL();

		}


		if(TimeCir_3rd)
		{
			if(Tpud==0)
//			if((power_on_flag&0x08)==0x08)
			{
				if(masterState_3rd==MS_IDLE)
				{
					if(ReadBuffer(&DaliCirBuffer_3rd,&DALI_Data))
					{
						DALI_3rd_Send(DALI_Data);
					}
					else
					{					
						dali_counter_3rd = 0  ;
						TimeCir_3rd=0;
//						Check_REL();
					}		
				
				}
			}
//			else Check_REL();
		}
				

		if(CheckFlag_1)										//1s一次检查
		{
			if(m24xx_read(EEPROM_24XX256,0X7259,0,&broadcast_flag,1)==I2C_NO_ERR)
			{
				if(broadcast_flag!=0x01)
				{
					broadcast_flag = 0;
				}
			}
			if(Tpud!=0)
			{
				Tpud--;
			}
					  			
			if(supply_1st)
			{												//检测供电
				supply_1st=0;
				power_on_flag |=(1ul<<1);
			}
			else 
			{
				power_on_flag &=~(1ul<<1);
//				Tpud=1;
			}
			if(supply_2nd)
			{
				supply_2nd=0;
				power_on_flag |= (1ul<<2);
			}
			else 
			{
				power_on_flag &=~(1ul<<2);
//				Tpud=1;
			}
			if(supply_3rd)
			{
				supply_3rd=0;
				power_on_flag |= (1ul<<3);
			}
			else 
			{
				power_on_flag &=~(1ul<<3) ;
//				Tpud=1;
			}
//			if(Tpud == 0)
			{
				Check_DALI_Send();								//检查是否有DALI数据需要发送
			}

			CheckFlag_1=0;
		}


		if(Check_Change_Flag)								 //20ms
		{	
			if(Tpud==0)
			{
				Check_Channel();								 //检查通道亮度是否变化，如果变化
			}
//			Check_REL();
//			exit_block_option++;
			Check_Change_Flag=0;
		}


		if(aux_check>1)
		{

			aux_value=(LPC_GPIO3->DATA&(1<<2));
			if((aux_flag&0x80)==(aux_value<<5))
			{
				if((LPC_GPIO3->DATA&(1<<2))==0)
				{
					if(aux_press!=0x04)
					{
						m24xx_read(EEPROM_24XX256,0xF1A,0,UARTBuf,7);
						UARTBuf[7]= CheckSum(UARTBuf,7);
						RcvNew=1;
						for(nop_i=0;nop_i<8;nop_i++)
						{
							UARTBuffer[nop_i]=UARTBuf[nop_i];
						}
						UARTSend(UARTBuffer,8);
						aux_flag &= 0x80;
				//		aux_check = 0;
					}
 
				}
				else
				{
					if(aux_release !=0x00)
					{
						m24xx_read(EEPROM_24XX256,0xF22,0,UARTBuf,7);
						UARTBuf[7]= CheckSum(UARTBuf,7);
						RcvNew=1;
						for(nop_i=0;nop_i<8;nop_i++)
						{
							UARTBuffer[nop_i]=UARTBuf[nop_i];
						}
						UARTSend(UARTBuffer,8);
						aux_flag &= 0x80;
				//		aux_check = 0;	
					}
				}
				aux_check = 0;
				aux_flag &=0x80;
			}
		}
    }  
}
#endif
		
/* EOF */
