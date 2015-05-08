/******************************************************************************/
/*  Copyright (c) 2011 NXP B.V.  All rights are reserved.                     */
/*  Reproduction in whole or in part is prohibited without the prior          */
/*  written consent of the copyright owner.                                   */
/*                                                                            */
/*  This software and any compilation or derivative thereof is, and           */
/*  shall remain the proprietary information of NXP and is                    */
/*  highly confidential in nature. Any and all use hereof is restricted       */
/*  and is subject to the terms and conditions set forth in the               */
/*  software license agreement concluded with NXP B.V.                        */
/*                                                                            */
/*  Under no circumstances is this software or any derivative thereof         */
/*  to be combined with any Open Source Software, exposed to, or in any       */
/*  way licensed under any Open License Terms without the express prior       */
/*  written permission of the copyright owner.                                */
/*                                                                            */
/*  For the purpose of the above, the term Open Source Software means         */
/*  any software that is licensed under Open License Terms. Open              */
/*  License Terms means terms in any license that require as a                */
/*  condition of use, modification and/or distribution of a work              */
/*                                                                            */
/*  1. the making available of source code or other materials                 */
/*     preferred for modification, or                                         */
/*                                                                            */
/*  2. the granting of permission for creating derivative                     */
/*     works, or                                                              */
/*                                                                            */
/*  3. the reproduction of certain notices or license terms                   */
/*     in derivative works or accompanying documentation, or                  */
/*                                                                            */
/*  4. the granting of a royalty-free license to any party                    */
/*     under Intellectual Property Rights                                     */
/*                                                                            */
/*  regarding the work and/or any work that contains, is combined with,       */
/*  requires or otherwise is based on the work.                               */
/*                                                                            */
/*  This software is provided for ease of recompilation only.                 */
/*  Modification and reverse engineering of this software are strictly        */
/*  prohibited.                                                               */
/*                                                                            */
/******************************************************************************/

#ifndef _DALI_MASTER_H
#define _DALI_MASTER_H

#define TURE		1
#define FALSE		0
#define BufferSize	255

#include "stdbool.h"
#include "stdint.h"



/***********************************************************/
/* Type definitions and defines                            */
/***********************************************************/

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


#define K1_OFF()		{LPC_GPIO3->DATA &= ~(1<<5);}
#define K2_OFF()		{LPC_GPIO2->DATA &= ~(1<<1);}
#define K3_OFF()		{LPC_GPIO3->DATA &= ~(1<<4);}
#define K1_ON()			{LPC_GPIO2->DATA &= ~(1<<5);}
#define K2_ON()			{LPC_GPIO0->DATA &= ~(1<<7);}
#define K3_ON()			{LPC_GPIO1->DATA &= ~(1<<9);}


#ifdef release
	#define LED_on()             { LPC_GPIO3->DATA &= ~(1<<1); WDTFeed();}		//
	#define LED_off()            { LPC_GPIO3->DATA |=  (1<<1); WDTFeed();}		// 看门狗喂狗程序，用于发布版本
    #define LED_Toggle()         { LPC_GPIO3->DATA ^=  (1<<1); WDTFeed();}
#else
	#define LED_on()             { LPC_GPIO3->DATA &= ~(1<<1); }
	#define LED_off()            { LPC_GPIO3->DATA |=  (1<<1); }					//未加看门狗，用于调试
    #define LED_Toggle()         { LPC_GPIO3->DATA ^=  (1<<1); }
#endif

#define release

typedef enum answerTypeTag
{
    ANSWER_NOT_AVAILABLE = 0,
    ANSWER_NOTHING_RECEIVED,
    ANSWER_GOT_DATA,
    ANSWER_INVALID_DATA
} answer_t;


/* state machine related definitions */
typedef enum stateTag
{
    MS_IDLE = 0,                        // bus idle
    MS_TX_SECOND_HALF_START_BIT,        // 
    MS_TX_DALI_FORWARD_FRAME,           // sending the dali forward frame
    MS_TX_STOP_BITS,                    //
    MS_SETTLING_BEFORE_BACKWARD,        // settling between forward and backward - stop bits
    MS_SETTLING_BEFORE_IDLE,            // settling before going to idle, after forward frame
    MS_WAITING_FOR_SLAVE_START_WINDOW,  // waiting for 7Te, start of slave Tx window
    MS_WAITING_FOR_SLAVE_START,         // start of slave Tx window
    MS_RECEIVING_ANSWER                 // receiving slave message
} MASTER_STATE;

/* definition of the captured edge data */
typedef struct capturedDataType_tag
{
    uint16_t  capturedTime;             // time stamp of signal edge
    uint8_t   bitLevel;                 // bit level *after* the edge
    uint8_t   levelType;                // indication of long or short duration *after* the edge
} capturedDataType;

typedef struct capturedFrameType_tag
{
    capturedDataType  capturedData[18]; // max 18 edges per backward frame
    uint8_t           capturedItems;    // counter of the captured edges

} capturedFrameType;


extern uint8_t BlockOption;
extern volatile bool         usbForwardFrameReceived;
extern volatile uint16_t     usbForwardFrame;
extern volatile uint8_t      usbBackwardFrame; // DALI slave answer
extern volatile answer_t     usbBackwardFrameAnswer;
extern volatile MASTER_STATE masterState;
extern volatile bool         waitForAnswer;
extern volatile uint32_t     daliForwardFrame; // converted DALI master command
extern volatile capturedFrameType     capturedFrame;    // data structure for the capture

/*********************************************************************************************************
定义UART收发环形缓冲区结构
*********************************************************************************************************/
typedef struct Dali_CIRBUF
{
	/* buffer for reception */
	uint16_t read_index, save_index;
	uint16_t buffer[BufferSize];
}CirBufType;


typedef struct channelDataType_tag
{
    uint8_t		dali_TX_flag;		//DALI通道发送标志位
	uint8_t		GoalLevel;          //通道需要变化到的目标电平    
	uint8_t		OringalLevel;		//通道变化前的电平
	uint8_t 	PresetOffset;       //场景偏移量          
    
    uint8_t		PrevDALILevel;		//先前的DALI电平值
	uint8_t 	flag_bit;  			//bit0:重复（1yes，0no）
	uint8_t		LogicChannel;		//逻辑通道号
	uint8_t 	PrePreset;			//先前场景
    
	uint8_t		MaxLevel;			//最大电平值
	uint8_t		AreaLink;			//区域连接值
	uint8_t		SwitchLevel;		//开关电平值
	uint8_t 	ShortAddr;			//短地址
    
	uint8_t 	Area;				//所属区
	uint8_t		AppendArea;			//附加区
	uint8_t	 	DALI_Channel;		//所属的dali通道
	uint16_t 	dali_data;			//需要发送的dali数据
	uint16_t	PastedTime;			//已经经过的渐变时间
	uint16_t	timeToGoal;	        //总共需要渐变的时间
} channelDataType;

extern const uint8_t LDS_Power[];

extern CirBufType DaliCirBuffer;
extern CirBufType DaliCirBuffer_2nd;
extern CirBufType DaliCirBuffer_3rd;

void DelayMS(uint32_t DelayTime);
uint16_t AddressingLow8bit(uint32_t LongAddress);
uint16_t AddressingMiddle8bit(uint32_t LongAddress);
uint16_t AddressingHigh8bit(uint32_t LongAddress);
bool ReadBuffer(CirBufType *DaliCirBuffer,uint16_t* data);
uint32_t DALI_ConvertForwardFrame(uint16_t forwardFrame) ;
bool DALI_CheckWaitForAnswer(uint16_t forwardFrame)	   ;
bool DALI_CheckRepeatCmd(uint16_t forwardFrame)		 ;
uint8_t CheckSum(uint8_t *buffer,uint8_t Num);
extern void For_Decode(void);
extern bool WriteBuffer(CirBufType *DaliCirBuffer,uint16_t data);
extern void DALI_Send(uint16_t forwardFrame);
extern void Enumerate_DALI_Ballast(void);
extern void DALI_Thread(void);
extern void Set_Ballast_Param(uint8_t DALI_Channel);
#endif /* _DALI_MASTER_H */

/* EOF */
