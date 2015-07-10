/******************************************************************************

                  版权所有 (C), 2001-2016, Enix Jin

 ******************************************************************************
  文 件 名   : apps.c
  版 本 号   : 初稿
  作    者   : Enix
  生成日期   : 2015年5月8日
  最近修改   :
  功能描述   : 开关量模块功能主函数
  函数列表   :
  修改历史   :
  1.日    期   : 2015年5月8日
    作    者   : Enix
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "lpc11xx.h"
#include "PanDali_cmd.h"
#include "Apps.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/

channelDataType	            ChannelData[CHANNELNUMS];       /* 继电器对应的通道属性 */
             
const Relay_Ctrl_STRU       stRelayCtrl[CHANNELNUMS]=       /* 控制继电器动作的IO口 */
{
    ((3,0),     (1,2)),
    ((1,1),     (1,0)),
    ((0,11),    (2,11)),
    ((1,10),    (0,9)),
    ((0,11),    (2,11)),
    ((0,8),     (2,2)),
    ((2,10),    (2,9)),
    ((0,7),     (0,6)),
    ((3,5),     (2,5)),
    ((2,4),     (3,4)),
    ((0,3),     (2,1)),
    ((2,8),     (2,7)),
    ((0,2),     (1,8)),    
};

uint32_t                    ulFlagCurrentRelayState = 0;    /* 当前继电器的状态，bit0代表继电器1状态（ON =
                                                               1，OFF = 0） */
uint32_t                    ulFlagExpectRelayState  = 0;    /* 当前继电器的状态，bit0代表继电器1状态（ON =
                                                               1，OFF = 0） */
uint8_t                     ucCheckRelayNumber;
uint8_t                     ucImmediateActRelayNumber;                                                              
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/
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
/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

uint32_t* GetExpRelayStateAddr( void)
{
    return &ulFlagExpectRelayState;
}

/*****************************************************************************
 函 数 名  : SetRelayState
 功能描述  : 设置回路状态的全局变量
 输入参数  : uint8_t ucRelayNum
             RELAY_STATE_ENUM8 enState
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年7月3日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void SetExpRelayState( uint8_t ucRelayNum, RELAY_STATE_ENUM8 enState )
{
    uint32_t *pucExpState;

    /* 参数检查 */
    /* 需要设置的回路大于最大回路数，不设置全局变量，直接返回 */
    if  ((0 == ucRelayNum)
        ||(CHANNELNUMS < ucRelayNum))
    {
        return;
    }

    /* 设置状态有误，直接返回 */
    if ( Relay_State_on < enState)   
    {
        return;
    }
    
    pucExpState = GetExpRelayStateAddr();
    
    switch ( enState)
    {
        case Relay_State_on :
            *pucExpState |= 1<<(ucRelayNum - 1);
            break;
        case Relay_State_off :
            *pucExpState &= ~(1<<(ucRelayNum - 1));
            break;
        default:
            return;
    }
    
    return;
}

/*****************************************************************************
 函 数 名  : RelayAct
 功能描述  : 控制自恢复继电器动作函数
 输入参数  : uint8_t ucConfig       
                RELAY_TRIGG_BEGIN   继电器触发
                
             uint8_t ucStartAddr    第几个回路动作
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月30日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void RelayAct( uint8_t ucConfig, uint8_t ucStartAddr)
{
    /* 参数检查 */
    /* 继电器动作状态不正确 */
    if (RELAY_TRIGOFF_END < ucConfig)
    {
        return;
    }
    /* 超出最大回路数 */
    if ( CHANNELNUMS < ucStartAddr )
    {
        return;
    }

    /* 根据继电器动作状态设置IO口状态 */
    switch ( ucConfig )
    {
        case RELAY_TRIGON_BEGIN :
            GPIOSetValue((uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOn.ucGPIO_Port,
                     (uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOn.ucGPIO_Pos, 0);
            break;
        case RELAY_TRIGON_END :
            GPIOSetValue((uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOn.ucGPIO_Port,
                     (uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOn.ucGPIO_Pos, 1);
            break;
        case RELAY_TRIGOFF_BEGIN :
            GPIOSetValue((uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOff.ucGPIO_Port,
                     (uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOff.ucGPIO_Pos, 0);
            break;
        case RELAY_TRIGOFF_END :
            GPIOSetValue((uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOff.ucGPIO_Port,
                     (uint32_t) stRelayCtrl[ucStartAddr-1].stRelayOff.ucGPIO_Pos, 0);
            break;
        default:
            break;
    }
  

}

/*****************************************************************************
 函 数 名  : RelayActFinish
 功能描述  : 继电器触发电平时间到，IO口恢复触发前状态
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年7月2日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void RelayActFinish(void)
{
    uint8_t i;

    for(i = 1;i <= CHANNELNUMS ; i++ )
    {
        RelayAct(RELAY_TRIGON_END, i);
        RelayAct(RELAY_TRIGOFF_END, i);
    }
    
    return;
}


/*****************************************************************************
 函 数 名  : CheckRelayThread
 功能描述  : 循环检查继电器是否需要动作的函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年6月20日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void CheckRelayThread( void )
{
    uint16_t     ucCurrentLuminanceLevel;
    /* 检查目标亮度是否比最大亮度高 */
    if(ChannelData[ucCheckRelayNumber].GoalLevel >ChannelData[ucCheckRelayNumber].MaxLevel)
    {
        /* 如果目标亮度比最大亮度高，则目标亮度修改为最大亮度 */
        ChannelData[ucCheckRelayNumber].GoalLevel = ChannelData[ucCheckRelayNumber].MaxLevel;
    }

    /* 判断继电器是否需要动作 */
    if(ChannelData[ucCheckRelayNumber].timeToGoal!=ChannelData[ucCheckRelayNumber].PastedTime)
    {
        /* 判断是打开继电器还是关闭继电器 */
        if (ChannelData[ucCheckRelayNumber].GoalLevel > ChannelData[ucCheckRelayNumber].OringalLevel)
        {
            /* 计算当前亮度值 */
            ChannelData[ucCheckRelayNumber].CurrentLevel \
                = (uint8_t) ( ChannelData[ucCheckRelayNumber].OringalLevel     \
                             + (((ChannelData[ucCheckRelayNumber].GoalLevel     \
                                 -ChannelData[ucCheckRelayNumber].OringalLevel)\
                               *ChannelData[ucCheckRelayNumber].PastedTime + 1)) \
                                / ChannelData[ucCheckRelayNumber].timeToGoal);

            /* 关闭回路动作 */
            /* 是否到达动作亮度（开关电平） */
            if (ChannelData[ucCheckRelayNumber].CurrentLevel == ChannelData[ucCheckRelayNumber].SwitchLevel)
            {
                /* 开启继电器动作 */
                RelayAct(RELAY_TRIGOFF_BEGIN, ucCheckRelayNumber);
                /* 置标志位 */
                ChannelData[ucCheckRelayNumber].flag_bit |= FlagBitTrigOffBegin;
                ChannelData[ucCheckRelayNumber].flag_bit &= ~FlagBitRelayStateOn;
            }    
        }
        else if (ChannelData[ucCheckRelayNumber].GoalLevel < ChannelData[ucCheckRelayNumber].OringalLevel)
        {
            /* 计算当前亮度值 */
            ChannelData[ucCheckRelayNumber].CurrentLevel \
                = (uint8_t) ( ChannelData[ucCheckRelayNumber].OringalLevel     \
                             - (((ChannelData[ucCheckRelayNumber].GoalLevel     \
                                 -ChannelData[ucCheckRelayNumber].OringalLevel)\
                               *ChannelData[ucCheckRelayNumber].PastedTime + 1)) \
                                / ChannelData[ucCheckRelayNumber].timeToGoal);
            /* 打开回路动作 */
            /* 是否到达动作亮度（开关电平） */
            if (ChannelData[ucCheckRelayNumber].CurrentLevel == ChannelData[ucCheckRelayNumber].SwitchLevel)
            {
                /* 开启继电器动作 */
                RelayAct(RELAY_TRIGON_BEGIN, ucCheckRelayNumber);
                ChannelData[ucCheckRelayNumber].flag_bit |= (FlagBitRelayStateOn | FlagBitTrigOnBegin);
            } 
        }
    }
    else
    {
        ChannelData[ucCheckRelayNumber].OringalLevel=ChannelData[ucCheckRelayNumber].GoalLevel;
		ChannelData[ucCheckRelayNumber].PastedTime=1;
		ChannelData[ucCheckRelayNumber].timeToGoal=1;
    }

    /* 回路动作检查成功，下次检查下一回路 */
    if (CHANNELNUMS < ucCheckRelayNumber )
    {
        ucCheckRelayNumber = 1;
    }
    else
    {
        ucCheckRelayNumber++;
    }

    /* 打开100ms定时器，继电器IO口恢复，具体看自保持继电器动作说明 */
        
    return;
}

/*****************************************************************************
 函 数 名  : Main_Thread
 功能描述  : 主线程，检查定时器超时，处理RS485命令
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年5月30日
    作    者   : Enix
    修改内容   : 新生成函数

*****************************************************************************/
void Main_Thread(void)
{
	uint8_t nop_i ;
	uint32_t aux_value;   

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
    }
}

