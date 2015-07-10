/******************************************************************************

                  ��Ȩ���� (C), 2001-2016, Enix Jin

 ******************************************************************************
  �� �� ��   : apps.c
  �� �� ��   : ����
  ��    ��   : Enix
  ��������   : 2015��5��8��
  ����޸�   :
  ��������   : ������ģ�鹦��������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2015��5��8��
    ��    ��   : Enix
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "lpc11xx.h"
#include "PanDali_cmd.h"
#include "Apps.h"
/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/

channelDataType	            ChannelData[CHANNELNUMS];       /* �̵�����Ӧ��ͨ������ */
             
const Relay_Ctrl_STRU       stRelayCtrl[CHANNELNUMS]=       /* ���Ƽ̵���������IO�� */
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

uint32_t                    ulFlagCurrentRelayState = 0;    /* ��ǰ�̵�����״̬��bit0����̵���1״̬��ON =
                                                               1��OFF = 0�� */
uint32_t                    ulFlagExpectRelayState  = 0;    /* ��ǰ�̵�����״̬��bit0����̵���1״̬��ON =
                                                               1��OFF = 0�� */
uint8_t                     ucCheckRelayNumber;
uint8_t                     ucImmediateActRelayNumber;                                                              
/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ��������                                     *
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
 * �궨��                                       *
 *----------------------------------------------*/

uint32_t* GetExpRelayStateAddr( void)
{
    return &ulFlagExpectRelayState;
}

/*****************************************************************************
 �� �� ��  : SetRelayState
 ��������  : ���û�·״̬��ȫ�ֱ���
 �������  : uint8_t ucRelayNum
             RELAY_STATE_ENUM8 enState
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��7��3��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void SetExpRelayState( uint8_t ucRelayNum, RELAY_STATE_ENUM8 enState )
{
    uint32_t *pucExpState;

    /* ������� */
    /* ��Ҫ���õĻ�·��������·����������ȫ�ֱ�����ֱ�ӷ��� */
    if  ((0 == ucRelayNum)
        ||(CHANNELNUMS < ucRelayNum))
    {
        return;
    }

    /* ����״̬����ֱ�ӷ��� */
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
 �� �� ��  : RelayAct
 ��������  : �����Իָ��̵�����������
 �������  : uint8_t ucConfig       
                RELAY_TRIGG_BEGIN   �̵�������
                
             uint8_t ucStartAddr    �ڼ�����·����
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��30��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void RelayAct( uint8_t ucConfig, uint8_t ucStartAddr)
{
    /* ������� */
    /* �̵�������״̬����ȷ */
    if (RELAY_TRIGOFF_END < ucConfig)
    {
        return;
    }
    /* ��������·�� */
    if ( CHANNELNUMS < ucStartAddr )
    {
        return;
    }

    /* ���ݼ̵�������״̬����IO��״̬ */
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
 �� �� ��  : RelayActFinish
 ��������  : �̵���������ƽʱ�䵽��IO�ڻָ�����ǰ״̬
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��7��2��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

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
 �� �� ��  : CheckRelayThread
 ��������  : ѭ�����̵����Ƿ���Ҫ�����ĺ���
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��6��20��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

*****************************************************************************/
void CheckRelayThread( void )
{
    uint16_t     ucCurrentLuminanceLevel;
    /* ���Ŀ�������Ƿ��������ȸ� */
    if(ChannelData[ucCheckRelayNumber].GoalLevel >ChannelData[ucCheckRelayNumber].MaxLevel)
    {
        /* ���Ŀ�����ȱ�������ȸߣ���Ŀ�������޸�Ϊ������� */
        ChannelData[ucCheckRelayNumber].GoalLevel = ChannelData[ucCheckRelayNumber].MaxLevel;
    }

    /* �жϼ̵����Ƿ���Ҫ���� */
    if(ChannelData[ucCheckRelayNumber].timeToGoal!=ChannelData[ucCheckRelayNumber].PastedTime)
    {
        /* �ж��Ǵ򿪼̵������ǹرռ̵��� */
        if (ChannelData[ucCheckRelayNumber].GoalLevel > ChannelData[ucCheckRelayNumber].OringalLevel)
        {
            /* ���㵱ǰ����ֵ */
            ChannelData[ucCheckRelayNumber].CurrentLevel \
                = (uint8_t) ( ChannelData[ucCheckRelayNumber].OringalLevel     \
                             + (((ChannelData[ucCheckRelayNumber].GoalLevel     \
                                 -ChannelData[ucCheckRelayNumber].OringalLevel)\
                               *ChannelData[ucCheckRelayNumber].PastedTime + 1)) \
                                / ChannelData[ucCheckRelayNumber].timeToGoal);

            /* �رջ�·���� */
            /* �Ƿ񵽴ﶯ�����ȣ����ص�ƽ�� */
            if (ChannelData[ucCheckRelayNumber].CurrentLevel == ChannelData[ucCheckRelayNumber].SwitchLevel)
            {
                /* �����̵������� */
                RelayAct(RELAY_TRIGOFF_BEGIN, ucCheckRelayNumber);
                /* �ñ�־λ */
                ChannelData[ucCheckRelayNumber].flag_bit |= FlagBitTrigOffBegin;
                ChannelData[ucCheckRelayNumber].flag_bit &= ~FlagBitRelayStateOn;
            }    
        }
        else if (ChannelData[ucCheckRelayNumber].GoalLevel < ChannelData[ucCheckRelayNumber].OringalLevel)
        {
            /* ���㵱ǰ����ֵ */
            ChannelData[ucCheckRelayNumber].CurrentLevel \
                = (uint8_t) ( ChannelData[ucCheckRelayNumber].OringalLevel     \
                             - (((ChannelData[ucCheckRelayNumber].GoalLevel     \
                                 -ChannelData[ucCheckRelayNumber].OringalLevel)\
                               *ChannelData[ucCheckRelayNumber].PastedTime + 1)) \
                                / ChannelData[ucCheckRelayNumber].timeToGoal);
            /* �򿪻�·���� */
            /* �Ƿ񵽴ﶯ�����ȣ����ص�ƽ�� */
            if (ChannelData[ucCheckRelayNumber].CurrentLevel == ChannelData[ucCheckRelayNumber].SwitchLevel)
            {
                /* �����̵������� */
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

    /* ��·�������ɹ����´μ����һ��· */
    if (CHANNELNUMS < ucCheckRelayNumber )
    {
        ucCheckRelayNumber = 1;
    }
    else
    {
        ucCheckRelayNumber++;
    }

    /* ��100ms��ʱ�����̵���IO�ڻָ������忴�Ա��̵ּ�������˵�� */
        
    return;
}

/*****************************************************************************
 �� �� ��  : Main_Thread
 ��������  : ���̣߳���鶨ʱ����ʱ������RS485����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��5��30��
    ��    ��   : Enix
    �޸�����   : �����ɺ���

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

