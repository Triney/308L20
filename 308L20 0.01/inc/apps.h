/******************************************************************************

                  ��Ȩ���� (C), 2001-2016, Enix Jin

 ******************************************************************************
  �� �� ��   : apps.h
  �� �� ��   : ����
  ��    ��   : Enix
  ��������   : 2015��5��9��
  ����޸�   :
  ��������   : apps.c ��ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2015��5��9��
    ��    ��   : Enix
    �޸�����   : �����ļ�

******************************************************************************/


#ifndef __APPS_H__
#define __APPS_H__
#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */
/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/

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

/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ö�ٶ���                                     *
 *----------------------------------------------*/

enum RELAY_STATE_ENUM
{
    Relay_State_off   = 0,    
    Relay_State_on    = 1        
}RELAY_STATE_ENUM8;

typedef uint8_t RELAY_STATE_ENUM8;
/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/

#define CHANNELNUMS             12
#define RELAY_TRIGON_BEGIN      0
#define RELAY_TRIGON_END        1
#define RELAY_TRIGOFF_BEGIN     2
#define RELAY_TRIGOFF_END       3



/*----------------------------------------------*
 * �ṹ�嶨��                                   *
 *----------------------------------------------*/
typedef struct
{
    uint8_t         ucGPIO_Port;
    uint8_t         ucGPIO_Pos;
}Relay_GPIO_POS_STRU;

typedef struct
{
    Relay_GPIO_POS_STRU                 stRelayOn;
    Relay_GPIO_POS_STRU                 stRelayOff;
}Relay_Ctrl_STRU;

extern Relay_Ctrl_STRU                  stRelayCtrl[CHANNELNUMS];

extern void DALI_Thread(void);
extern void CheckRelayThread( void );
extern void RelayActFinish(void);
extern void RelayAct( uint8_t ucConfig, uint8_t ucStartAddr);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __APPS_H__ */
