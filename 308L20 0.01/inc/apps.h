/******************************************************************************

                  版权所有 (C), 2001-2016, Enix Jin

 ******************************************************************************
  文 件 名   : apps.h
  版 本 号   : 初稿
  作    者   : Enix
  生成日期   : 2015年5月9日
  最近修改   :
  功能描述   : apps.c 的头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2015年5月9日
    作    者   : Enix
    修改内容   : 创建文件

******************************************************************************/


#ifndef __APPS_H__
#define __APPS_H__
#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */
/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/

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

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 枚举定义                                     *
 *----------------------------------------------*/

enum RELAY_STATE_ENUM
{
    Relay_State_off   = 0,    
    Relay_State_on    = 1        
}RELAY_STATE_ENUM8;

typedef uint8_t RELAY_STATE_ENUM8;
/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

#define CHANNELNUMS             12
#define RELAY_TRIGON_BEGIN      0
#define RELAY_TRIGON_END        1
#define RELAY_TRIGOFF_BEGIN     2
#define RELAY_TRIGOFF_END       3



/*----------------------------------------------*
 * 结构体定义                                   *
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
