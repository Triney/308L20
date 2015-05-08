#ifndef _PAN_DALI_CMD_H_
#define	_PAN_DALI_CMD_H_

#include "dali_master.h"

#include "main.h"
#include "i2c1.h"
#include "uart.h"
#include "timer32.h"
#include "gpio.h"
#include "wdt.h"
#include "dali_master_2nd.h"
#include "dali_master_3rd.h"


#ifndef NULL_PTR
    #define NULL_PTR    (0)
#endif

#define DeviceCode      0xEC
#define VersionHi 		0x01
#define VersionLo 	 	0x04

#define Preset_addr				0xf29
#define DALI_Data_base_addr		0x6629
#define Area_addr				0x6929
#define Area_append_addr 		0x69E9
#define Logic_Channel_addr 		0x6AA9
#define Max_level_addr			0x6B69
#define Switch_level_addr 		0x6E69
#define Switch_flag_addr		0x7229
#define Duplicate_addr			0x7241
#define AreaLink_addr			0x6f29	

enum Dim_ins
{
    Preset                  = 0xfe,
    Fade_area_to_off        = 0xfd,
    Program_level_to_preset = 0xfc,
    Reset_to_preset         = 0xfb,
    Configture_DMX          = 0xfa,
    Set_jion                = 0xf9,
    Panel_disable           = 0xf8,
    Panel_enable            = 0xf7,
    Panic                   = 0xf6,
    Un_panic                = 0xf5,
    PE_Speed                = 0xf4,
    Suspend_PE              = 0xf3,
    Restart_PE              = 0xf2,
    Set_air_link            = 0xf1,
    Clear_air_link          = 0xf0,
    Request_area_link       = 0xef,
    Relply_area_link        = 0xee,
    Suspend_motion          = 0xed,
    Resume_motion           = 0xec,
    Disable_motion_detector = 0xeb,
    Disable_motion_detector_this_preset = 0xea,
    Enable_motion_detector_current_preset = 0xe9,
    Disable_motion_detector_all_preset = 0xe8,
    Enable_motion_detector_all_preset  = 0xe7,
    Set_rmask               = 0xe6,
    Fade_channel_area_to_level = 0xe5,
    Reply_with_channel_levle= 0xe4,
    Request_channel_level   = 0xe3,
    Reply_current_preset    = 0xe2,
    Request_current_preset  = 0xe1,
    Preset_offset_and_bank  = 0xe0,
    Select_current_preset   = 0xdf,
    Save_current_preset     = 0xde,
    Restore_saved_preset    = 0xdd,
    Fade_to_off             = 0xdc,
    Fade_to_on              = 0xdb,
    Stop_fade               = 0xda,
    Fade_channel_to_preset  = 0xd9,
    Stop_fade_set_toggle_preset = 0xd8,
    Stop_fade_set_currten_preset = 0xd7,
    Fade_channel_to_toggle_preset = 0xd6,
    Fade_channel_to_current_preset = 0xd5,
    Toggle_Channel          = 0xd4,
    Fade_channel_level_0_1_sec = 0xd3,
    Fade_channel_level_1_sec= 0xd2,
    Fade_channel_level_1_min= 0xd1,
    Inc_level               = 0xd0,
    Dec_level               = 0xcf,
    Fade_area               = 0xce,
    Stop_fade_area          = 0xcd,
};
typedef uint8_t Dim_ins_enum8; 

typedef void (*pFuncCmd) (uint8_t *UARTBuffer,uint8_t *PresetValue,	uint8_t *ALvalue);

typedef struct
{
    Dim_ins_enum8   ins;
    pFuncCmd        pFuncDim;
}Dim_ins_stru;


enum Commissioning_ins
{
    Device_Identify                 = 0xfe,
    Setup                           = 0xfd,
    Reboot_Device                   = 0xfc,
    Read_EEPROM                     = 0xfb,
    Write_EEPROM                    = 0xfa,
    Echo_EEPROM                     = 0xf9,
    Vrms_Request                    = 0xf8,
    Phase_A_Vrms                    = 0xf7,
    CALIBRATE_Device_Phase_to_Vrms  = 0xf6,
    Light_Level_Request             = 0xf5,
    Light_Level_of_Device_Reply     = 0xf4,
    DMX_Port_Control                = 0xf3,
    DMX_Port_Scene_Capture          = 0xf2,     
    Start_Task                      = 0xf1,
    Stop_Task                       = 0xf0,
    Suspend_Task                    = 0xef,
    Resume_Task                     = 0xee,
    Enable_Event                    = 0xed,
    Disable_Event                   = 0xec,
    Trigger_Event                   = 0xeb,
    Flash_Read                      = 0xea,
    Flash_Read_Ack                  = 0xe9,
    Flash_Program_Enable            = 0xe8,
    Flash_Prog_Enable_Ack           = 0xe7,
    Flash_Write                     = 0xe6,
    Flash_Write_Ack                 = 0xe5,
    Flash_Erase                     = 0xe4,
    Flash_Erase_Ack                 = 0xe3,
    Read_RAM                        = 0xe2,
    Write_RAM                       = 0xe1,
    Echo_RAM                        = 0xe0,
    BLOCK_EE_Read_Enable            = 0xdf,
    BLOCK_EE_Read_Ack               = 0xde,
    BLOCK_EE_Write_Enable           = 0xdd,
    BLOCK_EE_Write_Ack              = 0xdc,
    BLOCK_Flash_Read_Enable         = 0xdb,
    BLOCK_Flash_Read_Ack            = 0xda,
    BLOCK_Flash_Write_Enable        = 0xd9,
    BLOCK_Flash_Write_Ack           = 0xd8,
    Request_Device_Status           = 0xd7,
    Report_Device_Status            = 0xd6,
    Request_Channel_Status          = 0xcb,
    Request_Physical_Channel_Level  = 0xb8,
    Set_Physical_Channel_Level      = 0xb7,
    DM320_dali_cmd                  = 0x6f
};

typedef uint8_t Commissioning_ins_enum8;

typedef void (*pCommissioningIns)(uint8_t *pBuffer, uint8_t *PresetValue);

typedef struct
{
    Commissioning_ins_enum8     ins;
    pCommissioningIns           pFuncCommissioning;
}CommissioningIns_stru;

extern uint8_t UARTBuffer[20];
extern uint8_t Box;
extern uint8_t xcode;
	
extern channelDataType	ChannelData[192];
    
void ProcessCMD(void);
extern uint8_t IsAreaAccept(uint8_t indx);	
extern uint8_t IsAppendAreaAccept(uint8_t indx);

#endif

