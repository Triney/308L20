#include "PanDali_cmd.h"


void DimIns_Preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    
    if((NULL_PTR == UARTBuffer)
        ||(NULL_PTR == PresetValue)
        ||(NULL_PTR == ALvalue))
        return;
    
    for(indx=0;indx<192;indx++)											   //寻找场景偏移量
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                UARTBuffer[4] = UARTBuffer[4]+ChannelData[indx].PresetOffset;
                break;
            }
        }							
    }

    if(UARTBuffer[4]>95)
    {
        return;
    }
    
    if(m24xx_read(EEPROM_24XX256,Preset_addr+UARTBuffer[4]*192,0,PresetValue,192) != I2C_NO_ERR)
    {
		return;        
    }
    
    dimTime =(uint32_t) (((UARTBuffer[6]<<8)|UARTBuffer[5])+2);	//调光时间，借用dimTime
    
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }
    
    for(indx=0;indx<192;indx++)
    {
        if(PresetValue[indx]!=0)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    if((ChannelData[indx].OringalLevel!=0)&&(ChannelData[indx].OringalLevel!=PresetValue[indx]))
                    {
                        if(ChannelData[indx].timeToGoal!=1)
                            ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                        ChannelData[indx].GoalLevel = PresetValue[indx];
                        ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                        ChannelData[indx].PrePreset = UARTBuffer[4];
                        ChannelData[indx].PastedTime = 1;
                    }
                }
             }													
        }
        PresetValue[indx] = ChannelData[indx].GoalLevel;
    }
    
    m24xx_write(EEPROM_24XX256,Preset_addr+(xcode+2)*192,0,PresetValue,192);
    
    for(indx=0;indx<192;indx++)
    {
        PresetValue[indx]=ChannelData[indx].PrePreset;
    }
    
    m24xx_write(EEPROM_24XX256,0X6c29,0,PresetValue,192);    
    return;
}


void DimIns_Fade_area_to_off(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint8_t AreaLink[4];
    uint32_t dimTime;
    for(indx=0;indx<192;indx++)											   //寻找场景偏移量
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                UARTBuffer[4] = UARTBuffer[4]+ChannelData[indx].PresetOffset;
                break;
            }
        }							
    }
        
    if(UARTBuffer[4]>95)
    {
        return;
    }
    
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].OringalLevel!=0)&&(ChannelData[indx].OringalLevel!=0xff))
                {
                    dimTime = (uint32_t) (((UARTBuffer[6]<<8)|UARTBuffer[5])+2);	//调光时间
                    if(dimTime>0xffff)
                    {
                        dimTime=0xffff;
                    }
                    if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = 0xff;
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;
                }
            }
         }
         else
         {
            if((UARTBuffer[1] >= ChannelData[indx].AreaLink)
                &&(UARTBuffer[1] <= (ChannelData[indx].AreaLink+24))
                &&(ChannelData[indx].AreaLink != 0xff))
            {
                if(m24xx_read(EEPROM_24XX256,AreaLink_addr+indx*4,0,AreaLink,4)==I2C_NO_ERR)
                {
                    dimTime = (uint32_t) ((AreaLink[1]<<16)|(AreaLink[2]<<8)|(AreaLink[3]));
                    if(
                        (dimTime>>(24-(UARTBuffer[1]-ChannelData[indx].AreaLink)))==0x01 
                      )
                    {
                        dimTime = (uint32_t) (((UARTBuffer[6]<<8)|UARTBuffer[5])+2);	//调光时间
                        
                        if(dimTime>0xffff)
                        {
                            dimTime=0xffff;
                        }
                        
                        if(ChannelData[indx].timeToGoal!=1)
                        {
                            ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                        }
                        ChannelData[indx].GoalLevel = 0xff;
                        ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                        ChannelData[indx].PastedTime = 1;	

                        }

                }
            }																	
        }
    }
    return;
}


void DimIns_Program_level_to_preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    if(m24xx_read(EEPROM_24XX256,Preset_addr+UARTBuffer[4]*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
                                        {
                if(IsAppendAreaAccept(indx))
                {
            
                    PresetValue[indx] = ChannelData[indx].GoalLevel;
                }
            }
        }
    }
    m24xx_write(EEPROM_24XX256,Preset_addr+UARTBuffer[4]*192,0,PresetValue,192);
    return;
}
    
void DimIns_Reset_to_preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    
    dimTime = (uint32_t) (((UARTBuffer[6]<<8)|UARTBuffer[5])+2);	//调光时间
    
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }
    
    if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+2)*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = PresetValue[indx];
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;					
                }
            }
        }							
    }
    return;
}

void DimIns_Panic(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+1)*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = PresetValue[indx];
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;					
                }
            }
        }							
    }    
    return;
}

void DimIns_Un_panic(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    
    dimTime = (uint32_t)(((UARTBuffer[6]<<8)|UARTBuffer[5])+2);	//调光时间
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }
    
    if(m24xx_read(EEPROM_24XX256,Preset_addr+(xcode+2)*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    if(ChannelData[indx].timeToGoal != 1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = PresetValue[indx];
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;					
                }
            }
        }							
    }
    return;
}
void DimIns_Set_air_link(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    if(m24xx_read(EEPROM_24XX256,AreaLink_addr,0,ALvalue,768)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    ALvalue[indx*4]=UARTBuffer[1];
                    ALvalue[indx*4+1]=UARTBuffer[6];														
                    ALvalue[indx*4+2]=UARTBuffer[5];
                    ALvalue[indx*4+3]=UARTBuffer[4];
                }
            }
            else
            {
                ALvalue[indx*4]=0xff;
                ALvalue[indx*4+1]=0;														
                ALvalue[indx*4+2]=0;
                ALvalue[indx*4+3]=0;
            }
        }
        m24xx_write(EEPROM_24XX256,AreaLink_addr,0,ALvalue,768);
    }
    return;
}

void DimIns_Clear_air_link(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
						
    uint8_t indx;
    for(indx=0;indx<192;indx++)
    {
        ALvalue[indx*4]=0xff;
        ALvalue[indx*4+1]=0;
        ALvalue[indx*4+2]=0;
        ALvalue[indx*4+3]=0;
    }
    m24xx_write(EEPROM_24XX256,AreaLink_addr,0,ALvalue,768);
    return;
}

void DimIns_Save_current_preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                UARTBuffer[4] = ChannelData[indx].PrePreset;
                break;
            }
        }
    }
    
    if(m24xx_read(EEPROM_24XX256,Preset_addr+UARTBuffer[4]*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
        
                PresetValue[indx] = ChannelData[indx].GoalLevel;
            }
        }
    }
    m24xx_write(EEPROM_24XX256,Preset_addr+UARTBuffer[4]*192,0,PresetValue,192);
    return;
}

void DimIns_Fade_channel_to_toggle_preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    if(m24xx_read(EEPROM_24XX256,Preset_addr+xcode*192,0,PresetValue,192)==I2C_NO_ERR)
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {
                if(IsAppendAreaAccept(indx))
                {
                    if(ChannelData[indx].timeToGoal != 1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel  = PresetValue[indx];
                    ChannelData[indx].timeToGoal = (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;					
                }
            }
        }							
    }

}

void DimIns_Request_channel_level(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t     indx;
//    uint32_t    dimTime;
    volatile uint8_t      *pucBackwardFrame;
    volatile answer_t     *pbUsbBackwardFrameAnswer;
    volatile MASTER_STATE *penMasterState;
    volatile bool         *pbWaitForAnswer;
    
    void (*pDaliSend)(uint16_t forwardFrame);
    void (*pDecodeDaliData)(void);
    
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {   
            if((ChannelData[indx].LogicChannel==UARTBuffer[5])&&((ChannelData[indx].flag_bit&0)==0x01))
            {
                UARTBuffer[3]=0xe4;
                switch(ChannelData[indx].DALI_Channel&0x03)
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
                
                while(*penMasterState != MS_IDLE)
                    __NOP();
                pDaliSend(ChannelData[UARTBuffer[5]].ShortAddr<<9|0x1A0);
                
                while(*penMasterState != MS_IDLE)
                    __NOP();
                
                if(*pbWaitForAnswer)
                {
                    pDecodeDaliData();
                    if(*pbUsbBackwardFrameAnswer == ANSWER_GOT_DATA)
                    {
                        UARTBuffer[4] = ~LDS_Power[*pucBackwardFrame];
                    }
                    else
                        UARTBuffer[4] = *pucBackwardFrame;
                }
                
                ChannelData[indx].GoalLevel = UARTBuffer[4];
                ChannelData[indx].OringalLevel = UARTBuffer[4];
                UARTBuffer[6] =	 ChannelData[indx].GoalLevel;
                UARTBuffer[7] = CheckSum(UARTBuffer,7);
                UARTSend(UARTBuffer,8);
                break;
            }
        }						
    }
    return;
}

void DimIns_Request_current_preset(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
//    uint32_t dimTime;
     for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {	
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==0)&&((ChannelData[indx].flag_bit&0x01)==0))	 //判断逻辑通道是否为1并且不是重复通道
                {
                    UARTBuffer[3]=0xe2;
                    UARTBuffer[5]=ChannelData[indx].PrePreset;
                    UARTBuffer[6]=0;
                    UARTBuffer[7]=CheckSum(UARTBuffer,7);
                    UARTSend(UARTBuffer,8);
                    break;
                }		
            }
        }
    }
    return;
}

void DimIns_Fade_to_off(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    
    dimTime = UARTBuffer[4]*5+2;	//调光时间
    
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }
    
    if(UARTBuffer[5]==0xff)
    {

        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {	
                if(IsAppendAreaAccept(indx))
                {
                    ChannelData[indx].flag_bit &= 0x7f;
                    if(ChannelData[indx].timeToGoal!=1)
                    ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    if(UARTBuffer[6]>ChannelData[indx].OringalLevel)
                    {
                        ChannelData[indx].GoalLevel=UARTBuffer[6];
                        ChannelData[indx].timeToGoal=(uint16_t) (dimTime*(ChannelData[indx].GoalLevel-ChannelData[indx].OringalLevel)/0xfe);								
                        ChannelData[indx].PastedTime=1;
                        if(ChannelData[indx].timeToGoal<2) 
                            ChannelData[indx].timeToGoal=2;		
                    }
                }
            }
        }	
    }
    else
    {
        for(indx=0;indx<192;indx++)
        {
            if(IsAreaAccept(indx))
            {	
                if(IsAppendAreaAccept(indx))
                {

                    if(ChannelData[indx].LogicChannel==UARTBuffer[5])
                    {	
                        ChannelData[indx].flag_bit &= 0x7f;
                        if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;											
                        if(UARTBuffer[6]>ChannelData[indx].OringalLevel)
                        {
                            ChannelData[indx].GoalLevel=UARTBuffer[6];
                            ChannelData[indx].timeToGoal=(uint16_t) (dimTime*(ChannelData[indx].GoalLevel-ChannelData[indx].OringalLevel)/0xfe);																		
                            ChannelData[indx].PastedTime = 1;
                            if(ChannelData[indx].timeToGoal<2) ChannelData[indx].timeToGoal=2;		
                        }
                        break;
                     }
                }
            }
        }
    }
    
    return;
}

void DimIns_Fade_to_on(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    
    dimTime = UARTBuffer[4]*5+2;	//调光时间，借用dimTime
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }

    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {	
            if(IsAppendAreaAccept(indx))
            {  	
                if((UARTBuffer[5]==0xff)||(ChannelData[indx].LogicChannel == UARTBuffer[5]))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    {
                        ChannelData[indx].flag_bit |= 0x80;
                        ChannelData[indx].GoalLevel=UARTBuffer[6];
                        ChannelData[indx].PastedTime = 1;
                        ChannelData[indx].timeToGoal=(uint16_t) (dimTime*(ChannelData[indx].OringalLevel-ChannelData[indx].GoalLevel)/0xfe);
                        if(ChannelData[indx].timeToGoal<2) 
                            ChannelData[indx].timeToGoal=2;		
                    }
                }
            }	
        }
    }
    
    return;
}

void DimIns_Stop_fade(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
//    uint32_t dimTime;
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
         {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
                {
                    ChannelData[indx].OringalLevel = (ChannelData[indx].OringalLevel+(((ChannelData[indx].GoalLevel&0xff)-(ChannelData[indx].OringalLevel))*(ChannelData[indx].PastedTime+1))/ChannelData[indx].timeToGoal);
                    if(ChannelData[indx].OringalLevel == 0)
                        ChannelData[indx].OringalLevel =1;
                    ChannelData[indx].GoalLevel = ChannelData[indx].OringalLevel;
                    ChannelData[indx].PastedTime = 1;
                    ChannelData[indx].timeToGoal = 1;
                }
            }	
    
        }
    }

}

void DimIns_Fade_channel_level_0_1_sec(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
//    uint32_t dimTime;
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                    ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = UARTBuffer[6];
                    ChannelData[indx].timeToGoal= (UARTBuffer[4]*5)+2;
                    ChannelData[indx].PastedTime = 1;
                }
            }
        }
    }
    
    return;
}

void DimIns_Fade_channel_level_1_sec(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                    ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    dimTime = UARTBuffer[4]*50+2;	//调光时间，借用dimTime
                    ChannelData[indx].GoalLevel = UARTBuffer[6];
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime+2;
                    ChannelData[indx].PastedTime = 1;
                }
            }
        }
    }
    
    return;
}

void DimIns_Fade_channel_level_1_min(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    uint32_t dimTime;
    dimTime = UARTBuffer[4]*3000+2;	//调光时间，借用dimTime
    if(dimTime>0xffff)
    {
        dimTime=0xffff;
    }

    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                    ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    ChannelData[indx].GoalLevel = UARTBuffer[6];
                    ChannelData[indx].timeToGoal= (uint16_t) dimTime;
                    ChannelData[indx].PastedTime = 1;
                }
            }
         }
     }
    
    return;
}

void DimIns_Preset_offset_and_bank(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
    for(indx=0;indx<192;indx++)
    {	
        if(IsAreaAccept(indx))
        {
            if(((UARTBuffer[2]==0xff)) 
            ||( (UARTBuffer[2]==ChannelData[indx].AppendArea)&&((UARTBuffer[2]&0x80)==0x80))
            ||(((UARTBuffer[2]&0x80)!=0x80)&&(((UARTBuffer[2]|ChannelData[indx].AppendArea)!=0)))  )		  //判断附加区
            {
                ChannelData[indx].PresetOffset = UARTBuffer[5];
            }
        }
    }
    Set_Offset();
    return;    
}

void DimIns_Inc_level(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
//    uint32_t dimTime;
    for(indx=0;indx<192;indx++)
    {
    if(IsAreaAccept(indx))
    {
        if(((UARTBuffer[2]==0xff)) 
        ||( (UARTBuffer[2]==ChannelData[indx].AppendArea)&&((UARTBuffer[2]&0x80)==0x80))
        ||(((UARTBuffer[2]&0x80)!=0x80)&&((UARTBuffer[2]|ChannelData[indx].AppendArea)!=0))  )		  //判断附加区
        {
    
            if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
            {
                if(ChannelData[indx].timeToGoal!=1)
                    ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                if(ChannelData[indx].GoalLevel!=0x01)
                {
                    ChannelData[indx].GoalLevel--;
                }
                ChannelData[indx].timeToGoal= UARTBuffer[4]+2;
                ChannelData[indx].PastedTime = 1;
            }
        }
     }
     }    
    return;
}

void DimIns_Dec_level(uint8_t *UARTBuffer, uint8_t *PresetValue, uint8_t *ALvalue)
{
    uint8_t indx;
//    uint32_t dimTime;
    for(indx=0;indx<192;indx++)
    {
        if(IsAreaAccept(indx))
        {
            if(IsAppendAreaAccept(indx))
            {
                if((ChannelData[indx].LogicChannel==UARTBuffer[5])||(UARTBuffer[5]==0xff))
                {
                    if(ChannelData[indx].timeToGoal!=1)
                        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
                    if(ChannelData[indx].GoalLevel!=0xff)
                    {
                        ChannelData[indx].GoalLevel++;
                    }
                    ChannelData[indx].timeToGoal= UARTBuffer[4]+2;
                    ChannelData[indx].PastedTime = 1;
                }
            }
         }
     }    
    return;
}

Dim_ins_stru Dim_Ins[] = 
{
    {Preset                         , DimIns_Preset                         },
    {Fade_area_to_off               , DimIns_Fade_area_to_off               },
    {Program_level_to_preset        , DimIns_Program_level_to_preset        },
    {Reset_to_preset                , DimIns_Reset_to_preset                },
    {Panic                          , DimIns_Panic                          },
    {Un_panic                       , DimIns_Un_panic                       },
    {Set_air_link                   , DimIns_Set_air_link                   },
    {Clear_air_link                 , DimIns_Clear_air_link                 },
    {Save_current_preset            , DimIns_Save_current_preset            },
    {Fade_channel_to_toggle_preset  , DimIns_Fade_channel_to_toggle_preset  },
    {Request_channel_level          , DimIns_Request_channel_level          },
    {Request_current_preset         , DimIns_Request_current_preset         },
    {Fade_to_off                    , DimIns_Fade_to_off                    },
    {Fade_to_on                     , DimIns_Fade_to_on                     },
    {Stop_fade                      , DimIns_Stop_fade                      },
    {Fade_channel_level_0_1_sec     , DimIns_Fade_channel_level_0_1_sec     },
    {Fade_channel_level_1_sec       , DimIns_Fade_channel_level_1_sec       },
    {Fade_channel_level_1_min       , DimIns_Fade_channel_level_1_min       },
    {Preset_offset_and_bank         , DimIns_Preset_offset_and_bank         },
    {Inc_level                      , DimIns_Inc_level                      },
    {Dec_level                      , DimIns_Dec_level                      }
};

uint8_t ucMaxDimIns  = (sizeof(Dim_Ins))/(sizeof(Dim_ins_stru));

void CmmssnIns_Setup(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    uint8_t indx;
        switch(UARTBuffer[4])
    {
        case 0x08:
        {
            K1_ON();
            K2_ON();
            K3_ON();
            DelayMS(20);
            GPIOSetValue(2,1,1);
            GPIOSetValue(1,9,1);
            GPIOSetValue(3,4,1);
            GPIOSetValue(2,5,1);
            GPIOSetValue(3,5,1);
            GPIOSetValue(0,7,1);
            /***************************/
            DALI_Send(0xFE80);
            DALI_2nd_Send(0xFE80);
            DALI_3rd_Send(0xFE80);
            while(masterState_3rd!=MS_IDLE){};
                
            DelayMS(500);
            
            for(indx = 7; indx != 0; indx--)
            {
                /* Flash Led */
                LPC_GPIO3->DATA ^= (1<<1);
            
            #ifdef release
                WDTFeed();
            #endif
                
                DelayMS(500);
            }
#if 0             
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();DelayMS(500);
            if((LPC_GPIO3->DATA&(1<<1))==(1<<1))
            {
                LED_on();
            }				
            else 	LED_off();
            
#endif
            /***************************/	
            Enumerate_DALI_Ballast();
            Set_Ballast_Param(1);
            Enumerate_DALI_Ballast_2nd();
            Set_Ballast_Param(2);
            Enumerate_DALI_Ballast_3rd();
            Set_Ballast_Param(3);
            Get_Channel_Param();

            UARTBuffer[3] = 0xfe;
            UARTBuffer[4] = VersionHi;
            UARTBuffer[5] = VersionLo;
            UARTBuffer[6] = 0x80;
            UARTBuffer[7] = CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
        }
            break;
        case 0x09:
            if(UARTBuffer[5] == 1)
            {
                for(indx=0;indx<192;indx++)
                {
                    PresetValue[indx]=0xff;	
                }
                for(indx=0;indx<4;indx++)
                {
                    m24xx_write(EEPROM_24XX256,0X6629+indx*192,0,PresetValue,192);
                }
            }
            Get_Channel_Param();
            break;
        default:break;

    }
    return;
}

void CmmssnIns_Reboot_Device(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    NVIC_SystemReset();
    return;
}

void CmmssnIns_Read_EEPROM(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    UARTBuffer[0] = 0xfa;
    UARTBuffer[1] = DeviceCode;
    UARTBuffer[2] = Box;
    UARTBuffer[3] = 0xf9;
    m24xx_read(EEPROM_24XX256,(UARTBuffer[4]<<8|UARTBuffer[5]),0,&UARTBuffer[6],1);
    UARTBuffer[7]=CheckSum(UARTBuffer,7);
    UARTSend(UARTBuffer,8);
    return;
}

void CmmssnIns_Write_EEPROM(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    if(I2C_NO_ERR ==  m24xx_write(EEPROM_24XX256,(UARTBuffer[4]<<8)|UARTBuffer[5],0,&UARTBuffer[6],1))
    {
            UARTBuffer[7] = UARTBuffer[7]+1;
    }
    UARTBuffer[3] = 0xf9;
    UARTSend(UARTBuffer,8);
    return;
}

void CmmssnIns_BLOCK_EE_Read_Enable(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    UARTBuffer[3] = 0xde;
    UARTBuffer[7] = UARTBuffer[7]+1;
    UARTSend(UARTBuffer,8);
    BlockOption=1;    
    return;
}

void CmmssnIns_BLOCK_EE_Write_Enable(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    UARTBuffer[3] = 0xdc;
    UARTBuffer[7] = UARTBuffer[7]+1;
    UARTSend(UARTBuffer,8);
    BlockWriteAddr=((UARTBuffer[4]<<8)|UARTBuffer[5]);
    BlockOption=1;
    return;    
}

void CmmssnIns_Request_Channel_Status(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
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
            UARTBuffer[6]=0x00;
            UARTBuffer[7]=CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
            return;                
    }
    
    switch (UARTBuffer[4])
    {
        case 1:
            while(*penMasterState!=MS_IDLE){;};
            pDaliSend((ChannelData[UARTBuffer[5]].ShortAddr<<9)|0x190);
            while(*penMasterState!=MS_IDLE){;};
            pDecodeDaliData();
            UARTBuffer[6] = *pucBackwardFrame;
            UARTBuffer[7] = CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
            break;
        case 2:
            while(*penMasterState!=MS_IDLE){;};
            pDaliSend((ChannelData[UARTBuffer[5]].ShortAddr<<9)|0x191);
            while(*penMasterState!=MS_IDLE){;};
            pDecodeDaliData();
            UARTBuffer[6]= *pucBackwardFrame & 0x01;
            UARTBuffer[7]=CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
            break;
        case 3:
            while(*penMasterState != MS_IDLE){;};
            pDaliSend((ChannelData[UARTBuffer[5]].ShortAddr<<9)|0x193);
            while(*penMasterState != MS_IDLE){;};
            pDecodeDaliData();
            UARTBuffer[6]= *pucBackwardFrame & 0x03;
            UARTBuffer[7]=CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
            break;
        case 4:
            while(*penMasterState != MS_IDLE){;};
            pDaliSend((ChannelData[UARTBuffer[5]].ShortAddr<<9)|0x192);
            while(*penMasterState != MS_IDLE){;};
            pDecodeDaliData();
            UARTBuffer[6]= *pucBackwardFrame & 0x01;
            UARTBuffer[7]=CheckSum(UARTBuffer,7);
            UARTSend(UARTBuffer,8);
            break;            
        default:
            break;
    }    
    return;
}

void CmmssnIns_Request_Physical_Channel_Level(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
     if((UARTBuffer[4]>0)&&(UARTBuffer[4]<193))
    {
         UARTBuffer[3] = 0xb9;
         if(ChannelData[UARTBuffer[4]].timeToGoal!=1)
             UARTBuffer[5] = ChannelData[UARTBuffer[4]].GoalLevel;
         else UARTBuffer[5]	= 0;
    }
    return;
}

void CmmssnIns_Set_Physical_Channel_Level(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    uint8_t indx;
    indx = UARTBuffer[4]-1;
    if(ChannelData[indx].timeToGoal!=1)
        ChannelData[indx].OringalLevel=(ChannelData[indx].OringalLevel+ChannelData[indx].GoalLevel)*ChannelData[indx].PastedTime*ChannelData[indx].timeToGoal;
    ChannelData[indx].GoalLevel = ~UARTBuffer[5];
    ChannelData[indx].timeToGoal= UARTBuffer[6]+2;
    ChannelData[indx].PastedTime = 1;
    return;
}

void CmmssnIns_DM320_dali_cmd(uint8_t *UARTBuffer, uint8_t *PresetValue)
{
    switch(UARTBuffer[4])
    {
        case 2:
            WriteBuffer(&DaliCirBuffer,UARTBuffer[5]<<8|UARTBuffer[6]);
            TimeCir_1st=1;
            break;
        case 1:
            WriteBuffer(&DaliCirBuffer_2nd,UARTBuffer[5]<<8|UARTBuffer[6]);
            TimeCir_2nd=1;
            break;
        case 0:
            WriteBuffer(&DaliCirBuffer_3rd,UARTBuffer[5]<<8|UARTBuffer[6]);
            TimeCir_3rd=1;									
            break;
        default:break;
    }
    return;
}

CommissioningIns_stru stCmmssnIns[] =
{
    {Setup                          , CmmssnIns_Setup                           },
    {Reboot_Device                  , CmmssnIns_Reboot_Device                   },
    {Read_EEPROM                    , CmmssnIns_Read_EEPROM                     },
    {Write_EEPROM                   , CmmssnIns_Write_EEPROM                    },
    {BLOCK_EE_Read_Enable           , CmmssnIns_BLOCK_EE_Read_Enable            },
    {BLOCK_EE_Write_Enable          , CmmssnIns_BLOCK_EE_Write_Enable           },
    {Request_Channel_Status         , CmmssnIns_Request_Channel_Status          },//此命令有疑义，待确认
    {Request_Physical_Channel_Level , CmmssnIns_Request_Physical_Channel_Level  },
    {Set_Physical_Channel_Level     , CmmssnIns_Set_Physical_Channel_Level      },
    {DM320_dali_cmd                 , CmmssnIns_DM320_dali_cmd                  }
};

uint8_t ucMaxCmmssnInsNum = sizeof(stCmmssnIns)/sizeof(CommissioningIns_stru);

void ProcessCMD(void)
{
//	uint16_t indx;
	uint8_t PresetValue[192];
	uint8_t ALvalue[768];
//	uint8_t AreaLink[4];
    uint8_t i;
//	uint32_t 	Temp_value;
    
    if(0xF5 == UARTBuffer[0])
    {
        for(i = 0; i < ucMaxDimIns; i++)
        {
            if(UARTBuffer[3] == Dim_Ins[i].ins)
            {
                Dim_Ins[i].pFuncDim(UARTBuffer,PresetValue,ALvalue);
                break;
            }
        }
    }
    else
    {
			if(0xFA == UARTBuffer[0])
			{
				if((DeviceCode == UARTBuffer[1])||(0xAA == UARTBuffer[1]))
				{
					if((UARTBuffer[2]==Box)||(0x55 == UARTBuffer[2]))
					{
                        for(i = 0; i < ucMaxCmmssnInsNum; i++)
                        {
                            if(stCmmssnIns[i].ins == UARTBuffer[3])
                            {
                                stCmmssnIns[i].pFuncCommissioning(UARTBuffer,PresetValue);
                                break;
                            }
                        }
                    }
                }
            }
    }
    return;
}

uint8_t IsAreaAccept(uint8_t indx)
{
	if((ChannelData[indx].Area==UARTBuffer[1])||(UARTBuffer[1]==0))
		return 1;
	else 
		return 0;
}

uint8_t IsAppendAreaAccept(uint8_t indx)
{

    if((((UARTBuffer[2]==0xff))) 
      ||( (UARTBuffer[2]==ChannelData[indx].AppendArea)&&((UARTBuffer[2]&0x80)==0x80))
      ||(((UARTBuffer[2]&0x80)!=0x80)&&(((ChannelData[indx].AppendArea &0x80)!= 0x80))&&((UARTBuffer[2]&ChannelData[indx].AppendArea)!=0))  )		  
    {
        return 1;
    }
    else
        return 0;
    

}
