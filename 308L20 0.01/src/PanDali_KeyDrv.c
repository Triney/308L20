#include "keyscan.h"
#include "PanDali_KeyDrv.h"

uint8_t PanelDali_SimulateRcvCmd(uint8_t *pBuffer, key_state_uint8_t enState)
{
    
    switch(enState)
    {
        case key_none:
        {
            break;
        }
        case key_left_turn:
        {
            *pBuffer++ = 0xf5;
            *pBuffer++ = 0;
            *pBuffer++ = 1;
        }
            
    }
}


