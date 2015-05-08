#ifndef _PANDALI_DRV_H
#define _PANDALI_DRV_H
#include "keyscan.h"

enum key_state
{
    key_none                        = 0,
    key_left_turn                   = 1,
    key_right_turn                  = 2,
    key_center_pressed              = 3,
    key_center_double               = 4,
    key_left_turn_center_pressed    = 5,
    key_right_turn_center_pressed   = 6
};

typedef uint8_t  key_state_uint8_t;

#endif

