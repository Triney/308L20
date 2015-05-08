#include "i2c1.h"
#include "dali_master.h"
#include "stdint.h"
#include "stdbool.h"
#include "uart.h"
/****************************************************/
/*Function Name:BlockRead
/*
/*读取EEPROM的数据并返回给上位机
/****************************************************/
extern unit8_t UARTBuffer[64];
uint8_t	temp_data[6];

void BlockRead(void)
{
	uint8_t ind;
	m24xx_read(EEPROM_24XX16,(UARTBuffer[4]<<8|UARTBuffer[5]),0,temp_data,6);
	UARTBuffer[7]=CheckSum(UARTBuffer,7);

}
