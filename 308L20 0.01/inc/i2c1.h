/*
www.mcu123.net 

LPC236X AT 24C16��ʾ����
IIC��ʾ����
2007.3.3����

��֤���Ѿ�����	 

�汾��v1.11

����ʱ�䣺2007.5.29  boy123

*/

#ifndef _I2C_H_
#define _I2C_H_

//---  LPC21XX I2C flags --------------------

#define  I2C_FLAG_AA    (1<<2)
#define  I2C_FLAG_SI    (1<<3)
#define  I2C_FLAG_STO   (1<<4)
#define  I2C_FLAG_STA   (1<<5)
#define  I2C_FLAG_I2EN  (1<<6)

//---- I2C Speed
#define  I2C_SPEED_100   0
#define  I2C_SPEED_400   1

//--- Errors

#define  I2C_NO_ERR                    0
#define  I2C_ERR_NO_RESPONSE           1
#define  I2C_ERR_WRONG_PARAM           2
#define  I2C_ERR_24XX_WR_TIMEOUT       3
#define  I2C_ERR_24XX_BAD_ADDR         4
#define  I2C_ERR_24XX_BAD_TYPE         5
#define  I2C_ERR_24XX_BAD_PAGESIZE     6
#define  I2C_ERR_24XX_WRONG_NUM        7



//--- EEPROM 24xx types

#define  EEPROM_24XX04                 1
#define  EEPROM_24XX08                 2
#define  EEPROM_24XX16                 3
#define  EEPROM_24XX32                 4
#define  EEPROM_24XX64                 5
#define  EEPROM_24XX128                6
#define  EEPROM_24XX256                7
#define  EEPROM_24XX512                8

//--- EEPROM 24xx max addr values

#define  EEPROM_MAX_ADDR_24XX04    0x01FF   //-- 512 Bytes
#define  EEPROM_MAX_ADDR_24XX08    0x03FF   //--  1 KBytes
#define  EEPROM_MAX_ADDR_24XX16    0x07FF   //--  2 KBytes
#define  EEPROM_MAX_ADDR_24XX32    0x0FFF   //--  4 KBytes
#define  EEPROM_MAX_ADDR_24XX64    0x1FFF   //--  8 KBytes
#define  EEPROM_MAX_ADDR_24XX128   0x3FFF   //-- 16 KBytes
#define  EEPROM_MAX_ADDR_24XX256   0x7FFF   //-- 32 KBytes
#define  EEPROM_MAX_ADDR_24XX512   0xFFFF   //-- 64 KBytes

//--- EEPROM 24xx write timeout ( > 5 mS)

#define  I2C_WR_24XX_TIMEOUT     10000

#define I2C_IDLE			0
#define I2C_STARTED			1
#define I2C_RESTARTED		2
#define I2C_REPEATED_START	3
#define DATA_ACK			4
#define DATA_NACK			5

#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA			0x00000004
#define I2CONSET_SI			0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC		0x00000040

#define I2DAT_I2C			0x00000000  /* I2C Data Reg */
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */


#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA			0x00000004
#define I2CONSET_SI			0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC		0x00000040

#define I2DAT_I2C			0x00000000  /* I2C Data Reg */
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH			0x00000180  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			0x00000180  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_HS_SCLH		0x00000020  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2SCLL_HS_SCLL		0x00000020  /* Fast Plus I2C SCL Duty Cycle Low Reg */




//---- Prototypes ---

void i2c_lpc_init(int Mode);

int m24xx_write(
    int eeprom_type,    //-- EEPROM type
    int eeprom_addr,    //-- start eeprom addr ( not included Hardware A2,A1,A0)
    int eeprom_cs_val,  //-- Hardware A2,A1,A0 (valid from 24XX32)
    uint8_t * buf,         //-- Data src buf
    int num);           //-- Bytes to write qty

int m24xx_read(
  int eeprom_type,    //-- EEPROM type
  int eeprom_addr,    //-- start eeprom addr ( not included Hardware A2,A1,A0)
  int eeprom_cs_val,  //-- Hardware A2,A1,A0 (valid from 24XX32)
  uint8_t * buf,         //-- Data dst buf
  int num);           //-- Bytes to read qty


#endif


