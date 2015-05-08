/**
 ******************************************************************************
 * @file	typedefs.h
 * @brief	Enter brief Description of this module.
 * @version	1.0
 * @date	03.05.2010
 * @author  NXP Semiconductors
 *
 * @details Enter detailed description of this module.
 *
 * ----------------------------------------------------------------------------\n
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 * ----------------------------------------------------------------------------\n
 ******************************************************************************
 */

#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef	WEAK
#define WEAK __attribute__ ((weak))
#endif
#ifndef	ALIAS
#define ALIAS(f) __attribute__ ((weak, alias (#f)))
#endif

/**
 *******************************************************************************
 * @typedef S8
 * @brief
 * Type definition for a signed 8 bit variable.
 *******************************************************************************
 */
typedef	char 	S8;
/**
 *******************************************************************************
 * @typedef S16
 * @brief
 * Type definition for a signed 16 bit variable.
 *******************************************************************************
 */
typedef	short	S16;
/**
 *******************************************************************************
 * @typedef S32
 * @brief
 * Type definition for a signed 32 bit variable.
 *******************************************************************************
 */
typedef	long	S32;
/**
 *******************************************************************************
 * @typedef S64
 * @brief
 * Type definition for a signed 64 bit variable.
 *******************************************************************************
 */
typedef long long S64;
/**
 *******************************************************************************
 * @typedef U8
 * @brief
 * Type definition for a unsigned 8 bit variable.
 *******************************************************************************
 */
typedef unsigned char	U8;
/**
 *******************************************************************************
 * @typedef U16
 * @brief
 * Type definition for a unsigned 16 bit variable.
 *******************************************************************************
 */
typedef	unsigned short  U16;
/**
 *******************************************************************************
 * @typedef U32
 * @brief
 * Type definition for a unsigned 32 bit variable.
 *******************************************************************************
 */
typedef	unsigned long	U32;
/**
 *******************************************************************************
 * @typedef U64
 * @brief
 * Type definition for a unsigned 64 bit variable.
 *******************************************************************************
 */
typedef unsigned long long U64;
/**
 *******************************************************************************
 * @typedef BOOL
 * @brief
 * Type definition for boolean variable.
 *******************************************************************************
 */
typedef	U8	BOOL;


#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif
#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#endif //__TYPEDEFS_H__
