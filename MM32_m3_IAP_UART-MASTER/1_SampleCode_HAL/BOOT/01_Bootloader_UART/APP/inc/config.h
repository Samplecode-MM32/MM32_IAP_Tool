/******************************************************************************
 * @file    config.h
 * @author  MM32 AE
 * @version V1.00
 * @date    10-June-2020
 * @brief   ......
 ******************************************************************************
 *  @attention
 * 
 *  THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 *  CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 *  TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 *  CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 *  HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 *  CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *  <H2><CENTER>&COPY; COPYRIGHT 2020 MINDMOTION </CENTER></H2>
******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __CONFIG_H__
#define __CONFIG_H__


#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/* Includes -----------------------------------------------------------------*/
#include "HAL_conf.h"
#include "HAL_device.h"


/* Exported constants -------------------------------------------------------*/

/* Define device flash size */
#if defined (MM32L3XX_N)
#define FLASH_SIZE          (128)/* Flash capacity of the selected chip (in K)*/
#define USER_FLASH_LAST_PAGE_ADDRESS  0x0801FC00
#define USER_FLASH_END_ADDRESS        0x0801FFFF /* 128 KBytes */
#elif (MM32F103xCxE_O)
#define FLASH_SIZE          (512)/* Flash capacity of the selected chip (in K)*/
#define USER_FLASH_LAST_PAGE_ADDRESS  0x0807FC00
#define USER_FLASH_END_ADDRESS        0x0807FFFF /* 512 KBytes */
#endif

/* Warning: Only one transfer protocol definition below can be selected! */
/*
//#define XMODEM_EN
//#define XMODEM1K_EN    
//#define YMODEM_EN    
//#define BINARY_EN    
//#define ASCII_EN    
*/
#define XMODEM_EN    /* transfer protocol define */
#if defined (XMODEM_EN) || defined (XMODEM1K_EN)
#include "xmodem.h"
#elif defined (YMODEM_EN)  || defined (YMODEM1K_EN)
#include "ymodem.h"
#elif defined (BINARY_EN)
#include "binary.h"
#elif defined (ASCII_EN)
#include "ascii.h"
#else	

#endif

/* transfer Baud rate define , 9600 or 115200  */
#define IAP_BAUD   115200  
#define APPLICATION_ADDRESS     (uint32_t)0x08002800
/* The parameter arrangement is :
UpgradeReqFlag + AppExsitFlag + AppBinCheck + UpBaud + бнбн
Note: Each parameter takes only one byte!
*/
#define PARA_ADDRESS     (uint32_t)0x08002400 
#define PARA_SIZE     (12) 

#define CHECKAPP_EN  0 /* Default is 0, no check for the app data */
#define DEBUG_EN  0 /* Default is 0, no putout log */
/* Default is 0, Ex factory burning does not need to merge hex */
#define MERGE_HEX_EN 0  

#define IAP_UART  UART1   /* Used for IAP */
//#define PRINTF_EN
#ifdef PRINTF_EN
#define PRINTF_UART  UART1   /* Used for printf */
#endif


/* Exported macro -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

