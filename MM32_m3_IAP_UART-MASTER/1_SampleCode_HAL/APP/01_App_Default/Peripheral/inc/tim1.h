/******************************************************************************
 * @file    tim1.h
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
#ifndef __TIM1_H__
#define __TIM1_H__


#ifdef __cplusplus
extern "C" {
#endif


#undef  EXTERN


#ifdef  __TIM1_C__
#define EXTERN
#else
#define EXTERN extern
#endif


/* Includes -----------------------------------------------------------------*/
#include "config.h"


/* Exported constants -------------------------------------------------------*/
#define UART_TIME_OUT  0X01
/* Exported types -----------------------------------------------------------*/
/* Exported macro -----------------------------------------------------------*/
/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/

EXTERN void TIM1_Processing(void);
EXTERN void TIM1_Configure(uint16_t hwPrescaler, uint16_t hwPeriod);


#ifdef __cplusplus
}
#endif


#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

