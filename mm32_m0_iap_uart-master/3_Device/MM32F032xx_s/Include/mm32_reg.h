////////////////////////////////////////////////////////////////////////////////
/// @file     mm32_reg.h
/// @author   AE TEAM
/// @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE SERIES OF
///           MM32 FIRMWARE LIBRARY.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
////////////////////////////////////////////////////////////////////////////////
#ifndef __MM32_REG_H
#define __MM32_REG_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"





#if defined ( __CC_ARM )
#pragma anon_unions
#endif

#ifndef HSE_STARTUP_TIMEOUT
#define HSE_STARTUP_TIMEOUT     (0x0500U)                                       ///< Time out for HSE start up.
#endif
#ifdef  CLOCK_12MHz
#ifndef HSE_VALUE
#define HSE_VALUE               (12000000U)                                     ///< Value of the External oscillator in Hz.
#endif
#else
#ifndef HSE_VALUE
#define HSE_VALUE               (8000000U)                                      ///< Value of the External oscillator in Hz.
#endif
#endif



#define HSI_VALUE_PLL_ON        (48000000U / 4)                                 ///< Value of the Internal oscillator in Hz.
#define HSI_DIV6                (48000000U / 6)                                 ///< Value of the Internal oscillator in Hz.

//#define HSI_VALUE_PLL_ON      (72000000 / 4)                                  ///< Value of the Internal
// oscillator in Hz. #define HSI_VALUE_PLL_OFF      (72000000 / 6)              ///<
// Value of the Internal oscillator in Hz.


#define LSI_VALUE               (40000U)                                        ///< Value of the Internal oscillator in Hz.



#ifndef HSI_VALUE
#define HSI_VALUE               HSI_DIV6
#endif










#define __MPU_PRESENT               (0)                                     ///< Cortex-M0 does not provide a MPU present or not
#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS            (2)                                     ///< Cortex-M0 uses 2 Bits for the Priority Levels
//#warning "__NVIC_PRIO_BITS not defined in device header file; using default!"
#endif
#define __Vendor_SysTickConfig      (0)                                     ///< Set to 1 if different SysTick Config is used

////////////////////////////////////////////////////////////////////////////////
/// @brief  MM32 MCU Interrupt Handle
////////////////////////////////////////////////////////////////////////////////
typedef enum IRQn {
    NonMaskableInt_IRQn             = -14,                                  ///< 2 Non Maskable Interrupt
    HardFault_IRQn                  = -13,                                  ///< 3 Cortex-M0 Hard Fault Interrupt
    SVC_IRQn                        = -5,                                   ///< 11 Cortex-M0 SV Call Interrupt
    PendSV_IRQn                     = -2,                                   ///< 14 Cortex-M0 Pend SV Interrupt
    SysTick_IRQn                    = -1,                                   ///< 15 Cortex-M0 System Tick Interrupt
    WWDG_IWDG_IRQn                  = 0,                                    ///< WatchDog Interrupt
    WWDG_IRQn                       = 0,                                    ///< Window WatchDog Interrupt
    PVD_IRQn                        = 1,                                    ///< PVD through EXTI Line detect Interrupt
    BKP_IRQn                        = 2,                                    ///< RTC through EXTI Line Interrupt
    RTC_IRQn                        = 2,                                    ///< RTC through EXTI Line Interrupt
    FLASH_IRQn                      = 3,                                    ///< FLASH Interrupt
    RCC_IRQn                        = 4,                                    ///< RCC Interrupt
    EXTI0_1_IRQn                    = 5,                                    ///< EXTI Line 0 and 1 Interrupts
    EXTI2_3_IRQn                    = 6,                                    ///< EXTI Line 2 and 3 Interrupts
    EXTI4_15_IRQn                   = 7,                                    ///< EXTI Line 4 to 15 Interrupts
    DMA1_Channel1_IRQn              = 9,                                    ///< DMA1 Channel 1 Interrupt
    DMA1_Channel2_3_IRQn            = 10,                                   ///< DMA1 Channel 2 and Channel 3 Interrupts
    DMA1_Channel4_5_IRQn            = 11,                                   ///< DMA1 Channel 4 and Channel 5 Interrupts
    ADC_IRQn                        = 12,                                   ///< ADC Interrupts
    ADC1_IRQn                       = 12,                                   ///< ADC Interrupts
    TIM1_BRK_UP_TRG_COM_IRQn        = 13,                                   ///< TIM1 Break, Update, Trigger and Commutation Interrupts
    TIM1_CC_IRQn                    = 14,                                   ///< TIM1 Capture Compare Interrupt
    TIM2_IRQn                       = 15,                                   ///< TIM2 Interrupt
    TIM3_IRQn                       = 16,                                   ///< TIM3 Interrupt
    TIM14_IRQn                      = 19,                                   ///< TIM14 Interrupt
    TIM16_IRQn                      = 21,                                   ///< TIM16 Interrupt
    TIM17_IRQn                      = 22,                                   ///< TIM17 Interrupt
    I2C1_IRQn                       = 23,                                   ///< I2C1 Interrupt
    SPI1_IRQn                       = 25,                                   ///< SPI1 Interrupt
    SPI2_IRQn                       = 26,                                   ///< SPI1 Interrupt
    UART1_IRQn                      = 27,                                   ///< UART1 Interrupt
    UART2_IRQn                      = 28,                                   ///< UART2 Interrupt
} IRQn_Type;





#include <core_cm0.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief  MM32 MCU Memory/Peripherals mapping
////////////////////////////////////////////////////////////////////////////////
#define FLASH_BASE                      (0x08000000U)                           ///< FLASH base address in the alias region
#define SRAM_BASE                       (0x20000000U)                           ///< SRAM base address in the alias region
#define PERIPH_BASE                     (0x40000000U)                           ///< Peripheral base address in the alias region



#define APB1PERIPH_BASE                 (PERIPH_BASE + 0x00000000)
#define APB2PERIPH_BASE                 (PERIPH_BASE + 0x00010000)
#define AHBPERIPH_BASE                  (PERIPH_BASE + 0x00020000)
#define APB3PERIPH_BASE                 (PERIPH_BASE + 0x00030000)
#define AHB2PERIPH_BASE                 (PERIPH_BASE + 0x08000000)
////////////////////////////////////////////////////////////////////////////////
/// @brief ADC Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1_BASE                       (APB2PERIPH_BASE + 0x2400)              ///< Base Address: 0x40012400

////////////////////////////////////////////////////////////////////////////////
/// @brief BKP Base Address Definition
////////////////////////////////////////////////////////////////////////////////


#define BKP_BASE                        (APB1PERIPH_BASE + 0x2840)              ///< Base Address: 0x40002840
////////////////////////////////////////////////////////////////////////////////
/// @brief CRC Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define CRC_BASE                        (AHBPERIPH_BASE + 0x3000)               ///< Base Address: 0x40023000

////////////////////////////////////////////////////////////////////////////////
/// @brief DBG Base Address Definition
////////////////////////////////////////////////////////////////////////////////

#define DBG_BASE                        (APB2PERIPH_BASE + 0x3400)              ///< Base Address: 0x40013400
////////////////////////////////////////////////////////////////////////////////
/// @brief DMA Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA1_BASE                       (AHBPERIPH_BASE + 0x0000)               ///< Base Address: 0x40020000
#define DMA1_Channel1_BASE              (AHBPERIPH_BASE + 0x0008)               ///< Base Address: 0x40020008
#define DMA1_Channel2_BASE              (AHBPERIPH_BASE + 0x001C)               ///< Base Address: 0x4002001C
#define DMA1_Channel3_BASE              (AHBPERIPH_BASE + 0x0030)               ///< Base Address: 0x40020030
#define DMA1_Channel4_BASE              (AHBPERIPH_BASE + 0x0044)               ///< Base Address: 0x40020044
#define DMA1_Channel5_BASE              (AHBPERIPH_BASE + 0x0058)               ///< Base Address: 0x40020058

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_BASE                       (APB2PERIPH_BASE + 0x0000)              ///< Base Address: 0x40010000

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_REG_BASE                  (AHBPERIPH_BASE + 0x2000)               ///< Base Address: 0x40022000

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO Base Address Definition
////////////////////////////////////////////////////////////////////////////////

#define GPIOA_BASE                  (AHB2PERIPH_BASE + 0x0000)              ///< Base Address: 0x48000000
#define GPIOB_BASE                  (AHB2PERIPH_BASE + 0x0400)              ///< Base Address: 0x48000400

#define GPIOC_BASE                  (AHB2PERIPH_BASE + 0x0800)              ///< Base Address: 0x48000800
#define GPIOD_BASE                  (AHB2PERIPH_BASE + 0x0C00)              ///< Base Address: 0x48000C00

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C1_BASE                       (APB1PERIPH_BASE + 0x5400)              ///< Base Address: 0x40005400

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_BASE                       (APB1PERIPH_BASE + 0x3000)              ///< Base Address: 0x40003000
////////////////////////////////////////////////////////////////////////////////
/// @brief OPTB Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define OB_BASE                         (0x1FFFF800U)                           ///< Flash Option Bytes base address
#define PROTECT_BASE                    (0x1FFE0000U)                           ///< Flash Protect Bytes base address
////////////////////////////////////////////////////////////////////////////////
/// @brief PWR Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define PWR_BASE                        (APB1PERIPH_BASE + 0x7000)              ///< Base Address: 0x40007000

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_BASE                        (AHBPERIPH_BASE + 0x1000)               ///< Base Address: 0x40021000
////////////////////////////////////////////////////////////////////////////////
/// @brief RTC Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_BASE                    (APB1PERIPH_BASE + 0x2800)              ///< Base Address: 0x40002800
////////////////////////////////////////////////////////////////////////////////
/// @brief SPI Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI2_BASE                       (APB1PERIPH_BASE + 0x3800)              ///< Base Address: 0x40003000
#define SPI1_BASE                       (APB2PERIPH_BASE + 0x3000)              ///< Base Address: 0x40003000

#define SYSCFG_BASE                     (APB2PERIPH_BASE + 0x0000)              ///< Base Address: 0x40010000




////////////////////////////////////////////////////////////////////////////////
/// @brief TIM Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM1_BASE                       (APB2PERIPH_BASE + 0x2C00)              ///< Base Address: 0x40012C00
#define TIM2_BASE                       (APB1PERIPH_BASE + 0x0000)              ///< Base Address: 0x40000000
#define TIM3_BASE                       (APB1PERIPH_BASE + 0x0400)              ///< Base Address: 0x40000400



#define TIM14_BASE                      (APB2PERIPH_BASE + 0x4000)              ///< Base Address: 0x40014000
#define TIM16_BASE                      (APB2PERIPH_BASE + 0x4400)              ///< Base Address: 0x40014400
#define TIM17_BASE                      (APB2PERIPH_BASE + 0x4800)              ///< Base Address: 0x40014800

////////////////////////////////////////////////////////////////////////////////
/// @brief UART Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define UART1_BASE                      (APB2PERIPH_BASE + 0x3800)              ///< Base Address: 0x40013800
#define UART2_BASE                      (APB1PERIPH_BASE + 0x4400)              ///< Base Address: 0x40004400
////////////////////////////////////////////////////////////////////////////////
/// @brief UID type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define UID_BASE                    (0x1FFFF7E8U)                           ///< Unique device ID register base address

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG Base Address Definition
////////////////////////////////////////////////////////////////////////////////
#define WWDG_BASE                       (APB1PERIPH_BASE + 0x2C00)              ///< Base Address: 0x40002C00

////////////////////////////////////////////////////////////////////////////////
/// @brief Analog-to-Digital Converter register
////////////////////////////////////////////////////////////////////////////////
#define USENCOMBINEREGISTER
#ifdef USENCOMBINEREGISTER

typedef struct {
    union {
        __IO u32 DR;                                                                ///< ADC data register,                             offset: 0x00
        __IO u32 ADDATA;
    };
    union {
        __IO u32 CFGR;                                                              ///< ADC configuration register,                    offset: 0x04
        __IO u32 ADCFG;
    };
    union {
        __IO u32 CR;                                                                ///< ADC control register,                          offset: 0x08
        __IO u32 ADCR;
    };
    union {
        __IO u32 CHSR;                                                              ///< ADC channel selection register,                offset: 0x0C
        __IO u32 ADCHS;
    };
    union {
        __IO u32 CMPR;                                                              ///< ADC window compare register,                   offset: 0x10
        __IO u32 ADCMPR;
    };
    union {
        __IO u32 SR;                                                                ///< ADC status register,                           offset: 0x14
        __IO u32 ADSTA;
    };
    union {
        __IO u32 CH0DR;                                                             ///< ADC channel0 data register,                    offset: 0x18
        __IO u32 ADDR0;
    };
    union {
        __IO u32 CH1DR;                                                             ///< ADC channel1 data register,                    offset: 0x1C
        __IO u32 ADDR1;
    };
    union {
        __IO u32 CH2DR;                                                             ///< ADC channel2 data register,                    offset: 0x20
        __IO u32 ADDR2;
    };
    union {
        __IO u32 CH3DR;                                                             ///< ADC channel3 data register,                    offset: 0x24
        __IO u32 ADDR3;
    };
    union {
        __IO u32 CH4DR;                                                             ///< ADC channel4 data register,                    offset: 0x28
        __IO u32 ADDR4;
    };
    union {
        __IO u32 CH5DR;                                                             ///< ADC channel5 data register,                    offset: 0x2C
        __IO u32 ADDR5;
    };
    union {
        __IO u32 CH6DR;                                                             ///< ADC channel6 data register,                    offset: 0x30
        __IO u32 ADDR6;
    };
    union {
        __IO u32 CH7DR;                                                             ///< ADC channel7 data register,                    offset: 0x34
        __IO u32 ADDR7;
    };
    union {
        __IO u32 CH8DR;                                                             ///< ADC channel8 data register,                    offset: 0x38
        __IO u32 ADDR8;
    };
    union {
        __IO u32 CH9DR;                                                             ///< ADC channel9 data register,                    offset: 0x3C
        __IO u32 ADDR9;
    };
    __IO u32 RESERVED0;                                                         ///<                                                offset: 0x40
    __IO u32 RESERVED1;                                                         ///<                                                offset: 0x44
    __IO u32 RESERVED2;                                                         ///<                                                offset: 0x48
    __IO u32 RESERVED3;                                                         ///<                                                offset: 0x4C
    union {
        __IO u32 CH14DR;                                                        ///< ADC channel14 data register,                   offset: 0x50
        __IO u32 ADDR14;
    };
    union {
        __IO u32 CH15DR;                                                        ///< ADC channel15 data register,                   offset: 0x54
        __IO u32 ADDR15;
    };
    __IO u32 SREXT;                                                             ///< ADC Extended Status Register,                  offset: 0x58
    __IO u32 CHANY0;                                                            ///< ADC any Chan Select Register 0,                offset: 0x5C
    __IO u32 CHANY1;                                                            ///< ADC any Chan Select Register 1,                offset: 0x60
    __IO u32 ANYCFG;                                                            ///< ADC any Chan config Register,                  offset: 0x64
    __IO u32 ANYCR;                                                             ///< ADC any Chan control Register,                 offset: 0x68
} ADC_TypeDef;

#endif
#ifdef USENNEWREGISTER
////////////////////////////////////////////////////////////////////////////////
/// @brief Analog-to-Digital Converter register
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 DR;                                                                ///< ADC data register,                             offset: 0x00
    __IO u32 CFGR;                                                              ///< ADC configuration register,                    offset: 0x04
    __IO u32 CR;                                                                ///< ADC control register,                          offset: 0x08
    __IO u32 CHSR;                                                              ///< ADC channel selection register,                offset: 0x0C
    __IO u32 CMPR;                                                              ///< ADC window compare register,                   offset: 0x10
    __IO u32 SR;                                                                ///< ADC status register,                           offset: 0x14
    __IO u32 CH0DR;                                                             ///< ADC channel0 data register,                    offset: 0x18
    __IO u32 CH1DR;                                                             ///< ADC channel1 data register,                    offset: 0x1C
    __IO u32 CH2DR;                                                             ///< ADC channel2 data register,                    offset: 0x20
    __IO u32 CH3DR;                                                             ///< ADC channel3 data register,                    offset: 0x24
    __IO u32 CH4DR;                                                             ///< ADC channel4 data register,                    offset: 0x28
    __IO u32 CH5DR;                                                             ///< ADC channel5 data register,                    offset: 0x2C
    __IO u32 CH6DR;                                                             ///< ADC channel6 data register,                    offset: 0x30
    __IO u32 CH7DR;                                                             ///< ADC channel7 data register,                    offset: 0x34
    __IO u32 CH8DR;                                                             ///< ADC channel8 data register,                    offset: 0x38
    __IO u32 CH9DR;                                                             ///< ADC channel9 data register,                    offset: 0x3C
    __IO u32 RESERVED0;                                                         ///<                                                offset: 0x40
    __IO u32 RESERVED1;                                                         ///<                                                offset: 0x44
    __IO u32 RESERVED2;                                                         ///<                                                offset: 0x48
    __IO u32 RESERVED3;                                                         ///<                                                offset: 0x4C
    __IO u32 CH14DR;                                                            ///< ADC channel14 data register,                   offset: 0x50
    __IO u32 CH15DR;                                                            ///< ADC channel15 data register,                   offset: 0x54
    __IO u32 SREXT;                                                             ///< ADC Extended Status Register,                  offset: 0x58
    __IO u32 CHANY0;                                                            ///< ADC any Chan Select Register 0,                offset: 0x5C
    __IO u32 CHANY1;                                                            ///< ADC any Chan Select Register 1,                offset: 0x60
    __IO u32 ANYCFG;                                                            ///< ADC any Chan config Register,                  offset: 0x64
    __IO u32 ANYCR;                                                             ///< ADC any Chan control Register,                 offset: 0x68
} ADC_TypeDef;
#endif
#ifdef USENOLDREGISTER
typedef struct {
    __IO u32 ADDATA;                                                            ///< ADC data register,                             offset: 0x00
    __IO u32 ADCFG;                                                             ///< ADC configuration register,                    offset: 0x04
    __IO u32 ADCR;                                                              ///< ADC control register,                          offset: 0x08
    __IO u32 ADCHS;                                                             ///< ADC channel selection register,                offset: 0x0C
    __IO u32 ADCMPR;                                                            ///< ADC window compare register,                   offset: 0x10
    __IO u32 ADSTA;                                                             ///< ADC status register,                           offset: 0x14
    __IO u32 ADDR0;                                                             ///< ADC channel0 data register,                    offset: 0x18
    __IO u32 ADDR1;                                                             ///< ADC channel1 data register,                    offset: 0x1C
    __IO u32 ADDR2;                                                             ///< ADC channel2 data register,                    offset: 0x20
    __IO u32 ADDR3;                                                             ///< ADC channel3 data register,                    offset: 0x24
    __IO u32 ADDR4;                                                             ///< ADC channel4 data register,                    offset: 0x28
    __IO u32 ADDR5;                                                             ///< ADC channel5 data register,                    offset: 0x2C
    __IO u32 ADDR6;                                                             ///< ADC channel6 data register,                    offset: 0x30
    __IO u32 ADDR7;                                                             ///< ADC channel7 data register,                    offset: 0x34
    __IO u32 ADDR8;                                                             ///< ADC channel8 data register,                    offset: 0x38
    __IO u32 ADDR9;                                                             ///< ADC channel9 data register,                    offset: 0x3C
    __IO u32 RESERVED0;                                                         ///<                                                offset: 0x40
    __IO u32 RESERVED1;                                                         ///<                                                offset: 0x44
    __IO u32 RESERVED2;                                                         ///<                                                offset: 0x48
    __IO u32 RESERVED3;                                                         ///<                                                offset: 0x4C
    __IO u32 ADDR14;                                                            ///< ADC channel14 data register,                   offset: 0x50
    __IO u32 ADDR15;                                                            ///< ADC channel15 data register,                   offset: 0x54
    __IO u32 SREXT;                                                             ///< ADC Extended Status Register,                  offset: 0x58
    __IO u32 CHANY0;                                                            ///< ADC any Chan Select Register 0,                offset: 0x5C
    __IO u32 CHANY1;                                                            ///< ADC any Chan Select Register 1,                offset: 0x60
    __IO u32 ANYCFG;                                                            ///< ADC any Chan config Register,                  offset: 0x64
    __IO u32 ANYCR;                                                             ///< ADC any Chan control Register,                 offset: 0x68
} ADC_TypeDef;
#endif
////////////////////////////////////////////////////////////////////////////////
/// @brief BKP Register Structure Definition
////////////////////////////////////////////////////////////////////////////////



#define BKP_NUMBER  20

typedef struct {
    __IO u32 RTCCR;                                                             ///< RTC clock calibration register,                offset: 0x00
    __IO u32 CR;                                                                ///< BKP control register,                          offset: 0x04
    __IO u32 CSR;                                                               ///< BKP control/status register,                   offset: 0x08
    __IO u32 RESERVED0;                                                         ///< Reserved,                                      offset: 0x0C
    __IO u32 DR1;                                                               ///< BKP data register 1,                           offset: 0x10
    __IO u32 DR2;                                                               ///< BKP data register 2,                           offset: 0x14
    __IO u32 DR3;                                                               ///< BKP data register 3,                           offset: 0x18
    __IO u32 DR4;                                                               ///< BKP data register 4,                           offset: 0x1C
    __IO u32 DR5;                                                               ///< BKP data register 5,                           offset: 0x20
    __IO u32 DR6;                                                               ///< BKP data register 6,                           offset: 0x24
    __IO u32 DR7;                                                               ///< BKP data register 7,                           offset: 0x28
    __IO u32 DR8;                                                               ///< BKP data register 8,                           offset: 0x2C
    __IO u32 DR9;                                                               ///< BKP data register 9,                           offset: 0x30
    __IO u32 DR10;                                                              ///< BKP data register 10                           offset: 0x34
    __IO u32 DR11;                                                              ///< BKP data register 11,                          offset: 0x38
    __IO u32 DR12;                                                              ///< BKP data register 12,                          offset: 0x3C
    __IO u32 DR13;                                                              ///< BKP data register 13,                          offset: 0x40
    __IO u32 DR14;                                                              ///< BKP data register 14,                          offset: 0x44
    __IO u32 DR15;                                                              ///< BKP data register 15,                          offset: 0x48
    __IO u32 DR16;                                                              ///< BKP data register 16,                          offset: 0x4C
    __IO u32 DR17;                                                              ///< BKP data register 17,                          offset: 0x50
    __IO u32 DR18;                                                              ///< BKP data register 18,                          offset: 0x54
    __IO u32 DR19;                                                              ///< BKP data register 19,                          offset: 0x58
    __IO u32 DR20;                                                              ///< BKP data register 20,                          offset: 0x5C
} BKP_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief CRC Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32   DR;                                                              ///< CRC data register,                             offset: 0x00
    __IO u32   IDR;                                                             ///< CRC independent data register,                 offset: 0x04
    __IO u32   CR;                                                              ///< CRC control register,                          offset: 0x08
} CRC_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief DEBUG Registers Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 IDCODE;                                                            ///< Code ID                                        offset: 0x00
    __IO u32 CR;                                                                ///< Control Register                               offset: 0x04
} DBGMCU_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief DMA Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CCR;                                                               ///< DMA channel x configuration register           offset: 0x00
    __IO u32 CNDTR;                                                             ///< DMA channel x number of data register          offset: 0x04
    __IO u32 CPAR;                                                              ///< DMA channel x peripheral address register      offset: 0x08
    __IO u32 CMAR;                                                              ///< DMA channel x memory address register          offset: 0x0C
} DMA_Channel_TypeDef;

typedef struct {
    __IO u32 ISR;                                                               ///< Interrupt Status Register                      offset: 0x00
    __IO u32 IFCR;                                                              ///< Interrupt Flag Clear Register                  offset: 0x04
} DMA_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI Registers Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CFGR;                                                              ///< configuration register,                        offset: 0x00
    u32 Reserved;                                                               ///< Reserved                                       offset: 0x04
    __IO u32 CR[4];                                                             ///< External interrupt configuration register,     offset: 0x08 - 0x17
    u32 Reserved1[0x100 - 0x06];                                                ///< Reserved space

    __IO u32 IMR;                                                               ///< Interrupt Mask Register                        offset: 0x00 + 0x400
    __IO u32 EMR;                                                               ///< Event Mask Register                            offset: 0x04 + 0x400
    __IO u32 RTSR;                                                              ///< Rising Trigger Status Register                 offset: 0x08 + 0x400
    __IO u32 FTSR;                                                              ///< Falling Trigger Status Register                offset: 0x0C + 0x400
    __IO u32 SWIER;                                                             ///< Software Interrupt Enable Register             offset: 0x10 + 0x400
    __IO u32 PR;                                                                ///< Pending Register                               offset: 0x14 + 0x400
} EXTI_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH Registers Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 ACR;                                                               ///< Access control Register                        offset: 0x00
    __IO u32 KEYR;                                                              ///< Key Register                                   offset: 0x04
    __IO u32 OPTKEYR;                                                           ///< Option byte key Register                       offset: 0x08
    __IO u32 SR;                                                                ///< State Register                                 offset: 0x0C
    __IO u32 CR;                                                                ///< Control Register                               offset: 0x10
    __IO u32 AR;                                                                ///< Address Register                               offset: 0x14
    __IO u32 RESERVED;
    __IO u32 OBR;                                                               ///< Option bytes Register                          offset: 0x1C
    __IO u32 WRPR;                                                              ///< Write protect Register                         offset: 0x20
} FLASH_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO Registers Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CRL;                                                               ///< Control Register Low,                          offset: 0x00
    __IO u32 CRH;                                                               ///< Control Register High,                         offset: 0x04
    __IO u32 IDR;                                                               ///< Input Data Register,                           offset: 0x08
    __IO u32 ODR;                                                               ///< Output Data Register,                          offset: 0x0C
    __IO u32 BSRR;                                                              ///< Bit Set or Reset Register,                     offset: 0x10
    __IO u32 BRR;                                                               ///< Bit Reset Register,                            offset: 0x14
    __IO u32 LCKR;                                                              ///< Lock Register,                                 offset: 0x18
    __IO u32 DCR;                                                               ///< Pin Output Open Drain Config Register,         offset: 0x1C
    __IO u32 AFRL;                                                              ///< Port Multiplexing Function Low Register,       offset: 0x20
    __IO u32 AFRH;                                                              ///< Port Multiplexing Function High Register,      offset: 0x24
} GPIO_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
#undef USENCOMBINEREGISTER
#undef USENNEWREGISTER
#undef USENOLDREGISTER
#define USENCOMBINEREGISTER
#ifdef USENCOMBINEREGISTER
typedef struct {
    union {
        __IO u16 CR;                                                           ///< Control Register                               offset: 0x00
        __IO u16 IC_CON;
    };
    __IO u16 RESERVED0;
    union {
        __IO u16 TAR;                                                          ///< Target Address Register                        offset: 0x04
        __IO u16 IC_TAR;
    };
    __IO u16 RESERVED1;
    union {
        __IO u16 SAR;                                                          ///< Slave Address Register                         offset: 0x08
        __IO u16 IC_SAR;
    };
    __IO u16 RESERVED2;
    __IO u16 IC_HS_MADDR_RESERVED;                                         ///< Reserved Register                              offset: 0x0C
    __IO u16 RESERVED3;
    union {
        __IO u16 DR;                                                           ///< Data Command Register                          offset: 0x10
        __IO u16 IC_DATA_CMD;
    };
    __IO u16 RESERVED4;
    union {
        __IO u16 SSHR;                                                         ///< SCL High Period Count for Std. Speed Register  offset: 0x14
        __IO u16 IC_SS_SCL_HCNT;
    };
    __IO u16 RESERVED5;
    union {
        __IO u16 SSLR;                                                         ///< SCL Low Period Count for Std. Speed Register   offset: 0x18
        __IO u16 IC_SS_SCL_LCNT;
    };
    __IO u16 RESERVED6;
    union {
        __IO u16 FSHR;                                                         ///< SCL High Period Count for Fast Speed Register  offset: 0x1C
        __IO u16 IC_FS_SCL_HCNT;
    };
    __IO u16 RESERVED7;
    union {
        __IO u16 FSLR;                                                         ///< SCL Low Period Count for Fast Speed Register   offset: 0x20
        __IO u16 IC_FS_SCL_LCNT;
    };
    __IO u16 RESERVED8;
    __IO u16 IC_HS_SCL_HCNT_RESERVED;                                      ///< Reserved Register                              offset: 0x24
    __IO u16 RESERVED9;
    __IO u16 IC_HS_SCL_LCNT_RESERVED;                                      ///< Reserved Register                              offset: 0x28
    __IO u16 RESERVED10;
    union {
        __IO u16 ISR;                                                          ///< Interrupt Status Register                      offset: 0x2C
        __IO u16 IC_INTR_STAT;
    };
    __IO u16 RESERVED11;
    union {
        __IO u16 IMR;                                                          ///< Interrupt Mask Register                        offset: 0x30
        __IO u16 IC_INTR_MASK;
    };
    __IO u16 RESERVED12;
    union {
        __IO u16 RAWISR;                                                       ///< RAW Interrupt Status Register                  offset: 0x34
        __IO u16 IC_RAW_INTR_STAT;
    };
    __IO u16 RESERVED13;
    union {
        __IO u16 RXTLR;                                                        ///< Receive FIFO Threshold Level Register          offset: 0x38
        __IO u16 IC_RX_TL;
    };
    __IO u16 RESERVED14;
    union {
        __IO u16 TXTLR;                                                        ///< Transmit FIFO Threshold Level Register         offset: 0x3C
        __IO u16 IC_TX_TL;
    };
    __IO u16 RESERVED15;
    union {
        __IO u16 ICR;                                                          ///< Clear All Interrupt Register                   offset: 0x40
        __IO u16 IC_CLR_INTR;
    };
    __IO u16 RESERVED16;
    union {
        __IO u16 RX_UNDER;                                                     ///< Clear RX_UNDER Interrupt Register              offset: 0x44
        __IO u16 IC_CLR_RX_UNDER;
    };
    __IO u16 RESERVED17;
    union {
        __IO u16 RX_OVER;                                                      ///< Clear RX_OVER Interrupt Register               offset: 0x48
        __IO u16 IC_CLR_RX_OVER;
    };
    __IO u16 RESERVED18;
    union {
        __IO u16 TX_OVER;                                                      ///< Clear TX_OVER Interrupt Register               offset: 0x4C
        __IO u16 IC_CLR_TX_OVER;
    };
    __IO u16 RESERVED19;
    union {
        __IO u16 RD_REQ;                                                       ///< Clear RD_REQ Interrupt Register                offset: 0x50
        __IO u16 IC_CLR_RD_REQ;
    };
    __IO u16 RESERVED20;
    union {
        __IO u16 TX_ABRT;                                                      ///< Clear TX_ABRT Interrupt Register               offset: 0x54
        __IO u16 IC_CLR_TX_ABRT;
    };
    __IO u16 RESERVED21;
    union {
        __IO u16 RX_DONE;                                                      ///< Clear RX_DONE Interrupt Register               offset: 0x58
        __IO u16 IC_CLR_RX_DONE;
    };
    __IO u16 RESERVED22;
    union {
        __IO u16 ACTIV;                                                        ///< Clear ACTIVITY Interrupt Register              offset: 0x5C
        __IO u16 IC_CLR_ACTIVITY;
    };
    __IO u16 RESERVED23;
    union {
        __IO u16 STOP;                                                         ///< Clear STOP_DET Interrupt Register              offset: 0x60
        __IO u16 IC_CLR_STOP_DET;
    };
    __IO u16 RESERVED24;
    union {
        __IO u16 START;                                                        ///< Clear START_DET Interrupt Register             offset: 0x64
        __IO u16 IC_CLR_START_DET;
    };
    __IO u16 RESERVED25;
    union {
        __IO u16 GC;                                                           ///< Clear GEN_CALL Interrupt Register              offset: 0x68
        __IO u16 IC_CLR_GEN_CALL;
    };
    __IO u16 RESERVED26;
    union {
        __IO u16 ENR;                                                          ///< Enable Register                                offset: 0x6C
        __IO u16 IC_ENABLE;
    };
    __IO u16 RESERVED27;
    union {
        __IO u32 SR;                                                           ///< Status Register                                offset: 0x70
        __IO u32 IC_STATUS;
    };
    union {
        __IO u32 TXFLR;                                                        ///< Transmit FIFO Level Register                   offset: 0x74
        __IO u32 IC_TXFLR;
    };
    union {
        __IO u32 RXFLR;                                                        ///< Receive FIFO Level Register                    offset: 0x78
        __IO u32 IC_RXFLR;
    };
    union {
        __IO u32 HOLD;                                                         ///< SDA Hold Time Register                         offset: 0x7C
        __IO u32 IC_SDA_HOLD;
    };
    __IO u32 RESERVED28;                                                   ///IC_TX_ABRT_SOURCE_RESERVED;
    __IO u32 RESERVED29;                                                   ///IC_SLV_DATA_NACK_ONLY_RESERVED;
    union {
        __IO u32 DMA;                                                          ///< DMA Control Register                           offset: 0x88
        __IO u32 IC_DMA_CR;
    };
    __IO u32 RESERVED30;                                                   ///IC_DMA_TDLR_RESERVED;
    __IO u32 RESERVED31;                                                   ///IC_DMA_RDLR_RESERVED;
    union {
        __IO u32 SETUP;                                                        ///< SDA Setup Time Register                        offset: 0x94
        __IO u32 IC_SDA_SETUP;
    };
    union {
        __IO u32 GCR;                                                          ///< ACK General Call Register                      offset: 0x98
        __IO u32 IC_ACK_GENERAL_CALL;
    };
} I2C_TypeDef;
#endif
#ifdef USENNEWREGISTER
typedef struct {
    __IO u16 CR;                                                                ///< Control Register                               offset: 0x00
    __IO u16 RESERVED0;
    __IO u16 TAR;                                                               ///< Target Address Register                        offset: 0x04
    __IO u16 RESERVED1;
    __IO u16 SAR;                                                               ///< Slave Address Register                         offset: 0x08
    __IO u16 RESERVED2;
    __IO u32 RESERVED3;
    __IO u16 DR;                                                                ///< Data Command Register                          offset: 0x10
    __IO u16 RESERVED4;
    __IO u16 SSHR;                                                              ///< SCL High Period Count for Std. Speed Register  offset: 0x14
    __IO u16 RESERVED5;
    __IO u16 SSLR;                                                              ///< SCL Low Period Count for Std. Speed Register   offset: 0x18
    __IO u16 RESERVED6;
    __IO u16 FSHR;                                                              ///< SCL High Period Count for Fast Speed Register  offset: 0x1C
    __IO u16 RESERVED7;
    __IO u16 FSLR;                                                              ///< SCL Low Period Count for Fast Speed Register   offset: 0x20
    __IO u16 RESERVED8;
    __IO u32 RESERVED9;
    __IO u32 RESERVED10;
    __IO u16 ISR;                                                               ///< Interrupt Status Register                      offset: 0x2C
    __IO u16 RESERVED11;
    __IO u16 IMR;                                                               ///< Interrupt Mask Register                        offset: 0x30
    __IO u16 RESERVED12;
    __IO u16 RAWISR;                                                            ///< RAW Interrupt Status Register                  offset: 0x34
    __IO u16 RESERVED13;
    __IO u16 RXTLR;                                                             ///< Receive FIFO Threshold Level Register          offset: 0x38
    __IO u16 RESERVED14;
    __IO u16 TXTLR;                                                             ///< Transmit FIFO Threshold Level Register         offset: 0x3C
    __IO u16 RESERVED15;
    __IO u16 ICR;                                                               ///< Clear All Interrupt Register                   offset: 0x40
    __IO u16 RESERVED16;
    __IO u16 RX_UNDER;                                                          ///< Clear RX_UNDER Interrupt Register              offset: 0x44
    __IO u16 RESERVED17;
    __IO u16 RX_OVER;                                                           ///< Clear RX_OVER Interrupt Register               offset: 0x48
    __IO u16 RESERVED18;
    __IO u16 TX_OVER;                                                           ///< Clear TX_OVER Interrupt Register               offset: 0x4C
    __IO u16 RESERVED19;
    __IO u16 RD_REQ;                                                            ///< Clear RD_REQ Interrupt Register                offset: 0x50
    __IO u16 RESERVED20;
    __IO u16 TX_ABRT;                                                           ///< Clear TX_ABRT Interrupt Register               offset: 0x54
    __IO u16 RESERVED21;
    __IO u16 RX_DONE;                                                           ///< Clear RX_DONE Interrupt Register               offset: 0x58
    __IO u16 RESERVED22;
    __IO u16 ACTIV;                                                             ///< Clear ACTIVITY Interrupt Register              offset: 0x5C
    __IO u16 RESERVED23;
    __IO u16 STOP;                                                              ///< Clear STOP_DET Interrupt Register              offset: 0x60
    __IO u16 RESERVED24;
    __IO u16 START;                                                             ///< Clear START_DET Interrupt Register             offset: 0x64
    __IO u16 RESERVED25;
    __IO u16 GC;                                                                ///< Clear GEN_CALL Interrupt Register              offset: 0x68
    __IO u16 RESERVED26;
    __IO u16 ENR;                                                               ///< Enable Register                                offset: 0x6C
    __IO u16 RESERVED27;
    __IO u32 SR;                                                                ///< Status Register                                offset: 0x70
    __IO u32 TXFLR;                                                             ///< Transmit FIFO Level Register                   offset: 0x74
    __IO u32 RXFLR;                                                             ///< Receive FIFO Level Register                    offset: 0x78
    __IO u32 HOLD;                                                              ///< SDA Hold Time Register                         offset: 0x7C
    __IO u32 RESERVED28;
    __IO u32 RESERVED29;
    __IO u32 DMA;                                                               ///< DMA Control Register                           offset: 0x88
    __IO u32 RESERVED30;
    __IO u32 RESERVED31;
    __IO u32 SETUP;                                                             ///< SDA Setup Time Register                        offset: 0x94
    __IO u32 GCR;                                                               ///< ACK General Call Register                      offset: 0x98
} I2C_TypeDef;
#endif
#ifdef USENOLDREGISTER
typedef struct {
    __IO u16 IC_CON;
    __IO u16 RESERVED0;
    __IO u16 IC_TAR;
    __IO u16 RESERVED1;
    __IO u16 IC_SAR;
    __IO u16 RESERVED2;
    __IO u16 IC_HS_MADDR_RESERVED;
    __IO u16 RESERVED3;
    __IO u16 IC_DATA_CMD;
    __IO u16 RESERVED4;
    __IO u16 IC_SS_SCL_HCNT;
    __IO u16 RESERVED5;
    __IO u16 IC_SS_SCL_LCNT;
    __IO u16 RESERVED6;
    __IO u16 IC_FS_SCL_HCNT;
    __IO u16 RESERVED7;
    __IO u16 IC_FS_SCL_LCNT;
    __IO u16 RESERVED8;
    __IO u16 IC_HS_SCL_HCNT_RESERVED;
    __IO u16 RESERVED9;
    __IO u16 IC_HS_SCL_LCNT_RESERVED;
    __IO u16 RESERVED10;
    __IO u16 IC_INTR_STAT;
    __IO u16 RESERVED11;
    __IO u16 IC_INTR_MASK;
    __IO u16 RESERVED12;
    __IO u16 IC_RAW_INTR_STAT;
    __IO u16 RESERVED13;
    __IO u16 IC_RX_TL;
    __IO u16 RESERVED14;
    __IO u16 IC_TX_TL;
    __IO u16 RESERVED15;
    __IO u16 IC_CLR_INTR;
    __IO u16 RESERVED16;
    __IO u16 IC_CLR_RX_UNDER;
    __IO u16 RESERVED17;
    __IO u16 IC_CLR_RX_OVER;
    __IO u16 RESERVED18;
    __IO u16 IC_CLR_TX_OVER;
    __IO u16 RESERVED19;
    __IO u16 IC_CLR_RD_REQ;
    __IO u16 RESERVED20;
    __IO u16 IC_CLR_TX_ABRT;
    __IO u16 RESERVED21;
    __IO u16 IC_CLR_RX_DONE;
    __IO u16 RESERVED22;
    __IO u16 IC_CLR_ACTIVITY;
    __IO u16 RESERVED23;
    __IO u16 IC_CLR_STOP_DET;
    __IO u16 RESERVED24;
    __IO u16 IC_CLR_START_DET;
    __IO u16 RESERVED25;
    __IO u16 IC_CLR_GEN_CALL;
    __IO u16 RESERVED26;
    __IO u16 IC_ENABLE;
    __IO u16 RESERVED27;
    __IO u32 IC_STATUS;
    __IO u32 IC_TXFLR;
    __IO u32 IC_RXFLR;
    __IO u32 IC_SDA_HOLD;
    __IO u32 IC_TX_ABRT_SOURCE_RESERVED;
    __IO u32 IC_SLV_DATA_NACK_ONLY_RESERVED;
    __IO u32 IC_DMA_CR;
    __IO u32 IC_DMA_TDLR_RESERVED;
    __IO u32 IC_DMA_RDLR_RESERVED;
    __IO u32 IC_SDA_SETUP;
    __IO u32 IC_ACK_GENERAL_CALL;
} I2C_TypeDef;
#endif
////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 KR;                                                                ///< Key Register                                   offset: 0x00
    __IO u32 PR;                                                                ///< Prescaler Register                             offset: 0x04
    __IO u32 RLR;                                                               ///< Reload Register                                offset: 0x08
    __IO u32 SR;                                                                ///< Status Register                                offset: 0x0C
    __IO u32 CR;                                                                ///< Control Register                               offset: 0x10
    __IO u32  IGEN;                                                              ///< Interrupt Generator Register                   offset: 0x14
} IWDG_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief  OPT Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u16 RDP;                                                               ///< Read Protect,                                  offset: 0x00
    __IO u16 USER;                                                              ///< User option byte,                              offset: 0x02
    __IO u16 Data0;                                                             ///< User data 0,                                   offset: 0x04
    __IO u16 Data1;                                                             ///< User data 1,                                   offset: 0x06
    __IO u16 WRP0;                                                              ///< Flash write protection option byte 0,          offset: 0x08
    __IO u16 WRP1;                                                              ///< Flash write protection option byte 1,          offset: 0x0A
    __IO u16 WRP2;                                                              ///< Flash write protection option byte 2,          offset: 0x0C
    __IO u16 WRP3;                                                              ///< Flash write protection option byte 3,          offset: 0x0E
} OB_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief  PROTECT BYTES Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u16 PROTECT_LEN0;                                                      ///< The length of Protect byte 0,                  offset: 0x00
    __IO u16 PROTECT_ADDR0;                                                     ///< Data of Protect byte 0,                        offset: 0x02
    __IO u16 PROTECT_LEN1;                                                      ///< The length of Protect byte 1,                  offset: 0x04
    __IO u16 PROTECT_ADDR1;                                                     ///< Data of Protect byte 1,                        offset: 0x06
    __IO u16 PROTECT_LEN2;                                                      ///< The length of Protect byte 2,                  offset: 0x08
    __IO u16 PROTECT_ADDR2;                                                     ///< Data of Protect byte 2,                        offset: 0x0A
    __IO u16 PROTECT_LEN3;                                                      ///< The length of Protect byte 3,                  offset: 0x0C
    __IO u16 PROTECT_ADDR3;                                                     ///< Data of Protect byte 3,                        offset: 0x0E
} PROTECT_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief PWR Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CR;                                                                ///< Control register                               offset: 0x00
    __IO u32 CSR;                                                               ///< Control Status register                        offset: 0x04
} PWR_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CR;                                                                ///< Control Register                               offset: 0x00
    __IO u32 CFGR;                                                              ///< Configuration Register                         offset: 0x04
    __IO u32 CIR;                                                               ///< Clock Interrupt Register                       offset: 0x08
    __IO u32 APB2RSTR;                                                          ///< Advanced Peripheral Bus 2 Reset Register       offset: 0x0C
    __IO u32 APB1RSTR;                                                          ///< Advanced Peripheral Bus 1 Reset Register       offset: 0x10

    __IO u32 AHBENR;                                                            ///< Advanced High Performance Bus Enable Register  offset: 0x14
    __IO u32 APB2ENR;                                                           ///< Advanced Peripheral Bus 2 Enable Register      offset: 0x18
    __IO u32 APB1ENR;                                                           ///< Advanced Peripheral Bus 1 Enable Register      offset: 0x1C


    __IO u32 BDCR;                                                              ///< Backup Domain Control Register                 offset: 0x20
    __IO u32 CSR;                                                               ///< Control Status Register                        offset: 0x24

    __IO u32 AHBRSTR;                                                           ///< Advanced High Performance Bus Reset Register   offset: 0x28
    __IO u32 CFGR2;                                                             ///< Advanced High Performance Bus Reset Register   offset: 0x2C
    __IO u32 RNG;                                                               ///< Reserved                                       offset: 0x30
    __IO u32 RESERVED4;                                                         ///< Reserved                                       offset: 0x34
    __IO u32 RESERVED5;                                                         ///< Reserved                                       offset: 0x38
    __IO u32 RESERVED6;                                                         ///< Reserved                                       offset: 0x3C
    __IO u32 SYSCFG;                                                            ///< System Configuration Register                  offset: 0x40
    __IO u32 HSIDLY;                                                            ///< HSI delay Register                             offset: 0x44
    __IO u32 HSEDLY;                                                            ///< HSE delay Register                             offset: 0x48
} RCC_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief RTC Registers Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    union {
        __IO u32 CR;                                                                ///< Control Register,                      offset: 0x00
        __IO u32 CRH;
    };
    union {
        __IO u32 CSR;                                                               ///< Control & Status Register,             offset: 0x04
        __IO u32 CRL;
    };
    __IO u32 PRLH;                                                              ///< Prescaler Reload Value High,           offset: 0x08
    __IO u32 PRLL;                                                              ///< Prescaler Reload Value Low,            offset: 0x0C
    __IO u32 DIVH;                                                              ///< Clock Divider High,                    offset: 0x10
    __IO u32 DIVL;                                                              ///< Clock Divider Low,                     offset: 0x14
    __IO u32 CNTH;                                                              ///< Counter High,                          offset: 0x18
    __IO u32 CNTL;                                                              ///< Counter Low,                           offset: 0x1C
    __IO u32 ALRH;                                                              ///< Alarm High,                            offset: 0x20
    __IO u32 ALRL;                                                              ///< Alarm Low,                             offset: 0x24
    __IO u32 MSRH;                                                              ///< Millisecond alarm high register        offset: 0x28
    __IO u32 MSRL;                                                              ///< Millisecond alarm low register         offset: 0x2C
} RTC_TypeDef;
////////////////////////////////////////////////////////////////////////////////
/// @brief SPI Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
#undef USENCOMBINEREGISTER
#undef USENNEWREGISTER
#undef USENOLDREGISTER
#define USENCOMBINEREGISTER
#ifdef USENCOMBINEREGISTER
typedef struct {
    union {
        __IO u32 TDR;                                                               ///< SPI transmit data register,                    offset: 0x00
        __IO u32 TXREG;
    };
    union {
        __IO u32 RDR;                                                               ///< SPI receive data register,                     offset: 0x04
        __IO u32 RXREG;
    };
    union {
        __IO u32 SR;                                                                ///< SPI current state register,                    offset: 0x08
        __IO u32 CSTAT;
    };
    union {
        __IO u32 ISR;                                                               ///< SPI interruput state register,                 offset: 0x0C
        __IO u32 INTSTAT;
    };
    union {
        __IO u32 IER;                                                               ///< SPI interruput enable register,                offset: 0x10
        __IO u32 INTEN;
    };
    union {
        __IO u32 ICR;                                                               ///< SPI interruput control register,               offset: 0x14
        __IO u32 INTCLR;
    };
    union {
        __IO u32 GCR;                                                               ///< SPI global control register,                   offset: 0x18
        __IO u32 GCTL;
    };
    union {
        __IO u32 CCR;                                                               ///< SPI common control register,                   offset: 0x1C
        __IO u32 CCTL;
    };
    union {
        __IO u32 BRR;                                                               ///< SPI baud rate control register,                offset: 0x20
        __IO u32 SPBRG;
    };
    union {
        __IO u32 RDNR;                                                              ///< SPI receive data number register,              offset: 0x24
        __IO u32 RXDNR;
    };
    union {
        __IO u32 NSSR;                                                              ///< SPI chip select register,                      offset: 0x28
        __IO u32 SCSR;
    };
    union {
        __IO u32 ECR;                                                               ///< SPI extand control register,                   offset: 0x2C
        __IO u32 EXTCTL;
    };
} SPI_TypeDef;
#endif
#ifdef USENNEWREGISTER
typedef struct {
    __IO u32 TDR;                                                               ///< SPI transmit data register,                    offset: 0x00
    __IO u32 RDR;                                                               ///< SPI receive data register,                     offset: 0x04
    __IO u32 SR;                                                                ///< SPI current state register,                    offset: 0x08
    __IO u32 ISR;                                                               ///< SPI interruput state register,                 offset: 0x0C
    __IO u32 IER;                                                               ///< SPI interruput enable register,                offset: 0x10
    __IO u32 ICR;                                                               ///< SPI interruput control register,               offset: 0x14
    __IO u32 GCR;                                                               ///< SPI global control register,                   offset: 0x18
    __IO u32 CCR;                                                               ///< SPI common control register,                   offset: 0x1C
    __IO u32 BRR;                                                               ///< SPI baud rate control register,                offset: 0x20
    __IO u32 RDNR;                                                              ///< SPI receive data number register,              offset: 0x24
    __IO u32 NSSR;                                                              ///< SPI chip select register,                      offset: 0x28
    __IO u32 ECR;                                                               ///< SPI extand control register,                   offset: 0x2C
} SPI_TypeDef;
#endif
#ifdef USENOLDREGISTER
typedef struct {
    __IO u32 TXREG;                                                             ///< SPI transmit data register,                    offset: 0x00
    __IO u32 RXREG;                                                             ///< SPI receive data register,                     offset: 0x04
    __IO u32 CSTAT;                                                             ///< SPI current state register,                    offset: 0x08
    __IO u32 INTSTAT;                                                           ///< SPI interruput state register,                 offset: 0x0C
    __IO u32 INTEN;                                                             ///< SPI interruput enable register,                offset: 0x10
    __IO u32 INTCLR;                                                            ///< SPI interruput control register,               offset: 0x14
    __IO u32 GCTL;                                                              ///< SPI global control register,                   offset: 0x18
    __IO u32 CCTL;                                                              ///< SPI common control register,                   offset: 0x1C
    __IO u32 SPBRG;                                                             ///< SPI baud rate control register,                offset: 0x20
    __IO u32 RXDNR;                                                             ///< SPI receive data number register,              offset: 0x24
    __IO u32 NSSR;                                                              ///< SPI chip select register,                      offset: 0x28
    __IO u32 EXTCTL;                                                            ///< SPI extand control register,                   offset: 0x2C
} SPI_TypeDef;
#endif

////////////////////////////////////////////////////////////////////////////////
/// @brief SysTem Configuration Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO uint32_t CFGR;                                                         ///< SYSCFG configuration register                  offset: 0x00
    __IO uint32_t RESERVED0;
    __IO uint32_t EXTICR[4];
} SYSCFG_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief Timer Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 CR1;                                                               ///< TIM control register 1,                        offset: 0x00
    __IO u32 CR2;                                                               ///< TIM control register 2,                        offset: 0x04
    __IO u32 SMCR;                                                              ///< TIM slave Mode Control register,               offset: 0x08
    __IO u32 DIER;                                                              ///< TIM DMA/interrupt enable register,             offset: 0x0C
    __IO u32 SR;                                                                ///< TIM status register,                           offset: 0x10
    __IO u32 EGR;                                                               ///< TIM event generation register,                 offset: 0x14
    __IO u32 CCMR1;                                                             ///< TIM capture/compare mode register 1,           offset: 0x18
    __IO u32 CCMR2;                                                             ///< TIM capture/compare mode register 2,           offset: 0x1C
    __IO u32 CCER;                                                              ///< TIM capture/compare enable register,           offset: 0x20
    __IO u32 CNT;                                                               ///< TIM counter register,                          offset: 0x24
    __IO u32 PSC;                                                               ///< TIM prescaler register,                        offset: 0x28
    __IO u32 ARR;                                                               ///< TIM auto-reload register,                      offset: 0x2C
    __IO u32 RCR;                                                               ///< TIM repetition counter register,               offset: 0x30
    __IO u32 CCR1;                                                              ///< TIM capture/compare register 1,                offset: 0x34
    __IO u32 CCR2;                                                              ///< TIM capture/compare register 2,                offset: 0x38
    __IO u32 CCR3;                                                              ///< TIM capture/compare register 3,                offset: 0x3C
    __IO u32 CCR4;                                                              ///< TIM capture/compare register 4,                offset: 0x40
    __IO u32 BDTR;                                                              ///< TIM break and dead-time register,              offset: 0x44
    __IO u32 DCR;                                                               ///< TIM DMA control register,                      offset: 0x48
    __IO u32 DMAR;                                                              ///< TIM DMA address for full transfer register,    offset: 0x4C
    __IO u32 OR;                                                            ///< Option register,                               offset: 0x50
    __IO u32 CCMR3;                                                         ///< TIM capture/compare mode register 3,           offset: 0x54
    __IO u32 CCR5;                                                          ///< TIM capture/compare register 5,                offset: 0x58
    __IO u32 PDER;                                                          ///< PWM/DMA repeat enable register,                offset: 0x5C
    __IO u32 CCR1FALL;                                                      ///< PWM shift count CCR1 register,                 offset: 0x60
    __IO u32 CCR2FALL;                                                      ///< PWM shift count CCR2 register,                 offset: 0x64
    __IO u32 CCR3FALL;                                                      ///< PWM shift count CCR3 register,                 offset: 0x68
    __IO u32 CCR4FALL;                                                      ///< PWM shift count CCR4 register,                 offset: 0x6c
    __IO u32 CCR5FALL;                                                      ///< PWM shift count CCR5 register,                 offset: 0x70
} TIM_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief UART Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
typedef struct {
    __IO u32 TDR;                                                               ///< Transmit Data Register,                        offset: 0x00
    __IO u32 RDR;                                                               ///< Receive Data Register,                         offset: 0x04
    __IO u32 CSR;                                                               ///< Current Status Register,                       offset: 0x08
    __IO u32 ISR;                                                               ///< Interrupt Status Register,                     offset: 0x0C
    __IO u32 IER;                                                               ///< Interrupt Enable Register,                     offset: 0x10
    __IO u32 ICR;                                                               ///< Interrupt Clear Register,                      offset: 0x14
    __IO u32 GCR;                                                               ///< Global Control Register,                       offset: 0x18
    __IO u32 CCR;                                                               ///< Config Control Register,                       offset: 0x1C
    __IO u32 BRR;                                                               ///< Baud Rate Register,                            offset: 0x20
    __IO u32 FRA;                                                               ///< Fraction Register,                             offset: 0x24

    __IO u32 RXAR;                                                              ///< Receive Address Register,                      offset: 0x28
    __IO u32 RXMR;                                                              ///< Receive Address Mask Register,                 offset: 0x2C
    __IO u32 SCR;                                                               ///< Smart Card Register,                           offset: 0x30

    __IO u32 IDLR;
} UART_TypeDef;

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG Register Structure Definition
////////////////////////////////////////////////////////////////////////////////
#undef USENCOMBINEREGISTER
#undef USENNEWREGISTER
#undef USENOLDREGISTER
#define USENCOMBINEREGISTER
#ifdef USENCOMBINEREGISTER
typedef struct {
    __IO u32 CR;                                                                ///< Control register                               offset: 0x00
    union {
        __IO u32 CFGR;                                                              ///< Configuration register                         offset: 0x04
        __IO uint32_t CFR;
    };
    __IO u32 SR;                                                                ///< Status register                                offset: 0x08
} WWDG_TypeDef;
#endif
#ifdef USENNEWREGISTER
typedef struct {
    __IO u32 CR;                                                                ///< Control register                               offset: 0x00
    __IO u32 CFGR;                                                              ///< Configuration register                         offset: 0x04
    __IO u32 SR;                                                                ///< Status register                                offset: 0x08
} WWDG_TypeDef;
#endif
#ifdef USENOLDREGISTER
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t CFR;
    __IO uint32_t SR;
} WWDG_TypeDef;
#endif
////////////////////////////////////////////////////////////////////////////////
/// @brief ADC type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1                            ((ADC_TypeDef*) ADC1_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief BKP type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define BKP                             ((BKP_TypeDef*) BKP_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief CRC type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define CRC                             ((CRC_TypeDef*) CRC_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief DBGMCU type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define DBGMCU                          ((DBGMCU_TypeDef*) DBG_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief DMA type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA1                            ((DMA_TypeDef*) DMA1_BASE)
#define DMA1_ch1                        ((DMA_Channel_TypeDef*) DMA1_Channel1_BASE)
#define DMA1_ch2                        ((DMA_Channel_TypeDef*) DMA1_Channel2_BASE)
#define DMA1_ch3                        ((DMA_Channel_TypeDef*) DMA1_Channel3_BASE)
#define DMA1_ch4                        ((DMA_Channel_TypeDef*) DMA1_Channel4_BASE)
#define DMA1_ch5                        ((DMA_Channel_TypeDef*) DMA1_Channel5_BASE)

#define DMA1_Channel1                   ((DMA_Channel_TypeDef*) DMA1_Channel1_BASE)
#define DMA1_Channel2                   ((DMA_Channel_TypeDef*) DMA1_Channel2_BASE)
#define DMA1_Channel3                   ((DMA_Channel_TypeDef*) DMA1_Channel3_BASE)
#define DMA1_Channel4                   ((DMA_Channel_TypeDef*) DMA1_Channel4_BASE)
#define DMA1_Channel5                   ((DMA_Channel_TypeDef*) DMA1_Channel5_BASE)



////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI                            ((EXTI_TypeDef*) EXTI_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH                           ((FLASH_TypeDef*) FLASH_REG_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIOA                           ((GPIO_TypeDef*) GPIOA_BASE)
#define GPIOB                           ((GPIO_TypeDef*) GPIOB_BASE)
#define GPIOC                           ((GPIO_TypeDef*) GPIOC_BASE)
#define GPIOD                           ((GPIO_TypeDef*) GPIOD_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C1                            ((I2C_TypeDef*)I2C1_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG                            ((IWDG_TypeDef*) IWDG_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief OPTB type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define OB                              ((OB_TypeDef*) OB_BASE)
#define PROTECT                         ((PROTECT_TypeDef*) PROTECT_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief PWR type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define PWR                             ((PWR_TypeDef*) PWR_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC                             ((RCC_TypeDef*) RCC_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief RTC type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC                         ((RTC_TypeDef*)RTC_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief SPI type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI2                            ((SPI_TypeDef*) SPI2_BASE)
#define SPI1                        ((SPI_TypeDef*) SPI1_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief TIM type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM1                            ((TIM_TypeDef*) TIM1_BASE)
#define TIM2                            ((TIM_TypeDef*) TIM2_BASE)
#define TIM3                            ((TIM_TypeDef*) TIM3_BASE)



#define TIM14                       ((TIM_TypeDef*) TIM14_BASE)
#define TIM16                       ((TIM_TypeDef*) TIM16_BASE)
#define TIM17                       ((TIM_TypeDef*) TIM17_BASE)

#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief UART type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define UART1                           ((UART_TypeDef*) UART1_BASE)
#define UART2                           ((UART_TypeDef*) UART2_BASE)

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG type pointer Definition
////////////////////////////////////////////////////////////////////////////////
#define WWDG                            ((WWDG_TypeDef*) WWDG_BASE)
////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_DR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_DR_DATA_Pos                (0)
#define  ADC_DR_DATA                    (0xFFFFU << ADC_DR_DATA_Pos)            ///< ADC 12bit convert data

#define  ADC_DR_CH_Pos                  (16)
#define  ADC_DR_CH                      (0x0FU <<  ADC_DR_CH_Pos)               ///< CHANNELSEL[19:16] (ADC current channel convert data)

#define  ADC_DR_CH0                     (0x00U << ADC_DR_CH_Pos)                ///< ADC Channel select  0
#define  ADC_DR_CH1                     (0x01U << ADC_DR_CH_Pos)                ///< ADC Channel select  1
#define  ADC_DR_CH2                     (0x02U << ADC_DR_CH_Pos)                ///< ADC Channel select  2
#define  ADC_DR_CH3                     (0x03U << ADC_DR_CH_Pos)                ///< ADC Channel select  3
#define  ADC_DR_CH4                     (0x04U << ADC_DR_CH_Pos)                ///< ADC Channel select  4
#define  ADC_DR_CH5                     (0x05U << ADC_DR_CH_Pos)                ///< ADC Channel select  5
#define  ADC_DR_CH6                     (0x06U << ADC_DR_CH_Pos)                ///< ADC Channel select  6
#define  ADC_DR_CH7                     (0x07U << ADC_DR_CH_Pos)                ///< ADC Channel select  7


#define  ADC_DR_CH8                     (0x08U << ADC_DR_CH_Pos)                ///< ADC Channel select  8
#define  ADC_DR_CH9                     (0x09U << ADC_DR_CH_Pos)                ///< ADC Channel select  9
#define  ADC_DR_TempSensor              (0x0EU << ADC_DR_CH_Pos)                ///< ADC Channel select  14
#define  ADC_DR_VoltRef                 (0x0FU << ADC_DR_CH_Pos)                ///< ADC Channel select  15


#define  ADC_DR_OVERRUN_Pos             (20)
#define  ADC_DR_OVERRUN                 (0x01U << ADC_DR_OVERRUN_Pos)           ///< ADC data will be cover
#define  ADC_DR_VALID_Pos               (21)
#define  ADC_DR_VALID                   (0x01U << ADC_DR_VALID_Pos)             ///< ADC data[11:0] is valid

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CFGR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_CFGR_ADEN_Pos              (0)
#define  ADC_CFGR_ADEN                  (0x01U << ADC_CFGR_ADEN_Pos)            ///< Enable ADC convert
#define  ADC_CFGR_ADWEN_Pos             (1)
#define  ADC_CFGR_ADWEN                 (0x01U << ADC_CFGR_ADWEN_Pos)           ///< Enable ADC window compare


#define  ADC_CFGR_RSLTCTL_Pos           (7)
#define  ADC_CFGR_RSLTCTL               (0x07U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select
#define  ADC_CFGR_RSLTCTL_12            (0x00U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select 12bit
#define  ADC_CFGR_RSLTCTL_11            (0x01U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select 11bit
#define  ADC_CFGR_RSLTCTL_10            (0x02U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select 10bit
#define  ADC_CFGR_RSLTCTL_9             (0x03U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select 9bit
#define  ADC_CFGR_RSLTCTL_8             (0x04U << ADC_CFGR_RSLTCTL_Pos)         ///< ADC resolution select 8bit

#define  ADC_CFGR_SAMCTL_Pos            (10)
#define  ADC_CFGR_SAMCTL                (0x0FU << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select
#define  ADC_CFGR_SAMCTL_1_5            (0x00U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 1.5t
#define  ADC_CFGR_SAMCTL_7_5            (0x01U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 7.5t
#define  ADC_CFGR_SAMCTL_13_5           (0x02U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 13.5t
#define  ADC_CFGR_SAMCTL_28_5           (0x03U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 28.5t
#define  ADC_CFGR_SAMCTL_41_5           (0x04U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 41.5t
#define  ADC_CFGR_SAMCTL_55_5           (0x05U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 55.5t
#define  ADC_CFGR_SAMCTL_71_5           (0x06U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 71.5t
#define  ADC_CFGR_SAMCTL_239_5          (0x07U << ADC_CFGR_SAMCTL_Pos)          ///< ADC sample time select 239.5t
#define ADC_CFGR_SAMCTL_2_5             (0x08U  << ADC_CFGR_SAMCTL_Pos)         ///< ADC sample time select 2.5t
#define ADC_CFGR_SAMCTL_3_5             (0x09U  << ADC_CFGR_SAMCTL_Pos)         ///< ADC sample time select 3.5t
#define ADC_CFGR_SAMCTL_4_5             (0x0AU  << ADC_CFGR_SAMCTL_Pos)         ///< ADC sample time select 4.5t
#define ADC_CFGR_SAMCTL_5_5             (0x0BU  << ADC_CFGR_SAMCTL_Pos)         ///< ADC sample time select 5.5t
#define ADC_CFGR_SAMCTL_6_5             (0x0CU  << ADC_CFGR_SAMCTL_Pos)         ///< ADC sample time select 6.5t

#define  ADC_CFGR_TEN_Pos           (2)
#define  ADC_CFGR_TEN               (0x01U << ADC_CFGR_TEN_Pos)                 ///< Enable ADC temperature sensor
#define  ADC_CFGR_VEN_Pos           (3)
#define  ADC_CFGR_VEN               (0x01U << ADC_CFGR_VEN_Pos)                 ///< Enable ADC voltage reference





#define ADC_CFGR_PRE_Pos            (4)
#define ADC_CFGR_PREL_Pos           (14)
#define ADC_CFGR_PRE                ((0x07U << ADC_CFGR_PRE_Pos) + (0x01U << ADC_CFGR_PREL_Pos))
#define ADC_CFGR_PRE_2              (0x00U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 2
#define ADC_CFGR_PRE_4              (0x01U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 4
#define ADC_CFGR_PRE_6              (0x02U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 6
#define ADC_CFGR_PRE_8              (0x03U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 8
#define ADC_CFGR_PRE_10             (0x04U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 10
#define ADC_CFGR_PRE_12             (0x05U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 12
#define ADC_CFGR_PRE_14             (0x06U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 14
#define ADC_CFGR_PRE_16             (0x07U  << ADC_CFGR_PRE_Pos)            ///< ADC preclk 16
#define ADC_CFGR_PRE_3              ((0x01U << ADC_CFGR_PREL_Pos) + (0x00U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 3
#define ADC_CFGR_PRE_5              ((0x01U << ADC_CFGR_PREL_Pos) + (0x01U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 5
#define ADC_CFGR_PRE_7              ((0x01U << ADC_CFGR_PREL_Pos) + (0x02U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 7
#define ADC_CFGR_PRE_9              ((0x01U << ADC_CFGR_PREL_Pos) + (0x03U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 9
#define ADC_CFGR_PRE_11             ((0x01U << ADC_CFGR_PREL_Pos) + (0x04U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 11
#define ADC_CFGR_PRE_13             ((0x01U << ADC_CFGR_PREL_Pos) + (0x05U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 13
#define ADC_CFGR_PRE_15             ((0x01U << ADC_CFGR_PREL_Pos) + (0x06U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 15
#define ADC_CFGR_PRE_17             ((0x01U << ADC_CFGR_PREL_Pos) + (0x07U << ADC_CFGR_PRE_Pos))    ///< ADC preclk 17






////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_CR_ADIE_Pos                (0)
#define  ADC_CR_ADIE                    (0x01U << ADC_CR_ADIE_Pos)              ///< ADC interrupt enable
#define  ADC_CR_ADWIE_Pos               (1)
#define  ADC_CR_ADWIE                   (0x01U << ADC_CR_ADWIE_Pos)             ///< ADC window compare interrupt enable
#define  ADC_CR_TRGEN_Pos               (2)
#define  ADC_CR_TRGEN                   (0x01U << ADC_CR_TRGEN_Pos)             ///< extranal trigger single start AD convert
#define  ADC_CR_DMAEN_Pos               (3)
#define  ADC_CR_DMAEN                   (0x01U << ADC_CR_DMAEN_Pos)             ///< ADC DMA enable
#define  ADC_CR_ADST_Pos                (8)
#define  ADC_CR_ADST                    (0x01U << ADC_CR_ADST_Pos)              ///< ADC start convert data
#define  ADC_CR_MODE_Pos                (9)
#define  ADC_CR_MODE                    (0x03U << ADC_CR_MODE_Pos)              ///< ADC convert mode
#define  ADC_CR_IMM                     (0x00U << ADC_CR_MODE_Pos)              ///< ADC imm convert mode
#define  ADC_CR_SCAN                    (0x01U << ADC_CR_MODE_Pos)              ///< ADC scan convert mode
#define  ADC_CR_CONTINUE                (0x02U << ADC_CR_MODE_Pos)              ///< ADC continue scan convert mode
#define  ADC_CR_ALIGN_Pos               (11)
#define  ADC_CR_ALIGN                   (0x01U << ADC_CR_ALIGN_Pos)             ///< ADC data align
#define  ADC_CR_LEFT                    (0x01U << ADC_CR_ALIGN_Pos)             ///< ADC data left align
#define  ADC_CR_RIGHT                   (0x00U << ADC_CR_ALIGN_Pos)             ///< ADC data right align
#define  ADC_CR_CMPCH_Pos               (12)
#define  ADC_CR_CMPCH                   (0x0FU << ADC_CR_CMPCH_Pos)             ///< CMPCH[15:12] ADC window compare channel0 convert data
#define  ADC_CR_CMPCH_0                 (0x00U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 0 Conversion Results
#define  ADC_CR_CMPCH_1                 (0x01U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 1 Conversion Results
#define  ADC_CR_CMPCH_2                 (0x02U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 2 Conversion Results
#define  ADC_CR_CMPCH_4                 (0x04U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 4 Conversion Results
#define  ADC_CR_CMPCH_5                 (0x05U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 5 Conversion Results
#define  ADC_CR_CMPCH_6                 (0x06U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 6 Conversion Results
#define  ADC_CR_CMPCH_7                 (0x07U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 7 Conversion Results
#define  ADC_CR_CMPCH_8                 (0x08U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 8 Conversion Results
#define  ADC_CR_CMPCH_9                 (0x09U << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 9 Conversion Results
#define  ADC_CR_CMPCH_10                (0x0AU << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 10 Conversion Results
#define  ADC_CR_CMPCH_11                (0x0BU << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 11 Conversion Results
#define  ADC_CR_CMPCH_13                (0x0DU << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 13 Conversion Results
#define  ADC_CR_CMPCH_14                (0x0EU << ADC_CR_CMPCH_Pos)             ///< Select Compare Channel 14 Conversion Results
#define  ADC_CR_CMPCH_ALL               (0x0FU << ADC_CR_CMPCH_Pos)             ///< Select Compare ALL Channel Conversion Results




#define ADC_CR_TRGSEL_H_Pos         (17)
#define ADC_CR_TRGSEL_L_Pos         (4)
#define ADC_CR_TRGSEL               ((0x03U << ADC_CR_TRGSEL_H_Pos) + (0x07U << ADC_CR_TRGSEL_L_Pos))   ///< TRGSEL[6:4][18:17] ADC external trigger source select
#define ADC_CR_T1_CC1               (0x00U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T1_CC1
#define ADC_CR_T1_CC2               (0x01U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T1_CC2
#define ADC_CR_T1_CC3               (0x02U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T1_CC3
#define ADC_CR_T2_CC2               (0x03U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T2_CC2
#define ADC_CR_T3_TRIG              (0x04U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T3_TRIG
#define ADC_CR_T1_CC4_CC5           (0x05U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T1_CC4_CC5
#define ADC_CR_T3_CC1               (0x06U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is T3_CC1
#define ADC_CR_EXTI_11              (0x07U << ADC_CR_TRGSEL_L_Pos)          ///< The external trigger source of the ADC is EXTI_11
#define ADC_CR_T1_TRIG              ((0x01U << ADC_CR_TRGSEL_H_Pos) + (0x00U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is T1_TRIG
#define ADC_CR_T2_CC1               ((0x01U << ADC_CR_TRGSEL_H_Pos) + (0x03U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is T2_CC1
#define ADC_CR_T3_CC4               ((0x01U << ADC_CR_TRGSEL_H_Pos) + (0x04U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is T3_CC4
#define ADC_CR_T2_TRIG              ((0x01U << ADC_CR_TRGSEL_H_Pos) + (0x05U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is T2_TRIG
#define ADC_CR_EXTI_15              ((0x01U << ADC_CR_TRGSEL_H_Pos) + (0x07U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is EXTI_15
#define ADC_CR_TIM1_CC4             ((0x02U << ADC_CR_TRGSEL_H_Pos) + (0x00U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is TIM1_CC4
#define ADC_CR_TIM1_CC5             ((0x02U << ADC_CR_TRGSEL_H_Pos) + (0x01U << ADC_CR_TRGSEL_L_Pos))   ///< The external trigger source of the ADC is TIM1_CC5

#define ADC_CR_SCANDIR_Pos          (16)
#define ADC_CR_SCANDIR              (0x01U << ADC_CR_SCANDIR_Pos)           ///< ADC scan direction

#define ADC_CR_TRGSHIFT_Pos         (19)
#define ADC_CR_TRGSHIFT             (0x07U << ADC_CR_TRGSHIFT_Pos)          ///< External trigger shift sample
#define ADC_CR_TRGSHIFT_0           (0x00U << ADC_CR_TRGSHIFT_Pos)          ///< No shift
#define ADC_CR_TRGSHIFT_4           (0x01U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 4 period
#define ADC_CR_TRGSHIFT_16          (0x02U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 16 period
#define ADC_CR_TRGSHIFT_32          (0x03U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 32 period
#define ADC_CR_TRGSHIFT_64          (0x04U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 64 period
#define ADC_CR_TRGSHIFT_128         (0x05U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 128 period
#define ADC_CR_TRGSHIFT_256         (0x06U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 256 period
#define ADC_CR_TRGSHIFT_512         (0x07U << ADC_CR_TRGSHIFT_Pos)          ///< Shift 512 period
#define  ADC_CR_TRG_EDGE_Pos        (24)
#define  ADC_CR_TRG_EDGE            (0x03U << ADC_CR_TRG_EDGE_Pos)          ///< ADC trig edge config
#define  ADC_CR_TRG_EDGE_DUAL       (0x00U << ADC_CR_TRG_EDGE_Pos)          ///< ADC dual edge trig mode
#define  ADC_CR_TRG_EDGE_DOWN       (0x01U << ADC_CR_TRG_EDGE_Pos)          ///< ADC down edge trig mode
#define  ADC_CR_TRG_EDGE_UP         (0x02U << ADC_CR_TRG_EDGE_Pos)          ///< ADC up   edge trig mode
#define  ADC_CR_TRG_EDGE_MASK       (0x03U << ADC_CR_TRG_EDGE_Pos)          ///< ADC mask edge trig mode

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_CHSR_CH0_Pos               (0)
#define  ADC_CHSR_CH0                   (0x01U << ADC_CHSR_CH0_Pos)             ///< Enable ADC channel 0
#define  ADC_CHSR_CH1_Pos               (1)
#define  ADC_CHSR_CH1                   (0x01U << ADC_CHSR_CH1_Pos)             ///< Enable ADC channel 1
#define  ADC_CHSR_CH2_Pos               (2)
#define  ADC_CHSR_CH2                   (0x01U << ADC_CHSR_CH2_Pos)             ///< Enable ADC channel 2
#define  ADC_CHSR_CH3_Pos               (3)
#define  ADC_CHSR_CH3                   (0x01U << ADC_CHSR_CH3_Pos)             ///< Enable ADC channel 3
#define  ADC_CHSR_CH4_Pos               (4)
#define  ADC_CHSR_CH4                   (0x01U << ADC_CHSR_CH4_Pos)             ///< Enable ADC channel 4
#define  ADC_CHSR_CH5_Pos               (5)
#define  ADC_CHSR_CH5                   (0x01U << ADC_CHSR_CH5_Pos)             ///< Enable ADC channel 5
#define  ADC_CHSR_CH6_Pos               (6)
#define  ADC_CHSR_CH6                   (0x01U << ADC_CHSR_CH6_Pos)             ///< Enable ADC channel 6
#define  ADC_CHSR_CH7_Pos               (7)
#define  ADC_CHSR_CH7                   (0x01U << ADC_CHSR_CH7_Pos)             ///< Enable ADC channel 7


#define  ADC_CHSR_CH8_Pos           (8)
#define  ADC_CHSR_CH8               (0x01U << ADC_CHSR_CH8_Pos)             ///< Enable ADC channel 8
#define  ADC_CHSR_CH9_Pos           (9)
#define  ADC_CHSR_CH9               (0x01U << ADC_CHSR_CH9_Pos)             ///< Enable ADC channel 9
#define  ADC_CHSR_CHT_Pos           (14)
#define  ADC_CHSR_CHT               (0x01U << ADC_CHSR_CHT_Pos)             ///< Enable Temperature Sensor
#define  ADC_CHSR_CHV_Pos           (15)
#define  ADC_CHSR_CHV               (0x01U << ADC_CHSR_CHV_Pos)             ///< Enable Voltage Sensor

#define  ADC_CHSR_CHALL             (0xC3FFU)                               ///< Enable ADC all channel

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CMPR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_CMPR_CMPLDATA_Pos          (0)
#define  ADC_CMPR_CMPLDATA              (0x0FFFU << ADC_CMPR_CMPLDATA_Pos)      ///< ADC 12bit window compare DOWN LEVEL DATA
#define  ADC_CMPR_CMPHDATA_Pos          (16)
#define  ADC_CMPR_CMPHDATA              (0x0FFFU << ADC_CMPR_CMPHDATA_Pos)      ///< ADC 12bit window compare UP LEVEL DATA

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_SR_ADIF_Pos                (0)
#define  ADC_SR_ADIF                    (0x01U << ADC_SR_ADIF_Pos)              ///< ADC convert complete flag
#define  ADC_SR_ADWIF_Pos               (1)
#define  ADC_SR_ADWIF                   (0x01U << ADC_SR_ADWIF_Pos)             ///< ADC compare flag
#define  ADC_SR_BUSY_Pos                (2)
#define  ADC_SR_BUSY                    (0x01U << ADC_SR_BUSY_Pos)              ///< ADC busy flag
#define  ADC_SR_CH_Pos                  (4)
#define  ADC_SR_CH                      (0x0FU << ADC_SR_CH_Pos)                ///< CHANNEL[7:4] ADC current channel
#define  ADC_SR_CH0                     (0x00U << ADC_SR_CH_Pos)                ///< Channel 0 is the current conversion channel
#define  ADC_SR_CH1                     (0x01U << ADC_SR_CH_Pos)                ///< Channel 1 is the current conversion channel
#define  ADC_SR_CH2                     (0x02U << ADC_SR_CH_Pos)                ///< Channel 2 is the current conversion channel
#define  ADC_SR_CH3                     (0x03U << ADC_SR_CH_Pos)                ///< Channel 3 is the current conversion channel
#define  ADC_SR_CH4                     (0x04U << ADC_SR_CH_Pos)                ///< Channel 4 is the current conversion channel
#define  ADC_SR_CH5                     (0x05U << ADC_SR_CH_Pos)                ///< Channel 5 is the current conversion channel
#define  ADC_SR_CH6                     (0x06U << ADC_SR_CH_Pos)                ///< Channel 6 is the current conversion channel
#define  ADC_SR_CH7                     (0x07U << ADC_SR_CH_Pos)                ///< Channel 7 is the current conversion channel
#define  ADC_SR_CH8                     (0x08U << ADC_SR_CH_Pos)                ///< Channel 8 is the current conversion channel
#define  ADC_SR_CH9                     (0x09U << ADC_SR_CH_Pos)                ///< Channel 9 is the current conversion channel
#define  ADC_SR_CH10                    (0x0AU << ADC_SR_CH_Pos)                ///< Channel 10 is the current conversion channel
#define  ADC_SR_CH11                    (0x0BU << ADC_SR_CH_Pos)                ///< Channel 11 is the current conversion channel
#define  ADC_SR_CH13                    (0x0DU << ADC_SR_CH_Pos)                ///< Channel 13 is the current conversion channel
#define  ADC_SR_CH14                    (0x0EU << ADC_SR_CH_Pos)                ///< Channel 14 is the current conversion channel
#define  ADC_SR_CH15                    (0x0FU << ADC_SR_CH_Pos)                ///< Channel 15 is the current conversion channel


#define  ADC_SR_VALID_Pos           (8)
#define  ADC_SR_VALID               (0x0FFFU << ADC_SR_VALID_Pos)           ///< VALID[19:8] ADC channel 0..11 valid flag
#define  ADC_SR_OVERRUN_Pos         (20)
#define  ADC_SR_OVERRUN             (0x0FFFU << ADC_SR_OVERRUN_Pos)         ///< OVERRUN[31:20] ADC channel 0..11 data covered flag

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHnDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_CHDR_DATA_Pos          (0)
#define  ADC_CHDR_DATA              (0xFFFFU << ADC_CHDR_DATA_Pos)          ///< ADC channel convert data
#define  ADC_CHDR_OVERRUN_Pos       (20)
#define  ADC_CHDR_OVERRUN           (0x01U << ADC_CHDR_OVERRUN_Pos)         ///< ADC data covered flag
#define  ADC_CHDR_VALID_Pos         (21)
#define  ADC_CHDR_VALID             (0x01U << ADC_CHDR_VALID_Pos)           ///< ADC data valid flag


////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_SREXT Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define  ADC_SREXT_VALID_Pos        (2)
#define  ADC_SREXT_VALID            (0x03U << ADC_SREXT_VALID_Pos)          ///< VALID[3:2] ADC channel 14..15 valid flag
#define  ADC_SREXT_OVERRUN_Pos      (6)
#define  ADC_SREXT_OVERRUN          (0x03U << ADC_SREXT_OVERRUN_Pos)        ///< OVERRUN[7:6] ADC channel 14..15 data covered flag

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHANY0 select Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1_CHANY0_SEL0_Pos        (0)                                     ///< CHANY_SEL0 (Bit 0)
#define ADC1_CHANY0_SEL0            (0x0FU << ADC1_CHANY0_SEL0_Pos)         ///< CHANY_SEL0 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL1_Pos        (4)                                     ///< CHANY_SEL1 (Bit 4)
#define ADC1_CHANY0_SEL1            (0x0FU << ADC1_CHANY0_SEL1_Pos)         ///< CHANY_SEL1 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL2_Pos        (8)                                     ///< CHANY_SEL2 (Bit 8)
#define ADC1_CHANY0_SEL2            (0x0FU << ADC1_CHANY0_SEL2_Pos)         ///< CHANY_SEL2 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL3_Pos        (12)                                    ///< CHANY_SEL3 (Bit 12)
#define ADC1_CHANY0_SEL3            (0x0FU << ADC1_CHANY0_SEL3_Pos)         ///< CHANY_SEL3 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL4_Pos        (16)                                    ///< CHANY_SEL4 (Bit 16)
#define ADC1_CHANY0_SEL4            (0x0FU << ADC1_CHANY0_SEL4_Pos)         ///< CHANY_SEL4 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL5_Pos        (20)                                    ///< CHANY_SEL5 (Bit 20)
#define ADC1_CHANY0_SEL5            (0x0FU << ADC1_CHANY0_SEL5_Pos)         ///< CHANY_SEL5 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL6_Pos        (24)                                    ///< CHANY_SEL6 (Bit 24)
#define ADC1_CHANY0_SEL6            (0x0FU << ADC1_CHANY0_SEL6_Pos)         ///< CHANY_SEL6 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY0_SEL7_Pos        (28)                                    ///< CHANY_SEL7 (Bit 28)
#define ADC1_CHANY0_SEL7            (0x0FU << ADC1_CHANY0_SEL7_Pos)         ///< CHANY_SEL7 (Bitfield-Mask: 0x0f)

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHANY0 select Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1_CHANY1_SEL8_Pos        (0)                                     ///< CHANY_SEL8 (Bit 0)
#define ADC1_CHANY1_SEL8            (0x0FU << ADC1_CHANY1_SEL8_Pos)         ///< CHANY_SEL8 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY1_SEL9_Pos        (4)                                     ///< CHANY_SEL9 (Bit 4)
#define ADC1_CHANY1_SEL9            (0x0FU << ADC1_CHANY1_SEL9_Pos)         ///< CHANY_SEL9 (Bitfield-Mask: 0x0f)

#define ADC1_CHANY1_SEL14_Pos       (24)                                    ///< CHANY_SEL14 (Bit 24)
#define ADC1_CHANY1_SEL14           (0x0FU << ADC1_CHANY1_SEL14_Pos)        ///< CHANY_SEL14 (Bitfield-Mask: 0x0f)
#define ADC1_CHANY1_SEL15_Pos       (28)                                    ///< CHANY_SEL15 (Bit 28)
#define ADC1_CHANY1_SEL15           (0x0FU << ADC1_CHANY1_SEL15_Pos)        ///< CHANY_SEL15 (Bitfield-Mask: 0x0f)

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHANY config number Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1_CHANY_CFG_NUM_Max      (16)                                    ///< CHANY_CFG_NUM Max Value is 16

////////////////////////////////////////////////////////////////////////////////
/// @brief ADC_CHANY mode enable Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define ADC1_CHANY_CR_MDEN_Pos      (0)                                     ///< CHANY_MDEN (Bit 0)
#define ADC1_CHANY_CR_MDEN          (0x01U << ADC1_CHANY_CR_MDEN_Pos)       ///< CHANY_MDEN (Bitfield-Mask: 0x01)

////////////////////////////////////////////////////////////////////////////////
/// @brief BKP_DRn Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define BKP_DR_DATA_Pos                 (0)
#define BKP_DR_DATA                     (0xFFFFU << BKP_DR_DATA)                ///< Backup data

////////////////////////////////////////////////////////////////////////////////
/// @brief BKP_RTCCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define BKP_RTCCR_CAL_Pos           (0)
#define BKP_RTCCR_CAL               (0x7FU << BKP_RTCCR_CAL_Pos)            ///< Calibration value
#define BKP_RTCCR_CCO_Pos           (7)
#define BKP_RTCCR_CCO               (0x01U << BKP_RTCCR_CCO_Pos)            ///< Calibration Clock Output
#define BKP_RTCCR_ASOE_Pos          (8)
#define BKP_RTCCR_ASOE              (0x01U << BKP_RTCCR_ASOE_Pos)           ///< Alarm or Second Output Enable
#define BKP_RTCCR_ASOS_Pos          (9)
#define BKP_RTCCR_ASOS              (0x01U << BKP_RTCCR_ASOS_Pos)           ///< Alarm or Second Output Selection

////////////////////////////////////////////////////////////////////////////////
/// @brief BKP_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define BKP_CR_TPE_Pos              (0)
#define BKP_CR_TPE                  (0x01U << BKP_CR_TPE_Pos)               ///< TAMPER pin enable
#define BKP_CR_TPAL_Pos             (1)
#define BKP_CR_TPAL                 (0x01U << BKP_CR_TPAL_Pos)              ///< TAMPER pin active level

////////////////////////////////////////////////////////////////////////////////
/// @brief BKP_CSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define BKP_CSR_CTE_Pos             (0)
#define BKP_CSR_CTE                 (0x01U << BKP_CSR_CTE_Pos)              ///< Clear Tamper event
#define BKP_CSR_CTI_Pos             (1)
#define BKP_CSR_CTI                 (0x01U << BKP_CSR_CTI_Pos)              ///< Clear Tamper Interrupt
#define BKP_CSR_TPIE_Pos            (2)
#define BKP_CSR_TPIE                (0x01U << BKP_CSR_TPIE_Pos)             ///< TAMPER Pin interrupt enable
#define BKP_CSR_TEF_Pos             (8)
#define BKP_CSR_TEF                 (0x01U << BKP_CSR_TEF_Pos)              ///< Tamper Event Flag
#define BKP_CSR_TIF_Pos             (9)
#define BKP_CSR_TIF                 (0x01U << BKP_CSR_TIF_Pos)              ///< Tamper Interrupt Flag
////////////////////////////////////////////////////////////////////////////////
/// @brief CRC_DR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define CRC_DR_DATA_Pos                 (0)
#define CRC_DR_DATA                     (0xFFFFFFFFU << CRC_DR_DATA_Pos)        ///< Data register bits
////////////////////////////////////////////////////////////////////////////////
/// @brief CRC_IDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define CRC_IDR_DATA_Pos                (0)
#define CRC_IDR_DATA                    (0xFFU << CRC_IDR_DATA_Pos)             ///< General-purpose 8-bit data register bits

////////////////////////////////////////////////////////////////////////////////
/// @brief CRC_CTRL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define CRC_CR_RESET_Pos                (0)
#define CRC_CR_RESET                    (0x01U << CRC_CR_RESET_Pos)             ///< RESET bit
////////////////////////////////////////////////////////////////////////////////
/// @brief DBGMCU_IDCODE Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DBGMCU_IDCODE_DEV_ID_Pos        (0)
#define DBGMCU_IDCODE_DEV_ID            (0xFFFFFFFFU << DBGMCU_IDCODE_DEV_ID_Pos)   ///< Device identifier

////////////////////////////////////////////////////////////////////////////////
/// @brief DBGMCU_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DBGMCU_CR_SLEEP_Pos             (0)
#define DBGMCU_CR_SLEEP                 (0x01U << DBGMCU_CR_SLEEP_Pos)          ///< Debug Sleep mode
#define DBGMCU_CR_STOP_Pos              (1)
#define DBGMCU_CR_STOP                  (0x01U << DBGMCU_CR_STOP_Pos)           ///< Debug Stop mode
#define DBGMCU_CR_STANDBY_Pos           (2)
#define DBGMCU_CR_STANDBY               (0x01U << DBGMCU_CR_STANDBY_Pos)        ///< Debug Standby mode
#define DBGMCU_CR_IWDG_STOP_Pos         (8)
#define DBGMCU_CR_IWDG_STOP             (0x01U << DBGMCU_CR_IWDG_STOP_Pos)      ///< Debug independent watchdog stopped when core is halted
#define DBGMCU_CR_WWDG_STOP_Pos         (9)
#define DBGMCU_CR_WWDG_STOP             (0x01U << DBGMCU_CR_WWDG_STOP_Pos)      ///< Debug window watchdog stopped when core is halted
#define DBGMCU_CR_TIM_STOP_Pos          (10)
#define DBGMCU_CR_TIM1_STOP             (0x01U << DBGMCU_CR_TIM_STOP_Pos)       ///< TIM1 counter stopped when core is halted
#define DBGMCU_CR_TIM2_STOP             (0x02U << DBGMCU_CR_TIM_STOP_Pos)       ///< TIM2 counter stopped when core is halted
#define DBGMCU_CR_TIM3_STOP             (0x04U << DBGMCU_CR_TIM_STOP_Pos)       ///< TIM3 counter stopped when core is halted

////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_ISR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_ISR_GIF1_Pos                (0)
#define DMA_ISR_GIF1                    (0x01U << DMA_ISR_GIF1_Pos)             ///< Channel 1 Global interrupt flag
#define DMA_ISR_TCIF1_Pos               (1)
#define DMA_ISR_TCIF1                   (0x01U << DMA_ISR_TCIF1_Pos)            ///< Channel 1 Transfer Complete flag
#define DMA_ISR_HTIF1_Pos               (2)
#define DMA_ISR_HTIF1                   (0x01U << DMA_ISR_HTIF1_Pos)            ///< Channel 1 Half Transfer flag
#define DMA_ISR_TEIF1_Pos               (3)
#define DMA_ISR_TEIF1                   (0x01U << DMA_ISR_TEIF1_Pos)            ///< Channel 1 Transfer Error flag
#define DMA_ISR_GIF2_Pos                (4)
#define DMA_ISR_GIF2                    (0x01U << DMA_ISR_GIF2_Pos)             ///< Channel 2 Global interrupt flag
#define DMA_ISR_TCIF2_Pos               (5)
#define DMA_ISR_TCIF2                   (0x01U << DMA_ISR_TCIF2_Pos)            ///< Channel 2 Transfer Complete flag
#define DMA_ISR_HTIF2_Pos               (6)
#define DMA_ISR_HTIF2                   (0x01U << DMA_ISR_HTIF2_Pos)            ///< Channel 2 Half Transfer flag
#define DMA_ISR_TEIF2_Pos               (7)
#define DMA_ISR_TEIF2                   (0x01U << DMA_ISR_TEIF2_Pos)            ///< Channel 2 Transfer Error flag
#define DMA_ISR_GIF3_Pos                (8)
#define DMA_ISR_GIF3                    (0x01U << DMA_ISR_GIF3_Pos)             ///< Channel 3 Global interrupt flag
#define DMA_ISR_TCIF3_Pos               (9)
#define DMA_ISR_TCIF3                   (0x01U << DMA_ISR_TCIF3_Pos)            ///< Channel 3 Transfer Complete flag
#define DMA_ISR_HTIF3_Pos               (10)
#define DMA_ISR_HTIF3                   (0x01U << DMA_ISR_HTIF3_Pos)            ///< Channel 3 Half Transfer flag
#define DMA_ISR_TEIF3_Pos               (11)
#define DMA_ISR_TEIF3                   (0x01U << DMA_ISR_TEIF3_Pos)            ///< Channel 3 Transfer Error flag
#define DMA_ISR_GIF4_Pos                (12)
#define DMA_ISR_GIF4                    (0x01U << DMA_ISR_GIF4_Pos)             ///< Channel 4 Global interrupt flag
#define DMA_ISR_TCIF4_Pos               (13)
#define DMA_ISR_TCIF4                   (0x01U << DMA_ISR_TCIF4_Pos)            ///< Channel 4 Transfer Complete flag
#define DMA_ISR_HTIF4_Pos               (14)
#define DMA_ISR_HTIF4                   (0x01U << DMA_ISR_HTIF4_Pos)            ///< Channel 4 Half Transfer flag
#define DMA_ISR_TEIF4_Pos               (15)
#define DMA_ISR_TEIF4                   (0x01U << DMA_ISR_TEIF4_Pos)            ///< Channel 4 Transfer Error flag
#define DMA_ISR_GIF5_Pos                (16)
#define DMA_ISR_GIF5                    (0x01U << DMA_ISR_GIF5_Pos)             ///< Channel 5 Global interrupt flag
#define DMA_ISR_TCIF5_Pos               (17)
#define DMA_ISR_TCIF5                   (0x01U << DMA_ISR_TCIF5_Pos)            ///< Channel 5 Transfer Complete flag
#define DMA_ISR_HTIF5_Pos               (18)
#define DMA_ISR_HTIF5                   (0x01U << DMA_ISR_HTIF5_Pos)            ///< Channel 5 Half Transfer flag
#define DMA_ISR_TEIF5_Pos               (19)
#define DMA_ISR_TEIF5                   (0x01U << DMA_ISR_TEIF5_Pos)            ///< Channel 5 Transfer Error flag


////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_IFCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_IFCR_CGIF1_Pos              (0)
#define DMA_IFCR_CGIF1                  (0x01U << DMA_IFCR_CGIF1_Pos)           ///< Channel 1 Global interrupt clearr
#define DMA_IFCR_CTCIF1_Pos             (1)
#define DMA_IFCR_CTCIF1                 (0x01U << DMA_IFCR_CTCIF1_Pos)          ///< Channel 1 Transfer Complete clear
#define DMA_IFCR_CHTIF1_Pos             (2)
#define DMA_IFCR_CHTIF1                 (0x01U << DMA_IFCR_CHTIF1_Pos)          ///< Channel 1 Half Transfer clear
#define DMA_IFCR_CTEIF1_Pos             (3)
#define DMA_IFCR_CTEIF1                 (0x01U << DMA_IFCR_CTEIF1_Pos)          ///< Channel 1 Transfer Error clear
#define DMA_IFCR_CGIF2_Pos              (4)
#define DMA_IFCR_CGIF2                  (0x01U << DMA_IFCR_CGIF2_Pos)           ///< Channel 2 Global interrupt clear
#define DMA_IFCR_CTCIF2_Pos             (5)
#define DMA_IFCR_CTCIF2                 (0x01U << DMA_IFCR_CTCIF2_Pos)          ///< Channel 2 Transfer Complete clear
#define DMA_IFCR_CHTIF2_Pos             (6)
#define DMA_IFCR_CHTIF2                 (0x01U << DMA_IFCR_CHTIF2_Pos)          ///< Channel 2 Half Transfer clear
#define DMA_IFCR_CTEIF2_Pos             (7)
#define DMA_IFCR_CTEIF2                 (0x01U << DMA_IFCR_CTEIF2_Pos)          ///< Channel 2 Transfer Error clear
#define DMA_IFCR_CGIF3_Pos              (8)
#define DMA_IFCR_CGIF3                  (0x01U << DMA_IFCR_CGIF3_Pos)           ///< Channel 3 Global interrupt clear
#define DMA_IFCR_CTCIF3_Pos             (9)
#define DMA_IFCR_CTCIF3                 (0x01U << DMA_IFCR_CTCIF3_Pos)          ///< Channel 3 Transfer Complete clear
#define DMA_IFCR_CHTIF3_Pos             (10)
#define DMA_IFCR_CHTIF3                 (0x01U << DMA_IFCR_CHTIF3_Pos)          ///< Channel 3 Half Transfer clear
#define DMA_IFCR_CTEIF3_Pos             (11)
#define DMA_IFCR_CTEIF3                 (0x01U << DMA_IFCR_CTEIF3_Pos)          ///< Channel 3 Transfer Error clear
#define DMA_IFCR_CGIF4_Pos              (12)
#define DMA_IFCR_CGIF4                  (0x01U << DMA_IFCR_CGIF4_Pos)           ///< Channel 4 Global interrupt clear
#define DMA_IFCR_CTCIF4_Pos             (13)
#define DMA_IFCR_CTCIF4                 (0x01U << DMA_IFCR_CTCIF4_Pos)          ///< Channel 4 Transfer Complete clear
#define DMA_IFCR_CHTIF4_Pos             (14)
#define DMA_IFCR_CHTIF4                 (0x01U << DMA_IFCR_CHTIF4_Pos)          ///< Channel 4 Half Transfer clear
#define DMA_IFCR_CTEIF4_Pos             (15)
#define DMA_IFCR_CTEIF4                 (0x01U << DMA_IFCR_CTEIF4_Pos)          ///< Channel 4 Transfer Error clear
#define DMA_IFCR_CGIF5_Pos              (16)
#define DMA_IFCR_CGIF5                  (0x01U << DMA_IFCR_CGIF5_Pos)           ///< Channel 5 Global interrupt clear
#define DMA_IFCR_CTCIF5_Pos             (17)
#define DMA_IFCR_CTCIF5                 (0x01U << DMA_IFCR_CTCIF5_Pos)          ///< Channel 5 Transfer Complete clear
#define DMA_IFCR_CHTIF5_Pos             (18)
#define DMA_IFCR_CHTIF5                 (0x01U << DMA_IFCR_CHTIF5_Pos)          ///< Channel 5 Half Transfer clear
#define DMA_IFCR_CTEIF5_Pos             (19)
#define DMA_IFCR_CTEIF5                 (0x01U << DMA_IFCR_CTEIF5_Pos)          ///< Channel 5 Transfer Error clear


////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_CCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_CCR_EN_Pos                  (0)
#define DMA_CCR_EN                      (0x01U << DMA_CCR_EN_Pos)               ///< Channel enabl
#define DMA_CCR_TCIE_Pos                (1)
#define DMA_CCR_TCIE                    (0x01U << DMA_CCR_TCIE_Pos)             ///< Transfer complete interrupt enable
#define DMA_CCR_HTIE_Pos                (2)
#define DMA_CCR_HTIE                    (0x01U << DMA_CCR_HTIE_Pos)             ///< Half Transfer interrupt enable
#define DMA_CCR_TEIE_Pos                (3)
#define DMA_CCR_TEIE                    (0x01U << DMA_CCR_TEIE_Pos)             ///< Transfer error interrupt enable
#define DMA_CCR_DIR_Pos                 (4)
#define DMA_CCR_DIR                     (0x01U << DMA_CCR_DIR_Pos)              ///< Data transfer direction
#define DMA_CCR_CIRC_Pos                (5)
#define DMA_CCR_CIRC                    (0x01U << DMA_CCR_CIRC_Pos)             ///< Circular mode
#define DMA_CCR_PINC_Pos                (6)
#define DMA_CCR_PINC                    (0x01U << DMA_CCR_PINC_Pos)             ///< Peripheral increment mode
#define DMA_CCR_MINC_Pos                (7)
#define DMA_CCR_MINC                    (0x01U << DMA_CCR_MINC_Pos)             ///< Memory increment mode

#define DMA_CCR_PSIZE_Pos               (8)
#define DMA_CCR_PSIZE                   (0x03U << DMA_CCR_PSIZE_Pos)            ///< PSIZE[1:0] bits (Peripheral size)
#define DMA_CCR_PSIZE_0                 (0x01U << DMA_CCR_PSIZE_Pos)            ///< Bit0
#define DMA_CCR_PSIZE_1                 (0x02U << DMA_CCR_PSIZE_Pos)            ///< Bit1

#define DMA_CCR_PSIZE_BYTE              (0x00U << DMA_CCR_PSIZE_Pos)            ///< DMA Peripheral Data Size Byte
#define DMA_CCR_PSIZE_HALFWORD          (0x01U << DMA_CCR_PSIZE_Pos)            ///< DMA Peripheral Data Size HalfWord
#define DMA_CCR_PSIZE_WORD              (0x02U << DMA_CCR_PSIZE_Pos)            ///< DMA Peripheral Data Size Word

#define DMA_CCR_MSIZE_Pos               (10)
#define DMA_CCR_MSIZE                   (0x03U << DMA_CCR_MSIZE_Pos)            ///< MSIZE[1:0] bits (Memory size)
#define DMA_CCR_MSIZE_0                 (0x01U << DMA_CCR_MSIZE_Pos)            ///< Bit0
#define DMA_CCR_MSIZE_1                 (0x02U << DMA_CCR_MSIZE_Pos)            ///< Bit1

#define DMA_CCR_MSIZE_BYTE              (0x00U << DMA_CCR_MSIZE_Pos)            ///< DMA Memory Data Size Byte
#define DMA_CCR_MSIZE_HALFWORD          (0x01U << DMA_CCR_MSIZE_Pos)            ///< DMA Memory Data Size HalfWord
#define DMA_CCR_MSIZE_WORD              (0x02U << DMA_CCR_MSIZE_Pos)            ///< DMA Memory Data Size Word

#define DMA_CCR_PL_Pos                  (12)
#define DMA_CCR_PL                      (0x03U << DMA_CCR_PL_Pos)               ///< PL[1:0] bits(Channel Priority level)
#define DMA_CCR_PL_0                    (0x01U << DMA_CCR_PL_Pos)               ///< Bit0
#define DMA_CCR_PL_1                    (0x02U << DMA_CCR_PL_Pos)               ///< Bit1

#define DMA_CCR_PL_Low                  (0x00U << DMA_CCR_PL_Pos)               ///< DMA Priority Low
#define DMA_CCR_PL_Medium               (0x01U << DMA_CCR_PL_Pos)               ///< DMA Priority Medium
#define DMA_CCR_PL_High                 (0x02U << DMA_CCR_PL_Pos)               ///< DMA Priority High
#define DMA_CCR_PL_VeryHigh             (0x03U << DMA_CCR_PL_Pos)               ///< DMA Priority VeryHigh
#define DMA_CCR_M2M_Pos                 (14)
#define DMA_CCR_M2M                     (0x01U << DMA_CCR_M2M_Pos)              ///< Memory to memory mode

#define DMA_CCR_ARE_Pos             (15)
#define DMA_CCR_ARE                 (0x01U << DMA_CCR_ARE_Pos)              ///< Auto-Reload Enable bit

////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_CNDTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_CNDTR_NDT_Pos               (0)
#define DMA_CNDTR_NDT                   (0xFFFFU << DMA_CNDTR_NDT_Pos)          ///< Number of data to Transfer

////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_CPAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_CPAR_PA_Pos                 (0)
#define DMA_CPAR_PA                     (0xFFFFFFFFU << DMA_CPAR_PA_Pos)        ///< Peripheral Address

////////////////////////////////////////////////////////////////////////////////
/// @brief DMA_CMAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define DMA_CMAR_MA_Pos                 (0)
#define DMA_CMAR_MA                     (0xFFFFFFFFU << DMA_CMAR_MA_Pos)        ///< Peripheral Address
////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_CFGR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_CFGR_MEMMODE_Pos       (0)
#define EXTI_CFGR_MEMMODE           (0x03U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config
#define EXTI_CFGR_MEMMODE_0         (0x01U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config Bit 0
#define EXTI_CFGR_MEMMODE_1         (0x02U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config Bit 1
#define EXTI_CFGR_FLASH_MEMORY      (0x00U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config Mode 0
#define EXTI_CFGR_SYSTEM_MEMORY     (0x01U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config Mode 1
#define EXTI_CFGR_SRAM_MEMORY       (0x03U << EXTI_CFGR_MEMMODE_Pos)        ///< EXTI_Memory Remap Config Mode 3
#define EXTI_CFGR_ADCDMA_Pos        (8)
#define EXTI_CFGR_ADCDMA            (0x01U << EXTI_CFGR_ADCDMA_Pos)         ///< ADC DMA remap
#define EXTI_CFGR_TIM16DMA_Pos      (11)
#define EXTI_CFGR_TIM16DMA          (0x01U << EXTI_CFGR_TIM16DMA_Pos)       ///< Timer 16 DMA remap
#define EXTI_CFGR_TIM17DMA_Pos      (12)
#define EXTI_CFGR_TIM17DMA          (0x01U << EXTI_CFGR_TIM17DMA_Pos)       ///< Timer 17 DMA remap

#define EXTI_CFGR_UART1TXDMA_Pos    (9)
#define EXTI_CFGR_UART1TXDMA        (0x01U << EXTI_CFGR_UART1TXDMA_Pos)     ///< UART1 TX DMA remap
#define EXTI_CFGR_UART1RXDMA_Pos    (10)
#define EXTI_CFGR_UART1RXDMA        (0x01U << EXTI_CFGR_UART1RXDMA_Pos)     ///< UART1 RX DMA remap

#define EXTI_CFGR_CSMCH1DMA_Pos     (13)
#define EXTI_CFGR_CSMCH1DMA         (0x01U << EXTI_CFGR_CSMCH1DMA_Pos)      ///< EXTI_Memory Remap Config
#define EXTI_CFGR_CSMCH2DMA_Pos     (14)
#define EXTI_CFGR_CSMCH2DMA         (0x01U << EXTI_CFGR_CSMCH2DMA_Pos)      ///< EXTI_Memory Remap Config
#define EXTI_CFGR_PA11PA12_Pos      (15)
#define EXTI_CFGR_PA11PA12          (0x01U << EXTI_CFGR_PA11PA12_Pos)       ///< EXTI_Memory Remap Config

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_CR1 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_CR1_EXTI0_Pos          (0)
#define EXTI_CR1_EXTI0              (0x0FU << EXTI_CR1_EXTI0_Pos)           ///< EXTI 0 configuration
#define EXTI_CR1_EXTI0_PA           (0x00U << EXTI_CR1_EXTI0_Pos)           ///< PA[0] pin
#define EXTI_CR1_EXTI0_PB           (0x01U << EXTI_CR1_EXTI0_Pos)           ///< PB[0] pin
#define EXTI_CR1_EXTI0_PC           (0x02U << EXTI_CR1_EXTI0_Pos)           ///< PC[0] pin
#define EXTI_CR1_EXTI0_PD           (0x03U << EXTI_CR1_EXTI0_Pos)           ///< PD[0] pin

#define EXTI_CR1_EXTI1_Pos          (4)
#define EXTI_CR1_EXTI1              (0x0FU << EXTI_CR1_EXTI1_Pos)           ///< EXTI 1 configuration
#define EXTI_CR1_EXTI1_PA           (0x00U << EXTI_CR1_EXTI1_Pos)           ///< PA[1] pin
#define EXTI_CR1_EXTI1_PB           (0x01U << EXTI_CR1_EXTI1_Pos)           ///< PB[1] pin
#define EXTI_CR1_EXTI1_PC           (0x02U << EXTI_CR1_EXTI1_Pos)           ///< PC[1] pin
#define EXTI_CR1_EXTI1_PD           (0x03U << EXTI_CR1_EXTI1_Pos)           ///< PD[1] pin

#define EXTI_CR1_EXTI2_Pos          (8)
#define EXTI_CR1_EXTI2              (0x0FU << EXTI_CR1_EXTI2_Pos)           ///< EXTI 2 configuration
#define EXTI_CR1_EXTI2_PA           (0x00U << EXTI_CR1_EXTI2_Pos)           ///< PA[2] pin
#define EXTI_CR1_EXTI2_PB           (0x01U << EXTI_CR1_EXTI2_Pos)           ///< PB[2] pin
#define EXTI_CR1_EXTI2_PC           (0x02U << EXTI_CR1_EXTI2_Pos)           ///< PC[2] pin
#define EXTI_CR1_EXTI2_PD           (0x03U << EXTI_CR1_EXTI2_Pos)           ///< PD[2] pin

#define EXTI_CR1_EXTI3_Pos          (12)
#define EXTI_CR1_EXTI3              (0x0FU << EXTI_CR1_EXTI3_Pos)           ///< EXTI 3 configuration
#define EXTI_CR1_EXTI3_PA           (0x00U << EXTI_CR1_EXTI3_Pos)           ///< PA[3] pin
#define EXTI_CR1_EXTI3_PB           (0x01U << EXTI_CR1_EXTI3_Pos)           ///< PB[3] pin
#define EXTI_CR1_EXTI3_PC           (0x02U << EXTI_CR1_EXTI3_Pos)           ///< PC[3] pin
#define EXTI_CR1_EXTI3_PD           (0x03U << EXTI_CR1_EXTI3_Pos)           ///< PD[3] pin

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_CR2 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_CR2_EXTI4_Pos          (0)
#define EXTI_CR2_EXTI4              (0x0FU << EXTI_CR2_EXTI4_Pos)           ///< EXTI 4 configuration
#define EXTI_CR2_EXTI4_PA           (0x00U << EXTI_CR2_EXTI4_Pos)           ///< PA[4] pin
#define EXTI_CR2_EXTI4_PB           (0x01U << EXTI_CR2_EXTI4_Pos)           ///< PB[4] pin
#define EXTI_CR2_EXTI4_PC           (0x02U << EXTI_CR2_EXTI4_Pos)           ///< PC[4] pin
#define EXTI_CR2_EXTI4_PD           (0x03U << EXTI_CR2_EXTI4_Pos)           ///< PD[4] pin

#define EXTI_CR2_EXTI5_Pos          (4)
#define EXTI_CR2_EXTI5              (0x0FU << EXTI_CR2_EXTI5_Pos)           ///< EXTI 5 configuration
#define EXTI_CR2_EXTI5_PA           (0x00U << EXTI_CR2_EXTI5_Pos)           ///< PA[5] pin
#define EXTI_CR2_EXTI5_PB           (0x01U << EXTI_CR2_EXTI5_Pos)           ///< PB[5] pin
#define EXTI_CR2_EXTI5_PC           (0x02U << EXTI_CR2_EXTI5_Pos)           ///< PC[5] pin
#define EXTI_CR2_EXTI5_PD           (0x03U << EXTI_CR2_EXTI5_Pos)           ///< PD[5] pin

#define EXTI_CR2_EXTI6_Pos          (8)
#define EXTI_CR2_EXTI6              (0x0FU << EXTI_CR2_EXTI6_Pos)           ///< EXTI 6 configuration
#define EXTI_CR2_EXTI6_PA           (0x00U << EXTI_CR2_EXTI6_Pos)           ///< PA[6] pin
#define EXTI_CR2_EXTI6_PB           (0x01U << EXTI_CR2_EXTI6_Pos)           ///< PB[6] pin
#define EXTI_CR2_EXTI6_PC           (0x02U << EXTI_CR2_EXTI6_Pos)           ///< PC[6] pin
#define EXTI_CR2_EXTI6_PD           (0x03U << EXTI_CR2_EXTI6_Pos)           ///< PD[6] pin

#define EXTI_CR2_EXTI7_Pos          (12)
#define EXTI_CR2_EXTI7              (0x0FU << EXTI_CR2_EXTI7_Pos)           ///< EXTI 7 configuration
#define EXTI_CR2_EXTI7_PA           (0x00U << EXTI_CR2_EXTI7_Pos)           ///< PA[7] pin
#define EXTI_CR2_EXTI7_PB           (0x01U << EXTI_CR2_EXTI7_Pos)           ///< PB[7] pin
#define EXTI_CR2_EXTI7_PC           (0x02U << EXTI_CR2_EXTI7_Pos)           ///< PC[7] pin
#define EXTI_CR2_EXTI7_PD           (0x03U << EXTI_CR2_EXTI7_Pos)           ///< PD[7] pin

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_CR3 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_CR3_EXTI8_Pos          (0)
#define EXTI_CR3_EXTI8              (0x0FU << EXTI_CR3_EXTI8_Pos)           ///< EXTI 8 configuration
#define EXTI_CR3_EXTI8_PA           (0x00U << EXTI_CR3_EXTI8_Pos)           ///< PA[8] pin
#define EXTI_CR3_EXTI8_PB           (0x01U << EXTI_CR3_EXTI8_Pos)           ///< PB[8] pin
#define EXTI_CR3_EXTI8_PC           (0x02U << EXTI_CR3_EXTI8_Pos)           ///< PC[8] pin
#define EXTI_CR3_EXTI8_PD           (0x03U << EXTI_CR3_EXTI8_Pos)           ///< PD[8] pin

#define EXTI_CR3_EXTI9_Pos          (4)
#define EXTI_CR3_EXTI9              (0x0FU << EXTI_CR3_EXTI9_Pos)           ///< EXTI 9 configuration
#define EXTI_CR3_EXTI9_PA           (0x00U << EXTI_CR3_EXTI9_Pos)           ///< PA[9] pin
#define EXTI_CR3_EXTI9_PB           (0x01U << EXTI_CR3_EXTI9_Pos)           ///< PB[9] pin
#define EXTI_CR3_EXTI9_PC           (0x02U << EXTI_CR3_EXTI9_Pos)           ///< PC[9] pin
#define EXTI_CR3_EXTI9_PD           (0x03U << EXTI_CR3_EXTI9_Pos)           ///< PD[9] pin

#define EXTI_CR3_EXTI10_Pos         (8)
#define EXTI_CR3_EXTI10             (0x0FU << EXTI_CR3_EXTI10_Pos)          ///< EXTI 10 configuration
#define EXTI_CR3_EXTI10_PA          (0x00U << EXTI_CR3_EXTI10_Pos)          ///< PA[10] pin
#define EXTI_CR3_EXTI10_PB          (0x01U << EXTI_CR3_EXTI10_Pos)          ///< PB[10] pin
#define EXTI_CR3_EXTI10_PC          (0x02U << EXTI_CR3_EXTI10_Pos)          ///< PC[10] pin
#define EXTI_CR3_EXTI10_PD          (0x03U << EXTI_CR3_EXTI10_Pos)          ///< PD[10] pin

#define EXTI_CR3_EXTI11_Pos         (12)
#define EXTI_CR3_EXTI11             (0x0FU << EXTI_CR3_EXTI11_Pos)          ///< EXTI 11 configuration
#define EXTI_CR3_EXTI11_PA          (0x00U << EXTI_CR3_EXTI11_Pos)          ///< PA[11] pin
#define EXTI_CR3_EXTI11_PB          (0x01U << EXTI_CR3_EXTI11_Pos)          ///< PB[11] pin
#define EXTI_CR3_EXTI11_PC          (0x02U << EXTI_CR3_EXTI11_Pos)          ///< PC[11] pin
#define EXTI_CR3_EXTI11_PD          (0x03U << EXTI_CR3_EXTI11_Pos)          ///< PD[11] pin

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_CR4 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_CR4_EXTI12_Pos         (0)
#define EXTI_CR4_EXTI12             (0x0FU << EXTI_CR4_EXTI12_Pos)          ///< EXTI 12 configuration
#define EXTI_CR4_EXTI12_PA          (0x00U << EXTI_CR4_EXTI12_Pos)          ///< PA[12] pin
#define EXTI_CR4_EXTI12_PB          (0x01U << EXTI_CR4_EXTI12_Pos)          ///< PB[12] pin
#define EXTI_CR4_EXTI12_PC          (0x02U << EXTI_CR4_EXTI12_Pos)          ///< PC[12] pin
#define EXTI_CR4_EXTI12_PD          (0x03U << EXTI_CR4_EXTI12_Pos)          ///< PD[12] pin

#define EXTI_CR4_EXTI13_Pos         (4)
#define EXTI_CR4_EXTI13             (0x0FU << EXTI_CR4_EXTI13_Pos)          ///< EXTI 13 configuration
#define EXTI_CR4_EXTI13_PA          (0x00U << EXTI_CR4_EXTI13_Pos)          ///< PA[13] pin
#define EXTI_CR4_EXTI13_PB          (0x01U << EXTI_CR4_EXTI13_Pos)          ///< PB[13] pin
#define EXTI_CR4_EXTI13_PC          (0x02U << EXTI_CR4_EXTI13_Pos)          ///< PC[13] pin
#define EXTI_CR4_EXTI13_PD          (0x03U << EXTI_CR4_EXTI13_Pos)          ///< PD[13] pin

#define EXTI_CR4_EXTI14_Pos         (8)
#define EXTI_CR4_EXTI14             (0x0FU << EXTI_CR4_EXTI14_Pos)          ///< EXTI 14 configuration
#define EXTI_CR4_EXTI14_PA          (0x00U << EXTI_CR4_EXTI14_Pos)          ///< PA[14] pin
#define EXTI_CR4_EXTI14_PB          (0x01U << EXTI_CR4_EXTI14_Pos)          ///< PB[14] pin
#define EXTI_CR4_EXTI14_PC          (0x02U << EXTI_CR4_EXTI14_Pos)          ///< PC[14] pin
#define EXTI_CR4_EXTI14_PD          (0x03U << EXTI_CR4_EXTI14_Pos)          ///< PD[14] pin

#define EXTI_CR4_EXTI15_Pos         (12)
#define EXTI_CR4_EXTI15             (0x0FU << EXTI_CR4_EXTI15_Pos)          ///< EXTI 15 configuration
#define EXTI_CR4_EXTI15_PA          (0x00U << EXTI_CR4_EXTI15_Pos)          ///< PA[15] pin
#define EXTI_CR4_EXTI15_PB          (0x01U << EXTI_CR4_EXTI15_Pos)          ///< PB[15] pin
#define EXTI_CR4_EXTI15_PC          (0x02U << EXTI_CR4_EXTI15_Pos)          ///< PC[15] pin
#define EXTI_CR4_EXTI15_PD          (0x03U << EXTI_CR4_EXTI15_Pos)          ///< PD[15] pin

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_IMR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_IMR_0_Pos                  (0)
#define EXTI_IMR_0                      (0x01U << EXTI_IMR_0_Pos)               ///< Interrupt Mask on line 0
#define EXTI_IMR_1_Pos                  (1)
#define EXTI_IMR_1                      (0x01U << EXTI_IMR_1_Pos)               ///< Interrupt Mask on line 1
#define EXTI_IMR_2_Pos                  (2)
#define EXTI_IMR_2                      (0x01U << EXTI_IMR_2_Pos)               ///< Interrupt Mask on line 2
#define EXTI_IMR_3_Pos                  (3)
#define EXTI_IMR_3                      (0x01U << EXTI_IMR_3_Pos)               ///< Interrupt Mask on line 3
#define EXTI_IMR_4_Pos                  (4)
#define EXTI_IMR_4                      (0x01U << EXTI_IMR_4_Pos)               ///< Interrupt Mask on line 4
#define EXTI_IMR_5_Pos                  (5)
#define EXTI_IMR_5                      (0x01U << EXTI_IMR_5_Pos)               ///< Interrupt Mask on line 5
#define EXTI_IMR_6_Pos                  (6)
#define EXTI_IMR_6                      (0x01U << EXTI_IMR_6_Pos)               ///< Interrupt Mask on line 6
#define EXTI_IMR_7_Pos                  (7)
#define EXTI_IMR_7                      (0x01U << EXTI_IMR_7_Pos)               ///< Interrupt Mask on line 7
#define EXTI_IMR_8_Pos                  (8)
#define EXTI_IMR_8                      (0x01U << EXTI_IMR_8_Pos)               ///< Interrupt Mask on line 8
#define EXTI_IMR_9_Pos                  (9)
#define EXTI_IMR_9                      (0x01U << EXTI_IMR_9_Pos)               ///< Interrupt Mask on line 9
#define EXTI_IMR_10_Pos                 (10)
#define EXTI_IMR_10                     (0x01U << EXTI_IMR_10_Pos)              ///< Interrupt Mask on line 10
#define EXTI_IMR_11_Pos                 (11)
#define EXTI_IMR_11                     (0x01U << EXTI_IMR_11_Pos)              ///< Interrupt Mask on line 11
#define EXTI_IMR_12_Pos                 (12)
#define EXTI_IMR_12                     (0x01U << EXTI_IMR_12_Pos)              ///< Interrupt Mask on line 12
#define EXTI_IMR_13_Pos                 (13)
#define EXTI_IMR_13                     (0x01U << EXTI_IMR_13_Pos)              ///< Interrupt Mask on line 13
#define EXTI_IMR_14_Pos                 (14)
#define EXTI_IMR_14                     (0x01U << EXTI_IMR_14_Pos)              ///< Interrupt Mask on line 14
#define EXTI_IMR_15_Pos                 (15)
#define EXTI_IMR_15                     (0x01U << EXTI_IMR_15_Pos)              ///< Interrupt Mask on line 15
#define EXTI_IMR_16_Pos                 (16)
#define EXTI_IMR_16                     (0x01U << EXTI_IMR_16_Pos)              ///< Interrupt Mask on line 16

#define EXTI_IMR_17_Pos             (17)
#define EXTI_IMR_17                 (0x01U << EXTI_IMR_17_Pos)              ///< Interrupt Mask on line 17

#define EXTI_IMR_18_Pos             (18)
#define EXTI_IMR_18                 (0x01U << EXTI_IMR_18_Pos)              ///< Interrupt Mask on line 18

#define EXTI_IMR_19_Pos                 (19)
#define EXTI_IMR_19                     (0x01U << EXTI_IMR_19_Pos)              ///< Interrupt Mask on line 19

#define EXTI_IMR_20_Pos             (20)
#define EXTI_IMR_20                 (0x01U << EXTI_IMR_20_Pos)              ///< Interrupt Mask on line 20



#define EXTI_IMR_24_Pos             (24)
#define EXTI_IMR_24                 (0x01U << EXTI_IMR_24_Pos)              ///< Interrupt Mask on line 24

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_EMR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_EMR_0_Pos                  (0)
#define EXTI_EMR_0                      (0x01U << EXTI_EMR_0_Pos)               ///< Event Mask on line 0
#define EXTI_EMR_1_Pos                  (1)
#define EXTI_EMR_1                      (0x01U << EXTI_EMR_1_Pos)               ///< Event Mask on line 1
#define EXTI_EMR_2_Pos                  (2)
#define EXTI_EMR_2                      (0x01U << EXTI_EMR_2_Pos)               ///< Event Mask on line 2
#define EXTI_EMR_3_Pos                  (3)
#define EXTI_EMR_3                      (0x01U << EXTI_EMR_3_Pos)               ///< Event Mask on line 3
#define EXTI_EMR_4_Pos                  (4)
#define EXTI_EMR_4                      (0x01U << EXTI_EMR_4_Pos)               ///< Event Mask on line 4
#define EXTI_EMR_5_Pos                  (5)
#define EXTI_EMR_5                      (0x01U << EXTI_EMR_5_Pos)               ///< Event Mask on line 5
#define EXTI_EMR_6_Pos                  (6)
#define EXTI_EMR_6                      (0x01U << EXTI_EMR_6_Pos)               ///< Event Mask on line 6
#define EXTI_EMR_7_Pos                  (7)
#define EXTI_EMR_7                      (0x01U << EXTI_EMR_7_Pos)               ///< Event Mask on line 7
#define EXTI_EMR_8_Pos                  (8)
#define EXTI_EMR_8                      (0x01U << EXTI_EMR_8_Pos)               ///< Event Mask on line 8
#define EXTI_EMR_9_Pos                  (9)
#define EXTI_EMR_9                      (0x01U << EXTI_EMR_9_Pos)               ///< Event Mask on line 9
#define EXTI_EMR_10_Pos                 (10)
#define EXTI_EMR_10                     (0x01U << EXTI_EMR_10_Pos)              ///< Event Mask on line 10
#define EXTI_EMR_11_Pos                 (11)
#define EXTI_EMR_11                     (0x01U << EXTI_EMR_11_Pos)              ///< Event Mask on line 11
#define EXTI_EMR_12_Pos                 (12)
#define EXTI_EMR_12                     (0x01U << EXTI_EMR_12_Pos)              ///< Event Mask on line 12
#define EXTI_EMR_13_Pos                 (13)
#define EXTI_EMR_13                     (0x01U << EXTI_EMR_13_Pos)              ///< Event Mask on line 13
#define EXTI_EMR_14_Pos                 (14)
#define EXTI_EMR_14                     (0x01U << EXTI_EMR_14_Pos)              ///< Event Mask on line 14
#define EXTI_EMR_15_Pos                 (15)
#define EXTI_EMR_15                     (0x01U << EXTI_EMR_15_Pos)              ///< Event Mask on line 15
#define EXTI_EMR_16_Pos                 (16)
#define EXTI_EMR_16                     (0x01U << EXTI_EMR_16_Pos)              ///< Event Mask on line 16

#define EXTI_EMR_17_Pos             (17)
#define EXTI_EMR_17                 (0x01U << EXTI_EMR_17_Pos)              ///< Event Mask on line 17

#define EXTI_EMR_18_Pos             (18)
#define EXTI_EMR_18                 (0x01U << EXTI_EMR_18_Pos)              ///< Event Mask on line 18

#define EXTI_EMR_19_Pos                 (19)
#define EXTI_EMR_19                     (0x01U << EXTI_EMR_19_Pos)              ///< Event Mask on line 19

#define EXTI_EMR_20_Pos             (20)
#define EXTI_EMR_20                 (0x01U << EXTI_EMR_20_Pos)              ///< Event Mask on line 20



#define EXTI_EMR_24_Pos             (24)
#define EXTI_EMR_24                 (0x01U << EXTI_EMR_24_Pos)              ///< Event Mask on line 24

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_RTSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_RTSR_0_Pos                 (0)
#define EXTI_RTSR_0                     (0x01U << EXTI_RTSR_0_Pos)              ///< Rising trigger event configuration bit of line 0
#define EXTI_RTSR_1_Pos                 (1)
#define EXTI_RTSR_1                     (0x01U << EXTI_RTSR_1_Pos)              ///< Rising trigger event configuration bit of line 1
#define EXTI_RTSR_2_Pos                 (2)
#define EXTI_RTSR_2                     (0x01U << EXTI_RTSR_2_Pos)              ///< Rising trigger event configuration bit of line 2
#define EXTI_RTSR_3_Pos                 (3)
#define EXTI_RTSR_3                     (0x01U << EXTI_RTSR_3_Pos)              ///< Rising trigger event configuration bit of line 3
#define EXTI_RTSR_4_Pos                 (4)
#define EXTI_RTSR_4                     (0x01U << EXTI_RTSR_4_Pos)              ///< Rising trigger event configuration bit of line 4
#define EXTI_RTSR_5_Pos                 (5)
#define EXTI_RTSR_5                     (0x01U << EXTI_RTSR_5_Pos)              ///< Rising trigger event configuration bit of line 5
#define EXTI_RTSR_6_Pos                 (6)
#define EXTI_RTSR_6                     (0x01U << EXTI_RTSR_6_Pos)              ///< Rising trigger event configuration bit of line 6
#define EXTI_RTSR_7_Pos                 (7)
#define EXTI_RTSR_7                     (0x01U << EXTI_RTSR_7_Pos)              ///< Rising trigger event configuration bit of line 7
#define EXTI_RTSR_8_Pos                 (8)
#define EXTI_RTSR_8                     (0x01U << EXTI_RTSR_8_Pos)              ///< Rising trigger event configuration bit of line 8
#define EXTI_RTSR_9_Pos                 (9)
#define EXTI_RTSR_9                     (0x01U << EXTI_RTSR_9_Pos)              ///< Rising trigger event configuration bit of line 9
#define EXTI_RTSR_10_Pos                (10)
#define EXTI_RTSR_10                    (0x01U << EXTI_RTSR_10_Pos)             ///< Rising trigger event configuration bit of line 10
#define EXTI_RTSR_11_Pos                (11)
#define EXTI_RTSR_11                    (0x01U << EXTI_RTSR_11_Pos)             ///< Rising trigger event configuration bit of line 11
#define EXTI_RTSR_12_Pos                (12)
#define EXTI_RTSR_12                    (0x01U << EXTI_RTSR_12_Pos)             ///< Rising trigger event configuration bit of line 12
#define EXTI_RTSR_13_Pos                (13)
#define EXTI_RTSR_13                    (0x01U << EXTI_RTSR_13_Pos)             ///< Rising trigger event configuration bit of line 13
#define EXTI_RTSR_14_Pos                (14)
#define EXTI_RTSR_14                    (0x01U << EXTI_RTSR_14_Pos)             ///< Rising trigger event configuration bit of line 14
#define EXTI_RTSR_15_Pos                (15)
#define EXTI_RTSR_15                    (0x01U << EXTI_RTSR_15_Pos)             ///< Rising trigger event configuration bit of line 15
#define EXTI_RTSR_16_Pos                (16)
#define EXTI_RTSR_16                    (0x01U << EXTI_RTSR_16_Pos)             ///< Rising trigger event configuration bit of line 16

#define EXTI_RTSR_17_Pos            (17)
#define EXTI_RTSR_17                (0x01U << EXTI_RTSR_17_Pos)             ///< Rising trigger event configuration bit of line 17

#define EXTI_RTSR_18_Pos            (18)
#define EXTI_RTSR_18                (0x01U << EXTI_RTSR_18_Pos)             ///< Rising trigger event configuration bit of line 18

#define EXTI_RTSR_19_Pos                (19)
#define EXTI_RTSR_19                    (0x01U << EXTI_RTSR_19_Pos)             ///< Rising trigger event configuration bit of line 19

#define EXTI_RTSR_20_Pos            (20)
#define EXTI_RTSR_20                (0x01U << EXTI_RTSR_20_Pos)             ///< Rising trigger event configuration bit of line 20



#define EXTI_RTSR_24_Pos            (24)
#define EXTI_RTSR_24                (0x01U << EXTI_RTSR_24_Pos)             ///< Rising trigger event configuration bit of line 24

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_FTSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_FTSR_0_Pos                 (0)
#define EXTI_FTSR_0                     (0x01U << EXTI_FTSR_0_Pos)              ///< Falling trigger event configuration bit of line 0
#define EXTI_FTSR_1_Pos                 (1)
#define EXTI_FTSR_1                     (0x01U << EXTI_FTSR_1_Pos)              ///< Falling trigger event configuration bit of line 1
#define EXTI_FTSR_2_Pos                 (2)
#define EXTI_FTSR_2                     (0x01U << EXTI_FTSR_2_Pos)              ///< Falling trigger event configuration bit of line 2
#define EXTI_FTSR_3_Pos                 (3)
#define EXTI_FTSR_3                     (0x01U << EXTI_FTSR_3_Pos)              ///< Falling trigger event configuration bit of line 3
#define EXTI_FTSR_4_Pos                 (4)
#define EXTI_FTSR_4                     (0x01U << EXTI_FTSR_4_Pos)              ///< Falling trigger event configuration bit of line 4
#define EXTI_FTSR_5_Pos                 (5)
#define EXTI_FTSR_5                     (0x01U << EXTI_FTSR_5_Pos)              ///< Falling trigger event configuration bit of line 5
#define EXTI_FTSR_6_Pos                 (6)
#define EXTI_FTSR_6                     (0x01U << EXTI_FTSR_6_Pos)              ///< Falling trigger event configuration bit of line 6
#define EXTI_FTSR_7_Pos                 (7)
#define EXTI_FTSR_7                     (0x01U << EXTI_FTSR_7_Pos)              ///< Falling trigger event configuration bit of line 7
#define EXTI_FTSR_8_Pos                 (8)
#define EXTI_FTSR_8                     (0x01U << EXTI_FTSR_8_Pos)              ///< Falling trigger event configuration bit of line 8
#define EXTI_FTSR_9_Pos                 (9)
#define EXTI_FTSR_9                     (0x01U << EXTI_FTSR_9_Pos)              ///< Falling trigger event configuration bit of line 9
#define EXTI_FTSR_10_Pos                (10)
#define EXTI_FTSR_10                    (0x01U << EXTI_FTSR_10_Pos)             ///< Falling trigger event configuration bit of line 10
#define EXTI_FTSR_11_Pos                (11)
#define EXTI_FTSR_11                    (0x01U << EXTI_FTSR_11_Pos)             ///< Falling trigger event configuration bit of line 11
#define EXTI_FTSR_12_Pos                (12)
#define EXTI_FTSR_12                    (0x01U << EXTI_FTSR_12_Pos)             ///< Falling trigger event configuration bit of line 12
#define EXTI_FTSR_13_Pos                (13)
#define EXTI_FTSR_13                    (0x01U << EXTI_FTSR_13_Pos)             ///< Falling trigger event configuration bit of line 13
#define EXTI_FTSR_14_Pos                (14)
#define EXTI_FTSR_14                    (0x01U << EXTI_FTSR_14_Pos)             ///< Falling trigger event configuration bit of line 14
#define EXTI_FTSR_15_Pos                (15)
#define EXTI_FTSR_15                    (0x01U << EXTI_FTSR_15_Pos)             ///< Falling trigger event configuration bit of line 15
#define EXTI_FTSR_16_Pos                (16)
#define EXTI_FTSR_16                    (0x01U << EXTI_FTSR_16_Pos)             ///< Falling trigger event configuration bit of line 16

#define EXTI_FTSR_17_Pos            (17)
#define EXTI_FTSR_17                (0x01U << EXTI_FTSR_17_Pos)             ///< Falling trigger event configuration bit of line 17

#define EXTI_FTSR_18_Pos            (18)
#define EXTI_FTSR_18                (0x01U << EXTI_FTSR_18_Pos)             ///< Falling trigger event configuration bit of line 18

#define EXTI_FTSR_19_Pos                (19)
#define EXTI_FTSR_19                    (0x01U << EXTI_FTSR_19_Pos)             ///< Falling trigger event configuration bit of line 19

#define EXTI_FTSR_20_Pos            (20)
#define EXTI_FTSR_20                (0x01U << EXTI_FTSR_20_Pos)             ///< Falling trigger event configuration bit of line 20



#define EXTI_FTSR_24_Pos            (24)
#define EXTI_FTSR_24                (0x01U << EXTI_FTSR_24_Pos)             ///< Falling trigger event configuration bit of line 24

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_SWIER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_SWIER_0_Pos                (0)
#define EXTI_SWIER_0                    (0x01U << EXTI_SWIER_0_Pos)             ///< Software Interrupt on line  0
#define EXTI_SWIER_1_Pos                (1)
#define EXTI_SWIER_1                    (0x01U << EXTI_SWIER_1_Pos)             ///< Software Interrupt on line  1
#define EXTI_SWIER_2_Pos                (2)
#define EXTI_SWIER_2                    (0x01U << EXTI_SWIER_2_Pos)             ///< Software Interrupt on line  2
#define EXTI_SWIER_3_Pos                (3)
#define EXTI_SWIER_3                    (0x01U << EXTI_SWIER_3_Pos)             ///< Software Interrupt on line  3
#define EXTI_SWIER_4_Pos                (4)
#define EXTI_SWIER_4                    (0x01U << EXTI_SWIER_4_Pos)             ///< Software Interrupt on line  4
#define EXTI_SWIER_5_Pos                (5)
#define EXTI_SWIER_5                    (0x01U << EXTI_SWIER_5_Pos)             ///< Software Interrupt on line  5
#define EXTI_SWIER_6_Pos                (6)
#define EXTI_SWIER_6                    (0x01U << EXTI_SWIER_6_Pos)             ///< Software Interrupt on line  6
#define EXTI_SWIER_7_Pos                (7)
#define EXTI_SWIER_7                    (0x01U << EXTI_SWIER_7_Pos)             ///< Software Interrupt on line  7
#define EXTI_SWIER_8_Pos                (8)
#define EXTI_SWIER_8                    (0x01U << EXTI_SWIER_8_Pos)             ///< Software Interrupt on line  8
#define EXTI_SWIER_9_Pos                (9)
#define EXTI_SWIER_9                    (0x01U << EXTI_SWIER_9_Pos)             ///< Software Interrupt on line  9
#define EXTI_SWIER_10_Pos               (10)
#define EXTI_SWIER_10                   (0x01U << EXTI_SWIER_10_Pos)            ///< Software Interrupt on line  10
#define EXTI_SWIER_11_Pos               (11)
#define EXTI_SWIER_11                   (0x01U << EXTI_SWIER_11_Pos)            ///< Software Interrupt on line  11
#define EXTI_SWIER_12_Pos               (12)
#define EXTI_SWIER_12                   (0x01U << EXTI_SWIER_12_Pos)            ///< Software Interrupt on line  12
#define EXTI_SWIER_13_Pos               (13)
#define EXTI_SWIER_13                   (0x01U << EXTI_SWIER_13_Pos)            ///< Software Interrupt on line  13
#define EXTI_SWIER_14_Pos               (14)
#define EXTI_SWIER_14                   (0x01U << EXTI_SWIER_14_Pos)            ///< Software Interrupt on line  14
#define EXTI_SWIER_15_Pos               (15)
#define EXTI_SWIER_15                   (0x01U << EXTI_SWIER_15_Pos)            ///< Software Interrupt on line  15
#define EXTI_SWIER_16_Pos               (16)
#define EXTI_SWIER_16                   (0x01U << EXTI_SWIER_16_Pos)            ///< Software Interrupt on line  16

#define EXTI_SWIER_17_Pos           (17)
#define EXTI_SWIER_17               (0x01U << EXTI_SWIER_17_Pos)            ///< Software Interrupt on line  17

#define EXTI_SWIER_18_Pos           (18)
#define EXTI_SWIER_18               (0x01U << EXTI_SWIER_18_Pos)            ///< Software Interrupt on line  18

#define EXTI_SWIER_19_Pos               (19)
#define EXTI_SWIER_19                   (0x01U << EXTI_SWIER_19_Pos)            ///< Software Interrupt on line  19

#define EXTI_SWIER_20_Pos           (20)
#define EXTI_SWIER_20               (0x01U << EXTI_SWIER_20_Pos)            ///< Software Interrupt on line  20



#define EXTI_SWIER_24_Pos           (24)
#define EXTI_SWIER_24               (0x01U << EXTI_SWIER_24_Pos)            ///< Software Interrupt on line  24

////////////////////////////////////////////////////////////////////////////////
/// @brief EXTI_PR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define EXTI_PR_0_Pos                   (0)
#define EXTI_PR_0                       (0x01U << EXTI_PR_0_Pos)                ///< Pending bit 0
#define EXTI_PR_1_Pos                   (1)
#define EXTI_PR_1                       (0x01U << EXTI_PR_1_Pos)                ///< Pending bit 1
#define EXTI_PR_2_Pos                   (2)
#define EXTI_PR_2                       (0x01U << EXTI_PR_2_Pos)                ///< Pending bit 2
#define EXTI_PR_3_Pos                   (3)
#define EXTI_PR_3                       (0x01U << EXTI_PR_3_Pos)                ///< Pending bit 3
#define EXTI_PR_4_Pos                   (4)
#define EXTI_PR_4                       (0x01U << EXTI_PR_4_Pos)                ///< Pending bit 4
#define EXTI_PR_5_Pos                   (5)
#define EXTI_PR_5                       (0x01U << EXTI_PR_5_Pos)                ///< Pending bit 5
#define EXTI_PR_6_Pos                   (6)
#define EXTI_PR_6                       (0x01U << EXTI_PR_6_Pos)                ///< Pending bit 6
#define EXTI_PR_7_Pos                   (7)
#define EXTI_PR_7                       (0x01U << EXTI_PR_7_Pos)                ///< Pending bit 7
#define EXTI_PR_8_Pos                   (8)
#define EXTI_PR_8                       (0x01U << EXTI_PR_8_Pos)                ///< Pending bit 8
#define EXTI_PR_9_Pos                   (9)
#define EXTI_PR_9                       (0x01U << EXTI_PR_9_Pos)                ///< Pending bit 9
#define EXTI_PR_10_Pos                  (10)
#define EXTI_PR_10                      (0x01U << EXTI_PR_10_Pos)               ///< Pending bit 10
#define EXTI_PR_11_Pos                  (11)
#define EXTI_PR_11                      (0x01U << EXTI_PR_11_Pos)               ///< Pending bit 11
#define EXTI_PR_12_Pos                  (12)
#define EXTI_PR_12                      (0x01U << EXTI_PR_12_Pos)               ///< Pending bit 12
#define EXTI_PR_13_Pos                  (13)
#define EXTI_PR_13                      (0x01U << EXTI_PR_13_Pos)               ///< Pending bit 13
#define EXTI_PR_14_Pos                  (14)
#define EXTI_PR_14                      (0x01U << EXTI_PR_14_Pos)               ///< Pending bit 14
#define EXTI_PR_15_Pos                  (15)
#define EXTI_PR_15                      (0x01U << EXTI_PR_15_Pos)               ///< Pending bit 15
#define EXTI_PR_16_Pos                  (16)
#define EXTI_PR_16                      (0x01U << EXTI_PR_16_Pos)               ///< Pending bit 16

#define EXTI_PR_17_Pos              (17)
#define EXTI_PR_17                  (0x01U << EXTI_PR_17_Pos)               ///< Pending bit 17

#define EXTI_PR_18_Pos              (18)
#define EXTI_PR_18                  (0x01U << EXTI_PR_18_Pos)               ///< Pending bit 18

#define EXTI_PR_19_Pos                  (19)
#define EXTI_PR_19                      (0x01U << EXTI_PR_19_Pos)               ///< Pending bit 19

#define EXTI_PR_20_Pos              (20)
#define EXTI_PR_20                  (0x01U << EXTI_PR_20_Pos)               ///< Pending bit 20



#define EXTI_PR_24_Pos              (24)
#define EXTI_PR_24                  (0x01U << EXTI_PR_24_Pos)               ///< Pending bit 24

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_ACR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_ACR_LATENCY_Pos           (0)
#define FLASH_ACR_LATENCY               (0x07U << FLASH_ACR_LATENCY_Pos)        ///< LATENCY[2:0] bits (Latency)
#define FLASH_ACR_LATENCY_0             (0x00U << FLASH_ACR_LATENCY_Pos)        ///< 0 waiting state
#define FLASH_ACR_LATENCY_1             (0x01U << FLASH_ACR_LATENCY_Pos)        ///< 1 waiting state   ?? BIT
#define FLASH_ACR_LATENCY_2             (0x02U << FLASH_ACR_LATENCY_Pos)        ///< 2 waiting state
#define FLASH_ACR_LATENCY_3             (0x03U << FLASH_ACR_LATENCY_Pos)        ///< 3 waiting state
#define FLASH_ACR_HLFCYA_Pos            (3)
#define FLASH_ACR_HLFCYA                (0x01U << FLASH_ACR_HLFCYA_Pos)         ///< Flash Half Cycle Access Enable
#define FLASH_ACR_PRFTBE_Pos            (4)
#define FLASH_ACR_PRFTBE                (0x01U << FLASH_ACR_PRFTBE_Pos)         ///< Prefetch Buffer Enable
#define FLASH_ACR_PRFTBS_Pos            (5)
#define FLASH_ACR_PRFTBS                (0x01U << FLASH_ACR_PRFTBS_Pos)         ///< Prefetch Buffer Status

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_KEYR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_KEYR_FKEY_Pos             (0)
#define FLASH_KEYR_FKEY                 (0xFFFFFFFFU << FLASH_KEYR_FKEY_Pos)    ///< FLASH Key

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_OPTKEYR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_OPTKEYR_OPTKEY_Pos        (0)
#define FLASH_OPTKEYR_OPTKEY            (0xFFFFFFFFU << FLASH_OPTKEYR_OPTKEY_Pos)   ///< Option Byte Key

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_SR_BUSY_Pos               (0)
#define FLASH_SR_BUSY                   (0x01U << FLASH_SR_BUSY_Pos)            ///< Busy
#define FLASH_SR_PGERR_Pos              (2)
#define FLASH_SR_PGERR                  (0x01U << FLASH_SR_PGERR_Pos)           ///< Programming Error
#define FLASH_SR_WRPRTERR_Pos           (4)
#define FLASH_SR_WRPRTERR               (0x01U << FLASH_SR_WRPRTERR_Pos)        ///< Write Protection Error
#define FLASH_SR_EOP_Pos                (5)
#define FLASH_SR_EOP                    (0x01U << FLASH_SR_EOP_Pos)             ///< End of operation

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_CR_PG_Pos                 (0)
#define FLASH_CR_PG                     (0x01U << FLASH_CR_PG_Pos)              ///< Programming
#define FLASH_CR_PER_Pos                (1)
#define FLASH_CR_PER                    (0x01U << FLASH_CR_PER_Pos)             ///< Page Erase
#define FLASH_CR_MER_Pos                (2)
#define FLASH_CR_MER                    (0x01U << FLASH_CR_MER_Pos)             ///< Mass Erase
#define FLASH_CR_OPTPG_Pos              (4)
#define FLASH_CR_OPTPG                  (0x01U << FLASH_CR_OPTPG_Pos)           ///< Option Byte Programming
#define FLASH_CR_OPTER_Pos              (5)
#define FLASH_CR_OPTER                  (0x01U << FLASH_CR_OPTER_Pos)           ///< Option Byte Erase
#define FLASH_CR_STRT_Pos               (6)
#define FLASH_CR_STRT                   (0x01U << FLASH_CR_STRT_Pos)            ///< Start
#define FLASH_CR_LOCK_Pos               (7)
#define FLASH_CR_LOCK                   (0x01U << FLASH_CR_LOCK_Pos)            ///< Lock
#define FLASH_CR_OPTWRE_Pos             (9)
#define FLASH_CR_OPTWRE                 (0x01U << FLASH_CR_OPTWRE_Pos)          ///< Option Bytes Write Enable
#define FLASH_CR_ERRIE_Pos              (10)
#define FLASH_CR_ERRIE                  (0x01U << FLASH_CR_ERRIE_Pos)           ///< Error Interrupt Enable
#define FLASH_CR_EOPIE_Pos              (12)
#define FLASH_CR_EOPIE                  (0x01U << FLASH_CR_EOPIE_Pos)           ///< End of operation interrupt enable

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_AR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_AR_FAR_Pos                (0)
#define FLASH_AR_FAR                    (0xFFFFFFFFU << FLASH_AR_FAR_Pos)       ///< Flash Address

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_OBR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_OBR_OPTERR_Pos            (0)
#define FLASH_OBR_OPTERR                (0x01U << FLASH_OBR_OPTERR_Pos)         ///< Option Byte Error

#define FLASH_OBR_USER_Pos              (2)
#define FLASH_OBR_USER                  (0xFFU << FLASH_OBR_USER_Pos)           ///< User Option Bytes

#define FLASH_OBR_WDG_SW                (0x01U << FLASH_OBR_USER_Pos)           ///< WDG_SW
#define FLASH_OBR_RST_STOP              (0x02U << FLASH_OBR_USER_Pos)           ///< nRST_STOP
#define FLASH_OBR_RST_STDBY             (0x04U << FLASH_OBR_USER_Pos)           ///< nRST_STDBY

#define FLASH_OBR_BOOT1             (0x10U << FLASH_OBR_USER_Pos)           ///< nBOOT1

#define FLASH_OBR_Data0_Pos             (10)
#define FLASH_OBR_Data0                 (0xFFU << FLASH_OBR_Data0_Pos)          ///< User data storage option byte
#define FLASH_OBR_Data1_Pos             (18)
#define FLASH_OBR_Data1                 (0xFFU << FLASH_OBR_Data1_Pos)          ///< User data storage option byte

////////////////////////////////////////////////////////////////////////////////
/// @brief FLASH_WRPR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define FLASH_WRPR_WRP_Pos              (0)
#define FLASH_WRPR_WRP                  (0xFFFFFFFFU << FLASH_WRPR_WRP_Pos)     ///< Write Protect

#define  GPIO_CRL_MODE_0                ((uint32_t)0x00000001)                  ///< Bit 0
#define  GPIO_CRL_MODE_1                ((uint32_t)0x00000002)                  ///< Bit 1

#define  GPIO_CRL_CNF_0                ((uint32_t)0x00000001)                   ///< Bit 0
#define  GPIO_CRL_CNF_1                ((uint32_t)0x00000002)                   ///< Bit 1

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_CRL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_CRL_MODE0_Pos              (0)
#define GPIO_CRL_MODE0                  (0x03U << GPIO_CRL_MODE0_Pos)           ///< MODE0[1:0] bits (portx mode bits, pin 0)
#define GPIO_CRL_MODE0_0                (0x01U << GPIO_CRL_MODE0_Pos)           ///< Bit 0
#define GPIO_CRL_MODE0_1                (0x02U << GPIO_CRL_MODE0_Pos)           ///< Bit 1

#define GPIO_CRL_CNF0_Pos               (2)
#define GPIO_CRL_CNF0                   (0x03U << GPIO_CRL_CNF0_Pos)            ///< CNF0[1:0] bits (portx configuration bits, pin 0)
#define GPIO_CRL_CNF0_0                 (0x01U << GPIO_CRL_CNF0_Pos)            ///< Bit 0
#define GPIO_CRL_CNF0_1                 (0x02U << GPIO_CRL_CNF0_Pos)            ///< Bit 1

#define GPIO_CRL_MODE1_Pos              (4)
#define GPIO_CRL_MODE1                  (0x03U << GPIO_CRL_MODE1_Pos)           ///< MODE1[1:0] bits (portx mode bits, pin 1)
#define GPIO_CRL_MODE1_0                (0x01U << GPIO_CRL_MODE1_Pos)           ///< Bit 0
#define GPIO_CRL_MODE1_1                (0x02U << GPIO_CRL_MODE1_Pos)           ///< Bit 1

#define GPIO_CRL_CNF1_Pos               (6)
#define GPIO_CRL_CNF1                   (0x03U << GPIO_CRL_CNF1_Pos)            ///< CNF1[1:0] bits (portx configuration bits, pin 1)
#define GPIO_CRL_CNF1_0                 (0x01U << GPIO_CRL_CNF1_Pos)            ///< Bit 0
#define GPIO_CRL_CNF1_1                 (0x02U << GPIO_CRL_CNF1_Pos)            ///< Bit 1

#define GPIO_CRL_MODE2_Pos              (8)
#define GPIO_CRL_MODE2                  (0x03U << GPIO_CRL_MODE2_Pos)           ///< MODE2[1:0] bits (portx mode bits, pin 2)
#define GPIO_CRL_MODE2_0                (0x01U << GPIO_CRL_MODE2_Pos)           ///< Bit 0
#define GPIO_CRL_MODE2_1                (0x02U << GPIO_CRL_MODE2_Pos)           ///< Bit 1

#define GPIO_CRL_CNF2_Pos               (10)
#define GPIO_CRL_CNF2                   (0x03U << GPIO_CRL_CNF2_Pos)            ///< CNF2[1:0] bits (portx configuration bits, pin 2)
#define GPIO_CRL_CNF2_0                 (0x01U << GPIO_CRL_CNF2_Pos)            ///< Bit 0
#define GPIO_CRL_CNF2_1                 (0x02U << GPIO_CRL_CNF2_Pos)            ///< Bit 1

#define GPIO_CRL_MODE3_Pos              (12)
#define GPIO_CRL_MODE3                  (0x03U << GPIO_CRL_MODE3_Pos)           ///< MODE3[1:0] bits (portx mode bits, pin 3)
#define GPIO_CRL_MODE3_0                (0x01U << GPIO_CRL_MODE3_Pos)           ///< Bit 0
#define GPIO_CRL_MODE3_1                (0x02U << GPIO_CRL_MODE3_Pos)           ///< Bit 1

#define GPIO_CRL_CNF3_Pos               (14)
#define GPIO_CRL_CNF3                   (0x03U << GPIO_CRL_CNF3_Pos)            ///< CNF3[1:0] bits (portx configuration bits, pin 3)
#define GPIO_CRL_CNF3_0                 (0x01U << GPIO_CRL_CNF3_Pos)            ///< Bit 0
#define GPIO_CRL_CNF3_1                 (0x02U << GPIO_CRL_CNF3_Pos)            ///< Bit 1


#define GPIO_CRL_MODE4_Pos              (16)
#define GPIO_CRL_MODE4                  (0x03U << GPIO_CRL_MODE4_Pos)           ///< MODE4[1:0] bits (portx mode bits, pin 4)
#define GPIO_CRL_MODE4_0                (0x01U << GPIO_CRL_MODE4_Pos)           ///< Bit 0
#define GPIO_CRL_MODE4_1                (0x02U << GPIO_CRL_MODE4_Pos)           ///< Bit 1

#define GPIO_CRL_CNF4_Pos               (18)
#define GPIO_CRL_CNF4                   (0x03U << GPIO_CRL_CNF4_Pos)            ///< CNF4[1:0] bits (portx configuration bits, pin 4)
#define GPIO_CRL_CNF4_0                 (0x01U << GPIO_CRL_CNF4_Pos)            ///< Bit 0
#define GPIO_CRL_CNF4_1                 (0x02U << GPIO_CRL_CNF4_Pos)            ///< Bit 1

#define GPIO_CRL_MODE5_Pos              (20)
#define GPIO_CRL_MODE5                  (0x03U << GPIO_CRL_MODE5_Pos)           ///< MODE5[1:0] bits (portx mode bits, pin 5)
#define GPIO_CRL_MODE5_0                (0x01U << GPIO_CRL_MODE5_Pos)           ///< Bit 0
#define GPIO_CRL_MODE5_1                (0x02U << GPIO_CRL_MODE5_Pos)           ///< Bit 1

#define GPIO_CRL_CNF5_Pos               (22)
#define GPIO_CRL_CNF5                   (0x03U << GPIO_CRL_CNF5_Pos)            ///< CNF5[1:0] bits (portx configuration bits, pin 5)
#define GPIO_CRL_CNF5_0                 (0x01U << GPIO_CRL_CNF5_Pos)            ///< Bit 0
#define GPIO_CRL_CNF5_1                 (0x02U << GPIO_CRL_CNF5_Pos)            ///< Bit 1

#define GPIO_CRL_MODE6_Pos              (24)
#define GPIO_CRL_MODE6                  (0x03U << GPIO_CRL_MODE6_Pos)           ///< MODE6[1:0] bits (portx mode bits, pin 6)
#define GPIO_CRL_MODE6_0                (0x01U << GPIO_CRL_MODE6_Pos)           ///< Bit 0
#define GPIO_CRL_MODE6_1                (0x02U << GPIO_CRL_MODE6_Pos)           ///< Bit 1

#define GPIO_CRL_CNF6_Pos               (26)
#define GPIO_CRL_CNF6                   (0x03U << GPIO_CRL_CNF6_Pos)            ///< CNF6[1:0] bits (portx configuration bits, pin 6)
#define GPIO_CRL_CNF6_0                 (0x01U << GPIO_CRL_CNF6_Pos)            ///< Bit 0
#define GPIO_CRL_CNF6_1                 (0x02U << GPIO_CRL_CNF6_Pos)            ///< Bit 1

#define GPIO_CRL_MODE7_Pos              (28)
#define GPIO_CRL_MODE7                  (0x03U << GPIO_CRL_MODE7_Pos)           ///< MODE7[1:0] bits (portx mode bits, pin 7)
#define GPIO_CRL_MODE7_0                (0x01U << GPIO_CRL_MODE7_Pos)           ///< Bit 0
#define GPIO_CRL_MODE7_1                (0x02U << GPIO_CRL_MODE7_Pos)           ///< Bit 1

#define GPIO_CRL_CNF7_Pos               (30)
#define GPIO_CRL_CNF7                   (0x03U << GPIO_CRL_CNF7_Pos)            ///< CNF7[1:0] bits (portx configuration bits, pin 7)
#define GPIO_CRL_CNF7_0                 (0x01U << GPIO_CRL_CNF7_Pos)            ///< Bit 0
#define GPIO_CRL_CNF7_1                 (0x02U << GPIO_CRL_CNF7_Pos)            ///< Bit 1

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_CRH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_CRH_MODE8_Pos              (0)
#define GPIO_CRH_MODE8                  (0x03U << GPIO_CRH_MODE8_Pos)           ///< MODE8[1:0] bits (portx mode bits, pin 0)
#define GPIO_CRH_MODE8_0                (0x01U << GPIO_CRH_MODE8_Pos)           ///< Bit 0
#define GPIO_CRH_MODE8_1                (0x02U << GPIO_CRH_MODE8_Pos)           ///< Bit 1
#define GPIO_CRH_CNF8_Pos               (2)
#define GPIO_CRH_CNF8                   (0x03U << GPIO_CRH_CNF8_Pos)            ///< CNF8[1:0] bits (portx configuration bits, pin 0)
#define GPIO_CRH_CNF8_0                 (0x01U << GPIO_CRH_CNF8_Pos)            ///< Bit 0
#define GPIO_CRH_CNF8_1                 (0x02U << GPIO_CRH_CNF8_Pos)            ///< Bit 1
#define GPIO_CRH_MODE9_Pos              (4)
#define GPIO_CRH_MODE9                  (0x03U << GPIO_CRH_MODE9_Pos)           ///< MODE9[1:0] bits (portx mode bits, pin 1)
#define GPIO_CRH_MODE9_0                (0x01U << GPIO_CRH_MODE9_Pos)           ///< Bit 0
#define GPIO_CRH_MODE9_1                (0x02U << GPIO_CRH_MODE9_Pos)           ///< Bit 1
#define GPIO_CRH_CNF9_Pos               (6)
#define GPIO_CRH_CNF9                   (0x03U << GPIO_CRH_CNF9_Pos)            ///< CNF9[1:0] bits (portx configuration bits, pin 1)
#define GPIO_CRH_CNF9_0                 (0x01U << GPIO_CRH_CNF9_Pos)            ///< Bit 0
#define GPIO_CRH_CNF9_1                 (0x02U << GPIO_CRH_CNF9_Pos)            ///< Bit 1
#define GPIO_CRH_MODE10_Pos             (8)
#define GPIO_CRH_MODE10                 (0x03U << GPIO_CRH_MODE10_Pos)          ///< MODE10[1:0] bits (portx mode bits, pin 2)
#define GPIO_CRH_MODE10_0               (0x01U << GPIO_CRH_MODE10_Pos)          ///< Bit 0
#define GPIO_CRH_MODE10_1               (0x02U << GPIO_CRH_MODE10_Pos)          ///< Bit 1
#define GPIO_CRH_CNF10_Pos              (10)
#define GPIO_CRH_CNF10                  (0x03U << GPIO_CRH_CNF10_Pos)           ///< CNF10[1:0] bits (portx configuration bits, pin 2)
#define GPIO_CRH_CNF10_0                (0x01U << GPIO_CRH_CNF10_Pos)           ///< Bit 0
#define GPIO_CRH_CNF10_1                (0x02U << GPIO_CRH_CNF10_Pos)           ///< Bit 1
#define GPIO_CRH_MODE11_Pos             (12)
#define GPIO_CRH_MODE11                 (0x03U << GPIO_CRH_MODE11_Pos)          ///< MODE11[1:0] bits (portx mode bits, pin 3)
#define GPIO_CRH_MODE11_0               (0x01U << GPIO_CRH_MODE11_Pos)          ///< Bit 0
#define GPIO_CRH_MODE11_1               (0x02U << GPIO_CRH_MODE11_Pos)          ///< Bit 1
#define GPIO_CRH_CNF11_Pos              (14)
#define GPIO_CRH_CNF11                  (0x03U << GPIO_CRH_CNF11_Pos)           ///< CNF11[1:0] bits (portx configuration bits, pin 3)
#define GPIO_CRH_CNF11_0                (0x01U << GPIO_CRH_CNF11_Pos)           ///< Bit 0
#define GPIO_CRH_CNF11_1                (0x02U << GPIO_CRH_CNF11_Pos)           ///< Bit 1
#define GPIO_CRH_MODE12_Pos             (16)
#define GPIO_CRH_MODE12                 (0x03U << GPIO_CRH_MODE12_Pos)          ///< MODE12[1:0] bits (portx mode bits, pin 4)
#define GPIO_CRH_MODE12_0               (0x01U << GPIO_CRH_MODE12_Pos)          ///< Bit 0
#define GPIO_CRH_MODE12_1               (0x02U << GPIO_CRH_MODE12_Pos)          ///< Bit 1
#define GPIO_CRH_CNF12_Pos              (18)
#define GPIO_CRH_CNF12                  (0x03U << GPIO_CRH_CNF12_Pos)           ///< CNF12[1:0] bits (portx configuration bits, pin 4)
#define GPIO_CRH_CNF12_0                (0x01U << GPIO_CRH_CNF12_Pos)           ///< Bit 0
#define GPIO_CRH_CNF12_1                (0x02U << GPIO_CRH_CNF12_Pos)           ///< Bit 1
#define GPIO_CRH_MODE13_Pos             (20)
#define GPIO_CRH_MODE13                 (0x03U << GPIO_CRH_MODE13_Pos)          ///< MODE13[1:0] bits (portx mode bits, pin 5)
#define GPIO_CRH_MODE13_0               (0x01U << GPIO_CRH_MODE13_Pos)          ///< Bit 0
#define GPIO_CRH_MODE13_1               (0x02U << GPIO_CRH_MODE13_Pos)          ///< Bit 1
#define GPIO_CRH_CNF13_Pos              (22)
#define GPIO_CRH_CNF13                  (0x03U << GPIO_CRH_CNF13_Pos)           ///< CNF13[1:0] bits (portx configuration bits, pin 5)
#define GPIO_CRH_CNF13_0                (0x01U << GPIO_CRH_CNF13_Pos)           ///< Bit 0
#define GPIO_CRH_CNF13_1                (0x02U << GPIO_CRH_CNF13_Pos)           ///< Bit 1
#define GPIO_CRH_MODE14_Pos             (24)
#define GPIO_CRH_MODE14                 (0x03U << GPIO_CRH_MODE14_Pos)          ///< MODE14[1:0] bits (portx mode bits, pin 6)
#define GPIO_CRH_MODE14_0               (0x01U << GPIO_CRH_MODE14_Pos)          ///< Bit 0
#define GPIO_CRH_MODE14_1               (0x02U << GPIO_CRH_MODE14_Pos)          ///< Bit 1
#define GPIO_CRH_CNF14_Pos              (26)
#define GPIO_CRH_CNF14                  (0x03U << GPIO_CRH_CNF14_Pos)           ///< CNF14[1:0] bits (portx configuration bits, pin 6)
#define GPIO_CRH_CNF14_0                (0x01U << GPIO_CRH_CNF14_Pos)           ///< Bit 0
#define GPIO_CRH_CNF14_1                (0x02U << GPIO_CRH_CNF14_Pos)           ///< Bit 1
#define GPIO_CRH_MODE15_Pos             (28)
#define GPIO_CRH_MODE15                 (0x03U << GPIO_CRH_MODE15_Pos)          ///< MODE15[1:0] bits (portx mode bits, pin 7)
#define GPIO_CRH_MODE15_0               (0x01U << GPIO_CRH_MODE15_Pos)          ///< Bit 0
#define GPIO_CRH_MODE15_1               (0x02U << GPIO_CRH_MODE15_Pos)          ///< Bit 1
#define GPIO_CRH_CNF15_Pos              (30)
#define GPIO_CRH_CNF15                  (0x03U << GPIO_CRH_CNF15_Pos)           ///< CNF15[1:0] bits (portx configuration bits, pin 7)
#define GPIO_CRH_CNF15_0                (0x01U << GPIO_CRH_CNF15_Pos)           ///< Bit 0
#define GPIO_CRH_CNF15_1                (0x02U << GPIO_CRH_CNF15_Pos)           ///< Bit 1

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_IDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_IDR_DATA_Pos               (0)
#define GPIO_IDR_DATA                   (0xFFFFU << GPIO_IDR_DATA_Pos)          ///< Port input data
////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_IDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_IDR_IDR0_Pos               (0)
#define GPIO_IDR_IDR0                   (0x01U << GPIO_IDR_IDR0_Pos)            ///< Portx Set bit 0
#define GPIO_IDR_IDR1_Pos               (1)
#define GPIO_IDR_IDR1                   (0x01U << GPIO_IDR_IDR1_Pos)            ///< Portx Set bit 1
#define GPIO_IDR_IDR2_Pos               (2)
#define GPIO_IDR_IDR2                   (0x01U << GPIO_IDR_IDR2_Pos)            ///< Portx Set bit 2
#define GPIO_IDR_IDR3_Pos               (3)
#define GPIO_IDR_IDR3                   (0x01U << GPIO_IDR_IDR3_Pos)            ///< Portx Set bit 3
#define GPIO_IDR_IDR4_Pos               (4)
#define GPIO_IDR_IDR4                   (0x01U << GPIO_IDR_IDR4_Pos)            ///< Portx Set bit 4
#define GPIO_IDR_IDR5_Pos               (5)
#define GPIO_IDR_IDR5                   (0x01U << GPIO_IDR_IDR5_Pos)            ///< Portx Set bit 5
#define GPIO_IDR_IDR6_Pos               (6)
#define GPIO_IDR_IDR6                   (0x01U << GPIO_IDR_IDR6_Pos)            ///< Portx Set bit 6
#define GPIO_IDR_IDR7_Pos               (7)
#define GPIO_IDR_IDR7                   (0x01U << GPIO_IDR_IDR7_Pos)            ///< Portx Set bit 7
#define GPIO_IDR_IDR8_Pos               (8)
#define GPIO_IDR_IDR8                   (0x01U << GPIO_IDR_IDR8_Pos)            ///< Portx Set bit 8
#define GPIO_IDR_IDR9_Pos               (9)
#define GPIO_IDR_IDR9                   (0x01U << GPIO_IDR_IDR9_Pos)            ///< Portx Set bit 9
#define GPIO_IDR_IDR10_Pos              (10)
#define GPIO_IDR_IDR10                  (0x01U << GPIO_IDR_IDR10_Pos)           ///< Portx Set bit 10
#define GPIO_IDR_IDR11_Pos              (11)
#define GPIO_IDR_IDR11                  (0x01U << GPIO_IDR_IDR11_Pos)           ///< Portx Set bit 11
#define GPIO_IDR_IDR12_Pos              (12)
#define GPIO_IDR_IDR12                  (0x01U << GPIO_IDR_IDR12_Pos)           ///< Portx Set bit 12
#define GPIO_IDR_IDR13_Pos              (13)
#define GPIO_IDR_IDR13                  (0x01U << GPIO_IDR_IDR13_Pos)           ///< Portx Set bit 13
#define GPIO_IDR_IDR14_Pos              (14)
#define GPIO_IDR_IDR14                  (0x01U << GPIO_IDR_IDR14_Pos)           ///< Portx Set bit 14
#define GPIO_IDR_IDR15_Pos              (15)
#define GPIO_IDR_IDR15                  (0x01U << GPIO_IDR_IDR15_Pos)           ///< Portx Set bit 15
////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_ODR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_ODR_DATA_Pos               (0)
#define GPIO_ODR_DATA                   (0xFFFF << GPIO_ODR_DATA_Pos)           ///< Port output data
////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_ODR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_ODR_ODR0_Pos               (0)
#define GPIO_ODR_ODR0                   (0x01U << GPIO_ODR_ODR0_Pos)            ///< Portx Set bit 0
#define GPIO_ODR_ODR1_Pos               (1)
#define GPIO_ODR_ODR1                   (0x01U << GPIO_ODR_ODR1_Pos)            ///< Portx Set bit 1
#define GPIO_ODR_ODR2_Pos               (2)
#define GPIO_ODR_ODR2                   (0x01U << GPIO_ODR_ODR2_Pos)            ///< Portx Set bit 2
#define GPIO_ODR_ODR3_Pos               (3)
#define GPIO_ODR_ODR3                   (0x01U << GPIO_ODR_ODR3_Pos)            ///< Portx Set bit 3
#define GPIO_ODR_ODR4_Pos               (4)
#define GPIO_ODR_ODR4                   (0x01U << GPIO_ODR_ODR4_Pos)            ///< Portx Set bit 4
#define GPIO_ODR_ODR5_Pos               (5)
#define GPIO_ODR_ODR5                   (0x01U << GPIO_ODR_ODR5_Pos)            ///< Portx Set bit 5
#define GPIO_ODR_ODR6_Pos               (6)
#define GPIO_ODR_ODR6                   (0x01U << GPIO_ODR_ODR6_Pos)            ///< Portx Set bit 6
#define GPIO_ODR_ODR7_Pos               (7)
#define GPIO_ODR_ODR7                   (0x01U << GPIO_ODR_ODR7_Pos)            ///< Portx Set bit 7
#define GPIO_ODR_ODR8_Pos               (8)
#define GPIO_ODR_ODR8                   (0x01U << GPIO_ODR_ODR8_Pos)            ///< Portx Set bit 8
#define GPIO_ODR_ODR9_Pos               (9)
#define GPIO_ODR_ODR9                   (0x01U << GPIO_ODR_ODR9_Pos)            ///< Portx Set bit 9
#define GPIO_ODR_ODR10_Pos              (10)
#define GPIO_ODR_ODR10                  (0x01U << GPIO_ODR_ODR10_Pos)           ///< Portx Set bit 10
#define GPIO_ODR_ODR11_Pos              (11)
#define GPIO_ODR_ODR11                  (0x01U << GPIO_ODR_ODR11_Pos)           ///< Portx Set bit 11
#define GPIO_ODR_ODR12_Pos              (12)
#define GPIO_ODR_ODR12                  (0x01U << GPIO_ODR_ODR12_Pos)           ///< Portx Set bit 12
#define GPIO_ODR_ODR13_Pos              (13)
#define GPIO_ODR_ODR13                  (0x01U << GPIO_ODR_ODR13_Pos)           ///< Portx Set bit 13
#define GPIO_ODR_ODR14_Pos              (14)
#define GPIO_ODR_ODR14                  (0x01U << GPIO_ODR_ODR14_Pos)           ///< Portx Set bit 14
#define GPIO_ODR_ODR15_Pos              (15)
#define GPIO_ODR_ODR15                  (0x01U << GPIO_ODR_ODR15_Pos)           ///< Portx Set bit 15

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_BSRR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_BSRR_BS0_Pos               (0)
#define GPIO_BSRR_BS0                   (0x01U << GPIO_BSRR_BS0_Pos)            ///< Portx Set bit 0
#define GPIO_BSRR_BS1_Pos               (1)
#define GPIO_BSRR_BS1                   (0x01U << GPIO_BSRR_BS1_Pos)            ///< Portx Set bit 1
#define GPIO_BSRR_BS2_Pos               (2)
#define GPIO_BSRR_BS2                   (0x01U << GPIO_BSRR_BS2_Pos)            ///< Portx Set bit 2
#define GPIO_BSRR_BS3_Pos               (3)
#define GPIO_BSRR_BS3                   (0x01U << GPIO_BSRR_BS3_Pos)            ///< Portx Set bit 3
#define GPIO_BSRR_BS4_Pos               (4)
#define GPIO_BSRR_BS4                   (0x01U << GPIO_BSRR_BS4_Pos)            ///< Portx Set bit 4
#define GPIO_BSRR_BS5_Pos               (5)
#define GPIO_BSRR_BS5                   (0x01U << GPIO_BSRR_BS5_Pos)            ///< Portx Set bit 5
#define GPIO_BSRR_BS6_Pos               (6)
#define GPIO_BSRR_BS6                   (0x01U << GPIO_BSRR_BS6_Pos)            ///< Portx Set bit 6
#define GPIO_BSRR_BS7_Pos               (7)
#define GPIO_BSRR_BS7                   (0x01U << GPIO_BSRR_BS7_Pos)            ///< Portx Set bit 7
#define GPIO_BSRR_BS8_Pos               (8)
#define GPIO_BSRR_BS8                   (0x01U << GPIO_BSRR_BS8_Pos)            ///< Portx Set bit 8
#define GPIO_BSRR_BS9_Pos               (9)
#define GPIO_BSRR_BS9                   (0x01U << GPIO_BSRR_BS9_Pos)            ///< Portx Set bit 9
#define GPIO_BSRR_BS10_Pos              (10)
#define GPIO_BSRR_BS10                  (0x01U << GPIO_BSRR_BS10_Pos)           ///< Portx Set bit 10
#define GPIO_BSRR_BS11_Pos              (11)
#define GPIO_BSRR_BS11                  (0x01U << GPIO_BSRR_BS11_Pos)           ///< Portx Set bit 11
#define GPIO_BSRR_BS12_Pos              (12)
#define GPIO_BSRR_BS12                  (0x01U << GPIO_BSRR_BS12_Pos)           ///< Portx Set bit 12
#define GPIO_BSRR_BS13_Pos              (13)
#define GPIO_BSRR_BS13                  (0x01U << GPIO_BSRR_BS13_Pos)           ///< Portx Set bit 13
#define GPIO_BSRR_BS14_Pos              (14)
#define GPIO_BSRR_BS14                  (0x01U << GPIO_BSRR_BS14_Pos)           ///< Portx Set bit 14
#define GPIO_BSRR_BS15_Pos              (15)
#define GPIO_BSRR_BS15                  (0x01U << GPIO_BSRR_BS15_Pos)           ///< Portx Set bit 15

#define GPIO_BSRR_BR0_Pos               (16)
#define GPIO_BSRR_BR0                   (0x01U << GPIO_BSRR_BR0_Pos)            ///< Portx Reset bit 0
#define GPIO_BSRR_BR1_Pos               (17)
#define GPIO_BSRR_BR1                   (0x01U << GPIO_BSRR_BR1_Pos)            ///< Portx Reset bit 1
#define GPIO_BSRR_BR2_Pos               (18)
#define GPIO_BSRR_BR2                   (0x01U << GPIO_BSRR_BR2_Pos)            ///< Portx Reset bit 2
#define GPIO_BSRR_BR3_Pos               (19)
#define GPIO_BSRR_BR3                   (0x01U << GPIO_BSRR_BR3_Pos)            ///< Portx Reset bit 3
#define GPIO_BSRR_BR4_Pos               (20)
#define GPIO_BSRR_BR4                   (0x01U << GPIO_BSRR_BR4_Pos)            ///< Portx Reset bit 4
#define GPIO_BSRR_BR5_Pos               (21)
#define GPIO_BSRR_BR5                   (0x01U << GPIO_BSRR_BR5_Pos)            ///< Portx Reset bit 5
#define GPIO_BSRR_BR6_Pos               (22)
#define GPIO_BSRR_BR6                   (0x01U << GPIO_BSRR_BR6_Pos)            ///< Portx Reset bit 6
#define GPIO_BSRR_BR7_Pos               (23)
#define GPIO_BSRR_BR7                   (0x01U << GPIO_BSRR_BR7_Pos)            ///< Portx Reset bit 7
#define GPIO_BSRR_BR8_Pos               (24)
#define GPIO_BSRR_BR8                   (0x01U << GPIO_BSRR_BR8_Pos)            ///< Portx Reset bit 8
#define GPIO_BSRR_BR9_Pos               (25)
#define GPIO_BSRR_BR9                   (0x01U << GPIO_BSRR_BR9_Pos)            ///< Portx Reset bit 9
#define GPIO_BSRR_BR10_Pos              (26)
#define GPIO_BSRR_BR10                  (0x01U << GPIO_BSRR_BR10_Pos)           ///< Portx Reset bit 10
#define GPIO_BSRR_BR11_Pos              (27)
#define GPIO_BSRR_BR11                  (0x01U << GPIO_BSRR_BR11_Pos)           ///< Portx Reset bit 11
#define GPIO_BSRR_BR12_Pos              (28)
#define GPIO_BSRR_BR12                  (0x01U << GPIO_BSRR_BR12_Pos)           ///< Portx Reset bit 12
#define GPIO_BSRR_BR13_Pos              (29)
#define GPIO_BSRR_BR13                  (0x01U << GPIO_BSRR_BR13_Pos)           ///< Portx Reset bit 13
#define GPIO_BSRR_BR14_Pos              (30)
#define GPIO_BSRR_BR14                  (0x01U << GPIO_BSRR_BR14_Pos)           ///< Portx Reset bit 14
#define GPIO_BSRR_BR15_Pos              (31)
#define GPIO_BSRR_BR15                  (0x01U << GPIO_BSRR_BR15_Pos)           ///< Portx Reset bit 15

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_BRR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_BRR_BR_Pos                 (0)
#define GPIO_BRR_BR                     (0xFFFFU << GPIO_BRR_BR_Pos)            ///< Portx Reset



////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_LCKR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_LCKR_LCK_Pos               (0)
#define GPIO_LCKR_LCK                   (0xFFFFU << GPIO_LCKR_LCK_Pos)          ///< Portx Lock
#define GPIO_LCKR_LCKK_Pos              (16)
#define GPIO_LCKR_LCKK                  (0x01U << GPIO_LCKR_LCKK_Pos)           ///< Lock key

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_DCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_DCR_PX0_Pos                (0)
#define GPIO_DCR_PX0                    (0x03U << GPIO_DCR_PX0_Pos)                  ///< PX0[1:0] bits (pinx configuration bits, pin 0)
#define GPIO_DCR_PX0_MODE0              (0x00U << GPIO_DCR_PX0_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX0_MODE1              (0x01U << GPIO_DCR_PX0_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX0_MODE2              (0x02U << GPIO_DCR_PX0_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX0_MODE3              (0x03U << GPIO_DCR_PX0_Pos)                  ///< Mode = 3
#define GPIO_DCR_PX1_Pos                (2)
#define GPIO_DCR_PX1                    (0x03U << GPIO_DCR_PX1_Pos)                  ///< PX1[1:0] bits (pinx configuration bits, pin 1)
#define GPIO_DCR_PX1_MODE0              (0x00U << GPIO_DCR_PX1_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX1_MODE1              (0x01U << GPIO_DCR_PX1_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX1_MODE2              (0x02U << GPIO_DCR_PX1_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX1_MODE3              (0x03U << GPIO_DCR_PX1_Pos)                  ///< Mode = 3
#define GPIO_DCR_PX2_Pos                (4)
#define GPIO_DCR_PX2                    (0x03U << GPIO_DCR_PX2_Pos)                  ///< PX2[1:0] bits (pinx configuration bits, pin 2)
#define GPIO_DCR_PX2_MODE0              (0x00U << GPIO_DCR_PX2_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX2_MODE1              (0x01U << GPIO_DCR_PX2_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX2_MODE2              (0x02U << GPIO_DCR_PX2_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX2_MODE3              (0x03U << GPIO_DCR_PX2_Pos)                  ///< Mode = 3
#define GPIO_DCR_PX3_Pos                (6)
#define GPIO_DCR_PX3                    (0x03U << GPIO_DCR_PX3_Pos)                  ///< PX3[1:0] bits (pinx configuration bits, pin 3)
#define GPIO_DCR_PX3_MODE0              (0x00U << GPIO_DCR_PX3_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX3_MODE1              (0x01U << GPIO_DCR_PX3_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX3_MODE2              (0x02U << GPIO_DCR_PX3_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX3_MODE3              (0x03U << GPIO_DCR_PX3_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX4_Pos                (8)
#define GPIO_DCR_PX4                    (0x03U << GPIO_DCR_PX4_Pos)                  ///< PX4[1:0] bits (pinx configuration bits, pin 4)
#define GPIO_DCR_PX4_MODE0              (0x00U << GPIO_DCR_PX4_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX4_MODE1              (0x01U << GPIO_DCR_PX4_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX4_MODE2              (0x02U << GPIO_DCR_PX4_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX4_MODE3              (0x03U << GPIO_DCR_PX4_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX5_Pos                (10)
#define GPIO_DCR_PX5                    (0x03U << GPIO_DCR_PX5_Pos)                  ///< PX5[1:0] bits (pinx configuration bits, pin 5)
#define GPIO_DCR_PX5_MODE0              (0x00U << GPIO_DCR_PX5_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX5_MODE1              (0x01U << GPIO_DCR_PX5_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX5_MODE2              (0x02U << GPIO_DCR_PX5_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX5_MODE3              (0x03U << GPIO_DCR_PX5_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX6_Pos                (12)
#define GPIO_DCR_PX6                    (0x03U << GPIO_DCR_PX6_Pos)                  ///< PX6[1:0] bits (pinx configuration bits, pin 6)
#define GPIO_DCR_PX6_MODE0              (0x00U << GPIO_DCR_PX6_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX6_MODE1              (0x01U << GPIO_DCR_PX6_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX6_MODE2              (0x02U << GPIO_DCR_PX6_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX6_MODE3              (0x03U << GPIO_DCR_PX6_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX7_Pos                (14)
#define GPIO_DCR_PX7                    (0x03U << GPIO_DCR_PX7_Pos)                  ///< PX7[1:0] bits (pinx configuration bits, pin 7)
#define GPIO_DCR_PX7_MODE0              (0x00U << GPIO_DCR_PX7_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX7_MODE1              (0x01U << GPIO_DCR_PX7_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX7_MODE2              (0x02U << GPIO_DCR_PX7_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX7_MODE3              (0x03U << GPIO_DCR_PX7_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX8_Pos                (16)
#define GPIO_DCR_PX8                    (0x03U << GPIO_DCR_PX8_Pos)                  ///< PX8[1:0] bits (pinx configuration bits, pin 8)
#define GPIO_DCR_PX8_MODE0              (0x00U << GPIO_DCR_PX8_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX8_MODE1              (0x01U << GPIO_DCR_PX8_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX8_MODE2              (0x02U << GPIO_DCR_PX8_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX8_MODE3              (0x03U << GPIO_DCR_PX8_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX9_Pos                (18)
#define GPIO_DCR_PX9                    (0x03U << GPIO_DCR_PX9_Pos)                  ///< PX9[1:0] bits (pinx configuration bits, pin 9)
#define GPIO_DCR_PX9_MODE0              (0x00U << GPIO_DCR_PX9_Pos)                  ///< Mode = 0
#define GPIO_DCR_PX9_MODE1              (0x01U << GPIO_DCR_PX9_Pos)                  ///< Mode = 1
#define GPIO_DCR_PX9_MODE2              (0x02U << GPIO_DCR_PX9_Pos)                  ///< Mode = 2
#define GPIO_DCR_PX9_MODE3              (0x03U << GPIO_DCR_PX9_Pos)                  ///< Mode = 3

#define GPIO_DCR_PX10_Pos               (20)
#define GPIO_DCR_PX10                   (0x03U << GPIO_DCR_PX10_Pos)                 ///< PX10[1:0] bits (pinx configuration bits, pin 10)
#define GPIO_DCR_PX10_MODE0             (0x00U << GPIO_DCR_PX10_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX10_MODE1             (0x01U << GPIO_DCR_PX10_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX10_MODE2             (0x02U << GPIO_DCR_PX10_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX10_MODE3             (0x03U << GPIO_DCR_PX10_Pos)                 ///< Mode = 3

#define GPIO_DCR_PX11_Pos               (22)
#define GPIO_DCR_PX11                   (0x03U << GPIO_DCR_PX11_Pos)                 ///< PX11[1:0] bits (pinx configuration bits, pin 11)
#define GPIO_DCR_PX11_MODE0             (0x00U << GPIO_DCR_PX11_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX11_MODE1             (0x01U << GPIO_DCR_PX11_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX11_MODE2             (0x02U << GPIO_DCR_PX11_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX11_MODE3             (0x03U << GPIO_DCR_PX11_Pos)                 ///< Mode = 3
#define GPIO_DCR_PX12_Pos               (24)
#define GPIO_DCR_PX12                   (0x03U << GPIO_DCR_PX12_Pos)                 ///< PX12[1:0] bits (pinx configuration bits, pin 12)
#define GPIO_DCR_PX12_MODE0             (0x00U << GPIO_DCR_PX12_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX12_MODE1             (0x01U << GPIO_DCR_PX12_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX12_MODE2             (0x02U << GPIO_DCR_PX12_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX12_MODE3             (0x03U << GPIO_DCR_PX12_Pos)                 ///< Mode = 3
#define GPIO_DCR_PX13_Pos               (26)
#define GPIO_DCR_PX13                   (0x03U << GPIO_DCR_PX13_Pos)                 ///< PX13[1:0] bits (pinx configuration bits, pin 13)
#define GPIO_DCR_PX13_MODE0             (0x00U << GPIO_DCR_PX13_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX13_MODE1             (0x01U << GPIO_DCR_PX13_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX13_MODE2             (0x02U << GPIO_DCR_PX13_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX13_MODE3             (0x03U << GPIO_DCR_PX13_Pos)                 ///< Mode = 3

#define GPIO_DCR_PX14_Pos               (28)
#define GPIO_DCR_PX14                   (0x03U << GPIO_DCR_PX14_Pos)                 ///< PX14[1:0] bits (pinx configuration bits, pin 14)
#define GPIO_DCR_PX14_MODE0             (0x00U << GPIO_DCR_PX14_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX14_MODE1             (0x01U << GPIO_DCR_PX14_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX14_MODE2             (0x02U << GPIO_DCR_PX14_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX14_MODE3             (0x03U << GPIO_DCR_PX14_Pos)                 ///< Mode = 3
#define GPIO_DCR_PX15_Pos               (30)
#define GPIO_DCR_PX15                   (0x03U << GPIO_DCR_PX15_Pos)                 ///< PX15[1:0] bits (pinx configuration bits, pin 15)
#define GPIO_DCR_PX15_MODE0             (0x00U << GPIO_DCR_PX15_Pos)                 ///< Mode = 0
#define GPIO_DCR_PX15_MODE1             (0x01U << GPIO_DCR_PX15_Pos)                 ///< Mode = 1
#define GPIO_DCR_PX15_MODE2             (0x02U << GPIO_DCR_PX15_Pos)                 ///< Mode = 2
#define GPIO_DCR_PX15_MODE3             (0x03U << GPIO_DCR_PX15_Pos)                 ///< Mode = 3
////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_AFRL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_AFRL_AFR0_Pos              (0)
#define GPIO_AFRL_AFR0                  (0x0FU << GPIO_AFRL_AFR0_Pos)           ///< Multiplexing function selection for bit 0 of portx
#define GPIO_AFRL_AFR1_Pos              (4)
#define GPIO_AFRL_AFR1                  (0x0FU << GPIO_AFRL_AFR1_Pos)           ///< Multiplexing function selection for bit 1 of portx
#define GPIO_AFRL_AFR2_Pos              (8)
#define GPIO_AFRL_AFR2                  (0x0FU << GPIO_AFRL_AFR2_Pos)           ///< Multiplexing function selection for bit 2 of portx
#define GPIO_AFRL_AFR3_Pos              (12)
#define GPIO_AFRL_AFR3                  (0x0FU << GPIO_AFRL_AFR3_Pos)           ///< Multiplexing function selection for bit 3 of portx
#define GPIO_AFRL_AFR4_Pos              (16)
#define GPIO_AFRL_AFR4                  (0x0FU << GPIO_AFRL_AFR4_Pos)           ///< Multiplexing function selection for bit 4 of portx
#define GPIO_AFRL_AFR5_Pos              (20)
#define GPIO_AFRL_AFR5                  (0x0FU << GPIO_AFRL_AFR5_Pos)           ///< Multiplexing function selection for bit 5 of portx
#define GPIO_AFRL_AFR6_Pos              (24)
#define GPIO_AFRL_AFR6                  (0x0FU << GPIO_AFRL_AFR6_Pos)           ///< Multiplexing function selection for bit 6 of portx
#define GPIO_AFRL_AFR7_Pos              (28)
#define GPIO_AFRL_AFR7                  (0x0FU << GPIO_AFRL_AFR7_Pos)           ///< Multiplexing function selection for bit 7 of portx

////////////////////////////////////////////////////////////////////////////////
/// @brief GPIO_AFRH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define GPIO_AFRH_AFR8_Pos              (0)
#define GPIO_AFRH_AFR8                  (0x0FU << GPIO_AFRH_AFR8_Pos)           ///< Multiplexing function selection for bit 8 of portx
#define GPIO_AFRH_AFR9_Pos              (4)
#define GPIO_AFRH_AFR9                  (0x0FU << GPIO_AFRH_AFR9_Pos)           ///< Multiplexing function selection for bit 9 of portx
#define GPIO_AFRH_AFR10_Pos             (8)
#define GPIO_AFRH_AFR10                 (0x0FU << GPIO_AFRH_AFR10_Pos)          ///< Multiplexing function selection for bit 10 of portx
#define GPIO_AFRH_AFR11_Pos             (12)
#define GPIO_AFRH_AFR11                 (0x0FU << GPIO_AFRH_AFR11_Pos)          ///< Multiplexing function selection for bit 11 of portx
#define GPIO_AFRH_AFR12_Pos             (16)
#define GPIO_AFRH_AFR12                 (0x0FU << GPIO_AFRH_AFR12_Pos)          ///< Multiplexing function selection for bit 12 of portx
#define GPIO_AFRH_AFR13_Pos             (20)
#define GPIO_AFRH_AFR13                 (0x0FU << GPIO_AFRH_AFR13_Pos)          ///< Multiplexing function selection for bit 13 of portx
#define GPIO_AFRH_AFR14_Pos             (24)
#define GPIO_AFRH_AFR14                 (0x0FU << GPIO_AFRH_AFR14_Pos)          ///< Multiplexing function selection for bit 14 of portx
#define GPIO_AFRH_AFR15_Pos             (28)
#define GPIO_AFRH_AFR15                 (0x0FU << GPIO_AFRH_AFR15_Pos)          ///< Multiplexing function selection for bit 15 of portx
#define GPIO_AF_MODEMASK                (0x0FU)                                 ///< Mode = 0
#define GPIO_AF_MODE0                   (0x00U)                                 ///< Mode = 0
#define GPIO_AF_MODE1                   (0x01U)                                 ///< Mode = 1
#define GPIO_AF_MODE2                   (0x02U)                                 ///< Mode = 2
#define GPIO_AF_MODE3                   (0x03U)                                 ///< Mode = 3
#define GPIO_AF_MODE4                   (0x04U)                                 ///< Mode = 4
#define GPIO_AF_MODE5                   (0x05U)                                 ///< Mode = 5
#define GPIO_AF_MODE6                   (0x06U)                                 ///< Mode = 6
#define GPIO_AF_MODE7                   (0x07U)                                 ///< Mode = 7


////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_CR_MASTER_Pos               (0)
#define I2C_CR_MASTER                   (0x01U << I2C_CR_MASTER_Pos)            ///< I2C master mode enable
#define I2C_CR_SPEED_Pos                (1)
#define I2C_CR_SPEED                    (0x03U << I2C_CR_SPEED_Pos)             ///< I2C speed mode
#define I2C_CR_STD                      (0x01U << I2C_CR_SPEED_Pos)             ///< I2C standard speed mode
#define I2C_CR_FAST                     (0x02U << I2C_CR_SPEED_Pos)             ///< I2C fast speed mode
#define I2C_CR_SLAVE10_Pos              (3)
#define I2C_CR_SLAVE10                  (0x01U << I2C_CR_SLAVE10_Pos)           ///< I2C slave mode responds to 10-bit address
#define I2C_CR_MASTER10_Pos             (4)
#define I2C_CR_MASTER10                 (0x01U << I2C_CR_MASTER10_Pos)          ///< I2C master mode responds to 10-bit address
#define I2C_CR_REPEN_Pos                (5)
#define I2C_CR_REPEN                    (0x01U << I2C_CR_REPEN_Pos)             ///< Enable send RESTART
#define I2C_CR_SLAVEDIS_Pos             (6)
#define I2C_CR_SLAVEDIS                 (0x01U << I2C_CR_SLAVEDIS_Pos)          ///< I2C slave mode disable
#define I2C_CR_STOPINT_Pos              (7)
#define I2C_CR_STOPINT                  (0x01U << I2C_CR_STOPINT_Pos)           ///< Generate STOP interrupt in slave mode
#define I2C_CR_EMPINT_Pos               (8)
#define I2C_CR_EMPINT                   (0x01U << I2C_CR_EMPINT_Pos)            ///< I2C TX_EMPTY interrupt

#define I2C_CR_STOP_Pos             (9)
#define I2C_CR_STOP                 (0x01U << I2C_CR_STOP_Pos)              ///< STOP signal enable
#define I2C_CR_RESTART_Pos          (10)
#define I2C_CR_RESTART              (0x01U << I2C_CR_RESTART_Pos)           ///< RESTART signal enable

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_TAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_TAR_ADDR_Pos                (0)
#define I2C_TAR_ADDR                    (0x03FFU << I2C_TAR_ADDR_Pos)           ///< Target address for master mode
#define I2C_TAR_GC_Pos                  (10)
#define I2C_TAR_GC                      (0x01U << I2C_TAR_GC_Pos)               ///< General Call or START byte
#define I2C_TAR_SPECIAL_Pos             (11)
#define I2C_TAR_SPECIAL                 (0x01U << I2C_TAR_SPECIAL_Pos)          ///< Special command enable like General Call or START byte

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_SAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_SAR_ADDR_Pos                (0)
#define I2C_SAR_ADDR                    (0x03FFU << I2C_SAR_ADDR_Pos)           ///< Slave address

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_DR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_DR_DAT_Pos                  (0)
#define I2C_DR_DAT                      (0xFFU << I2C_DR_DAT_Pos)               ///< The data to be transmitted or received
#define I2C_DR_CMD_Pos                  (8)
#define I2C_DR_CMD                      (0x01U << I2C_DR_CMD_Pos)               ///< Read or write command

#define I2C_DR_STOP_Pos             (9)
#define I2C_DR_STOP                 (0x01U << I2C_DR_STOP_Pos)              ///< STOP signal enable
#define I2C_DR_RESTART_Pos          (10)
#define I2C_DR_RESTART              (0x01U << I2C_DR_RESTART_Pos)           ///< RESTART signal enable

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_SSHR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_SSHR_CNT_Pos                (0)
#define I2C_SSHR_CNT                    (0xFFFFU << I2C_SSHR_CNT_Pos)           ///< SCL clock high period count for standard speed

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_SSLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_SSLR_CNT_Pos                (0)
#define I2C_SSLR_CNT                    (0xFFFFU << I2C_SSLR_CNT_Pos)           ///< SCL clock low period count for standard speed

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_FSHR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_FSHR_CNT_Pos                (0)
#define I2C_FSHR_CNT                    (0xFFFFU << I2C_FSHR_CNT_Pos)           ///< SCL clock high period count for fast speed

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_FSLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_FSLR_CNT_Pos                (0)
#define I2C_FSLR_CNT                    (0xFFFFU << I2C_FSLR_CNT_Pos)           ///< SCL clock low period count for fast speed

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_ISR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_ISR_RX_UNDER_Pos            (0)
#define I2C_ISR_RX_UNDER                (0x01U << I2C_ISR_RX_UNDER_Pos)         ///< RX_UNDER interrupt status
#define I2C_ISR_RX_OVER_Pos             (1)
#define I2C_ISR_RX_OVER                 (0x01U << I2C_ISR_RX_OVER_Pos)          ///< RX_OVER interrupt status
#define I2C_ISR_RX_FULL_Pos             (2)
#define I2C_ISR_RX_FULL                 (0x01U << I2C_ISR_RX_FULL_Pos)          ///< RX_FULL interrupt status
#define I2C_ISR_TX_OVER_Pos             (3)
#define I2C_ISR_TX_OVER                 (0x01U << I2C_ISR_TX_OVER_Pos)          ///< TX_OVER interrupt status
#define I2C_ISR_TX_EMPTY_Pos            (4)
#define I2C_ISR_TX_EMPTY                (0x01U << I2C_ISR_TX_EMPTY_Pos)         ///< TX_EMPTY interrupt status
#define I2C_ISR_RX_REQ_Pos              (5)
#define I2C_ISR_RX_REQ                  (0x01U << I2C_ISR_RX_REQ_Pos)           ///< RX_REQ interrupt status
#define I2C_ISR_TX_ABRT_Pos             (6)
#define I2C_ISR_TX_ABRT                 (0x01U << I2C_ISR_TX_ABRT_Pos)          ///< TX_ABRT interrupt status
#define I2C_ISR_RX_DONE_Pos             (7)
#define I2C_ISR_RX_DONE                 (0x01U << I2C_ISR_RX_DONE_Pos)          ///< RX_DONE interrupt status
#define I2C_ISR_ACTIV_Pos               (8)
#define I2C_ISR_ACTIV                   (0x01U << I2C_ISR_ACTIV_Pos)            ///< ACTIVITY interrupt status
#define I2C_ISR_STOP_Pos                (9)
#define I2C_ISR_STOP                    (0x01U << I2C_ISR_STOP_Pos)             ///< STOP_DET interrupt status
#define I2C_ISR_START_Pos               (10)
#define I2C_ISR_START                   (0x01U << I2C_ISR_START_Pos)            ///< START_DET interrupt status
#define I2C_ISR_GC_Pos                  (11)
#define I2C_ISR_GC                      (0x01U << I2C_ISR_GC_Pos)               ///< GEN_CALL interrupt status
#define I2C_ISR_RESTART_Pos             (12)
#define I2C_ISR_RESTART                 (0x01U << I2C_ISR_RESTART_Pos)          ///< RESTART_DET interrupt status
#define I2C_ISR_HOLD_Pos                (13)
#define I2C_ISR_HOLD                    (0x01U << I2C_ISR_HOLD_Pos)             ///< MST_ON_HOLD interrupt status

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_IMR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_IMR_RX_UNDER_Pos            (0)
#define I2C_IMR_RX_UNDER                (0x01U << I2C_IMR_RX_UNDER_Pos)         ///< RX_UNDER interrupt mask
#define I2C_IMR_RX_OVER_Pos             (1)
#define I2C_IMR_RX_OVER                 (0x01U << I2C_IMR_RX_OVER_Pos)          ///< RX_OVER interrupt mask
#define I2C_IMR_RX_FULL_Pos             (2)
#define I2C_IMR_RX_FULL                 (0x01U << I2C_IMR_RX_FULL_Pos)          ///< RX_FULL interrupt mask
#define I2C_IMR_TX_OVER_Pos             (3)
#define I2C_IMR_TX_OVER                 (0x01U << I2C_IMR_TX_OVER_Pos)          ///< TX_OVER interrupt mask
#define I2C_IMR_TX_EMPTY_Pos            (4)
#define I2C_IMR_TX_EMPTY                (0x01U << I2C_IMR_TX_EMPTY_Pos)         ///< TX_EMPTY interrupt mask
#define I2C_IMR_RX_REQ_Pos              (5)
#define I2C_IMR_RX_REQ                  (0x01U << I2C_IMR_RX_REQ_Pos)           ///< RX_REQ interrupt mask
#define I2C_IMR_TX_ABRT_Pos             (6)
#define I2C_IMR_TX_ABRT                 (0x01U << I2C_IMR_TX_ABRT_Pos)          ///< TX_ABRT interrupt mask
#define I2C_IMR_RX_DONE_Pos             (7)
#define I2C_IMR_RX_DONE                 (0x01U << I2C_IMR_RX_DONE_Pos)          ///< RX_DONE interrupt mask

#define I2C_IMR_ACTIV_Pos               (8)
#define I2C_IMR_ACTIV                   (0x01U << I2C_IMR_ACTIV_Pos)            ///< ACTIVITY interrupt status
#define I2C_IMR_STOP_Pos                (9)
#define I2C_IMR_STOP                    (0x01U << I2C_IMR_STOP_Pos)             ///< STOP_DET interrupt status
#define I2C_IMR_START_Pos               (10)
#define I2C_IMR_START                   (0x01U << I2C_IMR_START_Pos)            ///< START_DET interrupt status
#define I2C_IMR_GC_Pos                  (11)
#define I2C_IMR_GC                      (0x01U << I2C_IMR_GC_Pos)               ///< GEN_CALL interrupt status
#define I2C_IMR_RESTART_Pos             (12)
#define I2C_IMR_RESTART                 (0x01U << I2C_IMR_RESTART_Pos)          ///< RESTART_DET interrupt status
#define I2C_IMR_HOLD_Pos                (13)
#define I2C_IMR_HOLD                    (0x01U << I2C_IMR_HOLD_Pos)             ///< MST_ON_HOLD interrupt status


////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RAWISR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RAWISR_RX_UNDER_Pos         (0)
#define I2C_RAWISR_RX_UNDER             (0x01U << I2C_RAWISR_RX_UNDER_Pos)      ///< RX_UNDER raw interrupt status
#define I2C_RAWISR_RX_OVER_Pos          (1)
#define I2C_RAWISR_RX_OVER              (0x01U << I2C_RAWISR_RX_OVER_Pos)       ///< RX_OVER raw interrupt status
#define I2C_RAWISR_RX_FULL_Pos          (2)
#define I2C_RAWISR_RX_FULL              (0x01U << I2C_RAWISR_RX_FULL_Pos)       ///< RX_FULL raw interrupt status
#define I2C_RAWISR_TX_OVER_Pos          (3)
#define I2C_RAWISR_TX_OVER              (0x01U << I2C_RAWISR_TX_OVER_Pos)       ///< TX_OVER raw interrupt status
#define I2C_RAWISR_TX_EMPTY_Pos         (4)
#define I2C_RAWISR_TX_EMPTY             (0x01U << I2C_RAWISR_TX_EMPTY_Pos)      ///< TX_EMPTY raw interrupt status
#define I2C_RAWISR_RX_REQ_Pos           (5)
#define I2C_RAWISR_RX_REQ               (0x01U << I2C_RAWISR_RX_REQ_Pos)        ///< RX_REQ raw interrupt status
#define I2C_RAWISR_TX_ABRT_Pos          (6)
#define I2C_RAWISR_TX_ABRT              (0x01U << I2C_RAWISR_TX_ABRT_Pos)       ///< TX_ABRT raw interrupt status
#define I2C_RAWISR_RX_DONE_Pos          (7)
#define I2C_RAWISR_RX_DONE              (0x01U << I2C_RAWISR_RX_DONE_Pos)       ///< RX_DONE raw interrupt status

#define I2C_RAWISR_ACTIV_Pos            (8)
#define I2C_RAWISR_ACTIV                (0x01U << I2C_RAWISR_ACTIV_Pos)         ///< ACTIVITY interrupt status
#define I2C_RAWISR_STOP_Pos             (9)
#define I2C_RAWISR_STOP                 (0x01U << I2C_RAWISR_STOP_Pos)          ///< STOP_DET interrupt status
#define I2C_RAWISR_START_Pos            (10)
#define I2C_RAWISR_START                (0x01U << I2C_RAWISR_START_Pos)         ///< START_DET interrupt status
#define I2C_RAWISR_GC_Pos               (11)
#define I2C_RAWISR_GC                   (0x01U << I2C_RAWISR_GC_Pos)            ///< GEN_CALL interrupt status
#define I2C_RAWISR_RESTART_Pos          (12)
#define I2C_RAWISR_RESTART              (0x01U << I2C_RAWISR_RESTART_Pos)       ///< RESTART_DET interrupt status
#define I2C_RAWISR_HOLD_Pos             (13)
#define I2C_RAWISR_HOLD                 (0x01U << I2C_RAWISR_HOLD_Pos)          ///< MST_ON_HOLD interrupt status

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RXTLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RXTLR_Pos                   (0)
#define I2C_RXTLR_TL                    (0xFFU << I2C_RXTLR_Pos)                ///< Receive FIFO threshold level

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_TXTLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_TXTLR_Pos                   (0)
#define I2C_TXTLR_TL                    (0xFFU << I2C_TXTLR_Pos)                ///< Transmit FIFO threshold level

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_ICR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_ICR_Pos                     (0)
#define I2C_ICR                         (0x01U << I2C_ICR_Pos)                  ///< Read this register to clear the combined interrupt, all individual interrupts

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RX_UNDER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RX_UNDER_Pos                (0)
#define I2C_RX_UNDER                    (0x01U << I2C_RX_UNDER_Pos)             ///< Read this register to clear the RX_UNDER interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RX_OVER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RX_OVER_Pos                 (0)
#define I2C_RX_OVER                     (0x01U << I2C_RX_OVER_Pos)              ///< Read this register to clear the RX_OVER interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_TX_OVER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_TX_OVER_Pos                 (0)
#define I2C_TX_OVER                     (0x01U << I2C_TX_OVER_Pos)              ///< Read this register to clear the TX_OVER interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RD_REQ Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RD_REQ_Pos                  (0)
#define I2C_RD_REQ                      (0x01U << I2C_RD_REQ_Pos)               ///< Read this register to clear the RD_REQ interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_TX_ABRT Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_TX_ABRT_Pos                 (0)
#define I2C_TX_ABRT                     (0x01U << I2C_TX_ABRT_Pos)              ///< Read this register to clear the TX_ABRT interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RX_DONE Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RX_DONE_Pos                 (0)
#define I2C_RX_DONE                     (0x01U << I2C_RX_DONE_Pos)              ///< Read this register to clear the RX_DONE interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_ACTIV Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_ACTIV_Pos                   (0)
#define I2C_ACTIV                       (0x01U << I2C_ACTIV_Pos)                ///< Read this register to clear the ACTIVITY interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_STOP Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_STOP_Pos                    (0)
#define I2C_STOP                        (0x01U << I2C_STOP_Pos)                 ///< Read this register to clear the STOP_DET interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_START Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_START_Pos                   (0)
#define I2C_START                       (0x01U << I2C_START_Pos)                ///< Read this register to clear the START_DET interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_GC Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_GC_Pos                      (0)
#define I2C_GC                          (0x01U << I2C_GC_Pos)                   ///< Read this register to clear the GEN_CALL interrupt of the I2C_RAW_INTR_STAT register

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_ENR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_ENR_ENABLE_Pos              (0)
#define I2C_ENR_ENABLE                  (0x01U << I2C_ENR_ENABLE_Pos)           ///< I2C mode enable
#define I2C_ENR_ABORT_Pos               (1)
#define I2C_ENR_ABORT                   (0x01U << I2C_ENR_ABORT_Pos)            ///< I2C transfer abort

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_SR_ACTIV_Pos                (0)
#define I2C_SR_ACTIV                    (0x01U << I2C_SR_ACTIV_Pos)             ///< I2C activity status
#define I2C_SR_TFNF_Pos                 (1)
#define I2C_SR_TFNF                     (0x01U << I2C_SR_TFNF_Pos)              ///< Transmit FIFO not full
#define I2C_SR_TFE_Pos                  (2)
#define I2C_SR_TFE                      (0x01U << I2C_SR_TFE_Pos)               ///< Transmit FIFO completely empty
#define I2C_SR_RFNE_Pos                 (3)
#define I2C_SR_RFNE                     (0x01U << I2C_SR_RFNE_Pos)              ///< Receive FIFO not empty
#define I2C_SR_RFF_Pos                  (4)
#define I2C_SR_RFF                      (0x01U << I2C_SR_RFF_Pos)               ///< Receive FIFO completely full
#define I2C_SR_MST_ACTIV_Pos            (5)
#define I2C_SR_MST_ACTIV                (0x01U << I2C_SR_MST_ACTIV_Pos)         ///< Master FSM activity status
#define I2C_SR_SLV_ACTIV_Pos            (6)
#define I2C_SR_SLV_ACTIV                (0x01U << I2C_SR_SLV_ACTIV_Pos)         ///< Slave FSM activity status

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_TXFLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_TXFLR_CNT_Pos               (0)
#define I2C_TXFLR_CNT                   (0x03U << I2C_TXFLR_CNT_Pos)            ///< Number of valid data in the transmit FIFO

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_RXFLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_RXFLR_CNT_Pos               (0)
#define I2C_RXFLR_CNT                   (0x03U << I2C_RXFLR_CNT_Pos)            ///< Number of valid data in the receive FIFO

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_HOLD Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_HOLD_TXCNT_Pos              (0)
#define I2C_HOLD_TXCNT                  (0xFFFFU << I2C_HOLD_TXCNT_Pos)         ///< SDA hold time when I2C acts as a transmit
#define I2C_HOLD_RXCNT_Pos              (16)
#define I2C_HOLD_RXCNT                  (0xFFFFU << I2C_HOLD_RXCNT_Pos)         ///< SDA hold time when I2C acts as a receiver

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_DMA Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_DMA_RXEN_Pos                (0)
#define I2C_DMA_RXEN                    (0x01U << I2C_DMA_RXEN_Pos)             ///< Receive DMA enable
#define I2C_DMA_TXEN_Pos                (1)
#define I2C_DMA_TXEN                    (0x01U << I2C_DMA_TXEN_Pos)             ///< Transmit DMA enable

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_SETUP Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_SETUP_CNT_Pos               (0)
#define I2C_SETUP_CNT                   (0xFFU << I2C_SETUP_CNT_Pos)            ///< SDA setup

////////////////////////////////////////////////////////////////////////////////
/// @brief I2C_GCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define I2C_GCR_GC_Pos                  (0)
#define I2C_GCR_GC                      (0x01U << I2C_GCR_GC_Pos)               ///< ACK general call

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_KR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_KEYR_KEY_Pos               (0)
#define IWDG_KEYR_KEY                   (0xFFFFU << IWDG_KEYR_KEY_Pos)          ///< Key Value

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_PR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_PR_PRE_Pos                 (0)
#define IWDG_PR_PRE                     (0x07U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 4
#define IWDG_PR_PRE_DIV4                (0x00U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 4
#define IWDG_PR_PRE_DIV8                (0x01U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 8
#define IWDG_PR_PRE_DIV16               (0x02U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 16
#define IWDG_PR_PRE_DIV32               (0x03U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 32
#define IWDG_PR_PRE_DIV64               (0x04U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 64
#define IWDG_PR_PRE_DIV128              (0x05U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 128
#define IWDG_PR_PRE_DIV256              (0x06U << IWDG_PR_PRE_Pos)              ///< Prescaler divided by 256

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_RLR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_RLR_RL_Pos                 (0)
#define IWDG_RLR_RL                     (0x0FFFU << IWDG_RLR_RL_Pos)            ///< Watchdog counter reload value

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_SR_PVU_Pos                 (0)
#define IWDG_SR_PVU                     (0x01U << IWDG_SR_PVU_Pos)              ///< Watchdog prescaler value update
#define IWDG_SR_RVU_Pos                 (1)
#define IWDG_SR_RVU                     (0x01U << IWDG_SR_RVU_Pos)              ///< Watchdog counter reload value update

#define IWDG_SR_IVU_Pos                 (2)
#define IWDG_SR_IVU                     (0x01U << IWDG_SR_IVU_Pos)

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_CR_IRQSEL_Pos          (0)
#define IWDG_CR_IRQSEL              (0x01U << IWDG_CR_IRQSEL_Pos)           ///< IWDG overflow operation selection
#define IWDG_CR_IRQCLR_Pos          (1)
#define IWDG_CR_IRQCLR              (0x01U << IWDG_CR_IRQCLR_Pos)           ///< IWDG interrupt clear

////////////////////////////////////////////////////////////////////////////////
/// @brief IWDG_IGRN Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define IWDG_IGEN_IGEN_Pos          (0)
#define IWDG_IGEN_IGEN              (0xFFFU << IWDG_CR_IRQSEL_Pos)          ///< IWDG Interrupt Generate value


///////////////////////////////////////////////////////////////////////////////
//                  Nested Vectored Interrupt Controller
///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_ISER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_ISER_SETENA                (0xFFFFFFFFU)                           ///< Interrupt set enable bits
#define NVIC_ISER_SETENA_0              (0x00000001U)                           ///< bit 0
#define NVIC_ISER_SETENA_1              (0x00000002U)                           ///< bit 1
#define NVIC_ISER_SETENA_2              (0x00000004U)                           ///< bit 2
#define NVIC_ISER_SETENA_3              (0x00000008U)                           ///< bit 3
#define NVIC_ISER_SETENA_4              (0x00000010U)                           ///< bit 4
#define NVIC_ISER_SETENA_5              (0x00000020U)                           ///< bit 5
#define NVIC_ISER_SETENA_6              (0x00000040U)                           ///< bit 6
#define NVIC_ISER_SETENA_7              (0x00000080U)                           ///< bit 7
#define NVIC_ISER_SETENA_8              (0x00000100U)                           ///< bit 8
#define NVIC_ISER_SETENA_9              (0x00000200U)                           ///< bit 9
#define NVIC_ISER_SETENA_10             (0x00000400U)                           ///< bit 10
#define NVIC_ISER_SETENA_11             (0x00000800U)                           ///< bit 11
#define NVIC_ISER_SETENA_12             (0x00001000U)                           ///< bit 12
#define NVIC_ISER_SETENA_13             (0x00002000U)                           ///< bit 13
#define NVIC_ISER_SETENA_14             (0x00004000U)                           ///< bit 14
#define NVIC_ISER_SETENA_15             (0x00008000U)                           ///< bit 15
#define NVIC_ISER_SETENA_16             (0x00010000U)                           ///< bit 16
#define NVIC_ISER_SETENA_17             (0x00020000U)                           ///< bit 17
#define NVIC_ISER_SETENA_18             (0x00040000U)                           ///< bit 18
#define NVIC_ISER_SETENA_19             (0x00080000U)                           ///< bit 19
#define NVIC_ISER_SETENA_20             (0x00100000U)                           ///< bit 20
#define NVIC_ISER_SETENA_21             (0x00200000U)                           ///< bit 21
#define NVIC_ISER_SETENA_22             (0x00400000U)                           ///< bit 22
#define NVIC_ISER_SETENA_23             (0x00800000U)                           ///< bit 23
#define NVIC_ISER_SETENA_24             (0x01000000U)                           ///< bit 24
#define NVIC_ISER_SETENA_25             (0x02000000U)                           ///< bit 25
#define NVIC_ISER_SETENA_26             (0x04000000U)                           ///< bit 26
#define NVIC_ISER_SETENA_27             (0x08000000U)                           ///< bit 27
#define NVIC_ISER_SETENA_28             (0x10000000U)                           ///< bit 28
#define NVIC_ISER_SETENA_29             (0x20000000U)                           ///< bit 29
#define NVIC_ISER_SETENA_30             (0x40000000U)                           ///< bit 30
#define NVIC_ISER_SETENA_31             (0x80000000U)                           ///< bit 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_ICER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_ICER_CLRENA                (0xFFFFFFFFU)                           ///< Interrupt clear-enable bits
#define NVIC_ICER_CLRENA_0              (0x00000001U)                           ///< bit 0
#define NVIC_ICER_CLRENA_1              (0x00000002U)                           ///< bit 1
#define NVIC_ICER_CLRENA_2              (0x00000004U)                           ///< bit 2
#define NVIC_ICER_CLRENA_3              (0x00000008U)                           ///< bit 3
#define NVIC_ICER_CLRENA_4              (0x00000010U)                           ///< bit 4
#define NVIC_ICER_CLRENA_5              (0x00000020U)                           ///< bit 5
#define NVIC_ICER_CLRENA_6              (0x00000040U)                           ///< bit 6
#define NVIC_ICER_CLRENA_7              (0x00000080U)                           ///< bit 7
#define NVIC_ICER_CLRENA_8              (0x00000100U)                           ///< bit 8
#define NVIC_ICER_CLRENA_9              (0x00000200U)                           ///< bit 9
#define NVIC_ICER_CLRENA_10             (0x00000400U)                           ///< bit 10
#define NVIC_ICER_CLRENA_11             (0x00000800U)                           ///< bit 11
#define NVIC_ICER_CLRENA_12             (0x00001000U)                           ///< bit 12
#define NVIC_ICER_CLRENA_13             (0x00002000U)                           ///< bit 13
#define NVIC_ICER_CLRENA_14             (0x00004000U)                           ///< bit 14
#define NVIC_ICER_CLRENA_15             (0x00008000U)                           ///< bit 15
#define NVIC_ICER_CLRENA_16             (0x00010000U)                           ///< bit 16
#define NVIC_ICER_CLRENA_17             (0x00020000U)                           ///< bit 17
#define NVIC_ICER_CLRENA_18             (0x00040000U)                           ///< bit 18
#define NVIC_ICER_CLRENA_19             (0x00080000U)                           ///< bit 19
#define NVIC_ICER_CLRENA_20             (0x00100000U)                           ///< bit 20
#define NVIC_ICER_CLRENA_21             (0x00200000U)                           ///< bit 21
#define NVIC_ICER_CLRENA_22             (0x00400000U)                           ///< bit 22
#define NVIC_ICER_CLRENA_23             (0x00800000U)                           ///< bit 23
#define NVIC_ICER_CLRENA_24             (0x01000000U)                           ///< bit 24
#define NVIC_ICER_CLRENA_25             (0x02000000U)                           ///< bit 25
#define NVIC_ICER_CLRENA_26             (0x04000000U)                           ///< bit 26
#define NVIC_ICER_CLRENA_27             (0x08000000U)                           ///< bit 27
#define NVIC_ICER_CLRENA_28             (0x10000000U)                           ///< bit 28
#define NVIC_ICER_CLRENA_29             (0x20000000U)                           ///< bit 29
#define NVIC_ICER_CLRENA_30             (0x40000000U)                           ///< bit 30
#define NVIC_ICER_CLRENA_31             (0x80000000U)                           ///< bit 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_ISPR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_ISPR_SETPEND               (0xFFFFFFFFU)                           ///< Interrupt set-pending bits
#define NVIC_ISPR_SETPEND_0             (0x00000001U)                           ///< bit 0
#define NVIC_ISPR_SETPEND_1             (0x00000002U)                           ///< bit 1
#define NVIC_ISPR_SETPEND_2             (0x00000004U)                           ///< bit 2
#define NVIC_ISPR_SETPEND_3             (0x00000008U)                           ///< bit 3
#define NVIC_ISPR_SETPEND_4             (0x00000010U)                           ///< bit 4
#define NVIC_ISPR_SETPEND_5             (0x00000020U)                           ///< bit 5
#define NVIC_ISPR_SETPEND_6             (0x00000040U)                           ///< bit 6
#define NVIC_ISPR_SETPEND_7             (0x00000080U)                           ///< bit 7
#define NVIC_ISPR_SETPEND_8             (0x00000100U)                           ///< bit 8
#define NVIC_ISPR_SETPEND_9             (0x00000200U)                           ///< bit 9
#define NVIC_ISPR_SETPEND_10            (0x00000400U)                           ///< bit 10
#define NVIC_ISPR_SETPEND_11            (0x00000800U)                           ///< bit 11
#define NVIC_ISPR_SETPEND_12            (0x00001000U)                           ///< bit 12
#define NVIC_ISPR_SETPEND_13            (0x00002000U)                           ///< bit 13
#define NVIC_ISPR_SETPEND_14            (0x00004000U)                           ///< bit 14
#define NVIC_ISPR_SETPEND_15            (0x00008000U)                           ///< bit 15
#define NVIC_ISPR_SETPEND_16            (0x00010000U)                           ///< bit 16
#define NVIC_ISPR_SETPEND_17            (0x00020000U)                           ///< bit 17
#define NVIC_ISPR_SETPEND_18            (0x00040000U)                           ///< bit 18
#define NVIC_ISPR_SETPEND_19            (0x00080000U)                           ///< bit 19
#define NVIC_ISPR_SETPEND_20            (0x00100000U)                           ///< bit 20
#define NVIC_ISPR_SETPEND_21            (0x00200000U)                           ///< bit 21
#define NVIC_ISPR_SETPEND_22            (0x00400000U)                           ///< bit 22
#define NVIC_ISPR_SETPEND_23            (0x00800000U)                           ///< bit 23
#define NVIC_ISPR_SETPEND_24            (0x01000000U)                           ///< bit 24
#define NVIC_ISPR_SETPEND_25            (0x02000000U)                           ///< bit 25
#define NVIC_ISPR_SETPEND_26            (0x04000000U)                           ///< bit 26
#define NVIC_ISPR_SETPEND_27            (0x08000000U)                           ///< bit 27
#define NVIC_ISPR_SETPEND_28            (0x10000000U)                           ///< bit 28
#define NVIC_ISPR_SETPEND_29            (0x20000000U)                           ///< bit 29
#define NVIC_ISPR_SETPEND_30            (0x40000000U)                           ///< bit 30
#define NVIC_ISPR_SETPEND_31            (0x80000000U)                           ///< bit 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_ICPR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_ICPR_CLRPEND               (0xFFFFFFFFU)                           ///< Interrupt clear-pending bits
#define NVIC_ICPR_CLRPEND_0             (0x00000001U)                           ///< bit 0
#define NVIC_ICPR_CLRPEND_1             (0x00000002U)                           ///< bit 1
#define NVIC_ICPR_CLRPEND_2             (0x00000004U)                           ///< bit 2
#define NVIC_ICPR_CLRPEND_3             (0x00000008U)                           ///< bit 3
#define NVIC_ICPR_CLRPEND_4             (0x00000010U)                           ///< bit 4
#define NVIC_ICPR_CLRPEND_5             (0x00000020U)                           ///< bit 5
#define NVIC_ICPR_CLRPEND_6             (0x00000040U)                           ///< bit 6
#define NVIC_ICPR_CLRPEND_7             (0x00000080U)                           ///< bit 7
#define NVIC_ICPR_CLRPEND_8             (0x00000100U)                           ///< bit 8
#define NVIC_ICPR_CLRPEND_9             (0x00000200U)                           ///< bit 9
#define NVIC_ICPR_CLRPEND_10            (0x00000400U)                           ///< bit 10
#define NVIC_ICPR_CLRPEND_11            (0x00000800U)                           ///< bit 11
#define NVIC_ICPR_CLRPEND_12            (0x00001000U)                           ///< bit 12
#define NVIC_ICPR_CLRPEND_13            (0x00002000U)                           ///< bit 13
#define NVIC_ICPR_CLRPEND_14            (0x00004000U)                           ///< bit 14
#define NVIC_ICPR_CLRPEND_15            (0x00008000U)                           ///< bit 15
#define NVIC_ICPR_CLRPEND_16            (0x00010000U)                           ///< bit 16
#define NVIC_ICPR_CLRPEND_17            (0x00020000U)                           ///< bit 17
#define NVIC_ICPR_CLRPEND_18            (0x00040000U)                           ///< bit 18
#define NVIC_ICPR_CLRPEND_19            (0x00080000U)                           ///< bit 19
#define NVIC_ICPR_CLRPEND_20            (0x00100000U)                           ///< bit 20
#define NVIC_ICPR_CLRPEND_21            (0x00200000U)                           ///< bit 21
#define NVIC_ICPR_CLRPEND_22            (0x00400000U)                           ///< bit 22
#define NVIC_ICPR_CLRPEND_23            (0x00800000U)                           ///< bit 23
#define NVIC_ICPR_CLRPEND_24            (0x01000000U)                           ///< bit 24
#define NVIC_ICPR_CLRPEND_25            (0x02000000U)                           ///< bit 25
#define NVIC_ICPR_CLRPEND_26            (0x04000000U)                           ///< bit 26
#define NVIC_ICPR_CLRPEND_27            (0x08000000U)                           ///< bit 27
#define NVIC_ICPR_CLRPEND_28            (0x10000000U)                           ///< bit 28
#define NVIC_ICPR_CLRPEND_29            (0x20000000U)                           ///< bit 29
#define NVIC_ICPR_CLRPEND_30            (0x40000000U)                           ///< bit 30
#define NVIC_ICPR_CLRPEND_31            (0x80000000U)                           ///< bit 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_IABR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IABR_ACTIVE                (0xFFFFFFFFU)                           ///< Interrupt active flags
#define NVIC_IABR_ACTIVE_0              (0x00000001U)                           ///< bit 0
#define NVIC_IABR_ACTIVE_1              (0x00000002U)                           ///< bit 1
#define NVIC_IABR_ACTIVE_2              (0x00000004U)                           ///< bit 2
#define NVIC_IABR_ACTIVE_3              (0x00000008U)                           ///< bit 3
#define NVIC_IABR_ACTIVE_4              (0x00000010U)                           ///< bit 4
#define NVIC_IABR_ACTIVE_5              (0x00000020U)                           ///< bit 5
#define NVIC_IABR_ACTIVE_6              (0x00000040U)                           ///< bit 6
#define NVIC_IABR_ACTIVE_7              (0x00000080U)                           ///< bit 7
#define NVIC_IABR_ACTIVE_8              (0x00000100U)                           ///< bit 8
#define NVIC_IABR_ACTIVE_9              (0x00000200U)                           ///< bit 9
#define NVIC_IABR_ACTIVE_10             (0x00000400U)                           ///< bit 10
#define NVIC_IABR_ACTIVE_11             (0x00000800U)                           ///< bit 11
#define NVIC_IABR_ACTIVE_12             (0x00001000U)                           ///< bit 12
#define NVIC_IABR_ACTIVE_13             (0x00002000U)                           ///< bit 13
#define NVIC_IABR_ACTIVE_14             (0x00004000U)                           ///< bit 14
#define NVIC_IABR_ACTIVE_15             (0x00008000U)                           ///< bit 15
#define NVIC_IABR_ACTIVE_16             (0x00010000U)                           ///< bit 16
#define NVIC_IABR_ACTIVE_17             (0x00020000U)                           ///< bit 17
#define NVIC_IABR_ACTIVE_18             (0x00040000U)                           ///< bit 18
#define NVIC_IABR_ACTIVE_19             (0x00080000U)                           ///< bit 19
#define NVIC_IABR_ACTIVE_20             (0x00100000U)                           ///< bit 20
#define NVIC_IABR_ACTIVE_21             (0x00200000U)                           ///< bit 21
#define NVIC_IABR_ACTIVE_22             (0x00400000U)                           ///< bit 22
#define NVIC_IABR_ACTIVE_23             (0x00800000U)                           ///< bit 23
#define NVIC_IABR_ACTIVE_24             (0x01000000U)                           ///< bit 24
#define NVIC_IABR_ACTIVE_25             (0x02000000U)                           ///< bit 25
#define NVIC_IABR_ACTIVE_26             (0x04000000U)                           ///< bit 26
#define NVIC_IABR_ACTIVE_27             (0x08000000U)                           ///< bit 27
#define NVIC_IABR_ACTIVE_28             (0x10000000U)                           ///< bit 28
#define NVIC_IABR_ACTIVE_29             (0x20000000U)                           ///< bit 29
#define NVIC_IABR_ACTIVE_30             (0x40000000U)                           ///< bit 30
#define NVIC_IABR_ACTIVE_31             (0x80000000U)                           ///< bit 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI0 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR0_PRI_0                 (0x000000FFU)                           ///< Priority of interrupt 0
#define NVIC_IPR0_PRI_1                 (0x0000FF00U)                           ///< Priority of interrupt 1
#define NVIC_IPR0_PRI_2                 (0x00FF0000U)                           ///< Priority of interrupt 2
#define NVIC_IPR0_PRI_3                 (0xFF000000U)                           ///< Priority of interrupt 3

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI1 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR1_PRI_4                 (0x000000FFU)                           ///< Priority of interrupt 4
#define NVIC_IPR1_PRI_5                 (0x0000FF00U)                           ///< Priority of interrupt 5
#define NVIC_IPR1_PRI_6                 (0x00FF0000U)                           ///< Priority of interrupt 6
#define NVIC_IPR1_PRI_7                 (0xFF000000U)                           ///< Priority of interrupt 7

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI2 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR2_PRI_8                 (0x000000FFU)                           ///< Priority of interrupt 8
#define NVIC_IPR2_PRI_9                 (0x0000FF00U)                           ///< Priority of interrupt 9
#define NVIC_IPR2_PRI_10                (0x00FF0000U)                           ///< Priority of interrupt 10
#define NVIC_IPR2_PRI_11                (0xFF000000U)                           ///< Priority of interrupt 11

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI3 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR3_PRI_12                (0x000000FFU)                           ///< Priority of interrupt 12
#define NVIC_IPR3_PRI_13                (0x0000FF00U)                           ///< Priority of interrupt 13
#define NVIC_IPR3_PRI_14                (0x00FF0000U)                           ///< Priority of interrupt 14
#define NVIC_IPR3_PRI_15                (0xFF000000U)                           ///< Priority of interrupt 15

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI4 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR4_PRI_16                (0x000000FFU)                           ///< Priority of interrupt 16
#define NVIC_IPR4_PRI_17                (0x0000FF00U)                           ///< Priority of interrupt 17
#define NVIC_IPR4_PRI_18                (0x00FF0000U)                           ///< Priority of interrupt 18
#define NVIC_IPR4_PRI_19                (0xFF000000U)                           ///< Priority of interrupt 19

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI5 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR5_PRI_20                (0x000000FFU)                           ///< Priority of interrupt 20
#define NVIC_IPR5_PRI_21                (0x0000FF00U)                           ///< Priority of interrupt 21
#define NVIC_IPR5_PRI_22                (0x00FF0000U)                           ///< Priority of interrupt 22
#define NVIC_IPR5_PRI_23                (0xFF000000U)                           ///< Priority of interrupt 23

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI6 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR6_PRI_24                (0x000000FFU)                           ///< Priority of interrupt 24
#define NVIC_IPR6_PRI_25                (0x0000FF00U)                           ///< Priority of interrupt 25
#define NVIC_IPR6_PRI_26                (0x00FF0000U)                           ///< Priority of interrupt 26
#define NVIC_IPR6_PRI_27                (0xFF000000U)                           ///< Priority of interrupt 27

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI7 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR7_PRI_28                (0x000000FFU)                           ///< Priority of interrupt 28
#define NVIC_IPR7_PRI_29                (0x0000FF00U)                           ///< Priority of interrupt 29
#define NVIC_IPR7_PRI_30                (0x00FF0000U)                           ///< Priority of interrupt 30
#define NVIC_IPR7_PRI_31                (0xFF000000U)                           ///< Priority of interrupt 31

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI8 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR7_PRI_32                (0x000000FFU)                           ///< Priority of interrupt 32
#define NVIC_IPR7_PRI_33                (0x0000FF00U)                           ///< Priority of interrupt 33
#define NVIC_IPR7_PRI_34                (0x00FF0000U)                           ///< Priority of interrupt 34
#define NVIC_IPR7_PRI_35                (0xFF000000U)                           ///< Priority of interrupt 35

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI9 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR7_PRI_36                (0x000000FFU)                           ///< Priority of interrupt 36
#define NVIC_IPR7_PRI_37                (0x0000FF00U)                           ///< Priority of interrupt 37
#define NVIC_IPR7_PRI_38                (0x00FF0000U)                           ///< Priority of interrupt 38
#define NVIC_IPR7_PRI_39                (0xFF000000U)                           ///< Priority of interrupt 39

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI10 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR7_PRI_40                (0x000000FFU)                           ///< Priority of interrupt 40
#define NVIC_IPR7_PRI_41                (0x0000FF00U)                           ///< Priority of interrupt 41
#define NVIC_IPR7_PRI_42                (0x00FF0000U)                           ///< Priority of interrupt 42
#define NVIC_IPR7_PRI_43                (0xFF000000U)                           ///< Priority of interrupt 43

////////////////////////////////////////////////////////////////////////////////
/// @brief NVIC_PRI11 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define NVIC_IPR7_PRI_44                (0x000000FFU)                           ///< Priority of interrupt 44
#define NVIC_IPR7_PRI_45                (0x0000FF00U)                           ///< Priority of interrupt 45
#define NVIC_IPR7_PRI_46                (0x00FF0000U)                           ///< Priority of interrupt 46
#define NVIC_IPR7_PRI_47                (0xFF000000U)                           ///< Priority of interrupt 47

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_CPUID Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_CPUID_REVISION              (0x0000000FU)                           ///< Implementation defined revision number
#define SCB_CPUID_PARTNO                (0x0000FFF0U)                           ///< Number of processor within family
#define SCB_CPUID_Constant              (0x000F0000U)                           ///< Reads as 0x0F
#define SCB_CPUID_VARIANT               (0x00F00000U)                           ///< Implementation defined variant number
#define SCB_CPUID_IMPLEMENTER           (0xFF000000U)                           ///< Implementer code. ARM is 0x41

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_ICSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_ICSR_VECTACTIVE             (0x000001FFU)                           ///< Active ISR number field
#define SCB_ICSR_RETTOBASE              (0x00000800U)                           ///< All active exceptions minus the IPSR_current_exception yields the empty set
#define SCB_ICSR_VECTPENDING            (0x003FF000U)                           ///< Pending ISR number field
#define SCB_ICSR_ISRPENDING             (0x00400000U)                           ///< Interrupt pending flag
#define SCB_ICSR_ISRPREEMPT             (0x00800000U)                           ///< It indicates that a pending interrupt becomes active in the next running cycle
#define SCB_ICSR_PENDSTCLR              (0x02000000U)                           ///< Clear pending SysTick bit
#define SCB_ICSR_PENDSTSET              (0x04000000U)                           ///< Set pending SysTick bit
#define SCB_ICSR_PENDSVCLR              (0x08000000U)                           ///< Clear pending pendSV bit
#define SCB_ICSR_PENDSVSET              (0x10000000U)                           ///< Set pending pendSV bit
#define SCB_ICSR_NMIPENDSET             (0x80000000U)                           ///< Set pending NMI bit

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_VTOR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_VTOR_TBLOFF                 (0x1FFFFF80U)                           ///< Vector table base offset field
#define SCB_VTOR_TBLBASE                (0x20000000U)                           ///< Table base in code(0) or RAM(1)

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_AIRCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_AIRCR_VECTRESET             (0x00000001U)                           ///< System Reset bit
#define SCB_AIRCR_VECTCLRACTIVE         (0x00000002U)                           ///< Clear active vector bit
#define SCB_AIRCR_SYSRESETREQ           (0x00000004U)                           ///< Requests chip control logic to generate a reset
#define SCB_AIRCR_PRIGROUP              (0x00000700U)                           ///< PRIGROUP[2:0] bits (Priority group)
#define SCB_AIRCR_PRIGROUP_0            (0x00000100U)                           ///< Bit 0
#define SCB_AIRCR_PRIGROUP_1            (0x00000200U)                           ///< Bit 1
#define SCB_AIRCR_PRIGROUP_2            (0x00000400U)                           ///< Bit 2

#define SCB_AIRCR_PRIGROUP0             (0x00000000U)                           ///< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority)
#define SCB_AIRCR_PRIGROUP1             (0x00000100U)                           ///< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority)
#define SCB_AIRCR_PRIGROUP2             (0x00000200U)                           ///< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority)
#define SCB_AIRCR_PRIGROUP3             (0x00000300U)                           ///< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority)
#define SCB_AIRCR_PRIGROUP4             (0x00000400U)                           ///< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority)
#define SCB_AIRCR_PRIGROUP5             (0x00000500U)                           ///< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority)
#define SCB_AIRCR_PRIGROUP6             (0x00000600U)                           ///< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority)
#define SCB_AIRCR_PRIGROUP7             (0x00000700U)                           ///< Priority group=7 (no pre-emption priority, 8 bits of subpriority)

#define SCB_AIRCR_ENDIANESS             (0x00008000U)                           ///< Data endianness bit
#define SCB_AIRCR_VECTKEY               (0xFFFF0000U)                           ///< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT)

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_SCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_SCR_SLEEPONEXIT             (0x02U)                                 ///< Sleep on exit bit
#define SCB_SCR_SLEEPDEEP               (0x04U)                                 ///< Sleep deep bit
#define SCB_SCR_SEVONPEND               (0x10U)                                 ///< Wake up from WFE

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_CCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_CCR_NONBASETHRDENA          (0x0001U)                               ///< Thread mode can be entered from any level in Handler mode by controlled return value
#define SCB_CCR_USERSETMPEND            (0x0002U)                               ///< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception
#define SCB_CCR_UNALIGN_TRP             (0x0008U)                               ///< Trap for unaligned access
#define SCB_CCR_DIV_0_TRP               (0x0010U)                               ///< Trap on Divide by 0
#define SCB_CCR_BFHFNMIGN               (0x0100U)                               ///< Handlers running at priority -1 and -2
#define SCB_CCR_STKALIGN                (0x0200U)                               ///< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_SHPR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_SHPR_PRI_N                  (0x000000FFU)                           ///< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor
#define SCB_SHPR_PRI_N1                 (0x0000FF00U)                           ///< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved
#define SCB_SHPR_PRI_N2                 (0x00FF0000U)                           ///< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV
#define SCB_SHPR_PRI_N3                 (0xFF000000U)                           ///< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_SHCSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_SHCSR_MEMFAULTACT           (0x00000001U)                           ///< MemManage is active
#define SCB_SHCSR_BUSFAULTACT           (0x00000002U)                           ///< BusFault is active
#define SCB_SHCSR_USGFAULTACT           (0x00000008U)                           ///< UsageFault is active
#define SCB_SHCSR_SVCALLACT             (0x00000080U)                           ///< SVCall is active
#define SCB_SHCSR_MONITORACT            (0x00000100U)                           ///< Monitor is active
#define SCB_SHCSR_PENDSVACT             (0x00000400U)                           ///< PendSV is active
#define SCB_SHCSR_SYSTICKACT            (0x00000800U)                           ///< SysTick is active
#define SCB_SHCSR_USGFAULTPENDED        (0x00001000U)                           ///< Usage Fault is pended
#define SCB_SHCSR_MEMFAULTPENDED        (0x00002000U)                           ///< MemManage is pended
#define SCB_SHCSR_BUSFAULTPENDED        (0x00004000U)                           ///< Bus Fault is pended
#define SCB_SHCSR_SVCALLPENDED          (0x00008000U)                           ///< SVCall is pended
#define SCB_SHCSR_MEMFAULTENA           (0x00010000U)                           ///< MemManage enable
#define SCB_SHCSR_BUSFAULTENA           (0x00020000U)                           ///< Bus Fault enable
#define SCB_SHCSR_USGFAULTENA           (0x00040000U)                           ///< UsageFault enable

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_CFSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
///< MFSR
#define SCB_CFSR_IACCVIOL               (0x00000001U)                           ///< Instruction access violation
#define SCB_CFSR_DACCVIOL               (0x00000002U)                           ///< Data access violation
#define SCB_CFSR_MUNSTKERR              (0x00000008U)                           ///< Unstacking error
#define SCB_CFSR_MSTKERR                (0x00000010U)                           ///< Stacking error
#define SCB_CFSR_MMARVALID              (0x00000080U)                           ///< Memory Manage Address Register address valid flag
///< BFSR
#define SCB_CFSR_IBUSERR                (0x00000100U)                           ///< Instruction bus error flag
#define SCB_CFSR_PRECISERR              (0x00000200U)                           ///< Precise data bus error
#define SCB_CFSR_IMPRECISERR            (0x00000400U)                           ///< Imprecise data bus error
#define SCB_CFSR_UNSTKERR               (0x00000800U)                           ///< Unstacking error
#define SCB_CFSR_STKERR                 (0x00001000U)                           ///< Stacking error
#define SCB_CFSR_BFARVALID              (0x00008000U)                           ///< Bus Fault Address Register address valid flag
///< UFSR
#define SCB_CFSR_UNDEFINSTR             (0x00010000U)                           ///< The processor attempt to excecute an undefined instruction
#define SCB_CFSR_INVSTATE               (0x00020000U)                           ///< Invalid combination of EPSR and instruction
#define SCB_CFSR_INVPC                  (0x00040000U)                           ///< Attempt to load EXC_RETURN into pc illegally
#define SCB_CFSR_NOCP                   (0x00080000U)                           ///< Attempt to use a coprocessor instruction
#define SCB_CFSR_UNALIGNED              (0x01000000U)                           ///< Fault occurs when there is an attempt to make an unaligned memory access
#define SCB_CFSR_DIVBYZERO              (0x02000000U)                           ///< Fault occurs when SDIV or DIV instruction is used with a divisor of 0

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_HFSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_HFSR_VECTTBL                (0x00000002U)                           ///< Fault occures because of vector table read on exception processing
#define SCB_HFSR_FORCED                 (0x40000000U)                           ///< Hard Fault activated when a configurable Fault was received and cannot activate
#define SCB_HFSR_DEBUGEVT               (0x80000000U)                           ///< Fault related to debug

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_DFSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_DFSR_HALTED                 (0x01U)                                 ///< Halt request flag
#define SCB_DFSR_BKPT                   (0x02U)                                 ///< BKPT flag
#define SCB_DFSR_DWTTRAP                (0x04U)                                 ///< Data Watchpoint and Trace (DWT) flag
#define SCB_DFSR_VCATCH                 (0x08U)                                 ///< Vector catch flag
#define SCB_DFSR_EXTERNAL               (0x10U)                                 ///< External debug request flag

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_MMFAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_MMFAR_ADDRESS               (0xFFFFFFFFU)                           ///< Mem Manage fault address field

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_BFAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_BFAR_ADDRESS                (0xFFFFFFFFU)                           ///< Bus fault address field

////////////////////////////////////////////////////////////////////////////////
/// @brief SCB_AFSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SCB_AFSR_IMPDEF                 (0xFFFFFFFFU)                           ///< Implementation defined

////////////////////////////////////////////////////////////////////////////////
/// @brief PWR_CR register Bit definition
////////////////////////////////////////////////////////////////////////////////

#define PWR_CR_PDDS_Pos                 (1)
#define PWR_CR_PDDS                     (0x01U  << PWR_CR_PDDS_Pos)             ///< Power Down Deepsleep
#define PWR_CR_CWUF_Pos                 (2)
#define PWR_CR_CWUF                     (0x01U  << PWR_CR_CWUF_Pos)             ///< Clear Standby Flag
#define PWR_CR_CSBF_Pos                 (3)
#define PWR_CR_CSBF                     (0x01U  << PWR_CR_CSBF_Pos)             ///< Clear Standby Flag
#define PWR_CR_PVDE_Pos                 (4)
#define PWR_CR_PVDE                     (0x01U  << PWR_CR_PVDE_Pos)             ///< Power Voltage Detector Enable

#define PWR_CR_DBP_Pos              (8)
#define PWR_CR_DBP                  (0x01U  << PWR_CR_DBP_Pos)              ///< Domain Write Protction

#define PWR_CR_PLS_Pos                  (9)
#define PWR_CR_PLS                      (0x0FU  << PWR_CR_PLS_Pos)
#define PWR_CR_PLS_1V8                  (0x00U  << PWR_CR_PLS_Pos)              ///< PVD level 1.8V
#define PWR_CR_PLS_2V1                  (0x01U  << PWR_CR_PLS_Pos)              ///< PVD level 2.1V
#define PWR_CR_PLS_2V4                  (0x02U  << PWR_CR_PLS_Pos)              ///< PVD level 2.4V
#define PWR_CR_PLS_2V7                  (0x03U  << PWR_CR_PLS_Pos)              ///< PVD level 2.7V
#define PWR_CR_PLS_3V0                  (0x04U  << PWR_CR_PLS_Pos)              ///< PVD level 3.0V
#define PWR_CR_PLS_3V3                  (0x05U  << PWR_CR_PLS_Pos)              ///< PVD level 3.3V
#define PWR_CR_PLS_3V6                  (0x06U  << PWR_CR_PLS_Pos)              ///< PVD level 3.6V
#define PWR_CR_PLS_3V9                  (0x07U  << PWR_CR_PLS_Pos)              ///< PVD level 3.9V
#define PWR_CR_PLS_4V2                  (0x08U  << PWR_CR_PLS_Pos)              ///< PVD level 4.2V
#define PWR_CR_PLS_4V5                  (0x09U  << PWR_CR_PLS_Pos)              ///< PVD level 4.5V
#define PWR_CR_PLS_4V8                  (0x0AU  << PWR_CR_PLS_Pos)              ///< PVD level 4.8V


////////////////////////////////////////////////////////////////////////////////
/// @brief PWR_CSR register Bit definition
////////////////////////////////////////////////////////////////////////////////
#define PWR_CSR_WUF_Pos                 (0)
#define PWR_CSR_WUF                     (0x01U  << PWR_CSR_WUF_Pos)             ///< Wakeup Flag
#define PWR_CSR_SBF_Pos                 (1)
#define PWR_CSR_SBF                     (0x01U  << PWR_CSR_SBF_Pos)             ///< Standby Flag
#define PWR_CSR_PVDO_Pos                (2)
#define PWR_CSR_PVDO                    (0x01U  << PWR_CSR_PVDO_Pos)            ///< PVD Output
#define PWR_CSR_EWU_Pos                 (8)
#define PWR_CSR_EWUP                    (0x01U  << PWR_CSR_EWU_Pos)             ///< Enable WKUP pin


////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_CR_HSION_Pos                (0)
#define RCC_CR_HSION                    (0x01U << RCC_CR_HSION_Pos)             ///< Internal High Speed clock enable

#define RCC_CR_HSIRDY_Pos               (1)
#define RCC_CR_HSIRDY                   (0x01U << RCC_CR_HSIRDY_Pos)            ///< Internal High Speed clock ready flag

#define  RCC_CR_HSITEN_Pos              (2)
#define  RCC_CR_HSITEN                  (0x01U << RCC_CR_HSITEN_Pos)            ///< HSI internal temperature enable


#define  RCC_CR_HSICAL_Pos              (7)
#define  RCC_CR_HSICAL                  (0x9FU << RCC_CR_HSICAL_Pos)            ///< Internal High Speed clock Calibration
#define  RCC_CR_HSEON_Pos               (16)
#define  RCC_CR_HSEON                   (0x01U << RCC_CR_HSEON_Pos)             ///< External High Speed clock enable
#define  RCC_CR_HSERDY_Pos              (17)
#define  RCC_CR_HSERDY                  (0x01U << RCC_CR_HSERDY_Pos)            ///< External High Speed clock ready flag
#define  RCC_CR_HSEBYP_Pos              (18)
#define  RCC_CR_HSEBYP                  (0x01U << RCC_CR_HSEBYP_Pos)            ///< External High Speed clock Bypass
#define  RCC_CR_CSSON_Pos               (19)
#define  RCC_CR_CSSON                   (0x01U << RCC_CR_CSSON_Pos)             ///< Clock Security System enable


#define  RCC_CR_PLLDIV_Pos              (20)
#define  RCC_CR_PLLDIV                  (0x07U << RCC_CR_PLLDIV_Pos)            ///< PLLDIV[2:0] bits
#define  RCC_CR_PLLON_Pos               (24)
#define  RCC_CR_PLLON                   (0x01U << RCC_CR_PLLON_Pos)             ///< PLL enable
#define  RCC_CR_PLLRDY_Pos              (25)
#define  RCC_CR_PLLRDY                  (0x01U << RCC_CR_PLLRDY_Pos)            ///< PLL clock ready flag
#define  RCC_CR_PLLMUL_Pos              (26)
#define  RCC_CR_PLLMUL                  (0x3FU << RCC_CR_PLLMUL_Pos)            ///< PLLMUL[5:0] bits

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_CFGR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_CFGR_SW_Pos                 (0)
#define RCC_CFGR_SW                     (0x03U << RCC_CFGR_SW_Pos)              ///< SW[1:0] bits (System clock Switch)
#define RCC_CFGR_SW_HSI_DIV6            (0x00U << RCC_CFGR_SW_Pos)              ///< HSI/6 selected as system clock
#define RCC_CFGR_SW_HSE                 (0x01U << RCC_CFGR_SW_Pos)              ///< HSE selected as system clock
#define RCC_CFGR_SW_PLL                 (0x02U << RCC_CFGR_SW_Pos)              ///< PLL selected as system clock
#define RCC_CFGR_SW_LSI                 (0x03U << RCC_CFGR_SW_Pos)              ///< LSI selected as system clock

#define RCC_CFGR_SWS_Pos                (2)
#define RCC_CFGR_SWS                    (0x03U << RCC_CFGR_SWS_Pos)             ///< SWS[1:0] bits (System Clock Switch Status)
#define RCC_CFGR_SWS_HSI_DIV6           (0x00U << RCC_CFGR_SWS_Pos)             ///< HSI/6 oscillator used as system clock
#define RCC_CFGR_SWS_HSE                (0x01U << RCC_CFGR_SWS_Pos)             ///< HSE oscillator used as system clock
#define RCC_CFGR_SWS_PLL                (0x02U << RCC_CFGR_SWS_Pos)             ///< PLL used as system clock
#define RCC_CFGR_SWS_LSI                (0x03U << RCC_CFGR_SWS_Pos)             ///< LSI used as system clock

#define RCC_CFGR_HPRE_Pos               (4)
#define RCC_CFGR_HPRE                   (0x0FU << RCC_CFGR_HPRE_Pos)            ///< HPRE[3:0] bits (AHB prescaler)
#define RCC_CFGR_HPRE_DIV1              (0x00U << RCC_CFGR_HPRE_Pos)            ///< SYSCLK not divided
#define RCC_CFGR_HPRE_DIV2              (0x08U << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 2
#define RCC_CFGR_HPRE_DIV4              (0x09U << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 4
#define RCC_CFGR_HPRE_DIV8              (0x0AU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 8
#define RCC_CFGR_HPRE_DIV16             (0x0BU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 16
#define RCC_CFGR_HPRE_DIV64             (0x0CU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 64
#define RCC_CFGR_HPRE_DIV128            (0x0DU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 128
#define RCC_CFGR_HPRE_DIV256            (0x0EU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 256
#define RCC_CFGR_HPRE_DIV512            (0x0FU << RCC_CFGR_HPRE_Pos)            ///< SYSCLK divided by 512

#define RCC_CFGR_PPRE1_Pos              (8)
#define RCC_CFGR_PPRE1                  (0x07U << RCC_CFGR_PPRE1_Pos)           ///< PRE1[2:0] bits (APB1 prescaler)
#define RCC_CFGR_PPRE1_0                (0x01U << RCC_CFGR_PPRE1_Pos)           ///< Bit 0
#define RCC_CFGR_PPRE1_1                (0x02U << RCC_CFGR_PPRE1_Pos)           ///< Bit 1
#define RCC_CFGR_PPRE1_2                (0x04U << RCC_CFGR_PPRE1_Pos)           ///< Bit 2

#define RCC_CFGR_PPRE1_DIV1             (0x00U << RCC_CFGR_PPRE1_Pos)           ///< HCLK not divided
#define RCC_CFGR_PPRE1_DIV2             (0x04U << RCC_CFGR_PPRE1_Pos)           ///< HCLK divided by 2
#define RCC_CFGR_PPRE1_DIV4             (0x05U << RCC_CFGR_PPRE1_Pos)           ///< HCLK divided by 4
#define RCC_CFGR_PPRE1_DIV8             (0x06U << RCC_CFGR_PPRE1_Pos)           ///< HCLK divided by 8
#define RCC_CFGR_PPRE1_DIV16            (0x07U << RCC_CFGR_PPRE1_Pos)           ///< HCLK divided by 16

#define RCC_CFGR_PPRE2_Pos              (11)
#define RCC_CFGR_PPRE2                  (0x07U << RCC_CFGR_PPRE2_Pos)           ///< PRE2[2:0] bits (APB2 prescaler)
#define RCC_CFGR_PPRE2_DIV1             (0x00U << RCC_CFGR_PPRE2_Pos)           ///< HCLK not divided
#define RCC_CFGR_PPRE2_DIV2             (0x04U << RCC_CFGR_PPRE2_Pos)           ///< HCLK divided by 2
#define RCC_CFGR_PPRE2_DIV4             (0x05U << RCC_CFGR_PPRE2_Pos)           ///< HCLK divided by 4
#define RCC_CFGR_PPRE2_DIV8             (0x06U << RCC_CFGR_PPRE2_Pos)           ///< HCLK divided by 8
#define RCC_CFGR_PPRE2_DIV16            (0x07U << RCC_CFGR_PPRE2_Pos)           ///< HCLK divided by 16

#define RCC_CFGR_CKOFF_Pos              (14)
#define RCC_CFGR_CKOFF                  (0x01U << RCC_CFGR_CKOFF_Pos)           ///< Disable hardware to turn off the clock automatically

#define RCC_CFGR_PLLSRC_Pos         (16)
#define RCC_CFGR_PLLSRC             (0x01U << RCC_CFGR_PLLSRC_Pos)          ///< PLL entry clock source
#define RCC_CFGR_PLLXTPRE_Pos       (17)
#define RCC_CFGR_PLLXTPRE           (0x01U << RCC_CFGR_PLLXTPRE_Pos)        ///< HSE divider for PLL entry




#define RCC_CFGR_MCO_Pos            (24)
#define RCC_CFGR_MCO                (0x07U << RCC_CFGR_MCO_Pos)             ///< MCO[2:0] bits (Microcontroller Clock Output)
#define RCC_CFGR_MCO_NOCLOCK        (0x00U << RCC_CFGR_MCO_Pos)             ///< No clock
#define RCC_CFGR_MCO_HSI_DIV4       (0x01U << RCC_CFGR_MCO_Pos)             ///< HSI clock divided by 4 selected
#define RCC_CFGR_MCO_LSI            (0x02U << RCC_CFGR_MCO_Pos)             ///< LSI clock
#define RCC_CFGR_MCO_LSE            (0x03U << RCC_CFGR_MCO_Pos)             ///< LSE clock
#define RCC_CFGR_MCO_SYSCLK         (0x04U << RCC_CFGR_MCO_Pos)             ///< System clock selected
#define RCC_CFGR_MCO_HSI            (0x05U << RCC_CFGR_MCO_Pos)             ///< Internal 48 MHz RC oscillator clock selected
#define RCC_CFGR_MCO_HSE            (0x06U << RCC_CFGR_MCO_Pos)             ///< External 1-25 MHz oscillator clock selected
#define RCC_CFGR_MCO_PLL            (0x07U << RCC_CFGR_MCO_Pos)             ///< PLL clock divided by 2 selected
#define RCC_CFGR_MCO_PLLDIV2        (0x07U << RCC_CFGR_MCO_Pos)             ///< PLL clock divided by 2 selected
#define RCC_CFGR_MCO_PLLDIV12       (0x08U << RCC_CFGR_MCO_Pos)             ///< PLL clock divided by 12 selected
#define RCC_CFGR_MCO_PLLDIV24       (0x09U << RCC_CFGR_MCO_Pos)             ///< PLL clock divided by 24 selected
#define RCC_CFGR_PLLMUL_Pos         (30)
#define RCC_CFGR_PLLMUL             (0x11U << RCC_CFGR_PLLMUL_Pos)          ///< PLLMUL[1:0] bits
////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_CIR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_CIR_LSIRDYF_Pos             (0)
#define RCC_CIR_LSIRDYF                 (0x01U << RCC_CIR_LSIRDYF_Pos)          ///< LSI Ready Interrupt flag

#define RCC_CIR_LSERDYF_Pos         (1)
#define RCC_CIR_LSERDYF             (0x01U << RCC_CIR_LSERDYF_Pos)          ///< LSE Ready Interrupt flag

#define RCC_CIR_HSIRDYF_Pos             (2)
#define RCC_CIR_HSIRDYF                 (0x01U << RCC_CIR_HSIRDYF_Pos)          ///< HSI Ready Interrupt flag
#define RCC_CIR_HSERDYF_Pos             (3)
#define RCC_CIR_HSERDYF                 (0x01U << RCC_CIR_HSERDYF_Pos)          ///< HSE Ready Interrupt flag

#define RCC_CIR_PLLRDYF_Pos         (4)
#define RCC_CIR_PLLRDYF             (0x01U << RCC_CIR_PLLRDYF_Pos)          ///< PLL Ready Interrupt flag

#define RCC_CIR_CSSF_Pos                (7)
#define RCC_CIR_CSSF                    (0x01U << RCC_CIR_CSSF_Pos)             ///< Clock Security System Interrupt flag
#define RCC_CIR_LSIRDYIE_Pos            (8)
#define RCC_CIR_LSIRDYIE                (0x01U << RCC_CIR_LSIRDYIE_Pos)         ///< LSI Ready Interrupt Enable

#define RCC_CIR_LSERDYIE_Pos        (9)
#define RCC_CIR_LSERDYIE            (0x01U << RCC_CIR_LSERDYIE_Pos)         ///< LSE Ready Interrupt Enable

#define RCC_CIR_HSIRDYIE_Pos            (10)
#define RCC_CIR_HSIRDYIE                (0x01U << RCC_CIR_HSIRDYIE_Pos)         ///< HSI Ready Interrupt Enable
#define RCC_CIR_HSERDYIE_Pos            (11)
#define RCC_CIR_HSERDYIE                (0x01U << RCC_CIR_HSIRDYIE_Pos)         ///< HSE Ready Interrupt Enable

#define RCC_CIR_PLLRDYIE_Pos        (12)
#define RCC_CIR_PLLRDYIE            (0x01U << RCC_CIR_PLLRDYIE_Pos)         ///< PLL Ready Interrupt Enable

#define RCC_CIR_LSIRDYC_Pos             (16)
#define RCC_CIR_LSIRDYC                 (0x01U << RCC_CIR_LSIRDYC_Pos)          ///< LSI Ready Interrupt Clear

#define RCC_CIR_LSERDYC_Pos         (17)
#define RCC_CIR_LSERDYC             (0x01U << RCC_CIR_LSERDYC_Pos)          ///< LSE Ready Interrupt Clear

#define RCC_CIR_HSIRDYC_Pos             (18)
#define RCC_CIR_HSIRDYC                 (0x01U << RCC_CIR_HSIRDYC_Pos)          ///< HSI Ready Interrupt Clear
#define RCC_CIR_HSERDYC_Pos             (19)
#define RCC_CIR_HSERDYC                 (0x01U << RCC_CIR_HSERDYC_Pos)          ///< HSE Ready Interrupt Clear

#define RCC_CIR_PLLRDYC_Pos         (20)
#define RCC_CIR_PLLRDYC             (0x01U << RCC_CIR_PLLRDYC_Pos)          ///< PLL Ready Interrupt Clear

#define RCC_CIR_CSSC_Pos                (23)
#define RCC_CIR_CSSC                    (0x01U << RCC_CIR_CSSC_Pos)             ///< Clock Security System Interrupt Clear

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_APB2RSTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_APB2RSTR_EXTI_Pos       (0)
#define RCC_APB2RSTR_EXTI           (0x01U << RCC_APB2RSTR_EXTI_Pos)        ///< System Configuration register reset


#define RCC_APB2RSTR_ADC1_Pos       (9)
#define RCC_APB2RSTR_ADC1           (0x01U << RCC_APB2RSTR_ADC1_Pos)        ///< ADC interface reset

#define RCC_APB2RSTR_TIM1_Pos           (11)
#define RCC_APB2RSTR_TIM1               (0x01U << RCC_APB2RSTR_TIM1_Pos)        ///< TIM1 Timer reset
#define RCC_APB2RSTR_SPI1_Pos           (12)
#define RCC_APB2RSTR_SPI1               (0x01U << RCC_APB2RSTR_SPI1_Pos)        ///< SPI 1 reset
#define RCC_APB2RSTR_UART1_Pos          (14)
#define RCC_APB2RSTR_UART1              (0x01U << RCC_APB2RSTR_UART1_Pos)       ///< UART1 reset
#define RCC_APB2RSTR_TIM14_Pos      (16)
#define RCC_APB2RSTR_TIM14          (0x01U << RCC_APB2RSTR_TIM14_Pos)       ///< TIM14 Timer reset
#define RCC_APB2RSTR_TIM16_Pos      (17)
#define RCC_APB2RSTR_TIM16          (0x01U << RCC_APB2RSTR_TIM16_Pos)       ///< TIM16 Timer reset
#define RCC_APB2RSTR_TIM17_Pos      (18)
#define RCC_APB2RSTR_TIM17          (0x01U << RCC_APB2RSTR_TIM17_Pos)       ///< TIM17 Timer reset
#define RCC_APB2RSTR_DBGMCU_Pos     (22)
#define RCC_APB2RSTR_DBGMCU         (0x01U << RCC_APB2RSTR_DBGMCU_Pos)      ///< DBGMCU reset

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_AHB3RSTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_APB1RSTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_APB1RSTR_TIM2_Pos           (0)
#define RCC_APB1RSTR_TIM2               (0x01U << RCC_APB1RSTR_TIM2_Pos)        ///< Timer 2 reset
#define RCC_APB1RSTR_TIM3_Pos           (1)
#define RCC_APB1RSTR_TIM3               (0x01U << RCC_APB1RSTR_TIM3_Pos)        ///< Timer 3 reset
#define RCC_APB1RSTR_WWDG_Pos           (11)
#define RCC_APB1RSTR_WWDG               (0x01U << RCC_APB1RSTR_WWDG_Pos)        ///< Window Watchdog reset
#define RCC_APB1RSTR_SPI2_Pos           (14)
#define RCC_APB1RSTR_SPI2               (0x01U << RCC_APB1RSTR_SPI2_Pos)        ///< SPI 2 reset
#define RCC_APB1RSTR_UART2_Pos          (17)
#define RCC_APB1RSTR_UART2              (0x01U << RCC_APB1RSTR_UART2_Pos)       ///< UART 2 reset
#define RCC_APB1RSTR_I2C1_Pos           (21)
#define RCC_APB1RSTR_I2C1               (0x01U << RCC_APB1RSTR_I2C1_Pos)        ///< I2C 1 reset


#define RCC_APB1RSTR_PWR_Pos        (28)
#define RCC_APB1RSTR_PWR            (0x01U << RCC_APB1RSTR_PWR_Pos)         ///< Power interface reset
////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_AHB2RSTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_AHBENR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_AHBENR_DMA1_Pos             (0)
#define RCC_AHBENR_DMA1                 (0x01U << RCC_AHBENR_DMA1_Pos)          ///< DMA clock enable
#define RCC_AHBENR_SRAM_Pos             (2)
#define RCC_AHBENR_SRAM                 (0x01U << RCC_AHBENR_SRAM_Pos)          ///< SRAM interface clock enable
#define RCC_AHBENR_FLITF_Pos            (4)
#define RCC_AHBENR_FLITF                (0x01U << RCC_AHBENR_FLITF_Pos)         ///< FLITF clock enable
#define RCC_AHBENR_CRC_Pos              (6)
#define RCC_AHBENR_CRC                  (0x01U << RCC_AHBENR_CRC_Pos)           ///< Internal High Speed clock Calibration


#define RCC_AHBENR_GPIOA_Pos        (17)
#define RCC_AHBENR_GPIOA            (0x01U << RCC_AHBENR_GPIOA_Pos)         ///< GPIOA clock enable
#define RCC_AHBENR_GPIOB_Pos        (18)
#define RCC_AHBENR_GPIOB            (0x01U << RCC_AHBENR_GPIOB_Pos)         ///< GPIOB clock enable
#define RCC_AHBENR_GPIOC_Pos        (19)
#define RCC_AHBENR_GPIOC            (0x01U << RCC_AHBENR_GPIOC_Pos)         ///< GPIOC clock enable
#define RCC_AHBENR_GPIOD_Pos        (20)
#define RCC_AHBENR_GPIOD            (0x01U << RCC_AHBENR_GPIOD_Pos)         ///< GPIOD clock enable




////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_APB2ENR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_APB2ENR_EXTI_Pos        (0)
#define RCC_APB2ENR_EXTI            (0x01U << RCC_APB2ENR_EXTI_Pos)         ///< EXTI register enable


#define RCC_APB2ENR_ADC1_Pos            (9)
#define RCC_APB2ENR_ADC1                (0x01U << RCC_APB2ENR_ADC1_Pos)         ///< ADC 1 interface clock enable



#define RCC_APB2ENR_TIM1_Pos            (11)
#define RCC_APB2ENR_TIM1                (0x01U << RCC_APB2ENR_TIM1_Pos)         ///< TIM1 Timer clock enable
#define RCC_APB2ENR_SPI1_Pos            (12)
#define RCC_APB2ENR_SPI1                (0x01U << RCC_APB2ENR_SPI1_Pos)         ///< SPI 1 clock enable
#define RCC_APB2ENR_UART1_Pos           (14)
#define RCC_APB2ENR_UART1               (0x01U << RCC_APB2ENR_UART1_Pos)        ///< UART1 clock enable
#define RCC_APB2ENR_COMP_Pos            (15)
#define RCC_APB2ENR_COMP                (0x01U << RCC_APB2ENR_COMP_Pos)         ///< Comparator interface clock enable

#define RCC_APB2ENR_TIM14_Pos       (16)
#define RCC_APB2ENR_TIM14           (0x01U << RCC_APB2ENR_TIM14_Pos)        ///< TIM14 Timer clock enable

#define RCC_APB2ENR_TIM16_Pos       (17)
#define RCC_APB2ENR_TIM16           (0x01U << RCC_APB2ENR_TIM16_Pos)        ///< TIM16 Timer clock enable
#define RCC_APB2ENR_TIM17_Pos       (18)
#define RCC_APB2ENR_TIM17           (0x01U << RCC_APB2ENR_TIM17_Pos)        ///< TIM17 Timer clock enable
#define RCC_APB2ENR_DBGMCU_Pos      (22)
#define RCC_APB2ENR_DBGMCU          (0x01U << RCC_APB2ENR_DBGMCU_Pos)       ///< DBGMCU clock enable


////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_APB1ENR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_APB1ENR_TIM2_Pos            (0)
#define RCC_APB1ENR_TIM2                (0x01U << RCC_APB1ENR_TIM2_Pos)         ///< Timer 2 clock enable
#define RCC_APB1ENR_TIM3_Pos            (1)
#define RCC_APB1ENR_TIM3                (0x01U << RCC_APB1ENR_TIM3_Pos)         ///< Timer 3 clock enabled


#define RCC_APB1ENR_WWDG_Pos            (11)
#define RCC_APB1ENR_WWDG                (0x01U << RCC_APB1ENR_WWDG_Pos)         ///< Window Watchdog clock enable

#define RCC_APB1ENR_SPI2_Pos            (14)
#define RCC_APB1ENR_SPI2                (0x01U << RCC_APB1ENR_SPI2_Pos)         ///< SPI 2 clock enable

#define RCC_APB1ENR_UART2_Pos           (17)
#define RCC_APB1ENR_UART2               (0x01U << RCC_APB1ENR_UART2_Pos)        ///< UART 2 clock enable


#define RCC_APB1ENR_I2C1_Pos            (21)
#define RCC_APB1ENR_I2C1                (0x01U << RCC_APB1ENR_I2C1_Pos)         ///< I2C 1 clock enable


#define RCC_APB1ENR_BKP_Pos         (27)
#define RCC_APB1ENR_BKP             (0x01U << RCC_APB1ENR_BKP_Pos)          ///< Backup interface clock enable

#define RCC_APB1ENR_PWR_Pos             (28)
#define RCC_APB1ENR_PWR                 (0x01U << RCC_APB1ENR_PWR_Pos)          ///< Power interface clock enable


////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_BDCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_BDCR_LSEON_Pos          (0)
#define RCC_BDCR_LSEON              (0x01U << RCC_BDCR_LSEON_Pos)           ///< External Low Speed oscillator enable
#define RCC_BDCR_LSERDY_Pos         (1)
#define RCC_BDCR_LSERDY             (0x01U << RCC_BDCR_LSERDY_Pos)          ///< External Low Speed oscillator Ready
#define RCC_BDCR_LSEBYP_Pos         (2)
#define RCC_BDCR_LSEBYP             (0x01U << RCC_BDCR_LSEBYP_Pos)          ///< External Low Speed oscillator Bypass

#define RCC_BDCR_RTCSEL_Pos         (8)
#define RCC_BDCR_RTCSEL             (0x03U << RCC_BDCR_RTCSEL_Pos)          ///< RTCSEL[1:0] bits (RTC clock source selection)
#define RCC_BDCR_RTCSEL_LSE         (0x01U << RCC_BDCR_RTCSEL_Pos)          ///< LSE oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_LSI         (0x02U << RCC_BDCR_RTCSEL_Pos)          ///< LSI oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_HSE         (0x03U << RCC_BDCR_RTCSEL_Pos)          ///< HSE oscillator clock divided by 128 used as RTC clock

#define RCC_BDCR_RTCEN_Pos          (15)
#define RCC_BDCR_RTCEN              (0x01U << RCC_BDCR_RTCEN_Pos)           ///< RTC clock enable
#define RCC_BDCR_BDRST_Pos          (16)
#define RCC_BDCR_BDRST              (0x01U << RCC_BDCR_BDRST_Pos)           ///< Backup domain software reset

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_CSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_CSR_LSION_Pos               (0)
#define RCC_CSR_LSION                   (0x01U << RCC_CSR_LSION_Pos)            ///< Internal Low Speed oscillator enable
#define RCC_CSR_LSIRDY_Pos              (1)
#define RCC_CSR_LSIRDY                  (0x01U << RCC_CSR_LSIRDY_Pos)           ///< Internal Low Speed oscillator Ready
#define RCC_CSR_RMVF_Pos                (24)
#define RCC_CSR_RMVF                    (0x01U << RCC_CSR_RMVF_Pos)             ///< Remove reset flag
#define RCC_CSR_PINRSTF_Pos             (26)
#define RCC_CSR_PINRSTF                 (0x01U << RCC_CSR_PINRSTF_Pos)          ///< PIN reset flag

#define RCC_CSR_PORRSTF_Pos             (27)
#define RCC_CSR_PORRSTF                 (0x01U << RCC_CSR_PORRSTF_Pos)          ///< POR/PDR reset flag

#define RCC_CSR_SFTRSTF_Pos             (28)
#define RCC_CSR_SFTRSTF                 (0x01U << RCC_CSR_SFTRSTF_Pos)          ///< Software Reset flag

#define RCC_CSR_IWDGRSTF_Pos            (29)
#define RCC_CSR_IWDGRSTF                (0x01U << RCC_CSR_IWDGRSTF_Pos)         ///< Independent Watchdog reset flag

#define RCC_CSR_WWDGRSTF_Pos            (30)
#define RCC_CSR_WWDGRSTF                (0x01U << RCC_CSR_WWDGRSTF_Pos)         ///< Window watchdog reset flag

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_AHBRSTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_AHBRSTR_DMA1_Pos        (0)
#define RCC_AHBRSTR_DMA1            (0x01U << RCC_AHBRSTR_DMA1_Pos)         ///< DMA clock reset
#define RCC_AHBRSTR_SRAM_Pos        (2)
#define RCC_AHBRSTR_SRAM            (0x01U << RCC_AHBRSTR_SRAM_Pos)         ///< SRAM interface clock reset
#define RCC_AHBRSTR_FLITF_Pos       (4)
#define RCC_AHBRSTR_FLITF           (0x01U << RCC_AHBRSTR_FLITF_Pos)        ///< FLITF clock reset
#define RCC_AHBRSTR_CRC_Pos         (6)
#define RCC_AHBRSTR_CRC             (0x01U << RCC_AHBRSTR_CRC_Pos)          ///< Internal High Speed clock Calibration
#define RCC_AHBRSTR_GPIOA_Pos       (17)
#define RCC_AHBRSTR_GPIOA           (0x01U << RCC_AHBRSTR_GPIOA_Pos)        ///< GPIOA clock reset
#define RCC_AHBRSTR_GPIOB_Pos       (18)
#define RCC_AHBRSTR_GPIOB           (0x01U << RCC_AHBRSTR_GPIOB_Pos)        ///< GPIOB clock reset
#define RCC_AHBRSTR_GPIOC_Pos       (19)
#define RCC_AHBRSTR_GPIOC           (0x01U << RCC_AHBRSTR_GPIOC_Pos)        ///< GPIOC clock reset
#define RCC_AHBRSTR_GPIOD_Pos       (20)
#define RCC_AHBRSTR_GPIOD           (0x01U << RCC_AHBRSTR_GPIOD_Pos)        ///< GPIOD clock reset

////////////////////////////////////////////////////////////////////////////////
/// @brief RCC_EXTI Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RCC_CONFIG_PAGESIZE_Pos         (1)
#define RCC_CONFIG_PAGESIZE             (0x01U << RCC_CONFIG_PAGESIZE_Pos)      ///< Flash Page size
#define RCC_CONFIG_OSC_RTRIM_Pos        (8)
#define RCC_CONFIG_OSC_RTRIM            (0x07U << RCC_CONFIG_OSC_RTRIM_Pos)     ///< Oscillator feedback resistance trimming
#define RCC_CONFIG_OSC_ITRIM_Pos        (11)
#define RCC_CONFIG_OSC_ITRIM            (0x03U << RCC_CONFIG_OSC_ITRIM_Pos)     ///< Oscillator drive current trimming
#define RCC_CONFIG_OSC_LPFEN_Pos        (14)
#define RCC_CONFIG_OSC_LPFEN            (0x01U << RCC_CONFIG_OSC_LPFEN_Pos)     ///< Oscillator low pass filtering enable
////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_CR_SECIE_Pos            (0)
#define RTC_CR_SECIE                (0x01U << RTC_CR_SECIE_Pos)             ///< Second Interrupt Enable
#define RTC_CR_ALRIE_Pos            (1)
#define RTC_CR_ALRIE                (0x01U << RTC_CR_ALRIE_Pos)             ///< Alarm Interrupt Enable
#define RTC_CR_OWIE_Pos             (2)
#define RTC_CR_OWIE                 (0x01U << RTC_CR_OWIE_Pos)              ///< OverfloW Interrupt Enable

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_CSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_CSR_SECF_Pos            (0)
#define RTC_CSR_SECF                (0x01 << RTC_CSR_SECF_Pos)              ///< Second Flag
#define RTC_CSR_ALRF_Pos            (1)
#define RTC_CSR_ALRF                (0x01 << RTC_CSR_ALRF_Pos)              ///< Alarm Flag
#define RTC_CSR_OWF_Pos             (2)
#define RTC_CSR_OWF                 (0x01 << RTC_CSR_OWF_Pos)               ///< OverfloW Flag
#define RTC_CSR_RSF_Pos             (3)
#define RTC_CSR_RSF                 (0x01 << RTC_CSR_RSF_Pos)               ///< Registers Synchronized Flag
#define RTC_CSR_CNF_Pos             (4)
#define RTC_CSR_CNF                 (0x01 << RTC_CSR_CNF_Pos)               ///< Configuration Flag
#define RTC_CSR_RTOFF_Pos           (5)
#define RTC_CSR_RTOFF               (0x01 << RTC_CSR_RTOFF_Pos)             ///< RTC operation OFF
#define RTC_CSR_ALPEN_Pos           (6)
#define RTC_CSR_ALPEN               (0x01 << RTC_CSR_ALPEN_Pos)             ///< RTC Alarm Loop Enable
////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_PRLH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_PRLH_PRL_Pos            (0)
#define RTC_PRLH_PRL                (0x0F << RTC_PRLH_PRL_Pos)              ///< RTC Prescaler Reload Value High

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_PRLL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_PRLL_PRL_Pos            (0)
#define RTC_PRLL_PRL                (0xFFFFU << RTC_PRLL_PRL_Pos)           ///< RTC Prescaler Reload Value Low

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_DIVH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_DIVH_DIV_Pos            (0)
#define RTC_DIVH_DIV                (0x0F << RTC_DIVH_DIV_Pos)              ///< RTC Clock Divider High

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_DIVL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_DIVL_DIV_Pos            (0)
#define RTC_DIVL_DIV                (0xFFFFU << RTC_DIVL_DIV_Pos)           ///< RTC Clock Divider Low

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_CNTH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_CNTH_CNT_Pos            (0)
#define RTC_CNTH_CNT                (0xFFFFU << RTC_CNTH_CNT_Pos)           ///< RTC Counter High

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_CNTL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_CNTL_CNT_Pos            (0)
#define RTC_CNTL_CNT                (0xFFFFU << RTC_CNTL_CNT_Pos)           ///< RTC Counter Low

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_ALRH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_ALRH_ALR_Pos            (0)
#define RTC_ALRH_ALR                (0xFFFFU << RTC_ALRH_ALR_Pos)           ///< RTC Alarm High

////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_ALRL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_ALRL_ALR_Pos            (0)
#define RTC_ALRL_ALR                (0xFFFFU << RTC_ALRL_ALR_Pos)           ///< RTC Alarm Low


////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_MSRH Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_MSRH_MSR_Pos            (0)
#define RTC_MSRH_MSR                (0xFFFFU << RTC_MSRH_MSR_Pos)          ///< RTC MS Alarm Register High


////////////////////////////////////////////////////////////////////////////////
/// @brief RTC_MSRL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define RTC_MSRL_MSR_Pos            (0)
#define RTC_MSRL_MSR                (0xFFFFU << RTC_MSRL_MSR_Pos)           ///< RTC MS Alarm Register Low
////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_TDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_TDR_TXREG_Pos               (0)
#define SPI_TDR_TXREG                   (0xFFFFFFFFU << SPI_TDR_TXREG_Pos)      ///< Transmit data register

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_RDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_RDR_RXREG_Pos               (0)
#define SPI_RDR_RXREG                   (0xFFFFFFFFU << SPI_RDR_RXREG_Pos)      ///< Receive data register

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_SR_TXEPT_Pos                (0)
#define SPI_SR_TXEPT                    (0x01U << SPI_SR_TXEPT_Pos)             ///< Transmitter empty bit
#define SPI_SR_RXAVL_Pos                (1)
#define SPI_SR_RXAVL                    (0x01U << SPI_SR_RXAVL_Pos)             ///< Receive available byte data message
#define SPI_SR_TXFULL_Pos               (2)
#define SPI_SR_TXFULL                   (0x01U << SPI_SR_TXFULL_Pos)            ///< Transmitter FIFO full status bit
#define SPI_SR_RXAVL_4BYTE_Pos          (3)
#define SPI_SR_RXAVL_4BYTE              (0x01U << SPI_SR_RXAVL_4BYTE_Pos)       ///< Receive available 4 byte data message
#define SPI_SR_TXFADDR_Pos              (4)
#define SPI_SR_TXFADDR                  (0x0FU << SPI_SR_TXFADDR_Pos)           ///< Transmit FIFO address
#define SPI_SR_RXFADDR_Pos              (8)
#define SPI_SR_RXFADDR                  (0x0FU << SPI_SR_RXFADDR_Pos)           ///< Receive FIFO address

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_ISR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_ISR_TX_INTF_Pos             (0)
#define SPI_ISR_TX_INTF                 (0x01U << SPI_ISR_TX_INTF_Pos)          ///<  Transmit FIFO available interrupt flag bit
#define SPI_ISR_RX_INTF_Pos             (1)
#define SPI_ISR_RX_INTF                 (0x01U << SPI_ISR_RX_INTF_Pos)          ///<  Receive data available interrupt flag bit
#define SPI_ISR_UNDERRUN_INTF_Pos       (2)
#define SPI_ISR_UNDERRUN_INTF           (0x01U << SPI_ISR_UNDERRUN_INTF_Pos)    ///<  SPI underrun interrupt flag bit
#define SPI_ISR_RXOERR_INTF_Pos         (3)
#define SPI_ISR_RXOERR_INTF             (0x01U << SPI_ISR_RXOERR_INTF_Pos)      ///<  Receive overrun error interrupt flag bit
#define SPI_ISR_RXMATCH_INTF_Pos        (4)
#define SPI_ISR_RXMATCH_INTF            (0x01U << SPI_ISR_RXMATCH_INTF_Pos)     ///<  Receive data match the RXDNR number, the receive process will be completed and generate the interrupt
#define SPI_ISR_RXFULL_INTF_Pos         (5)
#define SPI_ISR_RXFULL_INTF             (0x01U << SPI_ISR_RXFULL_INTF_Pos)      ///<  RX FIFO full interrupt flag bit
#define SPI_ISR_TXEPT_INTF_Pos          (6)
#define SPI_ISR_TXEPT_INTF              (0x01U << SPI_ISR_TXEPT_INTF_Pos)       ///<  Transmitter empty interrupt flag bit

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_IER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_IER_TX_IEN_Pos              (0)
#define SPI_IER_TX_IEN                  (0x01U << SPI_IER_TX_IEN_Pos)           ///<  Transmit FIFO empty interrupt enable bit
#define SPI_IER_RX_IEN_Pos              (1)
#define SPI_IER_RX_IEN                  (0x01U << SPI_IER_RX_IEN_Pos)           ///<  Receive FIFO interrupt enable bit
#define SPI_IER_UNDERRUN_IEN_Pos        (2)
#define SPI_IER_UNDERRUN_IEN            (0x01U << SPI_IER_UNDERRUN_IEN_Pos)     ///<  Transmitter underrun interrupt enable bit
#define SPI_IER_RXOERR_IEN_Pos          (3)
#define SPI_IER_RXOERR_IEN              (0x01U << SPI_IER_RXOERR_IEN_Pos)       ///<  Overrun error interrupt enable bit
#define SPI_IER_RXMATCH_IEN_Pos         (4)
#define SPI_IER_RXMATCH_IEN             (0x01U << SPI_IER_RXMATCH_IEN_Pos)      ///<  Receive data complete interrupt enable bit
#define SPI_IER_RXFULL_IEN_Pos          (5)
#define SPI_IER_RXFULL_IEN              (0x01U << SPI_IER_RXFULL_IEN_Pos)       ///<  Receive FIFO full interrupt enable bit
#define SPI_IER_TXEPT_IEN_Pos           (6)
#define SPI_IER_TXEPT_IEN               (0x01U << SPI_IER_TXEPT_IEN_Pos)        ///<  Transmit empty interrupt enable bit

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_ICR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_ICR_TX_ICLR_Pos             (0)
#define SPI_ICR_TX_ICLR                 (0x01U << SPI_ICR_TX_ICLR_Pos)          ///<  Transmitter FIFO empty interrupt clear bit
#define SPI_ICR_RX_ICLR_Pos             (1)
#define SPI_ICR_RX_ICLR                 (0x01U << SPI_ICR_RX_ICLR_Pos)          ///<  Receive interrupt clear bit
#define SPI_ICR_UNDERRUN_ICLR_Pos       (2)
#define SPI_ICR_UNDERRUN_ICLR           (0x01U << SPI_ICR_UNDERRUN_ICLR_Pos)    ///<  Transmitter underrun interrupt clear bit
#define SPI_ICR_RXOERR_ICLR_Pos         (3)
#define SPI_ICR_RXOERR_ICLR             (0x01U << SPI_ICR_RXOERR_ICLR_Pos)      ///<  Overrun error interrupt clear bit
#define SPI_ICR_RXMATCH_ICLR_Pos        (4)
#define SPI_ICR_RXMATCH_ICLR            (0x01U << SPI_ICR_RXMATCH_ICLR_Pos)     ///<  Receive completed interrupt clear bit
#define SPI_ICR_RXFULL_ICLR_Pos         (5)
#define SPI_ICR_RXFULL_ICLR             (0x01U << SPI_ICR_RXFULL_ICLR_Pos)      ///<  Receiver buffer full interrupt clear bit
#define SPI_ICR_TXEPT_ICLR_Pos          (6)
#define SPI_ICR_TXEPT_ICLR              (0x01U << SPI_ICR_TXEPT_ICLR_Pos)       ///<  Transmitter empty interrupt clear bit

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_GCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_GCR_SPIEN_Pos               (0)
#define SPI_GCR_SPIEN                   (0x01U << SPI_GCR_SPIEN_Pos)            ///<  SPI select bit
#define SPI_GCR_IEN_Pos                 (1)
#define SPI_GCR_IEN                     (0x01U << SPI_GCR_IEN_Pos)              ///<  SPI interrupt enable bit
#define SPI_GCR_MODE_Pos                (2)
#define SPI_GCR_MODE                    (0x01U << SPI_GCR_MODE_Pos)             ///<  Master mode bit
#define SPI_GCR_TXEN_Pos                (3)
#define SPI_GCR_TXEN                    (0x01U << SPI_GCR_TXEN_Pos)             ///<  Transmit enable bit
#define SPI_GCR_RXEN_Pos                (4)
#define SPI_GCR_RXEN                    (0x01U << SPI_GCR_RXEN_Pos)             ///<  Receive enable bit

#define SPI_GCR_RXTLF_Pos               (5)
#define SPI_GCR_RXTLF                   (0x03U << SPI_GCR_RXTLF_Pos)            ///<  RX FIFO trigger level bit
#define SPI_GCR_RXTLF_One               (0x00U << SPI_GCR_RXTLF_Pos)            ///<
#define SPI_GCR_RXTLF_Half              (0x01U << SPI_GCR_RXTLF_Pos)            ///<

#define SPI_GCR_TXTLF_Pos               (7)
#define SPI_GCR_TXTLF                   (0x03U << SPI_GCR_TXTLF_Pos)            ///<  TX FIFO trigger level bit
#define SPI_GCR_TXTLF_One               (0x00U << SPI_GCR_TXTLF_Pos)            ///<
#define SPI_GCR_TXTLF_Half              (0x01U << SPI_GCR_TXTLF_Pos)            ///<

#define SPI_GCR_DMAEN_Pos               (9)
#define SPI_GCR_DMAEN                   (0x01U << SPI_GCR_DMAEN_Pos)            ///<  DMA access mode enable
#define SPI_GCR_NSS_Pos                 (10)
#define SPI_GCR_NSS                     (0x01U << SPI_GCR_NSS_Pos)              ///<  NSS select signal that from software or hardware
#define SPI_GCR_DWSEL_Pos               (11)
#define SPI_GCR_DWSEL                   (0x01U << SPI_GCR_DWSEL_Pos)            ///<  Valid byte or double-word data select signal

#define SPI_GCR_NSSTOG_Pos          (12)
#define SPI_GCR_NSSTOG              (0x01U << SPI_GCR_NSSTOG_Pos)           ///<  Slave select toggle

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_CCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_CCR_CPHA_Pos                (0)
#define SPI_CCR_CPHA                    (0x01U << SPI_CCR_CPHA_Pos)             ///<  Clock phase select bit
#define SPI_CCR_CPOL_Pos                (1)
#define SPI_CCR_CPOL                    (0x01U << SPI_CCR_CPOL_Pos)             ///<  Clock polarity select bit
#define SPI_CCR_LSBFE_Pos               (2)
#define SPI_CCR_LSBFE                   (0x01U << SPI_CCR_LSBFE_Pos)            ///<  LSI first enable bit
#define SPI_CCR_SPILEN_Pos              (3)
#define SPI_CCR_SPILEN                  (0x01U << SPI_CCR_SPILEN_Pos)           ///<  SPI character length bit
#define SPI_CCR_RXEDGE_Pos              (4)
#define SPI_CCR_RXEDGE                  (0x01U << SPI_CCR_RXEDGE_Pos)           ///<  Receive data edge select
#define SPI_CCR_TXEDGE_Pos              (5)
#define SPI_CCR_TXEDGE                  (0x01U << SPI_CCR_TXEDGE_Pos)           ///<  Transmit data edge select

#define SPI_CCR_CPHASEL_Pos         (6)
#define SPI_CCR_CPHASEL             (0x01U << SPI_CCR_CPHASEL)              ///<  CPHA polarity select

#define SPI_CCR_HISPD_Pos           (7)
#define SPI_CCR_HISPD               (0x01U << SPI_CCR_HISPD)                ///<  High speed slave mode

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_BRR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_BRR_DIVF_Pos                (0)
#define SPI_BRR_DIVF                    (0x0000FFFFU << SPI_BRR_DIVF_Pos)       ///<  SPI baud rate control register for baud rate

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_RDNR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_RDNR_RDN_Pos                (0)
#define SPI_RDNR_RDN                    (0xFFFFFFFFU << SPI_RDNR_RDN_Pos)       ///<  The register is used to hold a count of to be received bytes in next receive process

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_NSSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_NSSR_NSS_Pos                (0)
#define SPI_NSSR_NSS                    (0x01U << SPI_NSSR_NSS_Pos)             ///<  Chip select output signal in Master mode

////////////////////////////////////////////////////////////////////////////////
/// @brief SPI_ECR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define SPI_ECR_EXTLEN_Pos              (0)
#define SPI_ECR_EXTLEN                  (0x1FU << SPI_ECR_EXTLEN_Pos)           ///<  control SPI data length




////////////////////////////////////////////////////////////////////////////////
///                      System Configuration (SYSCFG)
////////////////////////////////////////////////////////////////////////////////

/// @brief SYSCFG_CFGR Register Bit definition
#define SYSCFG_CFGR_MEM_MODE               ((uint32_t)0x00000003)               ///< SYSCFG_Memory Remap Config
#define SYSCFG_CFGR_MEM_MODE_0             ((uint32_t)0x00000001)               ///< SYSCFG_Memory Remap Config Bit 0
#define SYSCFG_CFGR_MEM_MODE_1             ((uint32_t)0x00000002)               ///< SYSCFG_Memory Remap Config Bit 1
///
#define SYSCFG_CFGR_ADC_DMA_RMP            ((uint32_t)0x00000100)               ///< ADC DMA remap
#define SYSCFG_CFGR_UART1TX_DMA_RMP        ((uint32_t)0x00000200)               ///< UART1 TX DMA remap
#define SYSCFG_CFGR_UART1RX_DMA_RMP        ((uint32_t)0x00000400)               ///< UART1 RX DMA remap
#define SYSCFG_CFGR_TIM16_DMA_RMP          ((uint32_t)0x00000800)               ///< Timer 16 DMA remap
#define SYSCFG_CFGR_TIM17_DMA_RMP          ((uint32_t)0x00001000)               ///< Timer 17 DMA remap

/// @brief SYSCFG_EXTICR1 Register Bit definition
#define SYSCFG_EXTICR1_EXTI0               ((uint16_t)0x000F)                   ///< EXTI 0 configuration
#define SYSCFG_EXTICR1_EXTI1               ((uint16_t)0x00F0)                   ///< EXTI 1 configuration
#define SYSCFG_EXTICR1_EXTI2               ((uint16_t)0x0F00)                   ///< EXTI 2 configuration
#define SYSCFG_EXTICR1_EXTI3               ((uint16_t)0xF000)                   ///< EXTI 3 configuration

///  @brief  EXTI0 configuration
#define SYSCFG_EXTICR1_EXTI0_PA            ((uint16_t)0x0000)                   ///< PA[0] pin
#define SYSCFG_EXTICR1_EXTI0_PB            ((uint16_t)0x0001)                   ///< PB[0] pin
#define SYSCFG_EXTICR1_EXTI0_PC            ((uint16_t)0x0002)                   ///< PC[0] pin
#define SYSCFG_EXTICR1_EXTI0_PD            ((uint16_t)0x0003)                   ///< PD[0] pin

///  @brief  EXTI1 configuration
#define SYSCFG_EXTICR1_EXTI1_PA            ((uint16_t)0x0000)                   ///< PA[1] pin
#define SYSCFG_EXTICR1_EXTI1_PB            ((uint16_t)0x0010)                   ///< PB[1] pin
#define SYSCFG_EXTICR1_EXTI1_PC            ((uint16_t)0x0020)                   ///< PC[1] pin
#define SYSCFG_EXTICR1_EXTI1_PD            ((uint16_t)0x0030)                   ///< PD[1] pin

///  @brief  EXTI2 configuration
#define SYSCFG_EXTICR1_EXTI2_PA            ((uint16_t)0x0000)                   ///< PA[2] pin
#define SYSCFG_EXTICR1_EXTI2_PB            ((uint16_t)0x0100)                   ///< PB[2] pin
#define SYSCFG_EXTICR1_EXTI2_PC            ((uint16_t)0x0200)                   ///< PC[2] pin
#define SYSCFG_EXTICR1_EXTI2_PD            ((uint16_t)0x0300)                   ///< PD[2] pin


///  @brief  EXTI3 configuration
#define SYSCFG_EXTICR1_EXTI3_PA            ((uint16_t)0x0000)                   ///< PA[3] pin
#define SYSCFG_EXTICR1_EXTI3_PB            ((uint16_t)0x1000)                   ///< PB[3] pin
#define SYSCFG_EXTICR1_EXTI3_PC            ((uint16_t)0x2000)                   ///< PC[3] pin
#define SYSCFG_EXTICR1_EXTI3_PD            ((uint16_t)0x3000)                   ///< PD[3] pin


/// @brief SYSCFG_EXTICR2 Register Bit definition
#define SYSCFG_EXTICR2_EXTI4               ((uint16_t)0x000F)                   ///< EXTI 4 configuration
#define SYSCFG_EXTICR2_EXTI5               ((uint16_t)0x00F0)                   ///< EXTI 5 configuration
#define SYSCFG_EXTICR2_EXTI6               ((uint16_t)0x0F00)                   ///< EXTI 6 configuration
#define SYSCFG_EXTICR2_EXTI7               ((uint16_t)0xF000)                   ///< EXTI 7 configuration

/// @brief  EXTI4 configuration
#define SYSCFG_EXTICR2_EXTI4_PA            ((uint16_t)0x0000)                   ///< PA[4] pin
#define SYSCFG_EXTICR2_EXTI4_PB            ((uint16_t)0x0001)                   ///< PB[4] pin
#define SYSCFG_EXTICR2_EXTI4_PC            ((uint16_t)0x0002)                   ///< PC[4] pin
#define SYSCFG_EXTICR2_EXTI4_PD            ((uint16_t)0x0003)                   ///< PD[4] pin


/// @brief  EXTI5 configuration
#define SYSCFG_EXTICR2_EXTI5_PA            ((uint16_t)0x0000)                   ///< PA[5] pin
#define SYSCFG_EXTICR2_EXTI5_PB            ((uint16_t)0x0010)                   ///< PB[5] pin
#define SYSCFG_EXTICR2_EXTI5_PC            ((uint16_t)0x0020)                   ///< PC[5] pin
#define SYSCFG_EXTICR2_EXTI5_PD            ((uint16_t)0x0030)                   ///< PD[5] pin

/// @brief  EXTI6 configuration

#define SYSCFG_EXTICR2_EXTI6_PA            ((uint16_t)0x0000)                   ///< PA[6] pin
#define SYSCFG_EXTICR2_EXTI6_PB            ((uint16_t)0x0100)                   ///< PB[6] pin
#define SYSCFG_EXTICR2_EXTI6_PC            ((uint16_t)0x0200)                   ///< PC[6] pin
#define SYSCFG_EXTICR2_EXTI6_PD            ((uint16_t)0x0300)                   ///< PD[6] pin


/// @brief  EXTI7 configuration
#define SYSCFG_EXTICR2_EXTI7_PA            ((uint16_t)0x0000)                   ///< PA[7] pin
#define SYSCFG_EXTICR2_EXTI7_PB            ((uint16_t)0x1000)                   ///< PB[7] pin
#define SYSCFG_EXTICR2_EXTI7_PC            ((uint16_t)0x2000)                   ///< PC[7] pin
#define SYSCFG_EXTICR2_EXTI7_PD            ((uint16_t)0x3000)                   ///< PD[7] pin

/// @brief SYSCFG_EXTICR3 Register Bit definition
#define SYSCFG_EXTICR3_EXTI8               ((uint16_t)0x000F)                   ///< EXTI 8 configuration
#define SYSCFG_EXTICR3_EXTI9               ((uint16_t)0x00F0)                   ///< EXTI 9 configuration
#define SYSCFG_EXTICR3_EXTI10              ((uint16_t)0x0F00)                   ///< EXTI 10 configuration
#define SYSCFG_EXTICR3_EXTI11              ((uint16_t)0xF000)                   ///< EXTI 11 configuration

///  @brief  EXTI8 configuration
#define SYSCFG_EXTICR3_EXTI8_PA            ((uint16_t)0x0000)                   ///< PA[8] pin
#define SYSCFG_EXTICR3_EXTI8_PB            ((uint16_t)0x0001)                   ///< PB[8] pin
#define SYSCFG_EXTICR3_EXTI8_PC            ((uint16_t)0x0002)                   ///< PC[8] pin
#define SYSCFG_EXTICR3_EXTI8_PD            ((uint16_t)0x0003)                   ///< PD[8] pin


///  @brief  EXTI9 configuration

#define SYSCFG_EXTICR3_EXTI9_PA            ((uint16_t)0x0000)                   ///< PA[9] pin
#define SYSCFG_EXTICR3_EXTI9_PB            ((uint16_t)0x0010)                   ///< PB[9] pin
#define SYSCFG_EXTICR3_EXTI9_PC            ((uint16_t)0x0020)                   ///< PC[9] pin
#define SYSCFG_EXTICR3_EXTI9_PD            ((uint16_t)0x0030)                   ///< PD[9] pin

///  @brief  EXTI10 configuration
#define SYSCFG_EXTICR3_EXTI10_PA           ((uint16_t)0x0000)                   ///< PA[10] pin
#define SYSCFG_EXTICR3_EXTI10_PB           ((uint16_t)0x0100)                   ///< PB[10] pin
#define SYSCFG_EXTICR3_EXTI10_PC           ((uint16_t)0x0200)                   ///< PC[10] pin
#define SYSCFG_EXTICR3_EXTI10_PD           ((uint16_t)0x0300)                   ///< PE[10] pin

///  @brief  EXTI11 configuration
#define SYSCFG_EXTICR3_EXTI11_PA           ((uint16_t)0x0000)                   ///< PA[11] pin
#define SYSCFG_EXTICR3_EXTI11_PB           ((uint16_t)0x1000)                   ///< PB[11] pin
#define SYSCFG_EXTICR3_EXTI11_PC           ((uint16_t)0x2000)                   ///< PC[11] pin
#define SYSCFG_EXTICR3_EXTI11_PD           ((uint16_t)0x3000)                   ///< PD[11] pin


/// @brief SYSCFG_EXTICR4 Register Bit definition
#define SYSCFG_EXTICR4_EXTI12               ((uint16_t)0x000F)                  ///< EXTI 12 configuration
#define SYSCFG_EXTICR4_EXTI13               ((uint16_t)0x00F0)                  ///< EXTI 13 configuration
#define SYSCFG_EXTICR4_EXTI14               ((uint16_t)0x0F00)                  ///< EXTI 14 configuration
#define SYSCFG_EXTICR4_EXTI15               ((uint16_t)0xF000)                  ///< EXTI 15 configuration

#define SYSCFG_EXTICR4_EXTI12_PA            ((uint16_t)0x0000)                  ///< PA[12] pin for EXTI12
#define SYSCFG_EXTICR4_EXTI12_PB            ((uint16_t)0x0001)                  ///< PB[12] pin for EXTI12
#define SYSCFG_EXTICR4_EXTI12_PC            ((uint16_t)0x0002)                  ///< PC[12] pin for EXTI12
#define SYSCFG_EXTICR4_EXTI12_PD            ((uint16_t)0x0003)                  ///< PD[12] pin for EXTI12

#define SYSCFG_EXTICR4_EXTI13_PA            ((uint16_t)0x0000)                  ///< PA[13] pin for EXTI13
#define SYSCFG_EXTICR4_EXTI13_PB            ((uint16_t)0x0010)                  ///< PB[13] pin for EXTI13
#define SYSCFG_EXTICR4_EXTI13_PC            ((uint16_t)0x0020)                  ///< PC[13] pin for EXTI13
#define SYSCFG_EXTICR4_EXTI13_PD            ((uint16_t)0x0030)                  ///< PD[13] pin for EXTI13

#define SYSCFG_EXTICR4_EXTI14_PA            ((uint16_t)0x0000)                  ///< PA[14] pin for EXTI14
#define SYSCFG_EXTICR4_EXTI14_PB            ((uint16_t)0x0100)                  ///< PB[14] pin for EXTI14
#define SYSCFG_EXTICR4_EXTI14_PC            ((uint16_t)0x0200)                  ///< PC[14] pin for EXTI14
#define SYSCFG_EXTICR4_EXTI14_PD            ((uint16_t)0x0300)                  ///< PD[14] pin for EXTI14

#define SYSCFG_EXTICR4_EXTI15_PA            ((uint16_t)0x0000)                  ///< PA[15] pin for EXTI15
#define SYSCFG_EXTICR4_EXTI15_PB            ((uint16_t)0x1000)                  ///< PB[15] pin for EXTI15
#define SYSCFG_EXTICR4_EXTI15_PC            ((uint16_t)0x2000)                  ///< PC[15] pin for EXTI15
#define SYSCFG_EXTICR4_EXTI15_PD            ((uint16_t)0x3000)                  ///< PD[15] pin for EXTI15


////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CR1 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CR1_CEN_Pos                 (0)
#define TIM_CR1_CEN                     (0x01U << TIM_CR1_CEN_Pos)              ///< Counter enable
#define TIM_CR1_UDIS_Pos                (1)
#define TIM_CR1_UDIS                    (0x01U << TIM_CR1_UDIS_Pos)             ///< Update disable
#define TIM_CR1_URS_Pos                 (2)
#define TIM_CR1_URS                     (0x01U << TIM_CR1_URS_Pos)              ///< Update request source
#define TIM_CR1_OPM_Pos                 (3)
#define TIM_CR1_OPM                     (0x01U << TIM_CR1_OPM_Pos)              ///< One pulse mode
#define TIM_CR1_DIR_Pos                 (4)
#define TIM_CR1_DIR                     (0x01U << TIM_CR1_DIR_Pos)              ///< Direction
#define TIM_CR1_CMS_Pos                 (5)
#define TIM_CR1_CMS                     (0x03U << TIM_CR1_CMS_Pos)              ///< CMS[1:0] bits (Center-aligned mode selection)
#define TIM_CR1_CMS_EDGEALIGNED         (0x00U << TIM_CR1_CMS_Pos)              ///< Edge-aligned mode
#define TIM_CR1_CMS_CENTERALIGNED1      (0x01U << TIM_CR1_CMS_Pos)              ///< Center-aligned mode 1
#define TIM_CR1_CMS_CENTERALIGNED2      (0x02U << TIM_CR1_CMS_Pos)              ///< Center-aligned mode 2
#define TIM_CR1_CMS_CENTERALIGNED3      (0x03U << TIM_CR1_CMS_Pos)              ///< Center-aligned mode 3
#define TIM_CR1_ARPEN_Pos               (7)
#define TIM_CR1_ARPEN                   (0x01U << TIM_CR1_ARPEN_Pos)            ///< Auto-reload preload enable
#define TIM_CR1_CKD_Pos                 (8)
#define TIM_CR1_CKD                     (0x03U << TIM_CR1_CKD_Pos)              ///< CKD[1:0] bits (clock division)
#define TIM_CR1_CKD_DIV1                (0x00U << TIM_CR1_CKD_Pos)              ///< Divided by 1
#define TIM_CR1_CKD_DIV2                (0x01U << TIM_CR1_CKD_Pos)              ///< Divided by 2
#define TIM_CR1_CKD_DIV4                (0x02U << TIM_CR1_CKD_Pos)              ///< Divided by 4

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CR2 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CR2_CCPC_Pos                (0)
#define TIM_CR2_CCPC                    (0x01U << TIM_CR2_CCPC_Pos)             ///< Capture/Compare Preloaded Control
#define TIM_CR2_CCUS_Pos                (2)
#define TIM_CR2_CCUS                    (0x01U << TIM_CR2_CCUS_Pos)             ///< Capture/Compare Control Update Selection
#define TIM_CR2_CCDS_Pos                (3)
#define TIM_CR2_CCDS                    (0x01U << TIM_CR2_CCDS_Pos)             ///< Capture/Compare DMA Selection
#define TIM_CR2_MMS_Pos                 (4)
#define TIM_CR2_MMS                     (0x07U << TIM_CR2_MMS_Pos)              ///< MMS[2:0] bits (Master Mode Selection)
#define TIM_CR2_MMS_RESET               (0x00U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: Reset
#define TIM_CR2_MMS_ENABLE              (0x01U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: Enable
#define TIM_CR2_MMS_UPDATE              (0x02U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: Update
#define TIM_CR2_MMS_OC1                 (0x03U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: OC1
#define TIM_CR2_MMS_OC1REF              (0x04U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: OC1Ref
#define TIM_CR2_MMS_OC2REF              (0x05U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: OC2Ref
#define TIM_CR2_MMS_OC3REF              (0x06U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: OC3Ref
#define TIM_CR2_MMS_OC4REF              (0x07U << TIM_CR2_MMS_Pos)              ///< Master Mode Select: OC4Ref
#define TIM_CR2_TI1S_Pos                (7)
#define TIM_CR2_TI1S                    (0x01U << TIM_CR2_TI1S_Pos)             ///< TI1 Selection
#define TIM_CR2_OIS1_Pos                (8)
#define TIM_CR2_OIS1                    (0x01U << TIM_CR2_OIS1_Pos)             ///< Output Idle state 1 (OC1 output)
#define TIM_CR2_OIS1N_Pos               (9)
#define TIM_CR2_OIS1N                   (0x01U << TIM_CR2_OIS1N_Pos)            ///< Output Idle state 1 (OC1N output)
#define TIM_CR2_OIS2_Pos                (10)
#define TIM_CR2_OIS2                    (0x01U << TIM_CR2_OIS2_Pos)             ///< Output Idle state 2 (OC2 output)
#define TIM_CR2_OIS2N_Pos               (11)
#define TIM_CR2_OIS2N                   (0x01U << TIM_CR2_OIS2N_Pos)            ///< Output Idle state 2 (OC2N output)
#define TIM_CR2_OIS3_Pos                (12)
#define TIM_CR2_OIS3                    (0x01U << TIM_CR2_OIS3_Pos)             ///< Output Idle state 3 (OC3 output)
#define TIM_CR2_OIS3N_Pos               (13)
#define TIM_CR2_OIS3N                   (0x01U << TIM_CR2_OIS3N_Pos)            ///< Output Idle state 3 (OC3N output)
#define TIM_CR2_OIS4_Pos                (14)
#define TIM_CR2_OIS4                    (0x01U << TIM_CR2_OIS4_Pos)             ///< Output Idle state 4 (OC4 output)

#define TIM_CR2_OIS5_Pos            (16)
#define TIM_CR2_OIS5                (0x01U << TIM_CR2_OIS5_Pos)             ///< Output Idle state 5 (OC5 output)

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_SMCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_SMCR_SMS_Pos                (0)
#define TIM_SMCR_SMS                    (0x07U << TIM_SMCR_SMS_Pos)             ///< SMS[2:0] bits (Slave mode selection)
#define TIM_SMCR_SMS_OFF                (0x00U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: OFF
#define TIM_SMCR_SMS_ENCODER1           (0x01U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Encoder1
#define TIM_SMCR_SMS_ENCODER2           (0x02U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Encoder2
#define TIM_SMCR_SMS_ENCODER3           (0x03U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Encoder3
#define TIM_SMCR_SMS_RESET              (0x04U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Reset
#define TIM_SMCR_SMS_GATED              (0x05U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Gated
#define TIM_SMCR_SMS_TRIGGER            (0x06U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: Trigger
#define TIM_SMCR_SMS_EXTERNAL1          (0x07U << TIM_SMCR_SMS_Pos)             ///< Slave Mode select: External1

#define TIM_SMCR_OCCS_Pos               (3)
#define TIM_SMCR_OCCS                   (0x01U << TIM_SMCR_OCCS_Pos)            ///< Output compare clear selection

#define TIM_SMCR_TS_Pos                 (4)
#define TIM_SMCR_TS                     (0x07U << TIM_SMCR_TS_Pos)              ///< TS[2:0] bits (Trigger selection)
#define TIM_SMCR_TS_ITR0                (0x00U << TIM_SMCR_TS_Pos)              ///< Internal Trigger 0 (ITR0)
#define TIM_SMCR_TS_ITR1                (0x01U << TIM_SMCR_TS_Pos)              ///< Internal Trigger 1 (ITR1)
#define TIM_SMCR_TS_ITR2                (0x02U << TIM_SMCR_TS_Pos)              ///< Internal Trigger 2 (ITR2)
#define TIM_SMCR_TS_ITR3                (0x03U << TIM_SMCR_TS_Pos)              ///< Internal Trigger 3 (ITR3)
#define TIM_SMCR_TS_TI1F_ED             (0x04U << TIM_SMCR_TS_Pos)              ///< TI1 Edge Detector (TI1F_ED)
#define TIM_SMCR_TS_TI1FP1              (0x05U << TIM_SMCR_TS_Pos)              ///< Filtered Timer Input 1 (TI1FP1)
#define TIM_SMCR_TS_TI2FP2              (0x06U << TIM_SMCR_TS_Pos)              ///< Filtered Timer Input 2 (TI2FP2)
#define TIM_SMCR_TS_ETRF                (0x07U << TIM_SMCR_TS_Pos)              ///< External Trigger input (ETRF)
#define TIM_SMCR_MSM_Pos                (7)
#define TIM_SMCR_MSM                    (0x01U << TIM_SMCR_MSM_Pos)                            ///< Master/slave mode
#define TIM_SMCR_ETF_Pos                (8)
#define TIM_SMCR_ETF                    (0xFFU << TIM_SMCR_ETF_Pos)             ///< ETF[3:0] bits (External trigger filter)
#define TIM_SMCR_ETF_0                  (0x01U << TIM_SMCR_ETF_Pos)             ///< Bit 0
#define TIM_SMCR_ETF_1                  (0x02U << TIM_SMCR_ETF_Pos)             ///< Bit 1
#define TIM_SMCR_ETF_2                  (0x04U << TIM_SMCR_ETF_Pos)             ///< Bit 2
#define TIM_SMCR_ETF_3                  (0x08U << TIM_SMCR_ETF_Pos)             ///< Bit 3
#define TIM_SMCR_ETPS_Pos               (12)
#define TIM_SMCR_ETPS                   (0x03U << TIM_SMCR_ETPS_Pos)            ///< ETPS[1:0] bits (External trigger prescaler)
#define TIM_SMCR_ETPS_OFF               (0x00U << TIM_SMCR_ETPS_Pos)            ///< Prescaler OFF
#define TIM_SMCR_ETPS_DIV2              (0x01U << TIM_SMCR_ETPS_Pos)            ///< ETRP frequency divided by 2
#define TIM_SMCR_ETPS_DIV4              (0x02U << TIM_SMCR_ETPS_Pos)            ///< ETRP frequency divided by 4
#define TIM_SMCR_ETPS_DIV8              (0x03U << TIM_SMCR_ETPS_Pos)            ///< ETRP frequency divided by 8
#define TIM_SMCR_ECEN_Pos               (14)
#define TIM_SMCR_ECEN                   (0x01U << TIM_SMCR_ECEN_Pos)            ///< External clock enable
#define TIM_SMCR_ETP_Pos                (15)
#define TIM_SMCR_ETP                    (0x01U << TIM_SMCR_ETP_Pos)             ///< External trigger polarity

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_DIER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_DIER_UI_Pos                 (0)
#define TIM_DIER_UI                     (0x01U << TIM_DIER_UI_Pos)              ///< Update interrupt enable
#define TIM_DIER_CC1I_Pos               (1)
#define TIM_DIER_CC1I                   (0x01U << TIM_DIER_CC1I_Pos)            ///< Capture/Compare 1 interrupt enable
#define TIM_DIER_CC2I_Pos               (2)
#define TIM_DIER_CC2I                   (0x01U << TIM_DIER_CC2I_Pos)            ///< Capture/Compare 2 interrupt enable
#define TIM_DIER_CC3I_Pos               (3)
#define TIM_DIER_CC3I                   (0x01U << TIM_DIER_CC3I_Pos)            ///< Capture/Compare 3 interrupt enable
#define TIM_DIER_CC4I_Pos               (4)
#define TIM_DIER_CC4I                   (0x01U << TIM_DIER_CC4I_Pos)            ///< Capture/Compare 4 interrupt enable
#define TIM_DIER_COMI_Pos               (5)
#define TIM_DIER_COMI                   (0x01U << TIM_DIER_COMI_Pos)            ///< COM interrupt enable
#define TIM_DIER_TI_Pos                 (6)
#define TIM_DIER_TI                     (0x01U << TIM_DIER_TI_Pos)              ///< Trigger interrupt enable
#define TIM_DIER_BI_Pos                 (7)
#define TIM_DIER_BI                     (0x01U << TIM_DIER_BI_Pos)              ///< Break interrupt enable
#define TIM_DIER_UD_Pos                 (8)
#define TIM_DIER_UD                     (0x01U << TIM_DIER_UD_Pos)              ///< Update DMA request enable
#define TIM_DIER_CC1D_Pos               (9)
#define TIM_DIER_CC1D                   (0x01U << TIM_DIER_CC1D_Pos)            ///< Capture/Compare 1 DMA request enable
#define TIM_DIER_CC2D_Pos               (10)
#define TIM_DIER_CC2D                   (0x01U << TIM_DIER_CC2D_Pos)            ///< Capture/Compare 2 DMA request enable
#define TIM_DIER_CC3D_Pos               (11)
#define TIM_DIER_CC3D                   (0x01U << TIM_DIER_CC3D_Pos)            ///< Capture/Compare 3 DMA request enable
#define TIM_DIER_CC4D_Pos               (12)
#define TIM_DIER_CC4D                   (0x01U << TIM_DIER_CC4D_Pos)            ///< Capture/Compare 4 DMA request enable
#define TIM_DIER_COMD_Pos               (13)
#define TIM_DIER_COMD                   (0x01U << TIM_DIER_COMD_Pos)            ///< COM DMA request enable
#define TIM_DIER_TD_Pos                 (14)
#define TIM_DIER_TD                     (0x01U << TIM_DIER_TD_Pos)              ///< Trigger DMA request enable

#define TIM_DIER_CC5I_Pos               (16)
#define TIM_DIER_CC5I                   (0x01U << TIM_DIER_CC5I_Pos)            ///< Capture/Compare 5 interrupt enable
#define TIM_DIER_CC5D_Pos               (17)
#define TIM_DIER_CC5D                   (0x01U << TIM_DIER_CC5D_Pos)            ///< Capture/Compare 5 DMA request enable

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_SR_UI_Pos                   (0)
#define TIM_SR_UI                       (0x01U << TIM_SR_UI_Pos)                ///< Update interrupt Flag
#define TIM_SR_CC1I_Pos                 (1)
#define TIM_SR_CC1I                     (0x01U << TIM_SR_CC1I_Pos)              ///< Capture/Compare 1 interrupt Flag
#define TIM_SR_CC2I_Pos                 (2)
#define TIM_SR_CC2I                     (0x01U << TIM_SR_CC2I_Pos)              ///< Capture/Compare 2 interrupt Flag
#define TIM_SR_CC3I_Pos                 (3)
#define TIM_SR_CC3I                     (0x01U << TIM_SR_CC3I_Pos)              ///< Capture/Compare 3 interrupt Flag
#define TIM_SR_CC4I_Pos                 (4)
#define TIM_SR_CC4I                     (0x01U << TIM_SR_CC4I_Pos)              ///< Capture/Compare 4 interrupt Flag
#define TIM_SR_COMI_Pos                 (5)
#define TIM_SR_COMI                     (0x01U << TIM_SR_COMI_Pos)              ///< COM interrupt Flag
#define TIM_SR_TI_Pos                   (6)
#define TIM_SR_TI                       (0x01U << TIM_SR_TI_Pos)                ///< Trigger interrupt Flag
#define TIM_SR_BI_Pos                   (7)
#define TIM_SR_BI                       (0x01U << TIM_SR_BI_Pos)                ///< Break interrupt Flag
#define TIM_SR_CC1O_Pos                 (9)
#define TIM_SR_CC1O                     (0x01U << TIM_SR_CC1O_Pos)              ///< Capture/Compare 1 Overcapture Flag
#define TIM_SR_CC2O_Pos                 (10)
#define TIM_SR_CC2O                     (0x01U << TIM_SR_CC2O_Pos)              ///< Capture/Compare 2 Overcapture Flag
#define TIM_SR_CC3O_Pos                 (11)
#define TIM_SR_CC3O                     (0x01U << TIM_SR_CC3O_Pos)              ///< Capture/Compare 3 Overcapture Flag
#define TIM_SR_CC4O_Pos                 (12)
#define TIM_SR_CC4O                     (0x01U << TIM_SR_CC4O_Pos)              ///< Capture/Compare 4 Overcapture Flag

#define TIM_SR_CC5I_Pos                 (16)
#define TIM_SR_CC5I                     (0x01U << TIM_SR_CC5I_Pos)              ///< Capture/Compare 5 interrupt Flag

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_EGR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_EGR_UG_Pos                  (0)
#define TIM_EGR_UG                      (0x01U << TIM_EGR_UG_Pos)               ///< Update Generation
#define TIM_EGR_CC1G_Pos                (1)
#define TIM_EGR_CC1G                    (0x01U << TIM_EGR_CC1G_Pos)             ///< Capture/Compare 1 Generation
#define TIM_EGR_CC2G_Pos                (2)
#define TIM_EGR_CC2G                    (0x01U << TIM_EGR_CC2G_Pos)             ///< Capture/Compare 2 Generation
#define TIM_EGR_CC3G_Pos                (3)
#define TIM_EGR_CC3G                    (0x01U << TIM_EGR_CC3G_Pos)             ///< Capture/Compare 3 Generation
#define TIM_EGR_CC4G_Pos                (4)
#define TIM_EGR_CC4G                    (0x01U << TIM_EGR_CC4G_Pos)             ///< Capture/Compare 4 Generation
#define TIM_EGR_COMG_Pos                (5)
#define TIM_EGR_COMG                    (0x01U << TIM_EGR_COMG_Pos)             ///< Capture/Compare Control Update Generation
#define TIM_EGR_TG_Pos                  (6)
#define TIM_EGR_TG                      (0x01U << TIM_EGR_TG_Pos)               ///< Trigger Generation
#define TIM_EGR_BG_Pos                  (7)
#define TIM_EGR_BG                      (0x01U << TIM_EGR_BG_Pos)               ///< Break Generation

#define TIM_EGR_CC5G_Pos                (16)
#define TIM_EGR_CC5G                    (0x01U << TIM_EGR_CC5G_Pos)             ///< Capture/Compare 5 Generation

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCMR1 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCMR1_CC1S_Pos              (0)
#define TIM_CCMR1_CC1S                  (0x03U << TIM_CCMR1_CC1S_Pos)           ///< CC1S[1:0] bits (Capture/Compare 1 Selection)
#define TIM_CCMR1_CC1S_OC               (0x00U << TIM_CCMR1_CC1S_Pos)           ///< Channel is configured as output
#define TIM_CCMR1_CC1S_DIRECTTI         (0x01U << TIM_CCMR1_CC1S_Pos)           ///< Channel is configured as input, IC1 is mapped on TI1
#define TIM_CCMR1_CC1S_INDIRECTTI       (0x02U << TIM_CCMR1_CC1S_Pos)           ///< Channel is configured as input, IC1 is mapped on TI2
#define TIM_CCMR1_CC1S_TRC              (0x03U << TIM_CCMR1_CC1S_Pos)           ///< Channel is configured as input, IC1 is mapped on TRC
#define TIM_CCMR1_OC1FEN_Pos            (2)
#define TIM_CCMR1_OC1FEN                (0x01U << TIM_CCMR1_OC1FEN_Pos)         ///< Output Compare 1 Fast enable

#define TIM_CCMR1_OC1PEN_Pos            (3)
#define TIM_CCMR1_OC1PEN                (0x01U << TIM_CCMR1_OC1PEN_Pos)         ///< Output Compare 1 Preload enable
#define TIM_CCMR1_OC1M_Pos              (4)
#define TIM_CCMR1_OC1M                  (0x07U << TIM_CCMR1_OC1M_Pos)           ///< OC1M[2:0] bits (Output Compare 1 Mode)
#define TIM_CCMR1_OC1M_TIMING           (0x00U << TIM_CCMR1_OC1M_Pos)           ///< Timing
#define TIM_CCMR1_OC1M_ACTIVE           (0x01U << TIM_CCMR1_OC1M_Pos)           ///< Active
#define TIM_CCMR1_OC1M_INACTIVE         (0x02U << TIM_CCMR1_OC1M_Pos)           ///< Inactive
#define TIM_CCMR1_OC1M_TOGGLE           (0x03U << TIM_CCMR1_OC1M_Pos)           ///< Toggle
#define TIM_CCMR1_OC1M_FORCEINACTIVE    (0x04U << TIM_CCMR1_OC1M_Pos)           ///< Forceinactive
#define TIM_CCMR1_OC1M_FORCEACTIVE      (0x05U << TIM_CCMR1_OC1M_Pos)           ///< Forceactive
#define TIM_CCMR1_OC1M_PWM1             (0x06U << TIM_CCMR1_OC1M_Pos)           ///< PWM1
#define TIM_CCMR1_OC1M_PWM2             (0x07U << TIM_CCMR1_OC1M_Pos)           ///< PWM2

#define TIM_CCMR1_OC1CEN_Pos            (7)
#define TIM_CCMR1_OC1CEN                (0x01U << TIM_CCMR1_OC1CEN_Pos)         ///< Output Compare 1Clear Enable
#define TIM_CCMR1_CC2S_Pos              (8)
#define TIM_CCMR1_CC2S                  (0x03U << TIM_CCMR1_CC2S_Pos)           ///< CC2S[1:0] bits (Capture/Compare 2 Selection)
#define TIM_CCMR1_CC2S_OC               (0x00U << TIM_CCMR1_CC2S_Pos)           ///< Channel is configured as output
#define TIM_CCMR1_CC2S_DIRECTTI         (0x01U << TIM_CCMR1_CC2S_Pos)           ///< Channel is configured as input, IC2 is mapped on TI2
#define TIM_CCMR1_CC2S_INDIRECTTI       (0x02U << TIM_CCMR1_CC2S_Pos)           ///< Channel is configured as input, IC2 is mapped on TI1
#define TIM_CCMR1_CC2S_TRC              (0x03U << TIM_CCMR1_CC2S_Pos)           ///< Channel is configured as input, IC2 is mapped on TRC
#define TIM_CCMR1_OC2FEN_Pos            (10)
#define TIM_CCMR1_OC2FEN                (0x01U << TIM_CCMR1_OC2FEN_Pos)         ///< Output Compare 2 Fast enable
#define TIM_CCMR1_OC2PEN_Pos            (11)
#define TIM_CCMR1_OC2PEN                (0x01U << TIM_CCMR1_OC2PEN_Pos)         ///< Output Compare 2 Preload enable
#define TIM_CCMR1_OC2M_Pos              (12)
#define TIM_CCMR1_OC2M                  (0x07U << TIM_CCMR1_OC2M_Pos)           ///< OC2M[2:0] bits (Output Compare 2 Mode)
#define TIM_CCMR1_OC2M_TIMING           (0x00U << TIM_CCMR1_OC2M_Pos)           ///< Timing
#define TIM_CCMR1_OC2M_ACTIVE           (0x01U << TIM_CCMR1_OC2M_Pos)           ///< Active
#define TIM_CCMR1_OC2M_INACTIVE         (0x02U << TIM_CCMR1_OC2M_Pos)           ///< Inactive
#define TIM_CCMR1_OC2M_TOGGLE           (0x03U << TIM_CCMR1_OC2M_Pos)           ///< Toggle
#define TIM_CCMR1_OC2M_FORCEINACTIVE    (0x04U << TIM_CCMR1_OC2M_Pos)           ///< Forceinactive
#define TIM_CCMR1_OC2M_FORCEACTIVE      (0x05U << TIM_CCMR1_OC2M_Pos)           ///< Forceactive
#define TIM_CCMR1_OC2M_PWM1             (0x06U << TIM_CCMR1_OC2M_Pos)           ///< PWM1
#define TIM_CCMR1_OC2M_PWM2             (0x07U << TIM_CCMR1_OC2M_Pos)           ///< PWM2
#define TIM_CCMR1_OC2CEN_Pos            (15)
#define TIM_CCMR1_OC2CEN                (0x01U << TIM_CCMR1_OC2CEN_Pos)         ///< Output Compare 2 Clear Enable

#define TIM_CCMR1_IC1PSC_Pos            (2)
#define TIM_CCMR1_IC1PSC                (0x03U << TIM_CCMR1_IC1PSC_Pos)         ///< IC1PSC[1:0] bits (Input Capture 1 Prescaler)
#define TIM_CCMR1_IC1PSC_DIV1           (0x00U << TIM_CCMR1_IC1PSC_Pos)         ///< No Prescaler
#define TIM_CCMR1_IC1PSC_DIV2           (0x01U << TIM_CCMR1_IC1PSC_Pos)         ///< Capture is done once every 2 events
#define TIM_CCMR1_IC1PSC_DIV4           (0x02U << TIM_CCMR1_IC1PSC_Pos)         ///< Capture is done once every 4 events
#define TIM_CCMR1_IC1PSC_DIV8           (0x03U << TIM_CCMR1_IC1PSC_Pos)         ///< Capture is done once every 8 events
#define TIM_CCMR1_IC1F_Pos              (4)
#define TIM_CCMR1_IC1F                  (0x0FU << TIM_CCMR1_IC1F_Pos)           ///< IC1F[3:0] bits (Input Capture 1 Filter)
#define TIM_CCMR1_IC1F_0                (0x01U << TIM_CCMR1_IC1F_Pos)           ///< Bit 0
#define TIM_CCMR1_IC1F_1                (0x02U << TIM_CCMR1_IC1F_Pos)           ///< Bit 1
#define TIM_CCMR1_IC1F_2                (0x04U << TIM_CCMR1_IC1F_Pos)           ///< Bit 2
#define TIM_CCMR1_IC1F_3                (0x08U << TIM_CCMR1_IC1F_Pos)           ///< Bit 3

#define TIM_CCMR1_IC2PSC_Pos            (10)
#define TIM_CCMR1_IC2PSC                (0x03U << TIM_CCMR1_IC2PSC_Pos)         ///< IC2PSC[1:0] bits (Input Capture 2 Prescaler)
#define TIM_CCMR1_IC2PSC_DIV1           (0x00U << TIM_CCMR1_IC2PSC_Pos)         ///< No Prescaler
#define TIM_CCMR1_IC2PSC_DIV2           (0x01U << TIM_CCMR1_IC2PSC_Pos)         ///< Capture is done once every 2 events
#define TIM_CCMR1_IC2PSC_DIV4           (0x02U << TIM_CCMR1_IC2PSC_Pos)         ///< Capture is done once every 4 events
#define TIM_CCMR1_IC2PSC_DIV8           (0x03U << TIM_CCMR1_IC2PSC_Pos)         ///< Capture is done once every 8 events
#define TIM_CCMR1_IC2F_Pos              (12)
#define TIM_CCMR1_IC2F                  (0x0FU << TIM_CCMR1_IC2F_Pos)           ///< IC2F[3:0] bits (Input Capture 2 Filter)
#define TIM_CCMR1_IC2F_0                (0x01U << TIM_CCMR1_IC2F_Pos)           ///< Bit 0
#define TIM_CCMR1_IC2F_1                (0x02U << TIM_CCMR1_IC2F_Pos)           ///< Bit 1
#define TIM_CCMR1_IC2F_2                (0x04U << TIM_CCMR1_IC2F_Pos)           ///< Bit 2
#define TIM_CCMR1_IC2F_3                (0x08U << TIM_CCMR1_IC2F_Pos)           ///< Bit 3

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCMR2 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCMR2_CC3S_Pos              (0)
#define TIM_CCMR2_CC3S                  (0x03U << TIM_CCMR2_CC3S_Pos)           ///< CC3S[1:0] bits (Capture/Compare 3 Selection)
#define TIM_CCMR2_CC3S_OC               (0x00U << TIM_CCMR2_CC3S_Pos)           ///< Channel is configured as output
#define TIM_CCMR2_CC3S_DIRECTTI         (0x01U << TIM_CCMR2_CC3S_Pos)           ///< Channel is configured as input, IC3 is mapped on TI3
#define TIM_CCMR2_CC3S_INDIRECTTI       (0x02U << TIM_CCMR2_CC3S_Pos)           ///< Channel is configured as input, IC3 is mapped on TI4
#define TIM_CCMR2_CC3S_TRC              (0x03U << TIM_CCMR2_CC3S_Pos)           ///< Channel is configured as input, IC3 is mapped on TRC
#define TIM_CCMR2_OC3FEN_Pos            (2)
#define TIM_CCMR2_OC3FEN                (0x01U << TIM_CCMR2_OC3FEN_Pos)         ///< Output Compare 3 Fast enable
#define TIM_CCMR2_IC3PSC_Pos            (2)
#define TIM_CCMR2_IC3PSC                (0x03U << TIM_CCMR2_IC3PSC_Pos)         ///< IC3PSC[1:0] bits (Input Capture 3 Prescaler)
#define TIM_CCMR2_IC3PSC_DIV1           (0x00U << TIM_CCMR2_IC3PSC_Pos)         ///< No Prescaler
#define TIM_CCMR2_IC3PSC_DIV2           (0x01U << TIM_CCMR2_IC3PSC_Pos)         ///< Capture is done once every 2 events
#define TIM_CCMR2_IC3PSC_DIV4           (0x02U << TIM_CCMR2_IC3PSC_Pos)         ///< Capture is done once every 4 events
#define TIM_CCMR2_IC3PSC_DIV8           (0x03U << TIM_CCMR2_IC3PSC_Pos)         ///< Capture is done once every 8 events
#define TIM_CCMR2_OC3PEN_Pos            (3)
#define TIM_CCMR2_OC3PEN                (0x01U << TIM_CCMR2_OC3PEN_Pos)         ///< Output Compare 3 Preload enable
#define TIM_CCMR2_OC3M_Pos              (4)
#define TIM_CCMR2_OC3M                  (0x07U << TIM_CCMR2_OC3M_Pos)           ///< OC3M[2:0] bits (Output Compare 3 Mode)
#define TIM_CCMR2_OC3M_TIMING           (0x00U << TIM_CCMR2_OC3M_Pos)           ///< Timing
#define TIM_CCMR2_OC3M_ACTIVE           (0x01U << TIM_CCMR2_OC3M_Pos)           ///< Active
#define TIM_CCMR2_OC3M_INACTIVE         (0x02U << TIM_CCMR2_OC3M_Pos)           ///< Inactive
#define TIM_CCMR2_OC3M_TOGGLE           (0x03U << TIM_CCMR2_OC3M_Pos)           ///< Toggle
#define TIM_CCMR2_OC3M_FORCEINACTIVE    (0x04U << TIM_CCMR2_OC3M_Pos)           ///< Forceinactive
#define TIM_CCMR2_OC3M_FORCEACTIVE      (0x05U << TIM_CCMR2_OC3M_Pos)           ///< Forceactive
#define TIM_CCMR2_OC3M_PWM1             (0x06U << TIM_CCMR2_OC3M_Pos)           ///< PWM1
#define TIM_CCMR2_OC3M_PWM2             (0x07U << TIM_CCMR2_OC3M_Pos)           ///< PWM2
#define TIM_CCMR2_IC3F_Pos              (4)
#define TIM_CCMR2_IC3F                  (0x0FU << TIM_CCMR2_IC3F_Pos)           ///< IC3F[3:0] bits (Input Capture 3 Filter)
#define TIM_CCMR2_IC3F_0                (0x01U << TIM_CCMR2_IC3F_Pos)           ///< Bit 0
#define TIM_CCMR2_IC3F_1                (0x02U << TIM_CCMR2_IC3F_Pos)           ///< Bit 1
#define TIM_CCMR2_IC3F_2                (0x04U << TIM_CCMR2_IC3F_Pos)           ///< Bit 2
#define TIM_CCMR2_IC3F_3                (0x08U << TIM_CCMR2_IC3F_Pos)           ///< Bit 3
#define TIM_CCMR2_OC3CEN_Pos            (7)
#define TIM_CCMR2_OC3CEN                (0x01U << TIM_CCMR2_OC3CEN_Pos)         ///< Output Compare 3 Clear Enable
#define TIM_CCMR2_CC4S_Pos              (8)
#define TIM_CCMR2_CC4S                  (0x03U << TIM_CCMR2_CC4S_Pos)           ///< CC4S[1:0] bits (Capture/Compare 4 Selection)
#define TIM_CCMR2_CC4S_OC               (0x00U << TIM_CCMR2_CC4S_Pos)           ///< Channel is configured as output
#define TIM_CCMR2_CC4S_DIRECTTI         (0x01U << TIM_CCMR2_CC4S_Pos)           ///< Channel is configured as input, IC4 is mapped on TI4
#define TIM_CCMR2_CC4S_INDIRECTTI       (0x02U << TIM_CCMR2_CC4S_Pos)           ///< Channel is configured as input, IC4 is mapped on TI3
#define TIM_CCMR2_CC4S_TRC              (0x03U << TIM_CCMR2_CC4S_Pos)           ///< Channel is configured as input, IC4 is mapped on TRC
#define TIM_CCMR2_OC4FEN_Pos            (10)
#define TIM_CCMR2_OC4FEN                (0x01U << TIM_CCMR2_OC4FEN_Pos)         ///< Output Compare 4 Fast enable
#define TIM_CCMR2_OC4PEN_Pos            (11)
#define TIM_CCMR2_OC4PEN                (0x01U << TIM_CCMR2_OC4PEN_Pos)         ///< Output Compare 4 Preload enable
#define TIM_CCMR2_OC4M_Pos              (12)
#define TIM_CCMR2_OC4M                  (0x07U << TIM_CCMR2_OC4M_Pos)           ///< OC4M[2:0] bits (Output Compare 4 Mode)
#define TIM_CCMR2_OC4M_TIMING           (0x00U << TIM_CCMR2_OC4M_Pos)           ///< Timing
#define TIM_CCMR2_OC4M_ACTIVE           (0x01U << TIM_CCMR2_OC4M_Pos)           ///< Active
#define TIM_CCMR2_OC4M_INACTIVE         (0x02U << TIM_CCMR2_OC4M_Pos)           ///< Inactive
#define TIM_CCMR2_OC4M_TOGGLE           (0x03U << TIM_CCMR2_OC4M_Pos)           ///< Toggle
#define TIM_CCMR2_OC4M_FORCEINACTIVE    (0x04U << TIM_CCMR2_OC4M_Pos)           ///< Forceinactive
#define TIM_CCMR2_OC4M_FORCEACTIVE      (0x05U << TIM_CCMR2_OC4M_Pos)           ///< Forceactive
#define TIM_CCMR2_OC4M_PWM1             (0x06U << TIM_CCMR2_OC4M_Pos)           ///< PWM1
#define TIM_CCMR2_OC4M_PWM2             (0x07U << TIM_CCMR2_OC4M_Pos)           ///< PWM2
#define TIM_CCMR2_OC4CEN_Pos            (15)
#define TIM_CCMR2_OC4CEN                (0x01U << TIM_CCMR2_OC4CEN_Pos)         ///< Output Compare 4 Clear Enable
#define TIM_CCMR2_IC4PSC_Pos            (10)
#define TIM_CCMR2_IC4PSC                (0x03U << TIM_CCMR2_IC4PSC_Pos)         ///< IC4PSC[1:0] bits (Input Capture 4 Prescaler)
#define TIM_CCMR2_IC4PSC_DIV1           (0x00U << TIM_CCMR2_IC4PSC_Pos)         ///< No Prescaler
#define TIM_CCMR2_IC4PSC_DIV2           (0x01U << TIM_CCMR2_IC4PSC_Pos)         ///< Capture is done once every 2 events
#define TIM_CCMR2_IC4PSC_DIV4           (0x02U << TIM_CCMR2_IC4PSC_Pos)         ///< Capture is done once every 4 events
#define TIM_CCMR2_IC4PSC_DIV8           (0x03U << TIM_CCMR2_IC4PSC_Pos)         ///< Capture is done once every 8 events
#define TIM_CCMR2_IC4F_Pos              (12)
#define TIM_CCMR2_IC4F                  (0x0FU << TIM_CCMR2_IC4F_Pos)           ///< IC4F[3:0] bits (Input Capture 4 Filter)
#define TIM_CCMR2_IC4F_0                (0x01U << TIM_CCMR2_IC4F_Pos)           ///< Bit 0
#define TIM_CCMR2_IC4F_1                (0x02U << TIM_CCMR2_IC4F_Pos)           ///< Bit 1
#define TIM_CCMR2_IC4F_2                (0x04U << TIM_CCMR2_IC4F_Pos)           ///< Bit 2
#define TIM_CCMR2_IC4F_3                (0x08U << TIM_CCMR2_IC4F_Pos)           ///< Bit 3

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCER_CC1EN_Pos              (0)
#define TIM_CCER_CC1EN                  (0x01U << TIM_CCER_CC1EN_Pos)           ///< Capture/Compare 1 output enable
#define TIM_CCER_CC1P_Pos               (1)
#define TIM_CCER_CC1P                   (0x01U << TIM_CCER_CC1P_Pos)            ///< Capture/Compare 1 output Polarity
#define TIM_CCER_CC1NEN_Pos             (2)
#define TIM_CCER_CC1NEN                 (0x01U << TIM_CCER_CC1NEN_Pos)          ///< Capture/Compare 1 Complementary output enable
#define TIM_CCER_CC1NP_Pos              (3)
#define TIM_CCER_CC1NP                  (0x01U << TIM_CCER_CC1NP_Pos)           ///< Capture/Compare 1 Complementary output Polarity
#define TIM_CCER_CC2EN_Pos              (4)
#define TIM_CCER_CC2EN                  (0x01U << TIM_CCER_CC2EN_Pos)           ///< Capture/Compare 2 output enable
#define TIM_CCER_CC2P_Pos               (5)
#define TIM_CCER_CC2P                   (0x01U << TIM_CCER_CC2P_Pos)            ///< Capture/Compare 2 output Polarity
#define TIM_CCER_CC2NEN_Pos             (6)
#define TIM_CCER_CC2NEN                 (0x01U << TIM_CCER_CC2NEN_Pos)          ///< Capture/Compare 2 Complementary output enable
#define TIM_CCER_CC2NP_Pos              (7)
#define TIM_CCER_CC2NP                  (0x01U << TIM_CCER_CC2NP_Pos)           ///< Capture/Compare 2 Complementary output Polarity
#define TIM_CCER_CC3EN_Pos              (8)
#define TIM_CCER_CC3EN                  (0x01U << TIM_CCER_CC3EN_Pos)           ///< Capture/Compare 3 output enable
#define TIM_CCER_CC3P_Pos               (9)
#define TIM_CCER_CC3P                   (0x01U << TIM_CCER_CC3P_Pos)            ///< Capture/Compare 3 output Polarity
#define TIM_CCER_CC3NEN_Pos             (10)
#define TIM_CCER_CC3NEN                 (0x01U << TIM_CCER_CC3NEN_Pos)          ///< Capture/Compare 3 Complementary output enable
#define TIM_CCER_CC3NP_Pos              (11)
#define TIM_CCER_CC3NP                  (0x01U << TIM_CCER_CC3NP_Pos)           ///< Capture/Compare 3 Complementary output Polarity
#define TIM_CCER_CC4EN_Pos              (12)
#define TIM_CCER_CC4EN                  (0x01U << TIM_CCER_CC4EN_Pos)           ///< Capture/Compare 4 output enable
#define TIM_CCER_CC4P_Pos               (13)
#define TIM_CCER_CC4P                   (0x01U << TIM_CCER_CC4P_Pos)            ///< Capture/Compare 4 output Polarity

#define TIM_CCER_CC5EN_Pos          (16)
#define TIM_CCER_CC5EN              (0x01U << TIM_CCER_CC5EN_Pos)           ///< Capture/Compare 5 output enable
#define TIM_CCER_CC5P_Pos           (17)
#define TIM_CCER_CC5P               (0x01U << TIM_CCER_CC5P_Pos)            ///< Capture/Compare 5 output Polarity

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CNT Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_CNT_CNT                 (0xFFFFFFFFU)                           ///< Counter Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_PSC Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_PSC_PSC                     (0xFFFFU)                               ///< Prescaler Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_ARR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_ARR_ARR                 (0xFFFFFFFFU)                           ///< actual auto-reload Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_RCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_RCR_REP                     (0xFFU)                                 ///< Repetition Counter Value

#define TIM_RCR_REP_CNT_Pos         (8)
#define TIM_RCR_REP_CNT             (0xFFU << TIM_RCR_REP_CNT_Pos)          ///< Repetition counter value of real-time writing

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR1 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_CCR1_CCR1               (0xFFFFFFFFU)                           ///< Capture/Compare 1 Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR2 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_CCR2_CCR2               (0xFFFFFFFFU)                           ///< Capture/Compare 2 Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR3 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_CCR3_CCR3               (0xFFFFFFFFU)                           ///< Capture/Compare 3 Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR4 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define TIM_CCR4_CCR4               (0xFFFFFFFFU)                           ///< Capture/Compare 4 Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_BDTR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_BDTR_DTG_Pos                (0)
#define TIM_BDTR_DTG                    (0xFFU << TIM_BDTR_DTG_Pos)             ///< DTG[0:7] bits (Dead-Time Generator set-up)
#define TIM_BDTR_DTG_0                  (0x01U << TIM_BDTR_DTG_Pos)             ///< Bit 0
#define TIM_BDTR_DTG_1                  (0x02U << TIM_BDTR_DTG_Pos)             ///< Bit 1
#define TIM_BDTR_DTG_2                  (0x04U << TIM_BDTR_DTG_Pos)             ///< Bit 2
#define TIM_BDTR_DTG_3                  (0x08U << TIM_BDTR_DTG_Pos)             ///< Bit 3
#define TIM_BDTR_DTG_4                  (0x10U << TIM_BDTR_DTG_Pos)             ///< Bit 4
#define TIM_BDTR_DTG_5                  (0x20U << TIM_BDTR_DTG_Pos)             ///< Bit 5
#define TIM_BDTR_DTG_6                  (0x40U << TIM_BDTR_DTG_Pos)             ///< Bit 6
#define TIM_BDTR_DTG_7                  (0x80U << TIM_BDTR_DTG_Pos)             ///< Bit 7
#define TIM_BDTR_LOCK_Pos               (8)
#define TIM_BDTR_LOCK                   (0x03U << TIM_BDTR_LOCK_Pos)            ///< LOCK[1:0] bits (Lock Configuration)
#define TIM_BDTR_LOCK_OFF               (0x00U << TIM_BDTR_LOCK_Pos)            ///< Lock Off
#define TIM_BDTR_LOCK_1                 (0x01U << TIM_BDTR_LOCK_Pos)            ///< Lock Level 1
#define TIM_BDTR_LOCK_2                 (0x02U << TIM_BDTR_LOCK_Pos)            ///< Lock Level 2
#define TIM_BDTR_LOCK_3                 (0x03U << TIM_BDTR_LOCK_Pos)            ///< Lock Level 3
#define TIM_BDTR_OSSI_Pos               (10)
#define TIM_BDTR_OSSI                   (0x01U << TIM_BDTR_OSSI_Pos)            ///< Off-State Selection for Idle mode
#define TIM_BDTR_OSSR_Pos               (11)
#define TIM_BDTR_OSSR                   (0x01U << TIM_BDTR_OSSR_Pos)            ///< Off-State Selection for Run mode
#define TIM_BDTR_BKEN_Pos               (12)
#define TIM_BDTR_BKEN                   (0x01U << TIM_BDTR_BKEN_Pos)            ///< Break enable
#define TIM_BDTR_BKP_Pos                (13)
#define TIM_BDTR_BKP                    (0x01U << TIM_BDTR_BKP_Pos)             ///< Break Polarity
#define TIM_BDTR_AOEN_Pos               (14)
#define TIM_BDTR_AOEN                   (0x01U << TIM_BDTR_AOEN_Pos)            ///< Automatic Output enable
#define TIM_BDTR_MOEN_Pos               (15)
#define TIM_BDTR_MOEN                   (0x01U << TIM_BDTR_MOEN_Pos)            ///< Main Output enable

#define TIM_BDTR_DOEN_Pos           (16)
#define TIM_BDTR_DOEN               (0x01U << TIM_BDTR_DOEN_Pos)            ///< Direct Output enable

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_DCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_DCR_DBA_Pos                 (0)
#define TIM_DCR_DBA                     (0x1FU << TIM_DCR_DBA_Pos)              ///< DBA[4:0] bits (DMA Base Address)
#define TIM_DCR_DBA_0                   (0x01U << TIM_DCR_DBA_Pos)              ///< Bit 0
#define TIM_DCR_DBA_1                   (0x02U << TIM_DCR_DBA_Pos)              ///< Bit 1
#define TIM_DCR_DBA_2                   (0x04U << TIM_DCR_DBA_Pos)              ///< Bit 2
#define TIM_DCR_DBA_3                   (0x08U << TIM_DCR_DBA_Pos)              ///< Bit 3
#define TIM_DCR_DBA_4                   (0x10U << TIM_DCR_DBA_Pos)              ///< Bit 4
#define TIM_DCR_DBL_Pos                 (8)
#define TIM_DCR_DBL                     (0x1FU << TIM_DCR_DBL_Pos)              ///< DBL[4:0] bits (DMA Burst Length)
#define TIM_DCR_DBL_0                   (0x01U << TIM_DCR_DBL_Pos)              ///< Bit 0
#define TIM_DCR_DBL_1                   (0x02U << TIM_DCR_DBL_Pos)              ///< Bit 1
#define TIM_DCR_DBL_2                   (0x04U << TIM_DCR_DBL_Pos)              ///< Bit 2
#define TIM_DCR_DBL_3                   (0x08U << TIM_DCR_DBL_Pos)              ///< Bit 3
#define TIM_DCR_DBL_4                   (0x10U << TIM_DCR_DBL_Pos)              ///< Bit 4

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_DMAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_DMAR_DMAB                   (0xFFFFU)                               ///< DMA register for burst accesses

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_DMAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_OR_ETR_RMP_Pos          (0)
#define TIM_OR_ETR_RMP              (0x07U << TIM_OR_ETR_RMP_Pos)           ///< ETR GPIO remap
#define TIM_OR_ETR_RMP_GPIO         (0x00U << TIM_OR_ETR_RMP_Pos)           ///< GPIO input as ETR
#define TIM_OR_ETR_RMP_LSI          (0x01U << TIM_OR_ETR_RMP_Pos)           ///< LSI clock input as ETR
#define TIM_OR_ETR_RMP_LSE          (0x02U << TIM_OR_ETR_RMP_Pos)           ///< LSI clock input as ETR
#define TIM_OR_ETR_RMP_HSE_Div128   (0x03U << TIM_OR_ETR_RMP_Pos)           ///< HSE div 128 input as ETR


#define TIM_OR_TI4_RMP_Pos          (6)
#define TIM_OR_TI4_RMP              (0x03U << TIM_OR_TI4_RMP_Pos)           ///< TI4 GPIO remap

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCMR3 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCMR3_OC5FEN_Pos        (2)
#define TIM_CCMR3_OC5FEN            (0x01U << TIM_CCMR3_OC5FEN_Pos)         ///< Output Compare 5 Fast enable
#define TIM_CCMR3_OC5PEN_Pos        (3)
#define TIM_CCMR3_OC5PEN            (0x01U << TIM_CCMR3_OC5PEN_Pos)         ///< Output Compare 5 Preload enable
#define TIM_CCMR3_OC5M_Pos          (4)
#define TIM_CCMR3_OC5M              (0x07U << TIM_CCMR3_OC5M_Pos)           ///< OC5M[2:0] bits (Output Compare 5 Mode)
#define TIM_CCMR3_OC5CEN_Pos        (7)
#define TIM_CCMR3_OC5CEN            (0x01U << TIM_CCMR3_OC5CEN_Pos)         ///< Output Compare 5 Clear Enable

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR5 Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR5_CCR5               (0xFFFF)                                ///< Capture/Compare 5 Value

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_PDER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_PDER_CCDREPE_Pos        (0)
#define TIM_PDER_CCDREPE            (0x01U << TIM_PDER_CCDREPE_Pos)         ///< DMA request flow enable
#define TIM_PDER_CCR1SHIFTEN_Pos    (1)
#define TIM_PDER_CCR1SHIFTEN        (0x01U << TIM_PDER_CC1RSHIFTEN_Pos)     ///< CCR1 pwm shift enable
#define TIM_PDER_CCR2SHIFTEN_Pos    (2)
#define TIM_PDER_CCR2SHIFTEN        (0x01U << TIM_PDER_CC2RSHIFTEN_Pos)     ///< CCR2 pwm shift enable
#define TIM_PDER_CCR3SHIFTEN_Pos    (3)
#define TIM_PDER_CCR3SHIFTEN        (0x01U << TIM_PDER_CC3RSHIFTEN_Pos)     ///< CCR3 pwm shift enable
#define TIM_PDER_CCR4SHIFTEN_Pos    (4)
#define TIM_PDER_CCR4SHIFTEN        (0x01U << TIM_PDER_CC4RSHIFTEN_Pos)     ///< CCR4 pwm shift enable
#define TIM_PDER_CCR5SHIFTEN_Pos    (5)
#define TIM_PDER_CCR5SHIFTEN        (0x01U << TIM_PDER_CC5RSHIFTEN_Pos)     ///< CCR5 pwm shift enable

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR1FALL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR1FALL_CCR1FALL       (0xFFFFU)                               ///< Capture/compare value for ch1 when counting down in PWM center-aligned mode

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR2FALL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR2FALL_CCR2FALL       (0xFFFFU)                               ///< Capture/compare value for ch2 when counting down in PWM center-aligned mode

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR3FALL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR3FALL_CCR3FALL       (0xFFFFU)                               ///< Capture/compare value for ch3 when counting down in PWM center-aligned mode

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR4FALL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR4FALL_CCR4FALL       (0xFFFFU)                               ///< Capture/compare value for ch4 when counting down in PWM center-aligned mode

////////////////////////////////////////////////////////////////////////////////
/// @brief TIM_CCR5FALL Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define TIM_CCR5FALL_CCR5FALL       (0xFFFFU)                               ///< Capture/compare value for ch5 when counting down in PWM center-aligned mode

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_TDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_TDR_DATA_Pos               (0)
#define UART_TDR_DATA                   (0xFFU << UART_TDR_DATA_Pos)            ///< Transmit data register

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_RDR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_RDR_DATA_Pos               (0)
#define UART_RDR_DATA                   (0xFFU << UART_RDR_DATA_Pos)            ///< Receive data register

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_CSR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_CSR_TXC_Pos                (0)
#define UART_CSR_TXC                    (0x01U << UART_CSR_TXC_Pos)             ///< Transmit complete flag bit
#define UART_CSR_RXAVL_Pos              (1)
#define UART_CSR_RXAVL                  (0x01U << UART_CSR_RXAVL_Pos)           ///< Receive valid data flag bit
#define UART_CSR_TXFULL_Pos             (2)
#define UART_CSR_TXFULL                 (0x01U << UART_CSR_TXFULL_Pos)          ///< Transmit buffer full flag bit
#define UART_CSR_TXEPT_Pos              (3)
#define UART_CSR_TXEPT                  (0x01U << UART_CSR_TXEPT_Pos)           ///< Transmit buffer empty flag bit

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_ISR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_ISR_TX_Pos                 (0)
#define UART_ISR_TX                     (0x01U << UART_ISR_TX_Pos)              ///< Transmit buffer empty interrupt flag bit
#define UART_ISR_RX_Pos                 (1)
#define UART_ISR_RX                     (0x01U << UART_ISR_RX_Pos)              ///< Receive valid data interrupt flag bit

#define UART_ISR_TXC_Pos            (2)
#define UART_ISR_TXC                (0x01U << UART_ISR_TXC_Pos)             ///< Transmit complete interrupt flag bit

#define UART_ISR_RXOERR_Pos             (3)
#define UART_ISR_RXOERR                 (0x01U << UART_ISR_RXOERR_Pos)          ///< Receive overflow error interrupt flag bit
#define UART_ISR_RXPERR_Pos             (4)
#define UART_ISR_RXPERR                 (0x01U << UART_ISR_RXPERR_Pos)          ///< Parity error interrupt flag bit
#define UART_ISR_RXFERR_Pos             (5)
#define UART_ISR_RXFERR                 (0x01U << UART_ISR_RXFERR_Pos)          ///< Frame error interrupt flag bit
#define UART_ISR_RXBRK_Pos              (6)
#define UART_ISR_RXBRK                  (0x01U << UART_ISR_RXBRK_Pos)           ///< Receive frame break interrupt flag bit

#define UART_ISR_TXBRK_Pos          (7)
#define UART_ISR_TXBRK              (0x01U << UART_ISR_TXBRK_Pos)           ///< Transmit Break Frame Interrupt Flag Bit
#define UART_ISR_RXB8_Pos           (8)
#define UART_ISR_RXB8               (0x01U << UART_ISR_RXB8_Pos)            ///< Receive Bit 8 Interrupt Flag Bit

#define UART_ISR_RXIDLE_Pos         (9)
#define UART_ISR_RXIDLE             (0x01U << UART_ISR_RXIDLE_Pos)          ///< Receive Bit 8 Interrupt clear Bit

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_IER Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_IER_TX_Pos                 (0)
#define UART_IER_TX                     (0x01U << UART_IER_TX_Pos)              ///< Transmit buffer empty interrupt enable bit
#define UART_IER_RX_Pos                 (1)
#define UART_IER_RX                     (0x01U << UART_IER_RX_Pos)              ///< Receive buffer interrupt enable bit

#define UART_IER_TXC_Pos            (2)
#define UART_IER_TXC                (0x01U << UART_IER_TXC_Pos)             ///< Transmit complete interrupt enable bit

#define UART_IER_RXOERR_Pos             (3)
#define UART_IER_RXOERR                 (0x01U << UART_IER_RXOERR_Pos)          ///< Receive overflow error interrupt enable bit
#define UART_IER_RXPERR_Pos             (4)
#define UART_IER_RXPERR                 (0x01U << UART_IER_RXPERR_Pos)          ///< Parity error interrupt enable bit
#define UART_IER_RXFERR_Pos             (5)
#define UART_IER_RXFERR                 (0x01U << UART_IER_RXFERR_Pos)          ///< Frame error interrupt enable bit
#define UART_IER_RXBRK_Pos              (6)
#define UART_IER_RXBRK                  (0x01U << UART_IER_RXBRK_Pos)           ///< Receive frame break interrupt enable bit

#define UART_IER_TXBRK_Pos          (7)
#define UART_IER_TXBRK              (0x01U << UART_IER_TXBRK_Pos)           ///< Transmit Break Frame Interrupt Enable Bit
#define UART_IER_RXB8_Pos           (8)
#define UART_IER_RXB8               (0x01U << UART_IER_RXB8_Pos)            ///< Receive Bit 8 Interrupt Enable Bit

#define UART_IER_RXIDLE_Pos         (9)
#define UART_IER_RXIDLE             (0x01U << UART_IER_RXIDLE_Pos)          ///< Receive Bit 8 Interrupt clear Bit

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_ICR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////

#define UART_ICR_TX_Pos                 (0)
#define UART_ICR_TX                     (0x01U << UART_ICR_TX_Pos)              ///< Transmit buffer empty interrupt clear bit
#define UART_ICR_RX_Pos                 (1)
#define UART_ICR_RX                     (0x01U << UART_ICR_RX_Pos)              ///< Receive interrupt clear bit


#define UART_ICR_RXOERR_Pos             (3)
#define UART_ICR_RXOERR                 (0x01U << UART_ICR_RXOERR_Pos)          ///< Receive overflow error interrupt clear bit
#define UART_ICR_RXPERR_Pos             (4)
#define UART_ICR_RXPERR                 (0x01U << UART_ICR_RXPERR_Pos)          ///< Parity error interrupt clear bit
#define UART_ICR_RXFERR_Pos             (5)
#define UART_ICR_RXFERR                 (0x01U << UART_ICR_RXFERR_Pos)          ///< Frame error interrupt clear bit
#define UART_ICR_RXBRK_Pos              (6)
#define UART_ICR_RXBRK                  (0x01U << UART_ICR_RXBRK_Pos)           ///< Receive frame break interrupt clear bit


#define UART_ICR_RXIDLE_Pos         (9)
#define UART_ICR_RXIDLE             (0x01U << UART_ICR_RXIDLE_Pos)          ///< Receive Bit 8 Interrupt clear Bit

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_GCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_GCR_UART_Pos               (0)
#define UART_GCR_UART                   (0x01U << UART_GCR_UART_Pos)            ///< UART mode selection bit
#define UART_GCR_DMA_Pos                (1)
#define UART_GCR_DMA                    (0x01U << UART_GCR_DMA_Pos)             ///< DMA mode selection bit
#define UART_GCR_AUTOFLOW_Pos           (2)
#define UART_GCR_AUTOFLOW               (0x01U << UART_GCR_AUTOFLOW_Pos)        ///< Automatic flow control enable bit
#define UART_GCR_RX_Pos                 (3)
#define UART_GCR_RX                     (0x01U << UART_GCR_RX_Pos)              ///< Enable receive
#define UART_GCR_TX_Pos                 (4)
#define UART_GCR_TX                     (0x01U << UART_GCR_TX_Pos)              ///< Enable transmit

#define UART_GCR_SELB8_Pos          (7)
#define UART_GCR_SELB8              (0x01U << UART_GCR_SELB8_Pos)           ///< UART mode selection bit
#define UART_GCR_SWAP_Pos           (8)
#define UART_GCR_SWAP               (0x01U << UART_GCR_SWAP_Pos)            ///< DMA mode selection bit
#define UART_GCR_RXTOG_Pos          (9)
#define UART_GCR_RXTOG              (0x01U << UART_GCR_RXTOG_Pos)           ///< Automatic flow control enable bit
#define UART_GCR_TXTOG_Pos          (10)
#define UART_GCR_TXTOG              (0x01U << UART_GCR_TXTOG_Pos)           ///< Enable receive

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_CCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_CCR_PEN_Pos                (0)
#define UART_CCR_PEN                    (0x01U << UART_CCR_PEN_Pos)             ///< Parity enable bit
#define UART_CCR_PSEL_Pos               (1)
#define UART_CCR_PSEL                   (0x01U << UART_CCR_PSEL_Pos)            ///< Parity selection bit

#define UART_CCR_SPB_Pos                (2)
#define UART_CCR_SPB                    (0x01U << UART_CCR_SPB_Pos)             ///< Stop bit selection


#define UART_CCR_BRK_Pos                (3)
#define UART_CCR_BRK                    (0x01U << UART_CCR_BRK_Pos)             ///< UART transmit frame break
#define UART_CCR_CHAR_Pos               (4)
#define UART_CCR_CHAR                   (0x03U << UART_CCR_CHAR_Pos)            ///< UART width bit
#define UART_CCR_CHAR_5b                (0x00U << UART_CCR_CHAR_Pos)            ///< UART Word Length 5b
#define UART_CCR_CHAR_6b                (0x01U << UART_CCR_CHAR_Pos)            ///< UART Word Length 6b
#define UART_CCR_CHAR_7b                (0x02U << UART_CCR_CHAR_Pos)            ///< UART Word Length 7b
#define UART_CCR_CHAR_8b                (0x03U << UART_CCR_CHAR_Pos)            ///< UART Word Length 8b

#define UART_CCR_SPB1_Pos           (6)
#define UART_CCR_SPB1               (0x01U << UART_CCR_SPB1_Pos)            ///< Stop bit 1 selection
#define UART_CCR_B8RXD_Pos          (7)
#define UART_CCR_B8RXD              (0x01U << UART_CCR_B8RXD_Pos)           ///< Synchronous frame receive
#define UART_CCR_B8TXD_Pos          (8)
#define UART_CCR_B8TXD              (0x01U << UART_CCR_B8TXD_Pos)           ///< Synchronous frame transmit
#define UART_CCR_B8POL_Pos          (9)
#define UART_CCR_B8POL              (0x01U << UART_CCR_B8POL_Pos)           ///< Synchronous frame polarity control bit
#define UART_CCR_B8TOG_Pos          (10)
#define UART_CCR_B8TOG              (0x01U << UART_CCR_B8TOG_Pos)           ///< Synchronous frame auto toggle bit
#define UART_CCR_B8EN_Pos           (11)
#define UART_CCR_B8EN               (0x01U << UART_CCR_B8EN_Pos)            ///< Synchronous frame enable bit
#define UART_CCR_RWU_Pos            (12)
#define UART_CCR_RWU                (0x01U << UART_CCR_RWU_Pos)             ///< Receive wake up method
#define UART_CCR_WAKE_Pos           (13)
#define UART_CCR_WAKE               (0x01U << UART_CCR_WAKE_Pos)            ///< Wake up method

#define UART_CCR_LIN_Pos            (14)
#define UART_CCR_LIN                (0x01U << UART_CCR_LIN_Pos)             ///< Wake up method

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_BRR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_BRR_MANTISSA_Pos           (0)
#define UART_BRR_MANTISSA               (0xFFFFU << UART_BRR_MANTISSA_Pos)      ///< UART DIV MANTISSA

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_FRA Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_BRR_FRACTION_Pos           (0)
#define UART_BRR_FRACTION               (0x0FU << UART_BRR_FRACTION_Pos)        ///< UART DIV FRACTION

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_RXAR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_RXAR_ADDR_Pos          (0)
#define UART_RXAR_ADDR              (0xFFU << UART_RXAR_ADDR_Pos)           ///< Synchronous frame match address

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_RXMR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_RXMR_MASK_Pos          (0)
#define UART_RXMR_MASK              (0xFFU << UART_RXMR_MASK_Pos)           ///< Synchronous frame match address mask

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_SCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_SCR_SCEN_Pos           (0)
#define UART_SCR_SCEN               (0x01U << UART_SCR_SCEN_Pos)            ///< ISO7816 enable bit
#define UART_SCR_SCARB_Pos          (1)
#define UART_SCR_SCARB              (0x01U << UART_SCR_SCARB_Pos)           ///< ISO7816 check auto answer bit
#define UART_SCR_NACK_Pos           (2)
#define UART_SCR_NACK               (0x01U << UART_SCR_NACK_Pos)            ///< Master receive frame answer bit
#define UART_SCR_SCFCNT_Pos         (4)
#define UART_SCR_SCFCNT             (0xFFU << UART_SCR_SCFCNT_Pos)          ///< ISO7816 protection counter bit
#define UART_SCR_HDSEL_Pos          (12)
#define UART_SCR_HDSEL              (0x01U << UART_SCR_HDSEL_Pos)           ///< Single-line half-duplex mode selection bit

////////////////////////////////////////////////////////////////////////////////
/// @brief UART_SCR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define UART_IDLR_IDLR_Pos          (0)
#define UART_IDLR_IDLR              (0xFFFFU << UART_IDLR_IDLR_Pos)         ///< ISO7816 enable bit

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG_CR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define WWDG_CR_CNT_Pos                 (0)
#define WWDG_CR_CNT                     (0x7FU << WWDG_CR_CNT_Pos)              ///< T[6:0] bits (7-Bit counter (MSB to LSB))
#define WWDG_CR_WDGA_Pos                (7)
#define WWDG_CR_WDGA                    (0x01U << WWDG_CR_WDGA_Pos)             ///< Activation bit

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG_CFR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define WWDG_CFGR_WINDOW_Pos            (0)
#define WWDG_CFGR_WINDOW                (0x7FU << WWDG_CFGR_WINDOW_Pos)         ///< W[6:0] bits (7-bit window value)
#define WWDG_CFGR_WDGTB_Pos             (7)
#define WWDG_CFGR_WDGTB                 (0x03U << WWDG_CFGR_WDGTB_Pos)          ///< WDGTB[1:0] bits (Timer Base)
#define WWDG_CFGR_WDGTB_1               (0x00U << WWDG_CFGR_WDGTB_Pos)          ///< WDGTB[1:0] bits (Timer Base /1)
#define WWDG_CFGR_WDGTB_2               (0x01U << WWDG_CFGR_WDGTB_Pos)          ///< WDGTB[1:0] bits (Timer Base /2)
#define WWDG_CFGR_WDGTB_4               (0x02U << WWDG_CFGR_WDGTB_Pos)          ///< WDGTB[1:0] bits (Timer Base /4)
#define WWDG_CFGR_WDGTB_8               (0x03U << WWDG_CFGR_WDGTB_Pos)          ///< WDGTB[1:0] bits (Timer Base /8)
#define WWDG_CFGR_EWI_Pos               (9)
#define WWDG_CFGR_EWI                   (0x01U << WWDG_CFGR_EWI_Pos)            ///< Early Wakeup Interrupt

////////////////////////////////////////////////////////////////////////////////
/// @brief WWDG_SR Register Bit Definition
////////////////////////////////////////////////////////////////////////////////
#define WWDG_SR_EWIF_Pos                (0)
#define WWDG_SR_EWIF                    (0x01U << WWDG_SR_EWIF_Pos)             ///< Early Wakeup Interrupt Flag


////////////////////////////////////////////////////////////////////////////////
#include "mm32_reg_redefine_v1.h"
////////////////////////////////////////////////////////////////////////////////

/// @}

/// @}

/// @}

////////////////////////////////////////////////////////////////////////////////
#endif // __MM32_REG_H
////////////////////////////////////////////////////////////////////////////////
