/**
  ******************************************************************************
  * @file    stm32h563xx.h
  * @brief   CMSIS STM32H563xx Device Peripheral Access Layer Header File.
  ******************************************************************************
  */

#ifndef __STM32H563xx_H
#define __STM32H563xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M33 Processor and Core Peripherals
   */
#define __CM33_REV                0x0000U  /*!< Cortex-M33 revision r0p1                       */
#define __MPU_PRESENT             1U       /*!< STM32H5xx provides an MPU                      */
#define __VTOR_PRESENT            1U       /*!< VTOR present                                   */
#define __NVIC_PRIO_BITS          4U       /*!< STM32H5xx uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                    */
#define __DSP_PRESENT             1U       /*!< DSP extension present                          */
#define __SAUREGION_PRESENT       1U       /*!< SAU regions present                            */
#define __ICACHE_PRESENT          1U       /*!< Instruction cache present                      */
#define __DCACHE_PRESENT          1U       /*!< Data cache present                             */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32H563xx Interrupt Number Definition
 */
typedef enum
{
/******  Cortex-M33 Processor Exceptions Numbers *************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M33 Hard Fault Interrupt                 */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M33 Memory Management Interrupt          */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M33 Bus Fault Interrupt                  */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M33 Usage Fault Interrupt                */
  SecureFault_IRQn            = -9,     /*!< 7 Cortex-M33 Secure Fault Interrupt               */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M33 SV Call Interrupt                   */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M33 Debug Monitor Interrupt             */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M33 Pend SV Interrupt                   */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M33 System Tick Interrupt               */
/******  STM32 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM through EXTI Line detection Interrupt     */
  RTC_IRQn                    = 2,      /*!< RTC non-secure interrupt                          */
  RTC_S_IRQn                  = 3,      /*!< RTC secure interrupt                              */
  TAMP_IRQn                   = 4,      /*!< Tamper non-secure interrupt                       */
  RAMCFG_IRQn                 = 5,      /*!< RAMCFG global interrupt                           */
  FLASH_IRQn                  = 6,      /*!< FLASH non-secure global interrupt                 */
  FLASH_S_IRQn                = 7,      /*!< FLASH secure global interrupt                     */
  GTZC_IRQn                   = 8,      /*!< Global TrustZone Controller interrupt             */
  RCC_IRQn                    = 9,      /*!< RCC non-secure global interrupt                   */
  RCC_S_IRQn                  = 10,     /*!< RCC secure global interrupt                       */
  EXTI0_IRQn                  = 11,     /*!< EXTI Line0 interrupt                              */
  EXTI1_IRQn                  = 12,     /*!< EXTI Line1 interrupt                              */
  EXTI2_IRQn                  = 13,     /*!< EXTI Line2 interrupt                              */
  EXTI3_IRQn                  = 14,     /*!< EXTI Line3 interrupt                              */
  EXTI4_IRQn                  = 15,     /*!< EXTI Line4 interrupt                              */
  EXTI5_IRQn                  = 16,     /*!< EXTI Line5 interrupt                              */
  EXTI6_IRQn                  = 17,     /*!< EXTI Line6 interrupt                              */
  EXTI7_IRQn                  = 18,     /*!< EXTI Line7 interrupt                              */
  EXTI8_IRQn                  = 19,     /*!< EXTI Line8 interrupt                              */
  EXTI9_IRQn                  = 20,     /*!< EXTI Line9 interrupt                              */
  EXTI10_IRQn                 = 21,     /*!< EXTI Line10 interrupt                             */
  EXTI11_IRQn                 = 22,     /*!< EXTI Line11 interrupt                             */
  EXTI12_IRQn                 = 23,     /*!< EXTI Line12 interrupt                             */
  EXTI13_IRQn                 = 24,     /*!< EXTI Line13 interrupt                             */
  EXTI14_IRQn                 = 25,     /*!< EXTI Line14 interrupt                             */
  EXTI15_IRQn                 = 26,     /*!< EXTI Line15 interrupt                             */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm33.h"             /* Cortex-M33 processor and core peripherals */
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief General Purpose I/O
  */
typedef struct
{
  volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  volatile uint32_t CR;           /*!< RCC clock control register,                                     Address offset: 0x00  */
  volatile uint32_t HSICFGR;      /*!< RCC HSI configuration register,                                 Address offset: 0x04  */
  volatile uint32_t CRRCR;        /*!< RCC clock recovery RC register,                                 Address offset: 0x08  */
  volatile uint32_t CSICFGR;      /*!< RCC CSI configuration register,                                 Address offset: 0x0C  */
  volatile uint32_t CFGR1;        /*!< RCC clock configuration register 1,                             Address offset: 0x10  */
  volatile uint32_t CFGR2;        /*!< RCC clock configuration register 2,                             Address offset: 0x14  */
  volatile uint32_t RESERVED1;    /*!< Reserved,                                                       Address offset: 0x18  */
  volatile uint32_t PLL1CFGR;     /*!< RCC PLL1 configuration register,                                Address offset: 0x1C  */
  volatile uint32_t PLL2CFGR;     /*!< RCC PLL2 configuration register,                                Address offset: 0x20  */
  volatile uint32_t PLL3CFGR;     /*!< RCC PLL3 configuration register,                                Address offset: 0x24  */
  volatile uint32_t PLL1DIVR;     /*!< RCC PLL1 dividers configuration register,                       Address offset: 0x28  */
  volatile uint32_t PLL1FRACR;    /*!< RCC PLL1 fractional divider register,                           Address offset: 0x2C  */
  volatile uint32_t PLL2DIVR;     /*!< RCC PLL2 dividers configuration register,                       Address offset: 0x30  */
  volatile uint32_t PLL2FRACR;    /*!< RCC PLL2 fractional divider register,                           Address offset: 0x34  */
  volatile uint32_t PLL3DIVR;     /*!< RCC PLL3 dividers configuration register,                       Address offset: 0x38  */
  volatile uint32_t PLL3FRACR;    /*!< RCC PLL3 fractional divider register,                           Address offset: 0x3C  */
  volatile uint32_t RESERVED2;    /*!< Reserved,                                                       Address offset: 0x40  */
  volatile uint32_t CIER;         /*!< RCC clock interrupt enable register,                            Address offset: 0x44  */
  volatile uint32_t CIFR;         /*!< RCC clock interrupt flag register,                              Address offset: 0x48  */
  volatile uint32_t CICR;         /*!< RCC clock interrupt clear register,                             Address offset: 0x4C  */
  volatile uint32_t AHB1RSTR;     /*!< RCC AHB1 peripheral reset register,                             Address offset: 0x50  */
  volatile uint32_t AHB2RSTR;     /*!< RCC AHB2 peripheral reset register,                             Address offset: 0x54  */
  volatile uint32_t AHB4RSTR;     /*!< RCC AHB4 peripheral reset register,                             Address offset: 0x58  */
  volatile uint32_t RESERVED3;    /*!< Reserved,                                                       Address offset: 0x5C  */
  volatile uint32_t APB1RSTR1;    /*!< RCC APB1 peripheral reset register 1,                           Address offset: 0x60  */
  volatile uint32_t APB1RSTR2;    /*!< RCC APB1 peripheral reset register 2,                           Address offset: 0x64  */
  volatile uint32_t APB2RSTR;     /*!< RCC APB2 peripheral reset register,                             Address offset: 0x68  */
  volatile uint32_t APB3RSTR;     /*!< RCC APB3 peripheral reset register,                             Address offset: 0x6C  */
  volatile uint32_t RESERVED4;    /*!< Reserved,                                                       Address offset: 0x70  */
  volatile uint32_t AHB1ENR;      /*!< RCC AHB1 peripheral clock enable register,                      Address offset: 0x74  */
  volatile uint32_t AHB2ENR;      /*!< RCC AHB2 peripheral clock enable register,                      Address offset: 0x78  */
  volatile uint32_t AHB4ENR;      /*!< RCC AHB4 peripheral clock enable register,                      Address offset: 0x7C  */
  volatile uint32_t RESERVED5;    /*!< Reserved,                                                       Address offset: 0x80  */
  volatile uint32_t APB1ENR1;     /*!< RCC APB1 peripheral clock enable register 1,                    Address offset: 0x84  */
  volatile uint32_t APB1ENR2;     /*!< RCC APB1 peripheral clock enable register 2,                    Address offset: 0x88  */
  volatile uint32_t APB2ENR;      /*!< RCC APB2 peripheral clock enable register,                      Address offset: 0x8C  */
  volatile uint32_t APB3ENR;      /*!< RCC APB3 peripheral clock enable register,                      Address offset: 0x90  */
} RCC_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            0x08000000UL /*!< FLASH base address */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1 base address */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define APB3PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00040000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x02000000UL)
#define AHB4PERIPH_BASE       (PERIPH_BASE + 0x04000000UL)

/*!< AHB4 peripherals */
#define GPIOA_BASE            (AHB4PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB4PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB4PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB4PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB4PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB4PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB4PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB4PERIPH_BASE + 0x1C00UL)

#define RCC_BASE              (AHB4PERIPH_BASE + 0x4400UL)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/* Bits definition for RCC_AHB4ENR register */
#define RCC_AHB4ENR_GPIOAEN_Pos           (0U)
#define RCC_AHB4ENR_GPIOAEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOAEN_Pos)
#define RCC_AHB4ENR_GPIOAEN               RCC_AHB4ENR_GPIOAEN_Msk
#define RCC_AHB4ENR_GPIOBEN_Pos           (1U)
#define RCC_AHB4ENR_GPIOBEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOBEN_Pos)
#define RCC_AHB4ENR_GPIOBEN               RCC_AHB4ENR_GPIOBEN_Msk
#define RCC_AHB4ENR_GPIOCEN_Pos           (2U)
#define RCC_AHB4ENR_GPIOCEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOCEN_Pos)
#define RCC_AHB4ENR_GPIOCEN               RCC_AHB4ENR_GPIOCEN_Msk
#define RCC_AHB4ENR_GPIODEN_Pos           (3U)
#define RCC_AHB4ENR_GPIODEN_Msk           (0x1UL << RCC_AHB4ENR_GPIODEN_Pos)
#define RCC_AHB4ENR_GPIODEN               RCC_AHB4ENR_GPIODEN_Msk
#define RCC_AHB4ENR_GPIOEEN_Pos           (4U)
#define RCC_AHB4ENR_GPIOEEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOEEN_Pos)
#define RCC_AHB4ENR_GPIOEEN               RCC_AHB4ENR_GPIOEEN_Msk
#define RCC_AHB4ENR_GPIOFEN_Pos           (5U)
#define RCC_AHB4ENR_GPIOFEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOFEN_Pos)
#define RCC_AHB4ENR_GPIOFEN               RCC_AHB4ENR_GPIOFEN_Msk
#define RCC_AHB4ENR_GPIOGEN_Pos           (6U)
#define RCC_AHB4ENR_GPIOGEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOGEN_Pos)
#define RCC_AHB4ENR_GPIOGEN               RCC_AHB4ENR_GPIOGEN_Msk
#define RCC_AHB4ENR_GPIOHEN_Pos           (7U)
#define RCC_AHB4ENR_GPIOHEN_Msk           (0x1UL << RCC_AHB4ENR_GPIOHEN_Pos)
#define RCC_AHB4ENR_GPIOHEN               RCC_AHB4ENR_GPIOHEN_Msk

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32H563xx_H */
