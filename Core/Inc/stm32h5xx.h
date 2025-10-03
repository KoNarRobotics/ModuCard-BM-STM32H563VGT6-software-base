/**
  ******************************************************************************
  * @file    stm32h5xx.h
  * @brief   CMSIS STM32H5xx Device Peripheral Access Layer Header File.
  ******************************************************************************
  */

#ifndef __STM32H5xx_H
#define __STM32H5xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32H5)
#define STM32H5
#endif /* STM32H5 */

/* Uncomment the line below according to the target STM32H5 device used in your
   application
  */

#if !defined (STM32H563xx)
  #define STM32H563xx   /*!< STM32H563xx Devices */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */

#if defined(STM32H563xx)
  #include "stm32h563xx.h"
#else
 #error "Please select first the target STM32H5xx device used in your application (in stm32h5xx.h file)"
#endif

typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32H5xx_H */
