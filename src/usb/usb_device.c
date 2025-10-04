/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/USB_Device/App/usb_device.c
  * @author  MCD Application Team
  * @brief   This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "main.hpp"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO uint32_t remotewakeupon = 0;
uint8_t HID_Buffer[4];
USBD_HandleTypeDef hUsbDeviceFS;

extern PCD_HandleTypeDef hpcd_USB_FS;
extern USBD_DescriptorsTypeDef CDC_Desc;


/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */


/* USB Device Core handle declaration. */

/*
 * -- Insert your variables declaration here --
 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_Device_Init(void)
{
  /* USER CODE BEGIN USB_Device_Init_PreTreatment */
  /* USER CODE END USB_Device_Init_PreTreatment */
  hpcd_USB_DRD_FS.pData = &hUsbDeviceFS;
/* USER CODE END USB_Init 0 */

/* USER CODE BEGIN USB_Init 2 */
if(USBD_Init(&hUsbDeviceFS, &Class_Desc, 0) != USBD_OK)
      Error_Handler();

if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
      Error_Handler();

if(USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_Template_fops) != USBD_OK)
      Error_Handler();

if(USBD_Start(&hUsbDeviceFS) != USBD_OK)
      Error_Handler();

  /* Init Device Library, add supported class and start the library. */
  // if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, 0) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  // if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  // if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  /* USER CODE BEGIN USB_Device_Init_PostTreatment */

  /* USER CODE END USB_Device_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */
