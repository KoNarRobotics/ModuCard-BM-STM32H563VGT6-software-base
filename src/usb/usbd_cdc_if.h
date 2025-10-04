#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define APP_RX_DATA_SIZE 512
#define APP_TX_DATA_SIZE 512

extern USBD_CDC_ItfTypeDef USBD_CDC_Template_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t TEMPLATE_Transmit(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

