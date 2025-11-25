#pragma once
#include "fdcan.hpp"
#include "main.hpp"
#include "stmepic.hpp"
#include "uart.hpp"
#include "version.hpp"

// USB Device includes
#include "modu_card.hpp"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_composite_builder.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

namespace se = stmepic;

////////////////////////////////////////////////////////////////////////////////
//
//         DO NOT CHANGE THIS FILE UNLESS YOU KNOW WHAT YOU ARE DOING!!!
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// BOARD SETTINGS
extern std::shared_ptr<moducard::ModuCardBoard> modu_card_board;
extern uint32_t CAN_MODULE_BASE_ADDRESS;

////////////////////////////////////////////////////////////////////////////////
// HARDWARE INTERFACES
extern std::shared_ptr<se::CanBase> can1;
extern std::shared_ptr<se::CanBase> can2;
extern std::shared_ptr<se::UartBase> uart;

extern std::shared_ptr<moducard::ModuCardBoard> modu_card_board;

///////////////////////////////////////////////////////////////////////////////
/// ADDITIONAL PINS

extern se::GpioPin gpio_user_led_1;
extern se::GpioPin gpio_user_led_2;
extern se::GpioPin gpio_status_led;
extern se::GpioPin gpio_usr_button;

///////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTIONS
void config_usb_device();
void main_prog();
se::Status init_board();
