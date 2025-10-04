#pragma once
#include "fdcan.hpp"
#include "main.hpp"
#include "stmepic.hpp"
#include "version.hpp"

// USB Device includes
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_composite_builder.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

namespace se = stmepic;

void config_usb_device();

void main_prog();
