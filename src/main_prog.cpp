#include "main_prog.hpp"
#include "Timing.hpp"
#include "atmodem.hpp"
#include "can_messages.h"
#include "fdcan.hpp"
#include "logger.hpp"
#include "main.hpp"
#include "simple_task.hpp"
#include "uart.hpp"
#include "usbd_cdc_if.h"

std::shared_ptr<se::UART> uart5 = nullptr;
std::shared_ptr<se::FDCAN> fdcan1 = nullptr;
std::shared_ptr<se::FDCAN> fdcan2 = nullptr;

se::GpioPin gpio_boot_enable(*BOOT_EN_GPIO_Port, BOOT_EN_Pin);
se::GpioPin gpio_user_led_1(*USER_LED_1_GPIO_Port, USER_LED_1_Pin);
se::GpioPin gpio_user_led_2(*USER_LED_2_GPIO_Port, USER_LED_2_Pin);
se::GpioPin gpio_status_led(*STATUS_LED_GPIO_Port, STATUS_LED_Pin);
se::GpioPin gpio_usr_button(*USR_BUTTON_GPIO_Port, USR_BUTTON_Pin);

se::SimpleTask task_blink;

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
extern "C" {
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    se::Ticker::get_instance().irq_update_ticker();
    HAL_IncTick();
  }

  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}
}

Status init_board(se::SimpleTask &task, void *pvParameters) {
  gpio_user_led_1.write(0);
  gpio_user_led_2.write(0);
  gpio_status_led.write(0);

  se::Status stat = se::Status::OK();

  se::DeviceThreadedSettings settings;
  settings.uxStackDepth = 1024;
  settings.uxPriority = 2;
  settings.period = 10;

  // STMEPIC_RETURN_ON_ERROR(fdcan->add_callback(CAN_BAROMETER_STATUS_FRAME_ID,
  //                                             can_callback_bmp280_get_status,
  //                                             bmp280.get()));

  ///////////////////////////////////////////////////////////////////////////
  // HERE ADD MORE OF YOUR INIT CODE

  ///////////////////////////////////////////////////////////////////////////
  return stat;
}

Status task_blink_func(se::SimpleTask &task, void *pvParameters) {
  (void)pvParameters;

  if (!task.task_get_status().ok()) {
    gpio_status_led.toggle();
    return task.task_get_status();
  }

  return Status::OK();
}

void main_prog() {
  // START ALL INTERRUPTS
  HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);

  // INIT USB CO< port
  MX_USB_PCD_Init();
  MX_USB_Device_Init();

  // INIT LOGGER
  std::string version = std::to_string(VERSION_MAJOR) + "." +
                        std::to_string(VERSION_MINOR) + "." +
                        std::to_string(VERSION_BUILD);
  se::Logger::get_instance().init(se::LOG_LEVEL::LOG_LEVEL_DEBUG, true,
                                  TEMPLATE_Transmit, false, version);

  // INIT UART HANDLERS
  STMEPIC_ASSING_TO_OR_HRESET(uart5,
                              se::UART::Make(huart5, se::HardwareType::DMA));
  uart5->hardware_start();

  // INIT FDCAN HANDLER
  FDCAN_FilterTypeDef sFilterConfig1 = {};
  sFilterConfig1.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0; // 0x915;
  sFilterConfig1.FilterID2 = 0; // 0x1FFFFFFF; // all have to match
  // sFilterConfig1.RxBufferIndex       = 0;
  // sFilterConfig1.IsCalibrationMsg    = 0;

  se::FDcanFilterConfig filter_config1;
  filter_config1.filters.push_back(sFilterConfig1);
  filter_config1.fifo_number = se::FDCAN_FIFO::FDCAN_FIFO0;
  filter_config1.globalFilter_NonMatchingStd = FDCAN_REJECT;
  filter_config1.globalFilter_NonMatchingExt = FDCAN_REJECT;
  filter_config1.globalFilter_RejectRemoteStd = FDCAN_FILTER_REMOTE;
  filter_config1.globalFilter_RejectRemoteExt = FDCAN_FILTER_REMOTE;
  STMEPIC_ASSING_TO_OR_HRESET(
      fdcan1, se::FDCAN::Make(hfdcan1, filter_config1, nullptr, nullptr));

  FDCAN_FilterTypeDef sFilterConfig2 = {};
  sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig2.FilterIndex = 1;
  sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig2.FilterID1 = 0; // 0x915;
  sFilterConfig2.FilterID2 = 0; // 0x1FFFFFFF; // all have to match
  // sFilterConfig2.RxBufferIndex       = 0;
  // sFilterConfig2.IsCalibrationMsg    = 0;

  se::FDcanFilterConfig filter_config2;
  filter_config2.filters.push_back(sFilterConfig2);
  filter_config2.fifo_number = se::FDCAN_FIFO::FDCAN_FIFO0;
  filter_config2.globalFilter_NonMatchingStd = FDCAN_REJECT;
  filter_config2.globalFilter_NonMatchingExt = FDCAN_REJECT;
  filter_config2.globalFilter_RejectRemoteStd = FDCAN_FILTER_REMOTE;
  filter_config2.globalFilter_RejectRemoteExt = FDCAN_FILTER_REMOTE;

  STMEPIC_ASSING_TO_OR_HRESET(
      fdcan2, se::FDCAN::Make(hfdcan2, filter_config2, nullptr, nullptr));

  fdcan1->hardware_start();
  fdcan2->hardware_start();

  // START MAIN TASK
  task_blink.task_init(task_blink_func, nullptr, 100, init_board, 3500, 2,
                       "MainTask", false);
  task_blink.task_run();
}