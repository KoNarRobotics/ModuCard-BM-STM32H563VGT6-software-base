#pragma once
#include "main.h"
#include <memory.h>

namespace stmepic {

/**
 * @addtogroup W25Q_Driver
 * @brief W25Q QSPI Driver
 * @{
 */

/**
 * @defgroup W25Q_Param W25Q Chip's Parameters
 * @brief User's chip parameters
 * @{
 */
// YOUR CHIP'S SETTINGS
/// Mem size in MB-bit
#define MEM_FLASH_SIZE 128U // 256 MB-bit
/// Mem big block size in KB
#define MEM_BLOCK_SIZE 64U // 64 KB: 256 pages
/// Mem small block size in KB
#define MEM_SBLOCK_SIZE 32U // 32 KB: 128 pages
/// Mem sector size in KB
#define MEM_SECTOR_SIZE 4U // 4 KB : 16 pages
/// Mem page size in bytes
#define MEM_PAGE_SIZE 256U // 256 byte : 1 page
/// Blocks count
#define BLOCK_COUNT (MEM_FLASH_SIZE * 2) // 512 blocks
/// Sector count
#define SECTOR_COUNT (BLOCK_COUNT * 16) // 8192 sectors
/// Pages count
#define PAGE_COUNT (SECTOR_COUNT * 16) // 131'072 pages

/**@}*/

/**
 * @enum W25Q_STATE
 * @brief W25Q Return State
 * Lib's functions status returns
 * @{
 */
enum class W25Q_STATE {
  W25Q_OK = 0,          ///< Chip OK - Execution fine
  W25Q_BUSY = 1,        ///< Chip busy
  W25Q_PARAM_ERR = 2,   ///< Function parameters error
  W25Q_CHIP_ERR = 3,    ///< Chip error
  W25Q_SPI_ERR = 4,     ///< SPI Bus err
  W25Q_CHIP_IGNORE = 5, ///< Chip ignore state
};
/** @} */

/**
 * @struct W25Q_STATUS_REG
 * @brief  W25Q Status Registers
 * @TODO: Mem protected recognition
 *
 * Structure to check chip's status registers
 * @{
 */
struct W25Q_STATUS_REG {
  bool BUSY;  ///< Erase/Write in progress
  bool WEL;   ///< Write enable latch (1 - write allowed)
  bool QE;    ///< Quad SPI mode
  bool SUS;   ///< Suspend Status
  bool ADS;   ///< Current addr mode (0-3 byte / 1-4 byte)
  bool ADP;   ///< Power-up addr mode
  bool SLEEP; ///< Sleep Status
};
/** @} */

class W25qOctoSpi {
public:
  W25qOctoSpi(XSPI_HandleTypeDef &h) : hqspi(h){};
  /**
   * @brief W25Q Init function
   *
   * @param none
   * @return W25Q_STATE enum
   */
  W25Q_STATE W25Q_Init(void); ///< Initalize function

  W25Q_STATE W25Q_EnableVolatileSR(void); ///< Make Status Register Volatile
  W25Q_STATE
  W25Q_ReadStatusReg(uint8_t *reg_data,
                     uint8_t reg_num); ///< Read status register to variable
  W25Q_STATE
  W25Q_WriteStatusReg(uint8_t reg_data,
                      uint8_t reg_num); ///< Write status register from variable
  W25Q_STATE W25Q_ReadStatusStruct(
      W25Q_STATUS_REG *status); ///< Read all status registers to struct
  W25Q_STATE W25Q_IsBusy(void); ///< Check chip's busy status

  W25Q_STATE W25Q_ReadSByte(int8_t *buf, uint8_t pageShift,
                            uint32_t pageNum); ///< Read signed 8-bit variable
  W25Q_STATE W25Q_ReadByte(uint8_t *buf, uint8_t pageShift,
                           uint32_t pageNum); ///< Read 8-bit variable
  W25Q_STATE W25Q_ReadSWord(int16_t *buf, uint8_t pageShift,
                            uint32_t pageNum); ///< Read signed 16-bit variable
  W25Q_STATE W25Q_ReadWord(uint16_t *buf, uint8_t pageShift,
                           uint32_t pageNum); ///< Read 16-bit variable
  W25Q_STATE W25Q_ReadSLong(int32_t *buf, uint8_t pageShift,
                            uint32_t pageNum); ///< Read signed 32-bit variable
  W25Q_STATE W25Q_ReadLong(uint32_t *buf, uint8_t pageShift,
                           uint32_t pageNum); ///< Read 32-bit variable
  W25Q_STATE W25Q_ReadData(uint8_t *buf, uint16_t len, uint8_t pageShift,
                           uint32_t pageNum); ///< Read any 8-bit data
  W25Q_STATE W25Q_ReadRaw(uint8_t *buf, uint16_t data_len,
                          uint32_t rawAddr); ///< Read data from raw addr
  W25Q_STATE
  W25Q_SingleRead(uint8_t *buf, uint32_t len,
                  uint32_t Addr); ///< Read data from raw addr by single line

  // W25Q_STATE W25Q_EraseSector(uint32_t SectAddr);			///<
  // Erase 4KB Sector W25Q_STATE W25Q_EraseBlock(uint32_t BlockAddr, uint8_t
  // size); ///< Erase 32KB/64KB Sector W25Q_STATE W25Q_EraseChip(void);
  // ///< Erase all chip

  W25Q_STATE
  W25Q_ProgramSByte(int8_t buf, uint8_t pageShift,
                    uint32_t pageNum); ///< Program signed 8-bit variable
  W25Q_STATE W25Q_ProgramByte(uint8_t buf, uint8_t pageShift,
                              uint32_t pageNum); ///< Program 8-bit variable
  W25Q_STATE
  W25Q_ProgramSWord(int16_t buf, uint8_t pageShift,
                    uint32_t pageNum); ///< Program signed 16-bit variable
  W25Q_STATE W25Q_ProgramWord(uint16_t buf, uint8_t pageShift,
                              uint32_t pageNum); ///< Program 16-bit variable
  W25Q_STATE
  W25Q_ProgramSLong(int32_t buf, uint8_t pageShift,
                    uint32_t pageNum); ///< Program signed 32-bit variable
  W25Q_STATE W25Q_ProgramLong(uint32_t buf, uint8_t pageShift,
                              uint32_t pageNum); ///< Program 32-bit variable
  W25Q_STATE W25Q_ProgramData(uint8_t *buf, uint16_t len, uint8_t pageShift,
                              uint32_t pageNum); ///< Program any 8-bit data
  W25Q_STATE W25Q_ProgramRaw(uint8_t *buf, uint16_t data_len,
                             uint32_t rawAddr); ///< Program data to raw addr

  W25Q_STATE W25Q_ReadID(uint8_t *buf); ///< Read chip ID

  XSPI_HandleTypeDef &hqspi;

private:
  W25Q_STATUS_REG w25q_status;

  W25Q_STATE W25Q_WriteEnable(bool enable);      ///< Toggle WOL bit
  W25Q_STATE W25Q_EnableQSPI(bool enable);       ///< Toggle QE bit
  W25Q_STATE W25Q_Enter4ByteMode(bool enable);   ///< Toggle ADS bit
  W25Q_STATE W25Q_SetExtendedAddr(uint8_t Addr); ///< Set addr in 3-byte mode
  W25Q_STATE
  W25Q_GetExtendedAddr(uint8_t *outAddr); ///< Get addr in 3-byte mode

  uint32_t
  page_to_addr(uint32_t pageNum,
               uint8_t pageShift); ///< Translate page addr to byte addr
};

/// @}
} // namespace stmepic