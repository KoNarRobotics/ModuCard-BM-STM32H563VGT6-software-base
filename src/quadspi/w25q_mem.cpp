
#include "w25q_mem.hpp"
#include "main.h"

using namespace stmepic;

/**
 * @defgroup W25Q_Commands W25Q Chip's Commands
 * @brief W25Q Chip commands from datasheet
 * @{
 */
#define W25Q_WRITE_ENABLE                                                      \
  0x06U ///< sets WEL bit, must be set before any write/program/erase
#define W25Q_WRITE_DISABLE 0x04U      ///< resets WEL bit (state after power-up)
#define W25Q_ENABLE_VOLATILE_SR 0x50U ///< check 7.1 in datasheet
#define W25Q_READ_SR1 0x05U           ///< read status-register 1
#define W25Q_READ_SR2 0x35U           ///< read status-register 2
#define W25Q_READ_SR3 0x15U           ///< read ststus-register 3
#define W25Q_WRITE_SR1 0x01U          ///< write status-register 1 (8.2.5)
#define W25Q_WRITE_SR2 0x31U          ///< write status-register 2 (8.2.5)
#define W25Q_WRITE_SR3 0x11U          ///< write status-register 3 (8.2.5)
#define W25Q_READ_EXT_ADDR_REG                                                 \
  0xC8U ///< read extended addr reg (only in 3-byte mode)
#define W25Q_WRITE_EXT_ADDR_REG                                                \
  0xC5U ///< write extended addr reg (only in 3-byte mode)
#define W25Q_ENABLE_4B_MODE 0xB7U  ///< enable 4-byte mode (128+ MB address)
#define W25Q_DISABLE_4B_MODE 0xE9U ///< disable 4-byte mode (<=128MB)
#define W25Q_READ_DATA 0x03U       ///< read data by standard SPI
#define W25Q_READ_DATA_4B 0x13U    ///< read data by standard SPI in 4-byte mode
#define W25Q_FAST_READ 0x0BU       ///< highest FR speed (8.2.12)
#define W25Q_FAST_READ_4B 0x0CU    ///< fast read in 4-byte mode
#define W25Q_FAST_READ_DUAL_OUT 0x3BU ///< fast read in dual-SPI OUTPUT (8.2.14)
#define W25Q_FAST_READ_DUAL_OUT_4B                                             \
  0x3CU ///< fast read in dual-SPI OUTPUT in 4-byte mode
#define W25Q_FAST_READ_QUAD_OUT 0x6BU ///< fast read in quad-SPI OUTPUT (8.2.16)
#define W25Q_FAST_READ_QUAD_OUT_4B                                             \
  0x6CU ///< fast read in quad-SPI OUTPUT in 4-byte mode
#define W25Q_FAST_READ_DUAL_IO                                                 \
  0xBBU ///< fast read in dual-SPI I/O (address transmits by both lines)
#define W25Q_FAST_READ_DUAL_IO_4B                                              \
  0xBCU ///< fast read in dual-SPI I/O in 4-byte mode
#define W25Q_FAST_READ_QUAD_IO                                                 \
  0xEBU ///< fast read in quad-SPI I/O (address transmits by quad lines)
#define W25Q_FAST_READ_QUAD_IO_4B                                              \
  0xECU                           ///< fast read in quad-SPI I/O in 4-byte mode
#define W25Q_SET_BURST_WRAP 0x77U ///< use with quad-I/O (8.2.22)
#define W25Q_PAGE_PROGRAM 0x02U ///< program page (256bytes) by single SPI line
#define W25Q_PAGE_PROGRAM_4B                                                   \
  0x12U ///< program page by single SPI in 4-byte mode
#define W25Q_PAGE_PROGRAM_QUAD_INP                                             \
  0x32U ///< program page (256bytes) by quad SPI lines
#define W25Q_PAGE_PROGRAM_QUAD_INP_4B                                          \
  0x34U ///< program page by quad SPI in 4-byte mode
#define W25Q_SECTOR_ERASE                                                      \
  0x20U ///< sets all 4Kbyte sector with 0xFF (erases it)
#define W25Q_SECTOR_ERASE_4B                                                   \
  0x21U ///< sets all 4Kbyte sector with 0xFF in 4-byte mode
#define W25Q_32KB_BLOCK_ERASE 0x52U ///< sets all 32Kbyte block with 0xFF
#define W25Q_64KB_BLOCK_ERASE 0xD8U ///< sets all 64Kbyte block with 0xFF
#define W25Q_64KB_BLOCK_ERASE_4B                                               \
  0xDCU ///< sets all 64Kbyte sector with 0xFF in 4-byte mode
#define W25Q_CHIP_ERASE 0xC7U ///< fill all the chip with 0xFF
// #define W25Q_CHIP_ERASE 0x60U				///< another way
// to erase chip
#define W25Q_ERASEPROG_SUSPEND                                                 \
  0x75U ///< suspend erase/program operation (can be applied only when SUS=0,
        ///< BYSY=1)
#define W25Q_ERASEPROG_RESUME                                                  \
  0x7AU ///< resume erase/program operation (if SUS=1, BUSY=0)
#define W25Q_POWERDOWN 0xB9U ///< powers down the chip (power-up by reading ID)
#define W25Q_POWERUP 0xABU   ///< release power-down
#define W25Q_DEVID 0xABU     ///< read Device ID (same as powerup)
#define W25Q_FULLID 0x90U    ///< read Manufacturer ID & Device ID
#define W25Q_FULLID_DUAL_IO                                                    \
  0x92U ///< read Manufacturer ID & Device ID by dual I/O
#define W25Q_FULLID_QUAD_IO                                                    \
  0x94U                     ///< read Manufacturer ID & Device ID by quad I/O
#define W25Q_READ_UID 0x4BU ///< read unique chip 64-bit ID
#define W25Q_READ_JEDEC_ID 0x9FU      ///< read JEDEC-standard ID
#define W25Q_READ_SFDP 0x5AU          ///< read SFDP register parameters
#define W25Q_ERASE_SECURITY_REG 0x44U ///< erase security registers
#define W25Q_PROG_SECURITY_REG 0x42U  ///< program security registers
#define W25Q_READ_SECURITY_REG 0x48U  ///< read security registers
#define W25Q_IND_BLOCK_LOCK 0x36U     ///< make block/sector read-only
#define W25Q_IND_BLOCK_UNLOCK 0x39U   ///< disable block/sector protection
#define W25Q_READ_BLOCK_LOCK 0x3DU    ///< check block/sector protection
#define W25Q_GLOBAL_LOCK 0x7EU        ///< global read-only protection enable
#define W25Q_GLOBAL_UNLOCK 0x98U      ///< global read-only protection disable
#define W25Q_ENABLE_RST 0x66U         ///< enable software-reset ability
#define W25Q_RESET 0x99U              ///< make software reset
/// @}

#define w25q_delay(x)                                                          \
  HAL_Delay(x)               ///< Delay define to provide future support of RTOS
W25Q_STATUS_REG w25q_status; ///< Internal status structure instance

W25Q_STATE W25qOctoSpi::W25Q_Init(void) {
  W25Q_STATE state; // temp status variable

  // read id
  uint8_t id = 0;
  state = W25Q_ReadID(&id);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  // u can check id here

  // read chip's state to private lib's struct
  state = W25Q_ReadStatusStruct(NULL);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

#if MEM_FLASH_SIZE > 128 // if 4-byte mode

  /* If power-default 4-byte
   mode disabled */
  if (!w25q_status.ADP) {
    uint8_t buf_reg = 0;
    state = W25Q_ReadStatusReg(&buf_reg, 3);
    if (state != W25Q_STATE::W25Q_OK)
      return state;
    buf_reg |= 0b10; // set ADP bit
    state = W25Q_WriteStatusReg(buf_reg, 3);
    if (state != W25Q_STATE::W25Q_OK)
      return state;
  }

  /* If current 4-byte
   mode disabled */
  if (!w25q_status.ADS) {
    state = W25Q_Enter4ByteMode(1);
    if (state != W25Q_STATE::W25Q_OK)
      return state;
  }
#endif

  /* If Quad-SPI mode disabled */
  if (!w25q_status.QE) {
    uint8_t buf_reg = 0;
    state = W25Q_ReadStatusReg(&buf_reg, 2);
    if (state != W25Q_STATE::W25Q_OK)
      return state;
    buf_reg |= 0b10;
    state = W25Q_WriteStatusReg(buf_reg, 2);
    if (state != W25Q_STATE::W25Q_OK)
      return state;
  }

  // make another read
  state = W25Q_ReadStatusStruct(NULL);
  // return communication status
  return state;
}

/**
 * @}
 * @addtogroup W25Q_Reg Register Functions
 * @brief Operations with Status Registers
 * @{
 */

/**
 * @brief W25Q Enable Volatile SR
 * Makes status register volatile (temporary)
 *
 * @attention Func in development
 * @param none
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_EnableVolatileSR(void) {
  return W25Q_STATE::W25Q_PARAM_ERR;
}

/**
 * @brief W25Q Read Status Register
 * Read one status register
 *
 * @param[out] reg_data 1 byte
 * @param[in] reg_num Desired register 1..3
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadStatusReg(uint8_t *reg_data, uint8_t reg_num) {
  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...

  if (reg_num == 1)
    com.Instruction = W25Q_READ_SR1;
  else if (reg_num == 2)
    com.Instruction = W25Q_READ_SR2;
  else if (reg_num == 3)
    com.Instruction = W25Q_READ_SR3;
  else
    return W25Q_STATE::W25Q_PARAM_ERR;

  com.AddressMode = HAL_XSPI_ADDRESS_NONE;
  com.AddressWidth = HAL_XSPI_ADDRESS_NONE;
  com.Address = 0x0U;

  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_8_BITS;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  com.DataMode = HAL_XSPI_DATA_1_LINE;
  com.DataLength = 1;

  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
  com.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }
  if (HAL_XSPI_Receive(&hqspi, reg_data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }

  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Write Status Register
 * Write one status register
 *
 * @param[in] reg_data 1 byte
 * @param[in] reg_num Desired register 1..3
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_WriteStatusReg(uint8_t reg_data, uint8_t reg_num) {
  while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
    w25q_delay(1);

  W25Q_STATE state = W25Q_WriteEnable(1);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

  XSPI_RegularCmdTypeDef com;

  // com.InstructionMode = XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...

  if (reg_num == 1)
    com.Instruction = W25Q_WRITE_SR1;
  else if (reg_num == 2)
    com.Instruction = W25Q_WRITE_SR2;
  else if (reg_num == 3)
    com.Instruction = W25Q_WRITE_SR3;
  else
    return W25Q_STATE::W25Q_PARAM_ERR;

  // com.AddressMode = QSPI_ADDRESS_NONE;
  com.AddressMode = HAL_XSPI_ADDRESS_NONE;
  // com.AddressSize = QSPI_ADDRESS_NONE;
  com.AddressWidth = HAL_XSPI_ADDRESS_NONE;
  com.Address = 0x0U;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_8_BITS;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  // com.DataMode = QSPI_DATA_1_LINE;
  com.DataMode = HAL_XSPI_DATA_1_LINE;
  com.DataLength = 1;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  com.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }
  if (HAL_XSPI_Transmit(&hqspi, &reg_data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }

  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read Status Registers
 * Read all status registers to struct
 *
 * @param[out] status W25Q_STATUS_REG Pointer
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadStatusStruct(W25Q_STATUS_REG *status) {
  // buffer enum-variable
  W25Q_STATE state;

  // buffer register variables
  uint8_t SRs[3] = {
      0,
  };

  // first portion
  state = W25Q_ReadStatusReg(&SRs[0], 1);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

  // second portion
  state = W25Q_ReadStatusReg(&SRs[1], 2);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

  // third portion
  state = W25Q_ReadStatusReg(&SRs[2], 3);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  if (status) {
    status->BUSY = w25q_status.BUSY = SRs[0] & 0b1;
    status->WEL = w25q_status.WEL = (SRs[0] >> 1) & 0b1;
    status->QE = w25q_status.QE = (SRs[1] >> 1) & 0b1;
    status->SUS = w25q_status.SUS = (SRs[1] >> 7) & 0b1;
    status->ADS = w25q_status.ADS = SRs[2] & 0b1;
    status->ADP = w25q_status.ADP = (SRs[2] >> 1) & 0b1;
    status->SLEEP =
        w25q_status.SLEEP; // возможно нужно вынести в начало (тестить)
  }

  return state;
}

/**
 * @brief W25Q Check Busy flag
 * Fast checking Busy flag
 *
 * @param none
 * @return W25Q_STATE enum (W25Q_OK / W25Q_BUSY)
 */
W25Q_STATE W25qOctoSpi::W25Q_IsBusy(void) {
  W25Q_STATE state;
  uint8_t sr = 0;

  state = W25Q_ReadStatusReg(&sr, 1);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

  w25q_status.BUSY = sr & 0b1;

  return w25q_status.BUSY ? W25Q_STATE::W25Q_BUSY : W25Q_STATE::W25Q_OK;
}

/**
 * @}
 * @addtogroup W25Q_Read Read Functions
 * @brief Read operations - single data type variables or raw 8-bit blocks
 * @{
 */

/**
 * @brief W25Q Read single Signed Byte
 * Read signed 8-bit byte variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..255)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadSByte(int8_t *buf, uint8_t pageShift,
                                       uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data;
  W25Q_STATE state = W25Q_ReadRaw(&data, 1, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  memcpy(buf, &data, 1);
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read single Unsigned Byte
 * Read unsigned 8-bit byte variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..255)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadByte(uint8_t *buf, uint8_t pageShift,
                                      uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data;
  W25Q_STATE state = W25Q_ReadRaw(&data, 1, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  buf[0] = data;
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read single Signed Word
 * Read signed 16-bit word variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..254)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadSWord(int16_t *buf, uint8_t pageShift,
                                       uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 2)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[2];
  W25Q_STATE state = W25Q_ReadRaw(data, 2, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  memcpy(buf, data, 2);
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read single Unsigned Word
 * Read unsigned 16-bit word variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..254)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadWord(uint16_t *buf, uint8_t pageShift,
                                      uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 2)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[2];
  W25Q_STATE state = W25Q_ReadRaw(data, 2, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  memcpy(buf, data, 2);
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read single Signed Long
 * Read signed 32-bit long variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..252)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadSLong(int32_t *buf, uint8_t pageShift,
                                       uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 4)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[4];
  W25Q_STATE state = W25Q_ReadRaw(data, 4, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  memcpy(buf, data, 4);
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read single Signed Long
 * Read signed 32-bit long variable
 *
 * @param[out] buf Data to be read (single)
 * @param[in] pageShift Byte shift inside page (0..252)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadLong(uint32_t *buf, uint8_t pageShift,
                                      uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 4)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[4];
  W25Q_STATE state = W25Q_ReadRaw(data, 4, rawAddr);
  if (state != W25Q_STATE::W25Q_OK)
    return state;
  memcpy(buf, data, 4);
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read any 8-bit data
 * Read any 8-bit data from preffered page place
 *
 * @note Use memcpy to decode data
 * @param[out] buf Pointer to data to be read (single or array)
 * @param[in] len Length of data (1..256)
 * @param[in] pageShift Byte shift inside page (0..256 - len)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadData(uint8_t *buf, uint16_t len,
                                      uint8_t pageShift, uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || len == 0 || len > 256 || pageShift > 256 - len)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  return W25Q_ReadRaw(buf, len, rawAddr);
}

/**
 * @brief W25Q Read any 8-bit data from raw addr
 * Read any 8-bit data from preffered chip address
 *
 * @note Address is in [byte] size
 * @note Be carefull with page overrun
 * @param[out] buf Pointer to data to be written (single or array)
 * @param[in] data_len Length of data (1..256)
 * @param[in] rawAddr Start address of chip's cell
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadRaw(uint8_t *buf, uint16_t data_len,
                                     uint32_t rawAddr) {
  if (data_len > 256 || data_len == 0)
    return W25Q_STATE::W25Q_PARAM_ERR;

  while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
    w25q_delay(1);

  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
#if MEM_FLASH_SIZE > 128U
  com.Instruction = W25Q_FAST_READ_QUAD_IO_4B; // Command
  com.AddressSize = QSPI_ADDRESS_32_BITS;
#else
  com.Instruction = W25Q_FAST_READ_QUAD_IO; // Command
  // com.AddressSize = QSPI_ADDRESS_24_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
#endif
  // com.AddressMode = QSPI_ADDRESS_4_LINES;
  com.AddressMode = HAL_XSPI_ADDRESS_4_LINES;

  com.Address = rawAddr;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 6;
  // com.DataMode = QSPI_DATA_4_LINES;
  com.DataMode = HAL_XSPI_DATA_4_LINES;
  // com.NbData = data_len;
  com.DataLength = data_len;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  // if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;
  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;

  if (HAL_XSPI_Receive(&hqspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;

  // if (HAL_QSPI_Receive(&hqspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;

  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Read any 8-bit data from raw addr
 * Read any 8-bit data from preffered chip address by SINGLE SPI
 *
 * @note Works only with SINGLE SPI Line
 * @param[out] buf Pointer to data array
 * @param[in] len Length of array
 * @param[in] Addr Address to data
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_SingleRead(uint8_t *buf, uint32_t len,
                                        uint32_t Addr) {
  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  // com.InstructionMode = QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
#if MEM_FLASH_SIZE > 128U
  com.Instruction = W25Q_READ_DATA_4B; // Command
  // com.AddressSize = QSPI_ADDRESS_32_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
#else
  com.Instruction = W25Q_READ_DATA; // Command
  // com.AddressSize = QSPI_ADDRESS_24_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
#endif
  // com.AddressMode = QSPI_ADDRESS_1_LINE;
  com.AddressMode = HAL_XSPI_ADDRESS_1_LINE;

  com.Address = Addr;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  // com.DataMode = QSPI_DATA_1_LINE;
  com.DataMode = HAL_XSPI_DATA_1_LINE;
  // com.NbData = len;
  com.DataLength = len;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  // if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;

  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;

  // if (HAL_QSPI_Receive(&hqspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;
  if (HAL_XSPI_Receive(&hqspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;

  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Program single Signed Byte
 * Program signed 8-bit byte variable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..255)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramSByte(int8_t buf, uint8_t pageShift,
                                          uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data;
  memcpy(&data, &buf, 1);
  return W25Q_ProgramRaw(&data, 1, rawAddr);
}

/**
 * @brief W25Q Program single Unsigned Byte
 * Program unsigned 8-bit byte vairable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..255)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramByte(uint8_t buf, uint8_t pageShift,
                                         uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data;
  memcpy(&data, &buf, 1);
  return W25Q_ProgramRaw(&data, 1, rawAddr);
}

/**
 * @brief W25Q Program single Signed Word
 * Program signed 16-bit word vairable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..254)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramSWord(int16_t buf, uint8_t pageShift,
                                          uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 2)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[2];
  memcpy(data, &buf, 2);
  return W25Q_ProgramRaw(data, 2, rawAddr);
}

/**
 * @brief W25Q Program single Unsigned Word
 * Program unsigned 16-bit word vairable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..254)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramWord(uint16_t buf, uint8_t pageShift,
                                         uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 2)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[2];
  memcpy(data, &buf, 2);
  return W25Q_ProgramRaw(data, 2, rawAddr);
}

/**
 * @brief W25Q Program single Signed Long
 * Program signed 32-bit long vairable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..252)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramSLong(int32_t buf, uint8_t pageShift,
                                          uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 4)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[4];
  memcpy(data, &buf, 4);
  return W25Q_ProgramRaw(data, 4, rawAddr);
}

/**
 * @brief W25Q Program single Unigned Long
 * Program unsigned 32-bit long vairable
 *
 * @param[in] buf Data to be written (single)
 * @param[in] pageShift Byte shift inside page (0..252)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramLong(uint32_t buf, uint8_t pageShift,
                                         uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || pageShift > 256 - 4)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  uint8_t data[4];
  memcpy(data, &buf, 4);
  return W25Q_ProgramRaw(data, 4, rawAddr);
}

/**
 * @brief W25Q Program any 8-bit data
 * Program any 8-bit data to preffered page place
 *
 * @note Use memcpy to prepare data
 * @param[in] buf Pointer to data to be written (single or array)
 * @param[in] len Length of data (1..256)
 * @param[in] pageShift Byte shift inside page (0..256 - len)
 * @param[in] pageNum Page number (0..PAGE_COUNT-1)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramData(uint8_t *buf, uint16_t len,
                                         uint8_t pageShift, uint32_t pageNum) {
  if (pageNum >= PAGE_COUNT || len == 0 || len > 256 || pageShift > 256 - len)
    return W25Q_STATE::W25Q_PARAM_ERR;
  uint32_t rawAddr = page_to_addr(pageNum, pageShift);
  return W25Q_ProgramRaw(buf, len, rawAddr);
}

/**
 * @brief W25Q Program any 8-bit data to raw addr
 * Program any 8-bit data to preffered chip address
 *
 * @note Address is in [byte] size
 * @note Be carefull with page overrun
 * @param[in] buf Pointer to data to be written (single or array)
 * @param[in] data_len Length of data (1..256)
 * @param[in] rawAddr Start address of chip's cell
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ProgramRaw(uint8_t *buf, uint16_t data_len,
                                        uint32_t rawAddr) {
  if (data_len > 256 || data_len == 0)
    return W25Q_STATE::W25Q_PARAM_ERR;

  while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
    w25q_delay(1);

  W25Q_STATE state = W25Q_WriteEnable(1);
  if (state != W25Q_STATE::W25Q_OK)
    return state;

  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  // com.InstructionMode = QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
#if MEM_FLASH_SIZE > 128U
  com.Instruction = W25Q_PAGE_PROGRAM_QUAD_INP_4B; // Command
  // com.AddressSize = QSPI_ADDRESS_32_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
#else
  com.Instruction = W25Q_PAGE_PROGRAM_QUAD_INP; // Command
  // com.AddressSize = QSPI_ADDRESS_24_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
#endif
  // com.AddressMode = QSPI_ADDRESS_1_LINE;
  com.AddressMode = HAL_XSPI_ADDRESS_1_LINE;

  com.Address = rawAddr;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  // com.DataMode = QSPI_DATA_4_LINES;
  com.DataMode = HAL_XSPI_DATA_4_LINES;
  // com.NbData = data_len;
  com.DataLength = data_len;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
  com.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  // if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;

  // if (HAL_QSPI_Transmit(&hqspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK)
  //   return W25Q_STATE::W25Q_SPI_ERR;

  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;
  if (HAL_XSPI_Transmit(&hqspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_STATE::W25Q_SPI_ERR;

  while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
    w25q_delay(1);

  return W25Q_STATE::W25Q_OK;
}

/**
 * @}
 * @addtogroup W25Q_Erase Erase functions
 * @brief Erase sector, blocks or whole chip
 * @{
 */

/**
 * @brief W25Q Sector erase (4KB)
 * Minimal size operation to erase data
 *
 * @note Should be executed before writing
 * @param[in] SectAddr Sector start address
 * @return W25Q_STATE enum
 */
// W25Q_STATE W25qOctoSpi::W25Q_EraseSector(uint32_t SectAddr) {
//   if (SectAddr >= SECTOR_COUNT)
//     return W25Q_STATE::W25Q_PARAM_ERR;

//   while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
//     w25q_delay(1);

//   uint32_t rawAddr = SectAddr * MEM_SECTOR_SIZE * 1024U;

//   W25Q_STATE state = W25Q_WriteEnable(1);
//   if (state != W25Q_STATE::W25Q_OK)
//     return state;

//   QSPI_CommandTypeDef com;

//   com.InstructionMode = QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
// #if MEM_FLASH_SIZE > 128U
//   com.Instruction = W25Q_SECTOR_ERASE_4B; // Command
//   com.AddressSize = QSPI_ADDRESS_32_BITS;
// #else
//   com.Instruction = W25Q_SECTOR_ERASE; // Command
//   com.AddressSize = QSPI_ADDRESS_24_BITS;
// #endif
//   com.AddressMode = QSPI_ADDRESS_1_LINE;

//   com.Address = rawAddr;

//   com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//   com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
//   com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;

//   com.DummyCycles = 0;
//   com.DataMode = QSPI_DATA_NONE;
//   com.NbData = 0;

//   com.DdrMode = QSPI_DDR_MODE_DISABLE;
//   com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
//   com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

//   if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
//   HAL_OK)
//     return W25Q_STATE::W25Q_SPI_ERR;

//   while (W25Q_IsBusy() == W25Q_STATE::W25Q_BUSY)
//     w25q_delay(1);

//   return W25Q_STATE::W25Q_OK;
// }

/**
 * @}
 * @addtogroup W25Q_ID ID functions
 * @brief Who am I? Хто я?
 * @{
 */

/**
 * @brief W25Q Read ID
 * Function for reading chip ID
 *
 * @param[out] buf Pointer to output data (1 byte)
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_ReadID(uint8_t *buf) {
  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  // com.InstructionMode = QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.Instruction = W25Q_DEVID;                      // Command

  // com.AddressMode = QSPI_ADDRESS_1_LINE;
  com.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  // com.AddressSize = QSPI_ADDRESS_24_BITS;
  com.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  com.Address = 0x0U;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  // com.DataMode = QSPI_DATA_1_LINE;
  com.DataMode = HAL_XSPI_DATA_1_LINE;
  // com.NbData = 1;
  com.DataLength = 1;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
  com.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  // if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  //     HAL_OK) {
  //   return W25Q_STATE::W25Q_SPI_ERR;
  // }
  // if (HAL_QSPI_Receive(&hqspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  // HAL_OK) {
  //   return W25Q_STATE::W25Q_SPI_ERR;
  // }

  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }

  if (HAL_XSPI_Receive(&hqspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }
  return W25Q_STATE::W25Q_OK;
}

/**
 * @brief W25Q Toggle WEL bit
 * Toggle write enable latch
 *
 * @param[in] enable 1-enable write/0-disable write
 * @return W25Q_STATE enum
 */
W25Q_STATE W25qOctoSpi::W25Q_WriteEnable(bool enable) {
  // QSPI_CommandTypeDef com;
  XSPI_RegularCmdTypeDef com;

  // com.InstructionMode = QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
  com.Instruction = enable ? W25Q_WRITE_ENABLE : W25Q_WRITE_DISABLE;

  // com.AddressMode = QSPI_ADDRESS_NONE;
  com.AddressMode = HAL_XSPI_ADDRESS_NONE;
  // com.AddressSize = QSPI_ADDRESS_NONE;
  com.AddressWidth = HAL_XSPI_ADDRESS_NONE;
  com.Address = 0x0U;

  // com.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytes = HAL_XSPI_ALT_BYTES_NONE;
  // com.AlternateBytesSize = QSPI_ALTERNATE_BYTES_NONE;
  com.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_NONE;

  com.DummyCycles = 0;
  // com.DataMode = QSPI_DATA_NONE;
  com.DataMode = HAL_XSPI_DATA_NONE;
  // com.NbData = 0;
  com.DataLength = 0;

  // com.DdrMode = QSPI_DDR_MODE_DISABLE;
  com.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
  // com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  com.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  //

  // com.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  com.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
  com.AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
  com.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  // if (HAL_QSPI_Command(&hqspi, &com, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) !=
  //     HAL_OK) {
  //   return W25Q_STATE::W25Q_SPI_ERR;
  // }
  if (HAL_XSPI_Command(&hqspi, &com, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_STATE::W25Q_SPI_ERR;
  }
  w25q_delay(1); // Give a little time to sleep

  w25q_status.WEL = 1;

  return W25Q_STATE::W25Q_OK;
}

uint32_t W25qOctoSpi::page_to_addr(uint32_t pageNum, uint8_t pageShift) {
  return pageNum * MEM_PAGE_SIZE + pageShift;
}