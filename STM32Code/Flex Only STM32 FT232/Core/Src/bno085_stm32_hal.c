#include "bno085_stm32_hal.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define BNO085_I2C_ADDRESS       (0x4B << 1)
#define BNO085_I2C_TIMEOUT_MS    100U
#define BNO085_RESET_LOW_MS      10U
#define BNO085_BOOT_DELAY_MS     1000U
#define BNO085_SHTP_HEADER_SIZE  4U

static sh2_Hal_t bno085_hal;
static uint8_t bno085_is_open;
static uint8_t bno085_time_started;
static uint32_t bno085_last_cycle;
static uint64_t bno085_elapsed_cycles;

static uint32_t BNO085_STM32_GetTimeUs(sh2_Hal_t *self);

static int BNO085_STM32_Open(sh2_Hal_t *self)
{
  if (self != &bno085_hal || bno085_is_open)
  {
    return -1;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  bno085_time_started = 0U;
  bno085_last_cycle = 0U;
  bno085_elapsed_cycles = 0U;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(BNO085_RESET_LOW_MS);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(BNO085_BOOT_DELAY_MS);

  if (HAL_I2C_IsDeviceReady(&hi2c1, BNO085_I2C_ADDRESS, 3,
                            BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  bno085_is_open = 1U;
  return 0;
}

static void BNO085_STM32_Close(sh2_Hal_t *self)
{
  if (self == &bno085_hal && bno085_is_open)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    bno085_is_open = 0U;
  }
}

static int BNO085_STM32_Read(sh2_Hal_t *self, uint8_t *buffer,
                             unsigned buffer_len, uint32_t *timestamp_us)
{
  uint8_t header[BNO085_SHTP_HEADER_SIZE];
  uint16_t transfer_len;

  if (self != &bno085_hal || !bno085_is_open ||
      buffer == NULL || timestamp_us == NULL)
  {
    return -1;
  }

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) != GPIO_PIN_RESET)
  {
    return 0;
  }

  *timestamp_us = BNO085_STM32_GetTimeUs(self);

  if (HAL_I2C_Master_Receive(&hi2c1, BNO085_I2C_ADDRESS, header,
                             sizeof(header), BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  transfer_len = (uint16_t)(((uint16_t)header[1] << 8) | header[0]) & 0x7FFFU;
  if (transfer_len < BNO085_SHTP_HEADER_SIZE ||
      transfer_len > buffer_len ||
      transfer_len > SH2_HAL_MAX_TRANSFER_IN)
  {
    return -1;
  }

  if (HAL_I2C_Master_Receive(&hi2c1, BNO085_I2C_ADDRESS, buffer,
                             transfer_len, BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  return (int)transfer_len;
}

static int BNO085_STM32_Write(sh2_Hal_t *self, uint8_t *buffer,
                              unsigned buffer_len)
{
  if (self != &bno085_hal || !bno085_is_open ||
      buffer == NULL || buffer_len == 0U ||
      buffer_len > SH2_HAL_MAX_TRANSFER_OUT ||
      buffer_len > UINT16_MAX)
  {
    return -1;
  }

  if (HAL_I2C_Master_Transmit(&hi2c1, BNO085_I2C_ADDRESS, buffer,
                              (uint16_t)buffer_len,
                              BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  return (int)buffer_len;
}

static uint32_t BNO085_STM32_GetTimeUs(sh2_Hal_t *self)
{
  uint32_t cycle_count;

  (void)self;

  cycle_count = DWT->CYCCNT;
  if (bno085_time_started == 0U)
  {
    bno085_last_cycle = cycle_count;
    bno085_time_started = 1U;
  }
  else
  {
    bno085_elapsed_cycles += (uint32_t)(cycle_count - bno085_last_cycle);
    bno085_last_cycle = cycle_count;
  }

  return (uint32_t)(bno085_elapsed_cycles / (SystemCoreClock / 1000000U));
}

sh2_Hal_t *BNO085_STM32_HAL_Init(void)
{
  bno085_hal.open = BNO085_STM32_Open;
  bno085_hal.close = BNO085_STM32_Close;
  bno085_hal.read = BNO085_STM32_Read;
  bno085_hal.write = BNO085_STM32_Write;
  bno085_hal.getTimeUs = BNO085_STM32_GetTimeUs;

  return &bno085_hal;
}
