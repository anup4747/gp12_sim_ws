/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "bno085_stm32_hal.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLEX_STRAIGHT_RAW 2135U
#define FLEX_BENT_RAW 1144U
#define TELEMETRY_INTERVAL_MS 200U
#define MCP2515_CAN_ID_QUATERNION 0x100U
#define MCP2515_CAN_ID_FLEX 0x101U
#define MCP2515_CAN_ID_ACCELERATION 0x102U
#define MCP2515_SPI_TIMEOUT_MS 10U
#define MCP2515_MODE_TIMEOUT_MS 100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ADC raw value (12-bit: 0–4095) */
static uint32_t adc_raw = 0;

/* Transmit buffer for flex, quaternion, and acceleration telemetry */
static char tx_buf[256];

static volatile uint8_t bno_reset_seen;
static volatile uint8_t quat_valid;
static volatile uint8_t accel_valid;
static volatile float quat_i;
static volatile float quat_j;
static volatile float quat_k;
static volatile float quat_real = 1.0f;
static volatile float accel_x;
static volatile float accel_y;
static volatile float accel_z;
static uint8_t bno_connected;
static uint8_t mcp2515_ready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void BNO085_EventCallback(void *cookie, sh2_AsyncEvent_t *event);
static void BNO085_SensorCallback(void *cookie, sh2_SensorEvent_t *event);
static void FormatQuatComponent(char *buffer, size_t buffer_len, float value);
static void FormatAccelComponent(char *buffer, size_t buffer_len, float value);
static uint8_t MCP2515_Init(void);
static uint8_t MCP2515_SetMode(uint8_t mode);
static HAL_StatusTypeDef MCP2515_ReadRegister(uint8_t address, uint8_t *value);
static HAL_StatusTypeDef MCP2515_WriteRegisters(uint8_t address,
                                                const uint8_t *data,
                                                uint8_t length);
static HAL_StatusTypeDef MCP2515_BitModify(uint8_t address, uint8_t mask,
                                           uint8_t value);
static HAL_StatusTypeDef MCP2515_SendFrame(uint16_t id, const uint8_t *data,
                                           uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Send a null-terminated string over USART3.
 * @param  str  Pointer to the string to transmit.
 */
static void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief  Read a single conversion from ADC1 (Channel 14, PC4).
 * @return 12-bit ADC result (0–4095).
 */
static uint32_t ADC_ReadChannel(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t val = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return val;
}

static void BNO085_EventCallback(void *cookie, sh2_AsyncEvent_t *event)
{
  (void)cookie;

  if (event != NULL && event->eventId == SH2_RESET)
  {
    quat_valid = 0U;
    accel_valid = 0U;
    bno_reset_seen = 1U;
  }
}

static void BNO085_SensorCallback(void *cookie, sh2_SensorEvent_t *event)
{
  sh2_SensorValue_t value;

  (void)cookie;

  if (event == NULL || sh2_decodeSensorEvent(&value, event) != SH2_OK)
  {
    return;
  }

  if (value.sensorId == SH2_ACCELEROMETER)
  {
    accel_x = value.un.accelerometer.x;
    accel_y = value.un.accelerometer.y;
    accel_z = value.un.accelerometer.z;
    accel_valid = 1U;
    return;
  }

  if (value.sensorId != SH2_ROTATION_VECTOR)
  {
    return;
  }

  quat_i = value.un.rotationVector.i;
  quat_j = value.un.rotationVector.j;
  quat_k = value.un.rotationVector.k;
  quat_real = value.un.rotationVector.real;
  quat_valid = 1U;
}

static void FormatQuatComponent(char *buffer, size_t buffer_len, float value)
{
  int32_t scaled = (int32_t)(value * 10000.0f);
  const char *sign = (scaled < 0) ? "-" : " ";

  if (scaled < 0)
  {
    scaled = -scaled;
  }

  snprintf(buffer, buffer_len, "%s%ld.%04ld", sign,
           (long)(scaled / 10000), (long)(scaled % 10000));
}

static void FormatAccelComponent(char *buffer, size_t buffer_len, float value)
{
  int32_t scaled = (int32_t)(value * 100.0f);
  const char *sign = (scaled < 0) ? "-" : " ";

  if (scaled < 0)
  {
    scaled = -scaled;
  }

  snprintf(buffer, buffer_len, "%s%ld.%02ld", sign,
           (long)(scaled / 100), (long)(scaled % 100));
}

#define MCP2515_CMD_RESET 0xC0U
#define MCP2515_CMD_READ 0x03U
#define MCP2515_CMD_WRITE 0x02U
#define MCP2515_CMD_BIT_MODIFY 0x05U
#define MCP2515_CMD_RTS 0x80U
#define MCP2515_REG_CANSTAT 0x0EU
#define MCP2515_REG_CANCTRL 0x0FU
#define MCP2515_REG_CNF3 0x28U
#define MCP2515_REG_CNF2 0x29U
#define MCP2515_REG_CNF1 0x2AU
#define MCP2515_REG_CANINTE 0x2BU
#define MCP2515_REG_TXB0CTRL 0x30U
#define MCP2515_REG_TXB1CTRL 0x40U
#define MCP2515_REG_TXB2CTRL 0x50U
#define MCP2515_TXREQ_MASK 0x08U
#define MCP2515_MODE_MASK 0xE0U
#define MCP2515_MODE_CONFIG 0x80U
#define MCP2515_MODE_NORMAL 0x00U

static void MCP2515_Select(void)
{
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_RESET);
}

static void MCP2515_Deselect(void)
{
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MCP2515_Command(uint8_t command)
{
  HAL_StatusTypeDef status;

  MCP2515_Select();
  status = HAL_SPI_Transmit(&hspi1, &command, 1U, MCP2515_SPI_TIMEOUT_MS);
  MCP2515_Deselect();
  return status;
}

static HAL_StatusTypeDef MCP2515_ReadRegister(uint8_t address, uint8_t *value)
{
  uint8_t command[2] = {MCP2515_CMD_READ, address};
  uint8_t dummy = 0U;
  HAL_StatusTypeDef status;

  MCP2515_Select();
  status = HAL_SPI_Transmit(&hspi1, command, sizeof(command),
                            MCP2515_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(&hspi1, &dummy, value, 1U,
                                     MCP2515_SPI_TIMEOUT_MS);
  }
  MCP2515_Deselect();
  return status;
}

static HAL_StatusTypeDef MCP2515_WriteRegisters(uint8_t address,
                                                const uint8_t *data,
                                                uint8_t length)
{
  uint8_t command[2] = {MCP2515_CMD_WRITE, address};
  HAL_StatusTypeDef status;

  MCP2515_Select();
  status = HAL_SPI_Transmit(&hspi1, command, sizeof(command),
                            MCP2515_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length,
                              MCP2515_SPI_TIMEOUT_MS);
  }
  MCP2515_Deselect();
  return status;
}

static HAL_StatusTypeDef MCP2515_BitModify(uint8_t address, uint8_t mask,
                                           uint8_t value)
{
  uint8_t command[4] = {MCP2515_CMD_BIT_MODIFY, address, mask, value};
  HAL_StatusTypeDef status;

  MCP2515_Select();
  status = HAL_SPI_Transmit(&hspi1, command, sizeof(command),
                            MCP2515_SPI_TIMEOUT_MS);
  MCP2515_Deselect();
  return status;
}

static uint8_t MCP2515_SetMode(uint8_t mode)
{
  uint32_t start_tick;
  uint8_t can_status;

  if (MCP2515_BitModify(MCP2515_REG_CANCTRL, MCP2515_MODE_MASK, mode) != HAL_OK)
  {
    return 0U;
  }

  start_tick = HAL_GetTick();
  do
  {
    if (MCP2515_ReadRegister(MCP2515_REG_CANSTAT, &can_status) != HAL_OK)
    {
      return 0U;
    }
    if ((can_status & MCP2515_MODE_MASK) == mode)
    {
      return 1U;
    }
  } while ((uint32_t)(HAL_GetTick() - start_tick) < MCP2515_MODE_TIMEOUT_MS);

  return 0U;
}

static uint8_t MCP2515_Init(void)
{
  const uint8_t cnf1 = 0x00U;
  const uint8_t cnf2 = 0x90U;
  const uint8_t cnf3 = 0x02U;
  const uint8_t interrupts_disabled = 0x00U;

  MCP2515_Deselect();
  if (MCP2515_Command(MCP2515_CMD_RESET) != HAL_OK)
  {
    return 0U;
  }
  HAL_Delay(2U);

  if (!MCP2515_SetMode(MCP2515_MODE_CONFIG) ||
      MCP2515_WriteRegisters(MCP2515_REG_CNF1, &cnf1, 1U) != HAL_OK ||
      MCP2515_WriteRegisters(MCP2515_REG_CNF2, &cnf2, 1U) != HAL_OK ||
      MCP2515_WriteRegisters(MCP2515_REG_CNF3, &cnf3, 1U) != HAL_OK ||
      MCP2515_WriteRegisters(MCP2515_REG_CANINTE, &interrupts_disabled, 1U) != HAL_OK ||
      !MCP2515_SetMode(MCP2515_MODE_NORMAL))
  {
    return 0U;
  }

  return 1U;
}

static HAL_StatusTypeDef MCP2515_SendFrame(uint16_t id, const uint8_t *data,
                                           uint8_t length)
{
  static const uint8_t tx_control_addresses[3] = {
      MCP2515_REG_TXB0CTRL, MCP2515_REG_TXB1CTRL, MCP2515_REG_TXB2CTRL};
  uint8_t tx_control;
  uint8_t tx_buffer[13];
  uint8_t buffer_index;

  if (id > 0x7FFU || length > 8U || data == NULL)
  {
    return HAL_ERROR;
  }

  for (buffer_index = 0U; buffer_index < 3U; buffer_index++)
  {
    if (MCP2515_ReadRegister(tx_control_addresses[buffer_index],
                            &tx_control) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((tx_control & MCP2515_TXREQ_MASK) == 0U)
    {
      break;
    }
  }
  if (buffer_index == 3U)
  {
    return HAL_BUSY;
  }

  tx_buffer[0] = (uint8_t)(id >> 3);
  tx_buffer[1] = (uint8_t)((id & 0x07U) << 5);
  tx_buffer[2] = 0U;
  tx_buffer[3] = 0U;
  tx_buffer[4] = length;
  memcpy(&tx_buffer[5], data, length);

  if (MCP2515_WriteRegisters((uint8_t)(tx_control_addresses[buffer_index] + 1U),
                             tx_buffer, (uint8_t)(length + 5U)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2515_Command((uint8_t)(MCP2515_CMD_RTS | (1U << buffer_index)));
}

static int16_t MCP2515_ScaleQuaternion(float value)
{
  return (int16_t)(value * 10000.0f);
}

static int16_t MCP2515_ScaleAcceleration(float value)
{
  return (int16_t)(value * 100.0f);
}

static void MCP2515_StoreInt16LE(uint8_t *data, int16_t value)
{
  uint16_t bits = (uint16_t)value;
  data[0] = (uint8_t)bits;
  data[1] = (uint8_t)(bits >> 8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* -----------------------------------------------------------
   * Startup banner sent over UART to the PC (via FT232).
   * Open any serial terminal at 115200-8-N-1 to see output.
   * ----------------------------------------------------------- */
  UART_Print("\r\n=== STM32F405 Flex Sensor & BNO085 Telemetry ===\r\n");
  UART_Print("ADC: PC4 (CH14) | USART3: PB10/PB11 | 115200-8-N-1\r\n");
  UART_Print("BNO085 I2C1: PB8(SCL), PB9(SDA), INT: PC0, RST: PC1, addr: 0x4B\r\n");

  mcp2515_ready = MCP2515_Init();
  if (mcp2515_ready)
  {
    UART_Print("MCP2515: initialized for 500 kbit/s (8 MHz crystal)\r\n");
  }
  else
  {
    UART_Print("MCP2515: initialization failed; CAN telemetry disabled\r\n");
  }

  if (sh2_open(BNO085_STM32_HAL_Init(), BNO085_EventCallback, NULL) != SH2_OK)
  {
    UART_Print("BNO085: SH-2 initialization failed\r\n");
  }
  else if (sh2_setSensorCallback(BNO085_SensorCallback, NULL) != SH2_OK)
  {
    UART_Print("BNO085: sensor callback registration failed\r\n");
    sh2_close();
  }
  else
  {
    bno_connected = 1U;
    UART_Print("BNO085: connected; waiting for reset event and sensor data\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_telemetry_tick = HAL_GetTick() - TELEMETRY_INTERVAL_MS;
  while (1)
  {
    if (bno_connected)
    {
      sh2_service();

      if (bno_reset_seen)
      {
        sh2_SensorConfig_t config = {0};
        bno_reset_seen = 0U;
        config.reportInterval_us = 20000U;

        if (sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) != SH2_OK)
        {
          UART_Print("BNO085: failed to enable Rotation Vector report\r\n");
        }
        if (sh2_setSensorConfig(SH2_ACCELEROMETER, &config) != SH2_OK)
        {
          UART_Print("BNO085: failed to enable Accelerometer report\r\n");
        }
      }
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_telemetry_tick) >= TELEMETRY_INTERVAL_MS)
    {
      char str_i[14], str_j[14], str_k[14], str_real[14];
      char str_ax[14], str_ay[14], str_az[14];
      uint32_t voltage_mv;
      int32_t bend_pct;

      last_telemetry_tick = now;
      adc_raw = ADC_ReadChannel();
      voltage_mv = (adc_raw * 3300UL) / 4095UL;

      if (adc_raw >= FLEX_STRAIGHT_RAW)
      {
        bend_pct = 0;
      }
      else if (adc_raw <= FLEX_BENT_RAW)
      {
        bend_pct = 100;
      }
      else
      {
        bend_pct = (int32_t)((FLEX_STRAIGHT_RAW - adc_raw) * 100UL /
                             (FLEX_STRAIGHT_RAW - FLEX_BENT_RAW));
      }

      if (bno_connected && quat_valid)
      {
        FormatQuatComponent(str_i, sizeof(str_i), quat_i);
        FormatQuatComponent(str_j, sizeof(str_j), quat_j);
        FormatQuatComponent(str_k, sizeof(str_k), quat_k);
        FormatQuatComponent(str_real, sizeof(str_real), quat_real);
      }
      if (bno_connected && accel_valid)
      {
        FormatAccelComponent(str_ax, sizeof(str_ax), accel_x);
        FormatAccelComponent(str_ay, sizeof(str_ay), accel_y);
        FormatAccelComponent(str_az, sizeof(str_az), accel_z);
      }

      if (bno_connected && quat_valid && accel_valid)
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | Quat: [i:%s j:%s k:%s real:%s] | Accel (m/s^2): [ax:%s ay:%s az:%s]\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct, str_i, str_j, str_k, str_real,
                 str_ax, str_ay, str_az);
      }
      else if (bno_connected && quat_valid)
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | Quat: [i:%s j:%s k:%s real:%s] | Accel: waiting\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct, str_i, str_j, str_k, str_real);
      }
      else if (bno_connected && accel_valid)
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | Quat: waiting | Accel (m/s^2): [ax:%s ay:%s az:%s]\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct, str_ax, str_ay, str_az);
      }
      else if (bno_connected)
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | BNO085: FOUND, waiting for sensor data\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct);
      }
      else
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | BNO085: NOT FOUND\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct);
      }

      UART_Print(tx_buf);

      if (mcp2515_ready)
      {
        uint8_t flex_data[8] = {0};
        HAL_StatusTypeDef can_status;

        flex_data[0] = (uint8_t)adc_raw;
        flex_data[1] = (uint8_t)(adc_raw >> 8);
        flex_data[2] = (uint8_t)bend_pct;
        can_status = MCP2515_SendFrame(MCP2515_CAN_ID_FLEX, flex_data,
                                       sizeof(flex_data));
        if (can_status != HAL_OK)
        {
          UART_Print("MCP2515: flex CAN frame transmit failed\r\n");
        }

        if (bno_connected && quat_valid)
        {
          uint8_t quaternion_data[8];
          MCP2515_StoreInt16LE(&quaternion_data[0], MCP2515_ScaleQuaternion(quat_i));
          MCP2515_StoreInt16LE(&quaternion_data[2], MCP2515_ScaleQuaternion(quat_j));
          MCP2515_StoreInt16LE(&quaternion_data[4], MCP2515_ScaleQuaternion(quat_k));
          MCP2515_StoreInt16LE(&quaternion_data[6], MCP2515_ScaleQuaternion(quat_real));
          can_status = MCP2515_SendFrame(MCP2515_CAN_ID_QUATERNION,
                                         quaternion_data,
                                         sizeof(quaternion_data));
          if (can_status != HAL_OK)
          {
            UART_Print("MCP2515: quaternion CAN frame transmit failed\r\n");
          }
        }

        if (bno_connected && accel_valid)
        {
          uint8_t acceleration_data[8] = {0};
          MCP2515_StoreInt16LE(&acceleration_data[0],
                               MCP2515_ScaleAcceleration(accel_x));
          MCP2515_StoreInt16LE(&acceleration_data[2],
                               MCP2515_ScaleAcceleration(accel_y));
          MCP2515_StoreInt16LE(&acceleration_data[4],
                               MCP2515_ScaleAcceleration(accel_z));
          can_status = MCP2515_SendFrame(MCP2515_CAN_ID_ACCELERATION,
                                         acceleration_data,
                                         sizeof(acceleration_data));
          if (can_status != HAL_OK)
          {
            UART_Print("MCP2515: acceleration CAN frame transmit failed\r\n");
          }
        }
      }
    }

    HAL_Delay(1U);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP2515_CS_Pin */
  GPIO_InitStruct.Pin = MCP2515_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCP2515_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP2515_INT_Pin */
  GPIO_InitStruct.Pin = MCP2515_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Keep BNO085 RST inactive; detection does not reset or configure the sensor. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  /* Configure PC0 as input with pull-up for BNO085 active-LOW INT */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
