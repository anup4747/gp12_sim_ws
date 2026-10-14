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

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLEX_STRAIGHT_RAW 2135U
#define FLEX_BENT_RAW 1144U
#define TELEMETRY_INTERVAL_MS 200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ADC raw value (12-bit: 0–4095) */
static uint32_t adc_raw = 0;

/* Transmit buffer for flex readings and quaternion telemetry */
static char tx_buf[192];

static volatile uint8_t bno_reset_seen;
static volatile uint8_t quat_valid;
static volatile float quat_i;
static volatile float quat_j;
static volatile float quat_k;
static volatile float quat_real = 1.0f;
static uint8_t bno_connected;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void BNO085_EventCallback(void *cookie, sh2_AsyncEvent_t *event);
static void BNO085_SensorCallback(void *cookie, sh2_SensorEvent_t *event);
static void FormatQuatComponent(char *buffer, size_t buffer_len, float value);
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
    bno_reset_seen = 1U;
  }
}

static void BNO085_SensorCallback(void *cookie, sh2_SensorEvent_t *event)
{
  sh2_SensorValue_t value;

  (void)cookie;

  if (event == NULL ||
      sh2_decodeSensorEvent(&value, event) != SH2_OK ||
      value.sensorId != SH2_ROTATION_VECTOR)
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
  /* USER CODE BEGIN 2 */

  /* -----------------------------------------------------------
   * Startup banner sent over UART to the PC (via FT232).
   * Open any serial terminal at 115200-8-N-1 to see output.
   * ----------------------------------------------------------- */
  UART_Print("\r\n=== STM32F405 Flex Sensor & BNO085 Quaternion ===\r\n");
  UART_Print("ADC: PC4 (CH14) | USART3: PB10/PB11 | 115200-8-N-1\r\n");
  UART_Print("BNO085 I2C1: PB8(SCL), PB9(SDA), INT: PC0, RST: PC1, addr: 0x4B\r\n");

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
    UART_Print("BNO085: connected; waiting for reset event and quaternion data\r\n");
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
      }
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_telemetry_tick) >= TELEMETRY_INTERVAL_MS)
    {
      char str_i[14], str_j[14], str_k[14], str_real[14];
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
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | Quat: [i:%s j:%s k:%s real:%s]\r\n",
                 (unsigned long)(voltage_mv / 1000UL),
                 (unsigned long)(voltage_mv % 1000UL),
                 (long)bend_pct, str_i, str_j, str_k, str_real);
      }
      else if (bno_connected)
      {
        snprintf(tx_buf, sizeof(tx_buf),
                 "Volt: %lu.%03lu V | Bend: %3ld%% | BNO085: FOUND, waiting for quaternion\r\n",
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

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
