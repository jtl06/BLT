/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MODE1_CALIBRATION = 1,
  MODE2_TRIGGER     = 2,
  MODE3_MIC         = 3,
} ldat_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef ADC_BUF_LEN
#define ADC_BUF_LEN 256u
#endif

#define MODE2_SPACING_MS 5000u // 2 seconds

#define MIC_DEBOUNCE_US 20000u  // 20 ms

#define FW_VERSION "LDAT 1.0"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

extern uint16_t adc_dma_buf[ADC_BUF_LEN];
static uint16_t          g_light_threshold = 200;
static volatile uint32_t g_tests_remaining = 0;
static uint32_t          g_trial_idx = 0;
static volatile bool     g_waiting_for_light = false;
static volatile uint32_t g_t_start_us = 0;

static uint32_t          g_last_launch_ms = 0;

static volatile uint32_t g_last_mic_us = 0;


static uint8_t  rx_byte;
static char     cmd_buf[96];
static uint8_t  cmd_len = 0;

volatile uint16_t g_adc_dma_buf[ADC_BUF_LEN];

static volatile ldat_mode_t g_mode = MODE1_CALIBRATION;

volatile bool g_mic_triggered = false;
volatile bool g_is_timing = false;
volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;

volatile uint32_t g_last_trigger_time = 0;
const uint32_t DEBOUNCE_US = 200000; // 200ms debounce window in microseconds

volatile ldat_mode_t g_current_mode = MODE1_CALIBRATION;

static volatile bool g_btn_click_req = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void send_mouse_click(void);
static inline uint16_t adc_latest_sample(void); 
static void uart_start_rx_it(void);
static void send_line(const char *s);
static void handle_cmd(const char *cmd);
static inline uint32_t tim2_us(void);
static inline int usb_is_configured(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc_dma_buf, ADC_BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  printf("Continuous ADC-DMA running, buf=%u samples\r\n", ADC_BUF_LEN);
  printf("Initialized, JTL LDAT 1.0\r\n");
  HAL_TIM_Base_Start(&htim2);
  uart_start_rx_it();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {
  uint32_t now_ms = HAL_GetTick();

  // MODE2: launch trials every 500 ms when armed
  if (g_mode == MODE2_TRIGGER && g_tests_remaining && !g_waiting_for_light) {
    if ((now_ms - g_last_launch_ms) >= MODE2_SPACING_MS) {
      g_last_launch_ms = now_ms;
      g_waiting_for_light = true;
      send_mouse_click();
      g_t_start_us = tim2_us();  //start time
    }
  }

  // Complete a trial when light crosses threshold
  if (g_waiting_for_light && (adc_latest_sample() >= g_light_threshold)) {
    uint32_t t_end = tim2_us();
    uint32_t dt_us = (uint32_t)(t_end - g_t_start_us);

    ++g_trial_idx;
    char out[96];
    int n = snprintf(out, sizeof out, "DATA,%lu,%lu,%u\r\n",
                     (unsigned long)g_trial_idx,
                     (unsigned long)dt_us,
                     (unsigned)adc_latest_sample());
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)out, (uint16_t)n, HAL_MAX_DELAY);

    g_waiting_for_light = false;
    if (--g_tests_remaining == 0) {
      send_line("DONE\r\n");
    }
  }
  
  __WFI();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC1_Pin */
  GPIO_InitStruct.Pin = MIC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MIC1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static inline int usb_is_configured(void) {
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

static inline uint16_t adc_latest_sample(void)
{
    uint32_t dma_remaining = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    uint32_t idx           = (ADC_BUF_LEN - dma_remaining) & (ADC_BUF_LEN-1);
    return g_adc_dma_buf[idx];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MIC1_Pin && g_mode == MODE3_MIC) {
    uint32_t now = tim2_us();
    if ((uint32_t)(now - g_last_mic_us) > MIC_DEBOUNCE_US) {
      g_last_mic_us = now;
      if (g_tests_remaining && !g_waiting_for_light) {
        g_waiting_for_light = true;
        g_t_start_us = now; // mic click time
      }
    }
  } else if (GPIO_Pin == USER_BUTTON_Pin) {
    g_btn_click_req = true;
  }
}


void send_mouse_click(void)
{
  uint8_t hid_report[4] = {0};

  hid_report[0] = 0x01; 
  USBD_HID_SendReport(&hUsbDeviceFS, hid_report, sizeof(hid_report));
  HAL_Delay(10);

  hid_report[0] = 0x00;
  USBD_HID_SendReport(&hUsbDeviceFS, hid_report, sizeof(hid_report));
}

static void uart_start_rx_it(void) {
  HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
}

static inline void send_line(const char *s) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static inline uint32_t tim2_us(void) {
  return __HAL_TIM_GET_COUNTER(&htim2); // TIM2 configured 1 MHz, 32-bit
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &hlpuart1) {
    uint8_t c = rx_byte;

    if (c == '\n' || c == '\r') {
      if (cmd_len) {
        cmd_buf[cmd_len] = '\0';
        handle_cmd(cmd_buf);
        cmd_len = 0;
      }
    } else if (cmd_len < sizeof(cmd_buf)-1) {
      cmd_buf[cmd_len++] = (char)c;
    } else {
      cmd_len = 0;
    }
    HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
  }
}

static void handle_cmd(const char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    send_line("CMDS: HELLO, HELP, VERSION, STATUS, "
              "MODE1, MODE2, MODE3, CAL READ, CAL AUTO, CAL SET <thr>, TEST START <n>\r\n");

  } else if (strcmp(cmd, "HELLO") == 0) {
    send_line("ACK HELLO\r\n");

  } else if (strcmp(cmd, "VERSION") == 0) {
    char b[48]; snprintf(b, sizeof b, "VERSION %s\r\n", FW_VERSION); send_line(b);

  } else if (strcmp(cmd, "STATUS") == 0) {
    char b[96];
    snprintf(b, sizeof b, "STATUS mode=%u light=%u thr=%u waiting=%u left=%lu\r\n",
             (unsigned)g_mode, (unsigned)adc_latest_sample(), (unsigned)g_light_threshold,
             (unsigned)g_waiting_for_light, (unsigned long)g_tests_remaining);
    send_line(b);

  // ----- Modes -----
  } else if (strcmp(cmd, "MODE1") == 0) {
    g_mode = MODE1_CALIBRATION;  send_line("ACK MODE1\r\n");

  } else if (strcmp(cmd, "MODE2") == 0) {
    g_mode = MODE2_TRIGGER;      send_line("ACK MODE2\r\n");

  } else if (strcmp(cmd, "MODE3") == 0) {
    g_mode = MODE3_MIC;          send_line("ACK MODE3\r\n");

  // ----- Calibration helpers -----
  } else if (strcmp(cmd, "CAL READ") == 0) {
    char b[64];
    snprintf(b, sizeof b, "CAL,light=%u,thr=%u\r\n", (unsigned)adc_latest_sample(), (unsigned)g_light_threshold);
    send_line(b);

  } else if (strcmp(cmd, "CAL AUTO") == 0) {
    uint32_t thr = (adc_latest_sample() * 125u) / 100u;  // +25%
    if (thr > 4095u) thr = 4095u;
    g_light_threshold = (uint16_t)thr;
    char b[48];
    snprintf(b, sizeof b, "ACK CAL SET %u\r\n", (unsigned)g_light_threshold);
    send_line(b);

  } else if (strncmp(cmd, "CAL SET ", 8) == 0) {
    uint32_t v = strtoul(cmd+8, NULL, 10);
    if (v > 4095u) v = 4095u;
    g_light_threshold = (uint16_t)v;
    char b[48];
    snprintf(b, sizeof b, "ACK CAL SET %u\r\n", (unsigned)g_light_threshold);
    send_line(b);

  // ----- Test arming -----
  } else if (strncmp(cmd, "TEST START ", 11) == 0) {
    uint32_t n = strtoul(cmd+11, NULL, 10);
    g_trial_idx = 0;
    g_tests_remaining = n;
    g_waiting_for_light = false;
    send_line("DATA,trial,latency_us,light\r\n");  // CSV header
    char b[64];
    snprintf(b, sizeof b, "ACK TEST START %lu\r\n", (unsigned long)n);
    send_line(b);

    // MODE2: optionally launch first trial immediately
    if (g_mode == MODE2_TRIGGER && g_tests_remaining) {
      g_last_launch_ms = HAL_GetTick() - MODE2_SPACING_MS; // so it fires right away
    }

  } else {
    send_line("ERR UNKNOWN\r\n");
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &hlpuart1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
  }
}


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
