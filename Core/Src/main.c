/* USER CODE BEGIN Header */
/**
  * @file    main.c
  * @brief   BLT (Better Latency Tester) firmware entry.
  *
  * Project: BLT – input-to-photon latency tester.
  *
  * Copyright (c) 2025 Jacen Li
  * Licensed under the MIT License (see LICENSE).
  *
  * Portions of this file are generated from STM32CubeMX templates:
  *   Copyright (c) 2025 STMicroelectronics.
  *   Licensed under the BSD-3-Clause license.
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
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
} blt_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VERSION "BLT 0.1.1"

#define ADC_BUF_LEN 256u
#define MODE2_SPACING_US 5000000u // 5 s
#define MIC_DEBOUNCE_US 200000u  // 200 ms

#define ADC_SAMPLE_PERIOD_US 3u // 3us

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint16_t          g_light_threshold = 200;
static volatile uint32_t g_tests_remaining = 0;
static uint32_t          g_trial_idx = 0;

static volatile bool     g_waiting_for_light = false;
static volatile uint32_t g_t_start_us = 0;
static uint32_t          g_last_launch_us = 0;
static volatile uint32_t g_last_mic_us = 0;

static uint8_t  rx_byte;
static char     cmd_buf[96];
static uint8_t  cmd_len = 0;

volatile uint16_t g_adc_dma_buf[ADC_BUF_LEN];

static volatile blt_mode_t g_mode = MODE1_CALIBRATION;

static volatile uint32_t g_last_idx = 0;

//unused, for debug if needed
// static volatile bool g_btn_click_req = false; 



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_mouse_click(void);
static inline uint16_t adc_latest_sample(void); 
static void uart_start_rx_it(void);
static void send_line(const char *s);
static void handle_cmd(const char *cmd);
static inline uint32_t tim2_us(void);

static inline uint32_t adc_write_idx(void);
static bool detect_cross_and_stamp(uint16_t thr, uint32_t* t1_out);


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
  printf("Initialized %s\r\n", FW_VERSION);
  HAL_TIM_Base_Start(&htim2);
  uart_start_rx_it();
  // printf("%lu\n", USBD_HID_GetPollingInterval(&hUsbDeviceFS)); // send polling rate
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {
  uint32_t now_us = tim2_us();

  // MODE2: launch trials periodically, using USB as trigger
  if (g_mode == MODE2_TRIGGER && g_tests_remaining && !g_waiting_for_light) {
    if ((now_us - g_last_launch_us) >= MODE2_SPACING_US) {
      g_last_launch_us = tim2_us();
      g_last_idx = adc_write_idx();
      g_t_start_us = tim2_us();  //start time
      g_waiting_for_light = true;
      send_mouse_click();
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

/* USER CODE BEGIN 4 */
static inline uint16_t adc_latest_sample(void)
{
    uint32_t dma_remaining = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    uint32_t idx           = (ADC_BUF_LEN - dma_remaining) & (ADC_BUF_LEN-1);
    return g_adc_dma_buf[idx];
}


static inline uint32_t adc_write_idx(void){
    // NDTR = how many transfers LEFT; write index = prod
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    return (ADC_BUF_LEN - ndtr) & (ADC_BUF_LEN - 1);
}

static bool detect_cross_and_stamp(uint16_t thr, uint32_t* t1_out){
    uint32_t write = adc_write_idx();
    uint32_t idx   = g_last_idx;

    while (idx != write) {
        uint16_t s = g_adc_dma_buf[idx];
        if (s >= thr) {
            uint32_t now = tim2_us(); // tim2 ticks
            // How far behind "now" that sample is, in samples (mod ring)
            uint32_t samples_behind = (write - idx) & (ADC_BUF_LEN - 1);
            // sample time = 3 us
            uint32_t ticks_behind = samples_behind * ADC_SAMPLE_PERIOD_US;
            *t1_out = now - ticks_behind;
            g_last_idx = write; // consume everything up to 'write'
            return true;
        }
        idx = (idx + 1) & (ADC_BUF_LEN - 1);
    }
    g_last_idx = write;
    return false;
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
  } // else if (GPIO_Pin == USER_BUTTON_Pin) {
    // g_btn_click_req = true; //unused, for debug if needed
  // }
}


void send_mouse_click(void)
{
  uint8_t hid_report[4] = {0};

  hid_report[0] = 0x01; 
  USBD_HID_SendReport(&hUsbDeviceFS, hid_report, sizeof(hid_report));
  HAL_Delay(1);

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

  // Modes 
  } else if (strcmp(cmd, "MODE1") == 0) {
    g_mode = MODE1_CALIBRATION;  send_line("ACK MODE1\r\n");

  } else if (strcmp(cmd, "MODE2") == 0) {
    g_mode = MODE2_TRIGGER;      send_line("ACK MODE2\r\n");

  } else if (strcmp(cmd, "MODE3") == 0) {
    g_mode = MODE3_MIC;          send_line("ACK MODE3\r\n");

  // Calibration
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

  // Test arming
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
      g_last_launch_us = tim2_us() - MODE2_SPACING_US; // so it fires right away
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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    (void)hadc;
    if (g_waiting_for_light) {
        uint32_t t1;
        if (detect_cross_and_stamp(g_light_threshold, &t1)) {
            uint32_t dt_us = (uint32_t)(t1 - g_t_start_us);
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
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Same handling at full-complete boundary
    HAL_ADC_ConvHalfCpltCallback(hadc);
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
