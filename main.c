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
#include "NRF24_reg_addresses.h"
#include "NRF24.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

#define PLD_SIZE 6

#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_PIN_3   // CSN on PA3

#define NRF_CE_PORT   GPIOA
#define NRF_CE_PIN    GPIO_PIN_4   // CE on PA4

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
typedef struct __attribute__((packed)) {
  uint8_t hdr;   // 0xAA
  uint8_t seq;   // rolling counter
  int8_t  vx;    // forward/back [-127..127]
  int8_t  wz;    // turn        [-127..127]
  uint8_t flags; // bit0 = e-stop
  uint8_t crc;   // CRC-8 over bytes [0..4], poly 0x07, init 0x00
} ControlPkt;

static uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static int8_t ramp(int8_t cur, int8_t target, int8_t step){
  if (target > cur) return (target - cur > step) ? cur + step : target;
  if (target < cur) return (cur - target > step) ? cur - step : target;
  return cur;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#ifdef tx
//uint8_t data_R[PLD_SIZE] = {"Hello"};
//#else
//uint8_t data_T[PLD_SIZE];
//#endif
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  csn_high();
  ce_low();

  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(100);
  nrf24_set_crc(en_crc, _1byte);

  nrf24_pipe_pld_size(0, PLD_SIZE);
  uint8_t addr[5] = {'0', '0','0','0','1'};
  nrf24_open_rx_pipe(0,addr);
//  nrf24_open_tx_pipe[0,addr];
ce_high();
nrf24_listen();

/* State */
 uint32_t last_rx_ms = HAL_GetTick();
 // If you later add motors, keep ramped outputs here:
 int8_t Lout = 0, Rout = 0;

//uint8_t rx[PLD_SIZE] = {0};
//#ifdef tx
  //nrf24_stop_listen();
//#else
  //nrf24_listen();
//#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (nrf24_data_available())
	      {
	        ControlPkt p = {0};
	        nrf24_receive((uint8_t*)&p, sizeof(p));

	        /* Validate */
	        if (p.hdr != 0xAA) goto after_packet;
	        if (crc8((uint8_t*)&p, 5) != p.crc) goto after_packet;

	        last_rx_ms = HAL_GetTick();

	        /* E-stop flag? */
	        if (p.flags & 0x01) {
	          // TODO: drive motors: hiw_set_speed(1,0); hiw_set_speed(2,0);
	          Lout = Rout = 0;
	          // quick flash to indicate e-stop
	          for (int i=0;i<3;i++){ HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(60); }
	          continue;
	        }

	        /* Convert to floats in [-1..1] */
	        float v = ((float)p.vx) / 127.0f;   // forward/back
	        float w = ((float)p.wz) / 127.0f;   // turn

	        /* Tank mix */
	        float L = clampf(v + w, -1.0f, 1.0f);
	        float R = clampf(v - w, -1.0f, 1.0f);

	        /* First-run limit (raise later) */
	        const float LIMIT = 0.60f;
	        L = clampf(L, -LIMIT, LIMIT);
	        R = clampf(R, -LIMIT, LIMIT);

	        /* Map to -100..100 (for your motor driver later) */
	        int8_t Ltgt = (int8_t)lrintf(L * 100.0f);
	        int8_t Rtgt = (int8_t)lrintf(R * 100.0f);

	        /* Optional ramping (comment out if not needed yet) */
	        Lout = ramp(Lout, Ltgt, 3);
	        Rout = ramp(Rout, Rtgt, 3);

	        /* TODO: send to motor driver over I2C here
	           hiw_set_speed(1, Lout);
	           hiw_set_speed(2, Rout);
	        */

	        /* For now: LED indicates packets — blink faster with more forward */
	        uint32_t base = 400;
	        uint32_t delta = (uint32_t)(fabsf(v) * 300.0f);
	        uint32_t delayMs = base - delta;
	        if (w > 0.15f) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	        else           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	        HAL_Delay(delayMs);
	      }

	  after_packet:
	      /* Link-loss watchdog: stop if no packet for 300 ms */
	      if (HAL_GetTick() - last_rx_ms > 300) {
	        if (Lout || Rout) {
	          Lout = Rout = 0;
	          // TODO: drive motors: hiw_set_speed(1,0); hiw_set_speed(2,0);
	        }
	        // Slow blink to show we’re waiting for link
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	        HAL_Delay(250);
	      }
	    }
	  }

//#ifdef rx

//(nrf24_receive(data_R, sizeof(data_R));

  //HAL_Delay(1);
//#else
  //nrf24_stop_listen();
	//  if(nrf24_data_available()){
		//  nrf24_transmit(data_T, sizeof(data_T));
	  //}
	  //HAL_Delay(1);

//#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
