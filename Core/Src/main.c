/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Low-power environmental monitoring node
  *
  * This firmware implements an interrupt-driven, low-power environmental
  * monitoring system using an STM32 Nucleo-F401RE and a BME280 sensor.
  *
  * The MCU remains in STOP mode most of the time and wakes up periodically
  * via an RTC wake-up timer. On each wake event, the system clock is restored,
  * a single sensor measurement is performed over I2C, compensation is applied
  * using calibration data stored in sensor NVM, and the results are printed
  * over UART before returning to sleep.
  *
  * No external sensor libraries are used. All sensor communication and
  * compensation logic is implemented directly from the Bosch datasheet.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Forward declarations for local helper functions */
void uart_print(char *msg);
void I2C_Scan(void);
uint8_t BME280_ReadID(void);

/**
 * @brief Holds BME280 factory calibration coefficients.
 *
 * These values are read once from the sensor's non-volatile memory at boot
 * and reused for all subsequent compensation calculations.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData;

/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Shared intermediate value required for pressure and humidity compensation */
int32_t t_fine;

/* Flag set by RTC interrupt to signal a wake-up event */
volatile uint8_t rtc_wakeup_flag = 0;

/* Global calibration data (read once at boot) */
BME280_CalibData calib;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */

/* Low-level BME280 I2C access primitives */
void BME280_Read(uint8_t reg, uint8_t *data, uint16_t len);
void BME280_Write(uint8_t reg, uint8_t data);

#define BME280_ADDR (0x76 << 1)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Read registers from BME280 over I2C.
 *
 * Performs a register address write followed by a multi-byte read.
 */
void BME280_Read(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_I2C_Master_Transmit(&hi2c1, BME280_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, BME280_ADDR, data, len, HAL_MAX_DELAY);
}

/**
 * @brief Write a single register in the BME280.
 */
void BME280_Write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    HAL_I2C_Master_Transmit(&hi2c1, BME280_ADDR, buf, 2, HAL_MAX_DELAY);
}

/**
 * @brief Scan I2C bus for connected devices.
 *
 * Used during bring-up and debugging to verify I2C connectivity.
 */
void I2C_Scan(void)
{
    char msg[64];
    uint8_t found = 0;

    for (uint8_t addr = 8; addr < 0x78; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 2, 10) == HAL_OK)
        {
            snprintf(msg, sizeof(msg), "I2C device found at 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            found = 1;
        }
    }

    if (!found)
    {
        char no_dev[] = "No I2C devices found\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)no_dev, strlen(no_dev), HAL_MAX_DELAY);
    }
}

/**
 * @brief Lightweight UART print helper.
 *
 * Used only in main context (never inside interrupts).
 */
void uart_print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief Read factory calibration coefficients from BME280 NVM.
 *
 * These values are required for accurate compensation of raw ADC readings.
 */
void BME280_ReadCalibration(BME280_CalibData *cal)
{
    uint8_t buf[26];
    BME280_Read(0x88, buf, 26);

    cal->dig_T1 = (buf[1] << 8) | buf[0];
    cal->dig_T2 = (buf[3] << 8) | buf[2];
    cal->dig_T3 = (buf[5] << 8) | buf[4];

    cal->dig_P1 = (buf[7] << 8) | buf[6];
    cal->dig_P2 = (buf[9] << 8) | buf[8];
    cal->dig_P3 = (buf[11] << 8) | buf[10];
    cal->dig_P4 = (buf[13] << 8) | buf[12];
    cal->dig_P5 = (buf[15] << 8) | buf[14];
    cal->dig_P6 = (buf[17] << 8) | buf[16];
    cal->dig_P7 = (buf[19] << 8) | buf[18];
    cal->dig_P8 = (buf[21] << 8) | buf[20];
    cal->dig_P9 = (buf[23] << 8) | buf[22];

    BME280_Read(0xA1, &cal->dig_H1, 1);

    uint8_t buf2[7];
    BME280_Read(0xE1, buf2, 7);

    cal->dig_H2 = (buf2[1] << 8) | buf2[0];
    cal->dig_H3 = buf2[2];
    cal->dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F);
    cal->dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
    cal->dig_H6 = (int8_t)buf2[6];
}

/**
 * @brief Configure BME280 measurement settings.
 *
 * Sets oversampling and normal mode operation.
 */
void BME280_Init(void)
{
	    BME280_Write(0xF2, 0x01);  // Humidity oversampling x1
	    BME280_Write(0xF5, 0x00);  // Standby irrelevant in forced mode
	    BME280_Write(0xF4, 0x25);  // Temp & pressure x1, FORCED mode
}

/**
 * @brief Read raw uncompensated ADC values from BME280.
 *
 * Reads pressure, temperature, and humidity registers in a single burst.
 */
void BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum)
{
    uint8_t buf[8];

    // Data registers start at 0xF7
    BME280_Read(0xF7, buf, 8);

    // Pressure (20-bit)
    *raw_press = (int32_t)(
        (buf[0] << 12) |
        (buf[1] << 4)  |
        (buf[2] >> 4)
    );

    // Temperature (20-bit)
    *raw_temp = (int32_t)(
        (buf[3] << 12) |
        (buf[4] << 4)  |
        (buf[5] >> 4)
    );

    // Humidity (16-bit)
    *raw_hum = (int32_t)(
        (buf[6] << 8) |
        buf[7]
    );
}

/* Compensation functions follow Bosch reference implementation.
 * Fixed-point arithmetic is used for deterministic execution.
 */

int32_t BME280_Compensate_Temperature(int32_t adc_T, BME280_CalibData *cal)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)cal->dig_T1 << 1))) *
            ((int32_t)cal->dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)cal->dig_T1)) *
              ((adc_T >> 4) - ((int32_t)cal->dig_T1))) >> 12) *
            ((int32_t)cal->dig_T3)) >> 14;

    t_fine = var1 + var2;

    T = (t_fine * 5 + 128) >> 8;   // temperature in 0.01 °C

    return T;
}

uint32_t BME280_Compensate_Pressure(int32_t adc_P, BME280_CalibData *cal)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal->dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal->dig_P5) << 17);
    var2 = var2 + (((int64_t)cal->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal->dig_P3) >> 8) +
           ((var1 * (int64_t)cal->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) *
           ((int64_t)cal->dig_P1) >> 33;

    if (var1 == 0)
        return 0; // avoid division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)cal->dig_P7) << 4);

    return (uint32_t)p;  // pressure in Pa (Q24.8 format)
}

uint32_t BME280_Compensate_Humidity(int32_t adc_H, BME280_CalibData *cal)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - ((int32_t)76800);

    v_x1_u32r = (((((adc_H << 14) -
                     (((int32_t)cal->dig_H4) << 20) -
                     (((int32_t)cal->dig_H5) * v_x1_u32r)) +
                    ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)cal->dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t)cal->dig_H3)) >> 11) +
                        ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) *
                    ((int32_t)cal->dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r -
                (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                  ((int32_t)cal->dig_H1)) >> 4);

    if (v_x1_u32r < 0)
        v_x1_u32r = 0;
    if (v_x1_u32r > 419430400)
        v_x1_u32r = 419430400;

    return (uint32_t)(v_x1_u32r >> 12);  // humidity in %RH × 1024
}

/**
 * @brief RTC wake-up interrupt callback.
 *
 * ISR only sets a flag; all processing is deferred to main loop.
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    rtc_wakeup_flag = 1;
}

/**
 * @brief Perform one complete sensor measurement and print results.
 *
 * Called only after RTC wake-up. Executes quickly and does not block.
 */
void Read_And_Print_Sensor(void)
{
	BME280_Write(0xF4, 0x25);   // Trigger forced measurement
	HAL_Delay(10);             // Wait for conversion (~7ms worst case)

    int32_t raw_t, raw_p, raw_h;
    int32_t temp;
    uint32_t press, hum;

    BME280_ReadRaw(&raw_t, &raw_p, &raw_h);

    temp  = BME280_Compensate_Temperature(raw_t, &calib);
    press = BME280_Compensate_Pressure(raw_p, &calib);
    hum   = BME280_Compensate_Humidity(raw_h, &calib);

    char msg[120];
    snprintf(msg, sizeof(msg),
             "[RTC] T=%ld.%02ld C  P=%lu Pa  H=%lu %%\r\n",
             temp / 100,
             temp % 100,
             press / 256,
             hum / 1024);

    uart_print(msg);
}


/* USER CODE END 0 */

int main(void)
{
	 HAL_Init();
	    SystemClock_Config();

	    MX_GPIO_Init();
	    MX_USART2_UART_Init();
	    MX_I2C1_Init();
	    MX_RTC_Init();

	    uart_print("BOOT OK\r\n");

	    BME280_ReadCalibration(&calib);
	    BME280_Init();

	    uart_print("Sensor ready\r\n");

	    // Configure RTC ONCE
	    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	    HAL_RTCEx_SetWakeUpTimer_IT(
	        &hrtc,
	        5,
	        RTC_WAKEUPCLOCK_CK_SPRE_16BITS
	    );

	    /* Infinite loop */
	    /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /* Enter STOP mode */
	  if (rtc_wakeup_flag)
	         {
	             rtc_wakeup_flag = 0;

	             // REQUIRED after STOP
	             SystemClock_Config();

	             HAL_ResumeTick();

	             __HAL_RCC_USART2_CLK_ENABLE();
	             MX_USART2_UART_Init();

	             Read_And_Print_Sensor();
	         }
	  __HAL_RCC_USART2_CLK_DISABLE();
	  HAL_SuspendTick();

	         // Enter STOP mode
	         HAL_PWR_EnterSTOPMode(
	             PWR_LOWPOWERREGULATOR_ON,
	             PWR_STOPENTRY_WFI
	         );

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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
