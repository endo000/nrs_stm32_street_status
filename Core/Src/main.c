/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct DATA {
    int16_t Gyro_XYZ[3];
    int16_t Acc_XYZ[3];
    uint8_t READY;
};

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

/* USER CODE BEGIN PV */

struct DATA sensors_data;

float minmax_X = 350;
float minmax_Y = 300;
float minmax_roll = 25;
float minmax_pitch = 25;

char *status_str[7] = {"tilted to right", "tilted to left", "left side pit", "right side pit", "front pit",
                       "tilted down", "tilted up"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void pause() {
    uint32_t counter = 0;
    for (counter = 0; counter < 600; counter++) {
        asm("nop");
    }
}

void SPI1_write(uint8_t reg, uint8_t buffer) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    pause();
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
    pause();
    HAL_SPI_Transmit(&hspi1, &buffer, 1, 10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    pause();
}

uint8_t SPI1_read(uint8_t reg) {
    uint8_t buffer;
    reg |= 0x80;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    pause();
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
    pause();
    HAL_SPI_Receive(&hspi1, &buffer, 1, 10);
    pause();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    return buffer;
}

uint8_t I2C1_write(uint8_t address, uint8_t reg, uint8_t buffer) {
    address <<= 1;
    return HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 10);
}

uint8_t I2C1_read(uint8_t address, uint8_t reg) {
    uint8_t buffer;
    address <<= 1;
    HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &buffer,
                     1, 10);
    return buffer;
}


void initL3GD20() {
    uint8_t cip = SPI1_read(0x0F);
    if (cip != 0xD4 && cip != 0xD3)
        for (;;);

    // Initialize gyroscope
    SPI1_write(0x20, 0x0F); // ODR 95 Hz, Cut-Off 12.5
    SPI1_write(0x23, 0x10); // Full scale 500 dps
    SPI1_write(0x22, 0x08); // INT2

}

void initLSM303DLHC() {
    HAL_Delay(10);

    // Send sensor info
#define OLD_SENSOR 0x73
    I2C1_write(0x1E, 0x4F, OLD_SENSOR);
    HAL_Delay(100);

    // Initialize accelerometer
    I2C1_write(0x19, 0x20, 0x57); // ODR 100 Hz
    I2C1_write(0x19, 0x22, 0x10); // DRDY1 int. on INT1
    // High res. output mode
    // Block data update
    // Full-scale +/- 2G
    I2C1_write(0x19, 0x23, 0x88);
}

void readGyro_raw(int16_t *data) {
    // Read H and L registers;
    data[0] = (int16_t) ((SPI1_read(0x29) << 8) | SPI1_read(0x28)); // OUT_X
    data[1] = (int16_t) ((SPI1_read(0x2B) << 8) | SPI1_read(0x2A)); // OUT_Y
    data[2] = (int16_t) ((SPI1_read(0x2D) << 8) | SPI1_read(0x2C)); // OUT_Z
//    data[3] = SPI1_read(0x26); // OUT_TEMP
}

void readGyro(float *data, const int16_t *raw_data) {
    // FS = 500 -> Sensitivity - 17.50
    float scaler = 17.50f * 0.001f * 2;

    for (uint8_t i = 0; i < 3; i++) {
        data[i] = (float) raw_data[i] * scaler;
    }
//    data[3] = (float) ((int8_t) ~(raw_data[3] - 1) + 25);
//    data[3] = (float) (raw_data[3]);

    // Align axes
    float temp = data[0];
    data[0] = data[1] * -1;
    data[1] = temp * -1;
}

void readACC_raw(int16_t *data) {
    data[0] = (int16_t) ((I2C1_read(0x19, 0x29) << 8) | I2C1_read(0x19, 0x28)); // OUT_X_L
    data[1] = (int16_t) ((I2C1_read(0x19, 0x2B) << 8) | I2C1_read(0x19, 0x2A)); // OUT_Y_L
    data[2] = (int16_t) ((I2C1_read(0x19, 0x2D) << 8) | I2C1_read(0x19, 0x2C)); // OUT_Z_L
}

void readACC(float *data, const int16_t *raw_data) {
    // FS = 00 -> Sensitivity - 1
    // FS = 00 -> Measurement range - +/- 2
    float scale = 1 / 1.0f * 0.001f;
    for (uint8_t i = 0; i < 3; i++) {
        data[i] = (float) raw_data[i] * scale;
    }

    // Align axes
    data[1] *= -1;
}

uint16_t process_status() {
    float dataACC[3];
    float dataGyro[3];
    readACC(dataACC, sensors_data.Acc_XYZ);
    readGyro(dataGyro, sensors_data.Gyro_XYZ);

    float pitch = (float) (180.0f / M_PI *
                           atan2f(dataACC[0], sqrtf(dataACC[1] * dataACC[1] + dataACC[2] * dataACC[2])));
    float roll = (float) (180.0f / M_PI *
                          atan2f(dataACC[1], sqrtf(dataACC[0] * dataACC[0] + dataACC[2] * dataACC[2])));

    uint16_t status_code = 0;

    if (fabsf(roll) > minmax_roll) {
        if (roll > 0) {
            status_code |= 1;
        } else {
            status_code |= 1 << 1;
        }
    }
    if (fabsf(dataGyro[0]) > minmax_X) {
        if (dataGyro[0] > 0) {
            status_code |= 1 << 2;
        } else {
            status_code |= 1 << 3;
        }
    }
    if (fabsf(dataGyro[1]) > minmax_Y) {
        status_code |= 1 << 4;
    }
    if (fabsf(pitch) > minmax_pitch) {
        if (pitch > 0) {
            status_code |= 1 << 5;
        } else {
            status_code |= 1 << 6;
        }
    }

    return status_code;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int16_t data_raw[3];
    if (GPIO_Pin == GPIO_PIN_1) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
        readGyro_raw(data_raw);
        memcpy(sensors_data.Gyro_XYZ, data_raw, 3 * sizeof(int16_t));

        sensors_data.READY |= 1;
    } else if (GPIO_Pin == GPIO_PIN_4) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
        readACC_raw(data_raw);
        memcpy(sensors_data.Acc_XYZ, data_raw, 3 * sizeof(int16_t));

        sensors_data.READY |= 1 << 1;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    __HAL_SPI_ENABLE(&hspi1);
    __HAL_I2C_ENABLE(&hi2c1);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // set CS to 1

    initL3GD20();
    initLSM303DLHC();

    int16_t data_raw[3];
    readGyro_raw(data_raw);
    readACC_raw(data_raw);

    uint8_t mode = 0;
    uint8_t max_mode = 2;
    uint16_t packet_num = 0;

    uint8_t binary_data_size = 8;
    uint16_t binary_output[binary_data_size];
    uint16_t binary_status_output[3];

    binary_output[0] = 0xAAAB;
    binary_status_output[0] = 0xACAB;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
            mode++;
            if (mode == max_mode) {
                mode = 0;
            }

            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
                HAL_Delay(30);
            }
        }

        if (sensors_data.READY != 0b11) continue;

        sensors_data.READY = 0;

        if (mode == 0) {
            // Binary status
            binary_output[1] = packet_num;

            uint16_t status_code = process_status();

            binary_status_output[2] = status_code;

            CDC_Transmit_FS((uint8_t *) binary_status_output, 3 * sizeof(uint16_t));
        } else if (mode == 1) {
            // ASCII status
            char output[200], status[50];

            uint8_t status_code = process_status();

            if (status_code == 0) {
                sprintf(status, "normal");
            } else {
                uint8_t is_first = 1;
                for (uint8_t i = 0; i < 7; ++i) {
                    if (((status_code >> i) & 1) == 1) {
                        char temp[30];
                        sprintf(temp, "%s", status_str[i]);

                        if (is_first) {
                            sprintf(status, "%s", temp);
                            is_first = 0;
                        } else {
                            sprintf(status, "%s, %s", status, temp);
                        }
                    }
                }
            }

            sprintf(output,
                    "{\"Street analyze\": %5u, \"Code\": %3d, \"Status\": \"%s\"}\n\r",
                    packet_num, status_code, status);

            CDC_Transmit_FS((uint8_t *) output, strlen(output));
        }
        packet_num++;

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0000020B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                             | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                             | LD6_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DRDY_Pin MEMS_INT4_Pin MEMS_INT1_Pin */
    GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT4_Pin | MEMS_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                             LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                             LD6_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                          | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                          | LD6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = MEMS_INT3_Pin | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
