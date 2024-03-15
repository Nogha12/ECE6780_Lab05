/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void txCharacter(char data);
void txString(char* data);
char rxCharacter(void);
void writeByteI2C(char address, char data);
int readDataI2C(char address, int bytesToRead);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int clkSpeed;
  int targetBaud = 115200;
  
  int returnValue;
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIO B clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIO C clock
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable I2C 2 clock
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART 3 clock
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  
  // Configure GPIO B pins 11 and 13 for I2C Data and Clock
  GPIOB->MODER &= ~(GPIO_MODER_MODER11_Msk | GPIO_MODER_MODER13_Msk);
  GPIOB->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1; // alternate mode
  GPIOB->OTYPER |= GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13; // open-drain
  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR11 | GPIO_OSPEEDR_OSPEEDR13); // low-speed
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR13); // no pull-up/pull-down
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL13_Msk);
  GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos) | (0x5 << GPIO_AFRH_AFSEL13_Pos); // pin 11 to AF1, pin 13 to AF5
  
  // Configure GPIO B pin 14 
  GPIOB->MODER &= ~(GPIO_MODER_MODER14_Msk);
  GPIOB->MODER |= (GPIO_MODER_MODER14_0); // output mode
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14); // push-pull
  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR14); // low-speed
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR14); // no pull-up/pull-down
  
  // Configure GPIO C pin 0
  GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk);
  GPIOC->MODER |= (GPIO_MODER_MODER0_0); // output mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0); // push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0); // low-speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0); // no pull-up/pull-down
  
  // Configure GPIO C pins 10 and 11 to connect to USART 3
  GPIOC->MODER &= ~(GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);
  GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // alternate mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11); // push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11); // low-speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11); // no pull-up/pull-down
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
  GPIOC->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL10_Pos) | (0x1 << GPIO_AFRH_AFSEL11_Pos); // alternate function 1
  
  // Configure I2C 2
  I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLL_Msk);
  I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos); // set SCL low period to 19
  I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLH_Msk);
  I2C2->TIMINGR |= (0x0F << I2C_TIMINGR_SCLH_Pos); // set SCL high period to 15
  I2C2->TIMINGR &= ~(I2C_TIMINGR_SDADEL_Msk);
  I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos); // set data hold time to 2
  I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLDEL_Msk);
  I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos); // set data setup time to 4
  I2C2->TIMINGR &= ~(I2C_TIMINGR_PRESC_Msk);
  I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos); // set timing prescaler to 1
  
  // Configure USART 3
  clkSpeed = HAL_RCC_GetHCLKFreq();
  USART3->CR1 |= USART_CR1_RE | USART_CR1_TE; // enable TX and RX
  USART3->CR1 |= USART_CR1_RXNEIE; // enable interrupts from receive register not empty
  USART3->BRR &= 0x0;
  USART3->BRR |= clkSpeed / targetBaud; // set baud rate clock divisor

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  I2C2->CR1 |= I2C_CR1_PE; // enable I2C 2
  USART3->CR1 |= USART_CR1_UE; // enable USART 3
  
  GPIOB->ODR |= GPIO_ODR_14; // Set pin B14 high
  GPIOC->ODR |= GPIO_ODR_0; // Set pin C0 high
  
  writeByteI2C(0x69, 0x0F);
  returnValue = readDataI2C(0x69, 1);
  if (returnValue == 0xD3)
  {
    txString("WHO_AM_I is a match!\n\r");
  }
  else
  {
    txString("Error: WHO_AM_I does not match.\n\r");
    Error_Handler();
  }
  I2C2->CR2 |= I2C_CR2_STOP; // stop generation
  
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_Delay(500);
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Transmits a character via USART 3
  */
void txCharacter(char data)
{
  while ((USART3->ISR & (0x1 << 7)) == 0x0);
  USART3->TDR = data;
  return;
}

/**
  * @brief Transmits a string via USART 3
  */
void txString(char* data)
{
  for (int i = 0; data[i] != '\0'; i++)
  {
    txCharacter(data[i]);
  }
  return;
}

/**
  * @brief Receives a character via USART 3
  */
char rxCharacter(void)
{
  while ((USART3->ISR & (0x1 << 5)) == 0x0);
  return USART3->RDR;
}

void writeByteI2C(char address, char data)
{
  I2C2->CR2 &= ~(I2C_CR2_SADD_Msk);
  I2C2->CR2 |= (address << (I2C_CR2_SADD_Pos + 1)); // set slave address (7-bit)
  I2C2->CR2 &= ~(I2C_CR2_NBYTES_Msk);
  I2C2->CR2 |= (0x01 << I2C_CR2_NBYTES_Pos); // set number of bytes to 1
  I2C2->CR2 &= ~(I2C_CR2_RD_WRN); // set transfer direction to write
  I2C2->CR2 &= ~(I2C_CR2_AUTOEND); // software end mode
  
  I2C2->CR2 |= I2C_CR2_START; // start generation
  
  // Wait for TXIS or NACKF to be set and respond accordingly
  while ((I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
  {
    HAL_Delay(1);
  }
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    txString("Error: slave did not respond.\n\r");
    Error_Handler();
    return;
  }
  
  txString("Slave successfully responded.\n\r");
  I2C2->TXDR = data; // load data to write
  while ((I2C2->ISR & I2C_ISR_TC) == 0); // wait for transfer complete
  return;
}

int readDataI2C(char address, int bytesToRead)
{ 
  int data = 0;
  
  if (bytesToRead > sizeof(int))
  {
    txString("Warning: Too many bytes to read.\n\r");
    return -1;
  }
  
  I2C2->CR2 &= ~(I2C_CR2_SADD_Msk);
  I2C2->CR2 |= (address << (I2C_CR2_SADD_Pos + 1)); // set slave address (7-bit)
  I2C2->CR2 &= ~(I2C_CR2_NBYTES_Msk);
  I2C2->CR2 |= (bytesToRead << I2C_CR2_NBYTES_Pos); // set number of bytes
  I2C2->CR2 |= I2C_CR2_RD_WRN; // set transfer direction to read
  I2C2->CR2 &= ~(I2C_CR2_AUTOEND); // software end mode
  
  I2C2->CR2 |= I2C_CR2_START; // start generation
  
  for (int i = 0; i < bytesToRead; i++)
  {
    // Wait for TXIS or NACKF to be set and respond accordingly
    while ((I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)) == 0)
    {
      HAL_Delay(1);
    }
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      txString("Error: slave did not respond.\n\r");
      Error_Handler();
      return -1;
    }
    txString("Slave successfully read.\n\r");
    while ((I2C2->ISR & I2C_ISR_TC) == 0); // wait for transfer copmlete
    data |= I2C2->RXDR << i*8;
  }
  
  return data;
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
