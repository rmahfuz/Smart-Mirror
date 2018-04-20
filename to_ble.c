/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay (int a) {
    volatile int i,j;
    for (i=0 ; i < a ; i++) {
        j++;
    }
    return;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

    /*char buf [9] = "fghijk";
    char to_send = '\r';
    char* send_buf = "SF,2";*/

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
    char buf [20] = "qqqqqqqqqqqqqqqqqqqq"; // receiving buffer
    char to_send = '\r';
    char* send_buf = "SF,1";
	//send SF,1\r
	send_buf = "SF,1";
	HAL_UART_Transmit(&huart6, (uint8_t*)send_buf, 4, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 5, 5000);
	//delay(2700000); // about three sec
	asm("nop");
	//send SN,SmartMirror\r
	send_buf = "SN,bcd";
	HAL_UART_Transmit(&huart6, send_buf, 6, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	delay(900000); // about one sec
	HAL_UART_Receive(&huart6, buf, 5, 5000);
	asm("nop");
	//send SR,00000000
	send_buf = "SR,00000000";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 11, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	delay(3600000); // about four sec
	HAL_UART_Receive(&huart6, buf, 12, 5000);
	asm("nop");

	//send R,1\r
	send_buf = "R,1";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 3, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	//delay(3600000); // about four sec
	//HAL_UART_Receive(&huart6, buf2, 12, 5000);
	delay(3600000); // about four sec
	HAL_UART_Receive(&huart6, buf, 12, 5000); //should receive Reboot cmd

	asm("nop");

	//send A\r                  temporary addition
	send_buf = "A";
	HAL_UART_Transmit(&huart6, send_buf, 1, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	delay(900000); // about one sec
	HAL_UART_Receive(&huart6, buf, 5, 5000);
	asm("nop");
/////////////-------------------------------------------------------------------------------
	//send SS,00000001\r
	send_buf = "SS,00000001";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, 11, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send SR,00000000\r
	send_buf = "SR,00000000";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, 11, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send PZ\r
	send_buf = "PZ";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 2, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send PS,01020304-0506-0708-0900-0a0b0c0d0e0f\r
	send_buf = "PS,01020304-0506-0708-0900-0a0b0c0d0e0f";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, 39, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send PC,11223344-5566-7788-9900-aabbccddeeff,08,20\r
	send_buf = "PC,11223344-5566-7788-9900-aabbccddeeff,08,20";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 45, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send PC,11112222-3333-4444-5555-666677778888,08,20\r
	send_buf = "PC,11112222-3333-4444-5555-666677778888,08,20";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, 45, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send PC,99990000-aaaa-bbbb-cccc-ddddeeeeffff,08,20\r
	send_buf = "PC,99990000-aaaa-bbbb-cccc-ddddeeeeffff,08,20";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send,45, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	//send R,1\r
	send_buf = "R,1";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 3, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(2700000); // about three sec
	asm("nop");
	//send LS\r
	/*send_buf = "LS";
	HAL_UART_Transmit(&huart6, 2, sizeof(to_send), 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");*/
	send_buf = "SR,20000000";
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send,11, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(900000); // about one sec
	asm("nop");
	send_buf = "R,1";
	HAL_UART_Transmit(&huart6,(uint8_t*)&to_send, 3, 5000);
	HAL_UART_Transmit(&huart6, (uint8_t*)&to_send, sizeof(to_send), 5000);
	HAL_UART_Receive(&huart6, buf, 4, 5000);
	delay(2700000); // about three sec
	asm("nop");



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{
  //USART6->CR2 |= 0x8000; // i added this magic (setting bit 15 of USART6->CR2)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  huart6.Instance = USART6;
  //(huart6.Instance)->CR2 |= 0x8000; // i added this magic (setting bit 15 of USART6->CR2)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  //huart6.AdvancedInit.AdvFeatureInit = UART-ADVFEATURE_SWAP_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
