/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t rtc_adj[9];
uint8_t rtc_read[8];
uint8_t rtc_init[2] = {0x00, 0x00};
uint8_t rtc_radr = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*** DEFINES ***/

#define RS RS_GPIO_Port, RS_Pin
#define EN EN_GPIO_Port, EN_Pin
#define D4 D4_GPIO_Port, D4_Pin
#define D5 D5_GPIO_Port, D5_Pin
#define D6 D6_GPIO_Port, D6_Pin
#define D7 D7_GPIO_Port, D7_Pin

/* write_lcd -> write data or comand to lcd */


  void write_lcd(uint8_t data, uint8_t cmd_data)
 {

		if (cmd_data) HAL_GPIO_WritePin(RS, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(RS, GPIO_PIN_RESET);

		if(data & 0x80)	HAL_GPIO_WritePin(D7, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D7, GPIO_PIN_RESET);
		if(data & 0x40)	HAL_GPIO_WritePin(D6, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D6, GPIO_PIN_RESET);
		if(data & 0x20)	HAL_GPIO_WritePin(D5, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D5, GPIO_PIN_RESET);
		if(data & 0x10)	HAL_GPIO_WritePin(D4, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D4, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(EN, GPIO_PIN_SET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(EN, GPIO_PIN_RESET);

		if(data & 0x08)	HAL_GPIO_WritePin(D7, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D7, GPIO_PIN_RESET);
		if(data & 0x04)	HAL_GPIO_WritePin(D6, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D6, GPIO_PIN_RESET);
		if(data & 0x02)	HAL_GPIO_WritePin(D5, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D5, GPIO_PIN_RESET);
		if(data & 0x01)	HAL_GPIO_WritePin(D4, GPIO_PIN_SET);
		else						HAL_GPIO_WritePin(D4, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(EN, GPIO_PIN_SET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(EN, GPIO_PIN_RESET);

 }

 /* string_lcd -> write a string data to LCD */
  void string_lcd (char *string)
 {
	 while(*string) write_lcd(*string++, 1);
 }

 /* LCD_init -> initialize LCD 4bits */
  void init_LCD(void)
 {
	uint8_t i, buf[6] = {0x33, 0x32, 0x28, 0x06, 0x0c, 0x01};
	for(i=0; i<6; i++) write_lcd(buf[i],0);
 }

/* clear_LCD */
void clear_LCD(void)
{
	 write_lcd(0x01,0);
}

void escrita_relogio()
{
		write_lcd(0x83,0);
	  	string_lcd("  :  :  ");
	  	write_lcd(0xC3,0);
	  	string_lcd("  /  /  ");

	  	/* inicialização RTC */
	  	rtc_adj[0] = 0x00;  //endereço do registrador
	  	rtc_adj[1] = 0x00;	//segundos
	  	rtc_adj[2] = 0x46;	//minutos
	  	rtc_adj[3] = 0x09;	//hora
	  	rtc_adj[4] = 0x02;	//dia da semana
	  	rtc_adj[5] = 0x02;	//dia
	  	rtc_adj[6] = 0x04;	//mês
	  	rtc_adj[7] = 0x18;	//ano
	  	rtc_adj[8] = 0x10;	//controle

	  	//HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtc_adj, 9, 20);
	  	//HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtc_init, 2, 20);


}

 void leitura_relogio()
 {


	 /* leitura do RTC */
		  		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &rtc_radr, 1, 20); // envio do endereço do registrador

		  		HAL_I2C_Master_Receive(&hi2c1, 0xD1, rtc_read, 8, 20); //

		  		write_lcd(0x83,0); //hora
		  		write_lcd(((rtc_read[2] & 0x30)>>4) + 0x30, 1);
		  		write_lcd((rtc_read[2] & 0x0F) + 0x30, 1);

		  		write_lcd(0x86, 0);	//minuto
		  		write_lcd(((rtc_read[1] & 0x70 )>>4) + 0x30, 1);
		  		write_lcd((rtc_read[1] & 0x0F) + 0x30, 1);

		  		write_lcd(0x89, 0);	//segundos
		  		write_lcd(((rtc_read[0] & 0x70)>>4) + 0x30, 1);
		  		write_lcd((rtc_read[0] & 0x0F) + 0x30, 1);

		  		write_lcd(0xC3, 0);	//dia do mês
		  		write_lcd(((rtc_read[4] & 0x30)>>4) + 0x30, 1);
		  		write_lcd((rtc_read[4] & 0x0F) + 0x30, 1);

		  		write_lcd(0xC6, 0);	// mês
		  		write_lcd(((rtc_read[5] & 0x70)>>4) + 0x30, 1);
		  		write_lcd((rtc_read[5] & 0x0F) + 0x30, 1);

		  		write_lcd(0xC9, 0);	//ano
		  		write_lcd(((rtc_read[6] & 0x70)>>4) + 0x30, 1);
		  		write_lcd((rtc_read[6] & 0x0F) + 0x30, 1);

 }



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_TIM21_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Init(&htim21); //Configure the timer
  HAL_TIM_Base_Start_IT(&htim21); //Start the timer
  init_LCD();
  escrita_relogio();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  leitura_relogio();


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 1999;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 999;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLUE_Pin|GREEN_Pin|D7_Pin|D6_Pin 
                          |D5_Pin|D4_Pin|EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD1_Pin|RED_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BLUE_Pin GREEN_Pin D7_Pin D6_Pin 
                           D5_Pin D4_Pin EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin|GREEN_Pin|D7_Pin|D6_Pin 
                          |D5_Pin|D4_Pin|EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD1_Pin RED_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD1_Pin|RED_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
