/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Servo 1 Variables
int servo1_p_flag = 1;
int servo1_c_flag = 0;
int previous_state_s1 = s1_continue;
int current_position_s1 = 0;
int new_position_s1 = 0;
int n_loop_s1 = 0;
int n_wait_s1 = 0;
int difference_s1= 0;
int s1_delay_pos = 0;
int s1_waiting_f = 0;
int s1_positioning_f = 0;
int wait_i_s1 = 0;
int s1_loop_index_b = 0;
int s1_loop_index_e = 0;
int s1_i_loop = 0;
int s1_i_nloop = 0;
int s1_looping_f = 0;
int s1_nested_loop_error=0;
int r_s1_flag = 0;
int l_s1_flag = 0;
int s1_loop_wait = 0;
enum s_servo_1 ss1;


//Servo 2 Variables
int servo2_p_flag = 1;
int servo2_c_flag = 0;
int previous_state_s2 = s2_continue;
int current_position_s2 = 0;
int new_position_s2 = 0;
int n_loop_s2 = 0;
int n_wait_s2 = 0;
int difference_s2= 0;
int s2_delay_pos = 0;
int s2_waiting_f = 0;
int s2_positioning_f = 0;
int wait_i_s2 = 0;
int s2_loop_index_b = 0;
int s2_loop_index_e = 0;
int s2_i_loop = 0;
int s2_i_nloop = 0;
int s2_looping_f = 0;
int s2_nested_loop_error=0;
int r_s2_flag = 0;
int l_s2_flag = 0;
int s2_loop_wait = 0;
enum s_servo_2 ss2;

//UI Variables
int command_recieved= 0, command_valid = 0;
char ui_data[3];
int command_processed = 0;
int i = 0;
char * end_rec = "Recipe Has Ended\r\n";

//Recipes
/*DEMO RECIPE*/
char recipe1[] = { MOV | 0, MOV | 5, MOV | 0, MOV | 3, LOOP | 0 , MOV | 1, MOV | 4,MOV|5, END_LOOP, MOV | 0, MOV | 2, WAIT | 0 , MOV | 2, WAIT | 0, MOV | 2, MOV | 5, WAIT|31, WAIT|31, WAIT|31, MOV|4, RECIPE_END } ;
char recipe2[] = { MOV | 0, MOV | 5, MOV | 0, MOV | 3, LOOP | 0 , MOV | 1, MOV | 4,MOV|5, END_LOOP, MOV | 0, MOV | 2, WAIT | 0 , MOV | 2, WAIT | 0, MOV | 2, MOV | 5, WAIT|31, WAIT|31, WAIT|31, MOV|4, RECIPE_END } ;

/*RECIPE WITH ALL POSITIONS*/
//char recipe1[] = {  MOV | 0, MOV | 1, MOV | 2, MOV | 3, MOV | 4, MOV | 5, RECIPE_END } ;
//char recipe2[] = { MOV | 0, MOV | 1, MOV | 2, MOV | 3, MOV | 4, MOV | 5,  RECIPE_END } ;

/*RECIPE WITH COMMAND ERROR*/
//char recipe1[] = { MOV | 0, MOV | 1, MOV | 7,LOOP|2,MOV|3,MOV|5, MOV|3,END_LOOP, MOV | 4, MOV | 5, MOV | 4, RECIPE_END,  } ;

/*RECIPE WITH NESTED LOOP ERROR*/
//char recipe1[] = { MOV | 0, MOV | 1, MOV | 2,LOOP|2,MOV|3,MOV|5, MOV|3,LOOP|3,END_LOOP, MOV | 4, MOV | 5, MOV | 4, RECIPE_END,  } ;


int recipe_index_1 =  0;
int recipe_index_2 =  0;
int command_s1 = 0, value_s1 =0;
int command_s2 = 0, value_s2 =0;
int size_rec_s1 = 0;
int size_rec_s2 = 0;
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	char cmd_msg[] = "Please enter a command: ";
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Init(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	ss1 = s1_pause;
	ss2 = s2_pause;
	HAL_UART_Transmit(&huart2, (uint8_t *)cmd_msg, sizeof(cmd_msg), HAL_MAX_DELAY);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		get_UI_cmd();
		process_servo1(recipe1);
		process_servo2(recipe2);
		HAL_Delay(100);
	  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void get_UI_cmd(void)
{

	int n = 0;
	char servo1_cmd = '0';
	char servo2_cmd = '0';
	uint8_t CR_NL[] = {0x0D, 0x0A}; 
	char invalid_input_msg[58] = "\n\rInvalid Command...Please Enter P/p/C/c/R/r/L/l/N/n/B/b.\n";
	
	
//	HAL_UART_Receive(&huart2, (uint8_t *)ui_data, sizeof(ui_data)-1, HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t *)ui_data, sizeof(ui_data)-1, HAL_MAX_DELAY);		
//	HAL_UART_Transmit(&huart2, CR_NL, sizeof(CR_NL), HAL_MAX_DELAY);
	if(i < 3)
	{
		HAL_UART_Receive(&huart2, (uint8_t *)&ui_data[i], 1, 1);
		if (ui_data[i] == '\r' || ui_data[i] == 'P' || ui_data[i] == 'p' || ui_data[i] == 'C' || ui_data[i] == 'c' || ui_data[i] == 'R' || ui_data[i] == 'r' || ui_data[i] == 'L' || ui_data[i] == 'l' || ui_data[i] == 'N' || ui_data[i] == 'n' || ui_data[i] == 'B' || ui_data[i] == 'b'  )
		{
			HAL_UART_Transmit(&huart2, (uint8_t *)&ui_data[i], 1, HAL_MAX_DELAY);	
			i++;
		}
		else{
			
		}
	}
	else{
		command_recieved = 1;
		i = 0;
		
	}
	
	if(command_recieved){
	if(ui_data[0] == 'P' || ui_data[0] == 'p' || ui_data[0] == 'C' || ui_data[0] == 'c' || ui_data[0] == 'R' || ui_data[0] == 'r' || ui_data[0] == 'L' || ui_data[0] == 'l' || ui_data[0] == 'N' || ui_data[0] == 'n' || ui_data[0] == 'B' || ui_data[0] == 'b' )
	{
		if(ui_data[1] == 'P' || ui_data[1] == 'p' || ui_data[1] == 'C' || ui_data[1] == 'c' || ui_data[1] == 'R' || ui_data[1] == 'r' || ui_data[1] == 'L' || ui_data[1] == 'l' || ui_data[1] == 'N' || ui_data[1] == 'n' || ui_data[1] == 'B' || ui_data[1] == 'b')
		{	
			if(ui_data[2] == '\r')
			{
				command_valid = 1;
			}
			else{
				command_valid = 0;
				command_recieved = 0;
				reset_buffer();
				HAL_UART_Transmit(&huart2, (uint8_t*)"Command Invalid\r\n",strlen("Command Invalid\r\n"),100);
			}
			
		
		}
	}
}
	if(command_recieved && command_valid){
		
		command_valid = 0;
		command_recieved = 0;
	HAL_UART_Transmit(&huart2, CR_NL, sizeof(CR_NL), HAL_MAX_DELAY);
	servo1_cmd = ui_data[0];
	servo2_cmd = ui_data[1];
	/*Pause servo*/
	if (servo1_cmd == 'p' || servo1_cmd == 'P')
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		servo1_p_flag = 1;
		servo1_c_flag = 0;
	}
	/*continue servo*/
	if (servo1_cmd == 'c' || servo1_cmd == 'C')
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
		servo1_c_flag = 1;
		servo1_p_flag = 0;
	}
	/*Operations when servo is paused*/
	
		/*Move 1 position to the RIGHT if possible*/
	if (servo1_cmd == 'r' || servo1_cmd == 'R')
			{
				if (servo1_p_flag == 1)
					{
							if(current_position_s1 != 5){
								current_position_s1++;
								servo1_positions(current_position_s1);
								r_s1_flag = 1;
								
							}
							else{
								//Extreme Right Position!!!
							}
					}
					else{
						//Not in Paused state!!!
					}
					
				
			}
			/*Move 1 position to the LEFT if possible*/
	if (servo1_cmd == 'l' || servo1_cmd == 'L')
	{
			if (servo1_p_flag == 1)
					{
						if(current_position_s1 != 0){
							current_position_s1--;
								servo1_positions(current_position_s1);
								l_s1_flag = 1;
							}
							else{
								//Extreme Left Position!!!
							}
			
					}
				else 
					{
					}
			
	}
	
	if(servo1_cmd == 'n' || servo1_cmd == 'N'){
		//Do Nothing
	
	}
	
	if(servo1_cmd == 'b' || servo1_cmd == 'B'){
			recipe_index_1 = 0;	
			servo1_positions(0);
      		ss1 = s1_continue;
			servo1_p_flag = 0;
			servo1_c_flag = 1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

		
	}
	
	if (servo2_cmd == 'p' || servo2_cmd == 'P')
	{
		
		servo2_p_flag = 1;
		servo2_c_flag = 0;
	}
	/*continue servo*/
	if (servo2_cmd == 'c' || servo2_cmd == 'C')
	{
		
		servo2_c_flag = 1;
		servo2_p_flag = 0;
	}
	/*Operations when servo is paused*/
	
		/*Move 1 position to the RIGHT if possible*/
	if (servo2_cmd == 'r' || servo2_cmd == 'R')
			{
				if (servo2_p_flag == 1)
					{
							if(current_position_s2 != 5){
								current_position_s2 ++;
								servo2_positions(current_position_s2);
								r_s2_flag =1 ;
							}
							else{
								//Extreme Right Position!!!
							}
					}
				else {
						}
			
				
			}
			/*Move 1 position to the LEFT if possible*/
	if (servo2_cmd == 'l' || servo2_cmd == 'L')
	{
			if (servo2_p_flag == 1)
					{
							if(current_position_s2 != 0){
								current_position_s2 --;
								servo2_positions(current_position_s2);
								l_s2_flag=1;
							}
							else{
								//Extreme Left Position!!!
							}
			
					}
				else 
					{
					}
			
	}
	
	if(servo2_cmd == 'n' || servo2_cmd == 'N'){
		//Do Nothing
	
	}
	
	if(servo2_cmd == 'b' || servo2_cmd == 'B'){
			recipe_index_2 = 0;	
			servo2_positions(0);
			HAL_Delay(1000);
		    ss2 = s2_continue;
			servo2_p_flag = 0;
			servo2_c_flag = 1;
	}
	
	
		
	command_processed = 1;
	reset_buffer();
	}
	
}

void reset_buffer(void){
	int n=0;
	for(n=0;n<3;n++){
		ui_data[n] = 0;
	}

}

/*Position Servo 1*/
void servo1_positions(int s1){
	switch (s1)
	{
		case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_0);

			break;
		case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_1);

			break;
		case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_2);

			break;
		case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_3);

			break;
		case 4: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_4);

			break;
		case 5: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_POSITION_5);

			break;
	}

}

/*Position Servo 2*/
void servo2_positions(int s2){
	switch (s2)
	{
		case 0: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_0);

			break;
		case 1: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_1);

			break;
		case 2: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_2);

			break;
		case 3: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_3);

			break;

		case 4: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_4);

			break;
		case 5: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_POSITION_5);

			break;
	}

}


/*State Machine Implementation Servo 1*/
void process_servo1(char * str1){
	int j = 0;
	size_rec_s1 = sizeof(recipe1);
	switch(ss1){
		case s1_pause:
			
			if(servo1_c_flag)
				ss1 = previous_state_s1;
			
			else
				ss1 = s1_pause;
			
			break;
		case s1_continue:
			previous_state_s1 = s1_continue;
			if(servo1_p_flag)
				ss1 = s1_pause;
			else {
				if(r_s1_flag || l_s1_flag){
					r_s1_flag = 0;
					l_s1_flag = 0;
					servo1_positions(value_s1);
				}
				
				if(recipe_index_1 < size_rec_s1){
					command_s1 = str1[recipe_index_1]  & COMMAND_MASK ;
					if (command_s1 == MOV ){
						value_s1 = str1[recipe_index_1] & VALUE_MASK;
						if(value_s1 > 5){
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
							break;
						}
						new_position_s1 = value_s1;
						difference_s1 = abs(new_position_s1 - current_position_s1);
						servo1_positions(value_s1);
						s1_positioning_f = 1;
						ss1 = s1_positioning;
						current_position_s1 = new_position_s1;
						}
					else if (command_s1 == WAIT ){
						s1_waiting_f = 1 ;
						ss1 = s1_wait;
						n_wait_s1 = str1[recipe_index_1] & VALUE_MASK;
						if (n_wait_s1 > 31){
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

						}
						}
					else if (command_s1 == LOOP){
						s1_looping_f = 1;
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
						
						n_loop_s1 = str1[recipe_index_1] & VALUE_MASK;
						if (n_loop_s1 > 31){
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
						}
						s1_loop_index_b = recipe_index_1+1;
						j = s1_loop_index_b;
						while((int)str1[++j]!= END_LOOP);
						s1_loop_index_e = j;
						for(int n = s1_loop_index_b; n < s1_loop_index_e; n++){
							if((int)(str1[n]&COMMAND_MASK) == LOOP){
								s1_nested_loop_error = 1;
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
								s1_looping_f = 0;
								break;
							}
						}
						
						s1_i_loop = s1_loop_index_b;
						ss1 = s1_looping;
						
					}
					else if (command_s1 == RECIPE_END){
						HAL_UART_Transmit(&huart2, (uint8_t*)end_rec, strlen(end_rec),100);
						ss1 = s1_pause;
						servo1_c_flag = 0;
						servo1_p_flag = 1;
						break;
					}
					else if (command_s1 == SKIP_NEXT){
						recipe_index_1 ++;
					}
					else if (command_s1 == END_LOOP){
						
					}
					
					else{
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
						break;
					}
					recipe_index_1++;
				}
				else
					recipe_index_1 = 0;
			}
				
			break;
		case s1_positioning:
				if(s1_positioning_f){
					if( s1_delay_pos < 2 * difference_s1){
						s1_delay_pos ++ ;
					}
					else{
						s1_positioning_f = 0;
						s1_delay_pos = 0;
						ss1 = s1_continue;
					}
				
				}
		break;
		case s1_wait:
				if (s1_waiting_f){
				if((wait_i_s1 < n_wait_s1-3) )
				{
					//Waiting. . .
					wait_i_s1 ++;
				}
				else {
					s1_waiting_f = 0;
					wait_i_s1 = 0;
					ss1 = s1_continue;
				}
			}
				if(servo1_p_flag){
					ss1 = s1_pause; 
				}
				
		break;
		case s1_looping:
			previous_state_s1 = s1_looping;
			if(servo1_p_flag){
				ss1 = s1_pause;
			}
			if(s1_looping_f){
			if(s1_i_nloop < n_loop_s1){
				
				if (s1_i_loop < s1_loop_index_e){
					if (s1_loop_wait != 6){
						s1_loop_wait ++;
					}
					else{
						servo1_positions((str1[s1_i_loop] & VALUE_MASK));
						HAL_Delay(200);
						s1_i_loop ++;
						s1_loop_wait = 0;
					}
					
					
				}
				else{
					s1_i_nloop++;
					s1_i_loop = s1_loop_index_b;
				}
			}
		
			else{
				ss1 = s1_continue;
				s1_i_nloop = 0;
			}
		}
		break;
		
	}
		
}


/*State Machine Implementation Servo 2*/

void process_servo2(char * str2){
	int j = 0;
	size_rec_s2 = sizeof(recipe2);
	switch(ss2){
		case s2_pause:
			if(servo2_c_flag)
				ss2 = previous_state_s2;
			
			else
				ss2 = s2_pause;
			
			break;
		case s2_continue:
			previous_state_s2 = s2_continue;
			if(servo2_p_flag)
				ss2 = s2_pause;
			else {
				if(r_s2_flag || l_s2_flag){
					r_s2_flag = 0;
					l_s2_flag = 0;
					servo2_positions(value_s2);
				}
				
				if(recipe_index_2 < size_rec_s2){
					command_s2 = str2[recipe_index_2]  & COMMAND_MASK ;
					if (command_s2 == MOV ){
						value_s2 = str2[recipe_index_2] & VALUE_MASK;
						new_position_s2 = value_s2;
						difference_s2 = abs(new_position_s2 - current_position_s2);
						servo2_positions(value_s2);
						s2_positioning_f = 1;
						ss2 = s2_positioning;
						current_position_s2 = new_position_s2;
						}
					else if (command_s2 == WAIT ){
						s2_waiting_f = 1 ;
						ss2 = s2_wait;
						n_wait_s2 = str2[recipe_index_2] & VALUE_MASK;
						}
					else if (command_s2 == LOOP){
						s2_looping_f = 1;
						n_loop_s2 = str2[recipe_index_2] & VALUE_MASK;
						s2_loop_index_b = recipe_index_2+1;
						j = s2_loop_index_b;
						while((int)str2[++j]!= END_LOOP);
						s2_loop_index_e = j;
						for(int n = s2_loop_index_b; n < s2_loop_index_e; n++){
							if((int)(str2[n]&COMMAND_MASK) == LOOP){
								s2_nested_loop_error = 1;
								s2_looping_f = 0;
								break;
							}
						s2_i_loop = s2_loop_index_b;
						ss2 = s2_looping;
						}
					}
					else if (command_s2 == END_LOOP){
						
					}
					else if (command_s2 == SKIP_NEXT){
						recipe_index_2 ++;
					}
					else if (command_s2 == RECIPE_END){
						HAL_UART_Transmit(&huart2, (uint8_t*)end_rec, strlen(end_rec),100);
						ss2 = s2_pause;
						servo2_c_flag = 0;
						servo2_p_flag = 1;
						break;
					}
					
					else{
						break;
					
					}
					recipe_index_2++;
				}
				else
					recipe_index_2 = 0;
			}
				
			break;
		case s2_positioning:
				if(s2_positioning_f){
					if( s2_delay_pos < 2 * difference_s2){
						s2_delay_pos ++ ;
					}
					else{
						s2_positioning_f = 0;
						s2_delay_pos = 0;
						ss2 = s2_continue;
					}
				
				}
		break;
		case s2_wait:
				if (s2_waiting_f){
				if((wait_i_s2 < n_wait_s2-3) )
				{
					//Waiting. . .
					wait_i_s2 ++;
				}
				else {
					s2_waiting_f = 0;
					wait_i_s2 = 0;
					ss2 = s2_continue;
				}
			}
				if(servo2_p_flag){
					ss2 = s2_pause; 
				}
				
		break;
		case s2_looping:
			previous_state_s2 = s2_looping;
			if(servo2_p_flag){
				ss2 = s2_pause;
			}
			if(s2_looping_f){
			if(s2_i_nloop<n_loop_s2){
				
				if (s2_i_loop != s2_loop_index_e){
					if (s2_loop_wait != 6){
						s2_loop_wait ++;
					}
					else{
						servo2_positions((str2[s2_i_loop] & VALUE_MASK));
						HAL_Delay(200);
						s2_i_loop ++;
						s2_loop_wait = 0;
					}
					
					
				}
				else{
					s2_i_nloop++;
					s2_i_loop = s2_loop_index_b;
				}
			}
			else{
				ss2 = s2_continue;
				s2_i_nloop = 0;
			}
		}
		break;
		
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/