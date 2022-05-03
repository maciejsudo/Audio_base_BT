/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"


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
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* Definitions for DSP_I2S_Task */
osThreadId_t DSP_I2S_TaskHandle;
const osThreadAttr_t DSP_I2S_Task_attributes = {
  .name = "DSP_I2S_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#define NUMBER_F 512
#define NUMBER_U16 2048




//4 * 5 = 20
float iir_statesleft[20];
float iir_statesright[20];

uint16_t rxBuf[NUMBER_U16*2];
uint16_t txBuf[NUMBER_U16*2];

float in_bufferl [NUMBER_F*2];
float in_bufferr [NUMBER_F*2];
float out_bufferl [NUMBER_F*2];
float outbufferr [NUMBER_F*2];


arm_biquad_casd_df1_inst_f32 iir_settingsleft, iir_settingsright;
uint8_t callback = 0;


float32_t param_coeffs[25];
int8_t everyG[5];
uint16_t everyF[5] = {1000,1000,1000,1000,1000}; //init values!
float everyBW[5] = {1,1,1,1,1}; /// from 1 - 100 in percents of octave!

uint8_t IsChange = 5; //defaultowo jest zmiana w stanach 0,1,2,3,4
uint8_t volumeR =24; //MAX value
uint8_t volumeL =24; //MAX value



////////////////////////////////////////////////////////////
//timer for diode blinking (timer for UART transmitt optionally):
uint8_t timer_state=0;
////////////////////////////////////////////////////////////
//UART variables:
uint8_t receive_flag=0;

//RX:
char Received[6];
char sign;
uint16_t value;


//projecting coeffs variables:
float al, A, Ap1, Am1, sA, w0, bw, Q, coss, a0;





/////////////////////////////////////////////////////////////




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
void Start_DSP_I2S_Task(void *argument);
void Start_UART_Task(void *argument);

/* USER CODE BEGIN PFP */


void calc_shelf_coeffs(int8_t dBGain, uint8_t band_num, uint16_t f0, float BW)
{
	w0 = M_PI*f0/24000.0; // omega 0 dla f center
	coss = cos(w0);



	Q = pow(2.0, BW);
	Q = sqrt(Q)/(Q-1); //jezeli jest to w formie Q
	al = sin(w0)/(2.0*Q);

	A = ((float)dBGain)*0.025; // gain (NOT dB)
	A = pow(10.0, A);

	Ap1 = A+1.0;
	Am1 = A-1.0;
	sA = sqrt(A);


	if(band_num == 3) //low - shelf
	{

	a0 = Ap1+Am1*coss+2 * sA*al;
	param_coeffs[2 +(5*band_num)] = (A*(Ap1-Am1*coss-2*sA*al)/a0); //b2
	param_coeffs[1 +(5*band_num)] = 2.0*(A*(Am1-Ap1*coss)/a0); //b1
	param_coeffs[0 +(5*band_num)] = (A*(Ap1-Am1*coss+2*sA*al)/a0); //b0
	param_coeffs[4 +(5*band_num)] = -((Ap1+Am1*coss-2*sA*al)/a0); //a2
	param_coeffs[3 +(5*band_num)] = -2.0*(-(Am1+Ap1*coss)/a0); //a1
	}
	if(band_num == 4) //high - shelf
	{

	a0 = Ap1-Am1*coss+2 * sA*al;
	param_coeffs[2 +(5*band_num)] = (A*(Ap1+Am1*coss-2*sA*al)/a0); //b2
	param_coeffs[1 +(5*band_num)] = 2.0*(-A*(Am1+Ap1*coss)/a0); //b1
	param_coeffs[0 +(5*band_num)] = (A*(Ap1+Am1*coss+2*sA*al)/a0); //b0
	param_coeffs[4 +(5*band_num)] = -((Ap1-Am1*coss-2*sA*al)/a0); //a2
	param_coeffs[3 +(5*band_num)] = -2.0*((Am1-Ap1*coss)/a0); //a1

	}


}

void calc_peakEQ_coeffs(int8_t dBGain, uint8_t band_num, uint16_t f0, float BW)
{
	//f0 = 20.0*pow(2.0, al/12.0); // peak freq
	w0 = M_PI*f0/24000.0; // omega 0 dla f center
	coss = cos(w0);

	A = ((float)dBGain)*0.025; // gain (NOT dB)
	A = pow(10.0, A);

	bw = 0.1 + (float)BW*0.05; // bandwith in octaves

	Q = pow(2.0,bw);
	Q = sqrt(Q)/(Q-1);

	al = sin(w0)/(2.0*Q);

	a0 = 1.0+al/A;

	param_coeffs[2 +(5*band_num)]= (1.0-al*A)/a0; //b2
	param_coeffs[1 +(5*band_num)] = (-2*coss/a0); //b1
	param_coeffs[0 +(5*band_num)] = (1.0+al*A)/a0; //b0
	param_coeffs[4 +(5*band_num)] = -(1.0-al/A)/a0; //a2
	param_coeffs[3 +(5*band_num)] = -(-2*coss/a0); //a1

}




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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  ///* FPU initialization */
  SCB->CPACR |= ((3 << 10*2)|(3 << 11*2));

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */



// setting default peak filters:
  calc_peakEQ_coeffs(0, 0, 1000, 1);
  calc_peakEQ_coeffs(0, 1, 1000, 1);
  calc_peakEQ_coeffs(0, 2, 1000, 1);
// setting default shelf filters:
  calc_shelf_coeffs(0, 3, 500, 1.0);
  calc_shelf_coeffs(0, 4, 500, 1.0);
// ARM Biquad Direct form 2 structure init:

  arm_biquad_cascade_df1_init_f32(&iir_settingsleft, 5, &param_coeffs[0], &iir_statesleft[0]);
  arm_biquad_cascade_df1_init_f32(&iir_settingsright, 5, &param_coeffs[0], &iir_statesright[0]);


  //start i2s with 2048 samples transmission => 4096*u16 words
  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, txBuf, rxBuf, NUMBER_U16);

  //optionally//timer for plot transmission:
  HAL_TIM_Base_Start_IT(&htim10);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DSP_I2S_Task */
  DSP_I2S_TaskHandle = osThreadNew(Start_DSP_I2S_Task, NULL, &DSP_I2S_Task_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(Start_UART_Task, NULL, &UART_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 33599;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TASK_dsp_state_GPIO_Port, TASK_dsp_state_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TASK_main_state_Pin|TASK_uart_state_Pin|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TASK_dsp_state_Pin */
  GPIO_InitStruct.Pin = TASK_dsp_state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TASK_dsp_state_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TASK_main_state_Pin TASK_uart_state_Pin LD4_Pin LD3_Pin
                           LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = TASK_main_state_Pin|TASK_uart_state_Pin|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	callback = 1;
}
HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	callback = 2;
}


HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	receive_flag=1;
	HAL_UART_Receive_IT(&huart2, &Received, 6);
}







/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_DSP_I2S_Task */
/**
* @brief Function implementing the DSP_I2S_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DSP_I2S_Task */
void Start_DSP_I2S_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	  int offset_r_ptr;
	  int offset_w_ptr, w_ptr;
  /* Infinite loop */
  for(;;)
  {
	  /////-I2S-transmission part///////////////////////////////////////////////////////////////
  	  if (callback != 0) {

  		  if (callback == 1)
  		  {
  			  offset_r_ptr = 0;
  			  offset_w_ptr = 0;
  			  w_ptr = 0;
  		  }

  		  else if (callback == 2) {
			  offset_r_ptr = NUMBER_U16;
			  offset_w_ptr = NUMBER_F;
			  w_ptr = NUMBER_F;
  		  }


  		  for (int i=offset_r_ptr; i<offset_r_ptr+NUMBER_U16; i=i+4)
  		  {
			  in_bufferl[w_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
			  in_bufferr[w_ptr] = (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
			  w_ptr++;
  		  }


  		  arm_biquad_cascade_df1_f32(&iir_settingsleft, &in_bufferl[offset_w_ptr], &out_bufferl[offset_w_ptr], NUMBER_F);
  		  arm_biquad_cascade_df1_f32(&iir_settingsright, &in_bufferr[offset_w_ptr], &outbufferr[offset_w_ptr], NUMBER_F);

  		  w_ptr = offset_w_ptr;

  		  for (int i=offset_r_ptr; i<offset_r_ptr+NUMBER_U16; i=i+4)
  		  {
			  txBuf[i] =     (((int)(out_bufferl[w_ptr] * (volumeL * 0.0416)))>>16)&0xFFFF;
			  txBuf[i+1] =  ((int)(out_bufferl[w_ptr] * (volumeL * 0.0416)))&0xFFFF;

			  txBuf[i+2] =  (((int)(outbufferr[w_ptr] * (volumeR * 0.0416)))>>16)&0xFFFF;
			  txBuf[i+3] =  ((int)(outbufferr[w_ptr] * (volumeR * 0.0416)))&0xFFFF;
			  w_ptr++;
  		  }

  		  callback = 0;

  	  }


  	  //checking if there's need to update  and calculating coefficients:
	  if(IsChange ==0 || IsChange ==1|| IsChange ==2) //set coeffs according to band
	  {
	  calc_peakEQ_coeffs(everyG[IsChange],IsChange, everyF[IsChange], everyBW[IsChange]);

	  IsChange = 5;
	  }
	  else if(IsChange ==3 || IsChange ==4 )
	  {
	  calc_shelf_coeffs(everyG[IsChange], IsChange, everyF[IsChange], everyBW[IsChange]);
	  IsChange = 5;
	  }
	  else if(IsChange == 6)// clear settings
	  {
		  calc_peakEQ_coeffs(0, 0, 1000, 1);
		  calc_peakEQ_coeffs(0, 1, 1000, 1);
		  calc_peakEQ_coeffs(0, 2, 1000, 1);
		  calc_shelf_coeffs(0, 3, 1000, 1);
		  calc_shelf_coeffs(0, 4, 1000, 1);
		  IsChange =5;
	  }


    osDelay(1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_UART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_Task */
void Start_UART_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_Task */
	HAL_UART_Receive_IT(&huart2, &Received, 6);

  /* Infinite loop */
  for(;;)
  {

	  ///UART part//////////////////////////////////////////////////////////////////////////////

  if(timer_state==1)
  {
	  	 //blinking diode - work signalization;
		 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		 timer_state=0;
  }

  if(receive_flag==1)// RECEIVE PART:
  {
	  //message decoding
	  sign = Received[0];
	  Received[0] = '0';
	  sscanf(Received,"%d",&value);

	  switch(sign)
	  {
//PEAK FILTERS SET:
	  case 'a':
		  everyF[0] = value;					//frequency 0
		  IsChange =0;
		  break;
	  case 'b':
		  everyBW[0] =  0.01 *value;			//BW in octaves 0
		  IsChange =0;
		  break;
	  case 'c':
		  everyG[0] = value-12;					//gain value 0
		  IsChange =0;
		  break;
	  case 'd':
		  everyF[1] = value;					//frequency 1
		  IsChange =1;
		  break;
	  case 'e':
		  everyBW[1] =  0.01 *value;			//BW in octaves 1
		  IsChange =1;
		  break;
	  case 'f':
		  everyG[1] = value-12;					//gain value 1
		  IsChange =1;
		  break;
	  case 'g':
		  everyF[2] = value;					//frequency 2
		  IsChange =2;
		  break;
	  case 'h':
		  everyBW[2] =  0.01 *value;			//BW in octaves 2
		  IsChange =2;
		  break;
	  case 'i':
		  everyG[2] = value-12;					//gain value 2
		  IsChange =2;
		  break;

//SHELF FILTERS SET:

	  case 'j':
		  everyF[3] = value;					//freqency (slope center) 3
		  IsChange =3;
		  break;
	  case 'k':
		  everyBW[3] =  0.01 *value;			// S - parameter 3
		  IsChange =3;
		  break;
	  case 'l':
		  everyG[3] = value-12;					//gain value 3
		  IsChange =3;
		  break;

	  case 'm':
		  everyF[4] = value;					//freqency (slope center) 4
		  IsChange =4;
		  break;
	  case 'n':
		  everyBW[4] =  0.01 *value;			//S - parameter 4
		  IsChange =4;
		  break;
	  case 'o':
		  everyG[4] = value-12;					//gain value 4
		  IsChange =4;
		  break;




//VOLUME SET:
	  case 'L':
		  volumeL = value;						//left channel volume level
		  break;
	  case 'R':
		  volumeR = value;						//right channel volume level
		  break;

//RESET
	  case 'z':
		  IsChange =6;							//set all settings to default
		  break;

		  break;

	  }
// message received RED LED signalisation:
	  	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	  	  osDelay(250);
	  	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	  	  receive_flag=0;

  }
    osDelay(1);
  }
  //osThreadTerminate(NULL);
  /* USER CODE END Start_UART_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance == TIM10){ // Jeżeli przerwanie pochodzi od timera 10
	timer_state=1;
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

