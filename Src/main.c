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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "message_util.h"
#include "motor_controller.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define SAMPLES	512 /* 256 real party and 256 imaginary parts */
#define FFT_SIZE	SAMPLES / 2	/* FFT size is always the same size as we have samples, so 256 in our case */

float32_t Input[SAMPLES];
float32_t Output[FFT_SIZE];
float32_t maxValue;
uint32_t maxIndex;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
/* Private variables ---------------------------------------------------------*/
#define RECV_BUFF_SIZE 64
#define SEND_BUFF_SIZE 64

char* bufftr = "Hello!\n\r";
char* resp_msg_err = "invalid_message!\n\r";
uint8_t buffrec = 0;
uint8_t rxbuff[RECV_BUFF_SIZE];
uint8_t rxbuff_idx = 0;
//__IO ITStatus UartReady = RESET;
__IO ITStatus recv_msg_flag = RESET; //declare volatile so that the variable could be changed in interrupt


MatLab_Message_TypeDef matlab_msg;
PID_Algo_Params_TypeDef pid_params;

/* DMA Timeout event structure
 * Note: prevCNDTR initial value must be set to maximum size of DMA buffer!
*/
DMA_Event_t dma_uart_rx = {0,0,DMA_BUF_SIZE};

uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */
uint8_t data[DMA_BUF_SIZE] = {'\0'};    /* Data buffer that contains newly received data */
uint16_t data_len = 0;


/* ADC variables */
uint16_t duty = 0, fade=50;



//ADC value
uint16_t adc_value[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClockHSE_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void PWM_Set_Duty(uint16_t);
void DMA_Init(void);
void HandleMessage(MatLab_Message_TypeDef ml_msg);
void MatLab_Send_Param_Handler(uint8_t *data, uint8_t len);
//void Control_Motor(uint8_t *data, uint8_t len);
void Get_Position();
void Read_ADC();
void Set_PWM(uint8_t *data);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  SystemClockHSE_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Configure LED3, LED4, LED5 and LED6 */
//  BSP_LED_Init(LED3);// led orange
  BSP_LED_Init(LED4); // led green
//  BSP_LED_Init(LED5); //led red
  BSP_LED_Init(LED6); // led blue


  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  /* Receive Data register not empty interrupt */
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* Enable the UART Transmition Complete Interrupt */
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
//  __HAL_UART_FLUSH_DRREGISTER(&huart2);
//  if (HAL_UART_Receive_DMA(&huart2, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK)
//  {
//
//  }

    __HAL_UART_FLUSH_DRREGISTER(&huart3);
    if (HAL_UART_Receive_DMA(&huart3, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK)
    {

    }

//  __HAL_UART_FLUSH_DRREGISTER(&huart1);
//  if (HAL_UART_Receive_DMA(&huart1, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK)
//  {
//
//  }
  /* Disable Half Transfer Interrupt */
 // __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);


  /* ADC code */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_UART_Transmit_IT(&huart3, (uint8_t*)bufftr, 8);

  BSP_LED_Toggle(LED6); //TX-blue
//  Motor_Forward_Drive(100);
  printf("hello swv\n");
  while (1)
  {
	  arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
	  arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);
	  if (recv_msg_flag == SET) {
//		  BSP_LED_Toggle(LED4); // green
		  printf("received message\n");
		  uint8_t err;
		  MatLab_Message_TypeDef msg;
		  err = MatLab_Message_Parser(&msg, data, data_len);

		  if (err == MSG_PARSER_SUCCESS) {
			  HandleMessage(msg);
		  } else {
				MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, &err, 1);

		  }
//		  MATLAB_Message_Handler(data, data_len);
//		  HAL_UART_Transmit(&huart2, data, data_len, 5); //echo
//		  HAL_UART_Transmit(&huart2, (uint8_t*)bufftr, 8, 5);

		  recv_msg_flag = RESET;

	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//
//    /**Configure the main internal regulator output voltage
//    */
//  __HAL_RCC_PWR_CLK_ENABLE();
//
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//    /**Initializes the CPU, AHB and APB busses clocks
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//    /**Initializes the CPU, AHB and APB busses clocks
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//    /**Configure the Systick interrupt time
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//    /**Configure the Systick
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//
//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);




//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//
//  __HAL_RCC_PWR_CLK_ENABLE();
//
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//
//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */



void SystemClockHSE_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
  /* -1- Select HSI as system clock source to allow modification of the PLL configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* -2- Enable HSE Oscillator, select it as PLL source and finally activate the PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application)*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/* DMA Configuration */
void DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_rx.Instance = DMA1_Stream5;
    //hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if(HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart2,hdmarx, hdma_usart2_rx);

    /* DMA Interrupt Configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
//  UartReady = SET;

  /* Turn LED6 on: Transfer in transmission process is correct */
//  BSP_LED_On(LED6);
//  HAL_Delay(50);
//  BSP_LED_Off(LED6);
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* *****************************************************************
	 * DMA with Timeout Event
	 * ****************************************************************
	 */

	uint16_t i, pos, start, length;
	uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(UartHandle->hdmarx);


	/* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
	if (dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE) {
		dma_uart_rx.flag = 0;
		return;
	}

	/* Determine start position in DMA buffer based on previous CNDTR value */
	start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ?
			(DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

	if (dma_uart_rx.flag) /* Timeout event */
	{
		/* Determine new data length based on previous DMA_CNDTR value:
		 *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
		 *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
		 */
		length =
				(dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ?
						(dma_uart_rx.prevCNDTR - currCNDTR) :
						(DMA_BUF_SIZE - currCNDTR);
		dma_uart_rx.prevCNDTR = currCNDTR;
		dma_uart_rx.flag = 0;
	} else /* DMA Rx Complete event */
	{
		length = DMA_BUF_SIZE - start;
		dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
	}

	/* Copy and Process new data */
	for (i = 0, pos = start; i < length; ++i, ++pos) {
		data[i] = dma_rx_buf[pos];
	}
	data_len = length;
	recv_msg_flag = SET;
	HAL_UART_DMAStop(&huart3);
//	HAL_UART_DMAStop(&huart1);
	dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
	HAL_UART_Receive_DMA(&huart3, dma_rx_buf, DMA_BUF_SIZE);
//	HAL_UART_Receive_DMA(&huart1, dma_rx_buf, DMA_BUF_SIZE);
	__HAL_DMA_SET_COUNTER(UartHandle->hdmarx, DMA_BUF_SIZE);



//	uint8_t err;
//	matlab_msg = MatLab_Message_Parser(data, length, &err);
//	HAL_UART_Transmit_IT(&huart2, &(matlab_msg.len), 1);
//	HAL_UART_Transmit_IT(&huart2, matlab_msg.payload, 2);


//	HAL_UART_Transmit_IT(&huart2, data, length);
}

//void PWM_Set_Duty(uint16_t duty_cycle)
//{
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle);
//}


void HandleMessage(MatLab_Message_TypeDef ml_msg) {
	switch (ml_msg.cmd_type) {
	case MATLAB_CMD_SEND_PARAM:
		/* TODO - Call SendParam function */
		MatLab_Send_Param_Handler(ml_msg.payload.data, ml_msg.payload.len);
		break;
	case MATLAB_CMD_GET_POS:
		/* TODO - Call GetPos function */
		Get_Position();
		break;
	case MATLAB_CMD_READ_ADC:
		Read_ADC();
		break;
	case MATLAB_CMD_RESTART:
		/* TODO - Call Soft reset function */
		break;
	case MATLAB_CMD_SET_PWM:
		Set_PWM(ml_msg.payload.data);
		break;

		break;
	default:
		MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, (uint8_t *)MSG_REPLY_NAK_CMD_INVALID, 1);
	}
}

void Get_Position()
{
	uint8_t pos = 0;

//	uint8_t pin_d_1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

//	uint16_t port_value = HAL_GPIO_ReadPort(GPIOD);
//	port_value &= 0x00FF;
	pos = Read_Encoder_Position();
	MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, &pos, 1);
//	MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, &pin_d_1, 1);
//	MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, pin_values, 8);
}

void Read_ADC()
{
	uint8_t buff[6];
	uint16_t adc1_in0, adc1_in1, adc1_in11;

	HAL_ADC_Stop_DMA(&hadc1);
	adc1_in0 = adc_value[0];
	adc1_in1 = adc_value[1];
	adc1_in11 = adc_value[2];

	buff[0] = (uint8_t) (adc1_in0 & 0x00ff);
	buff[1] = (uint8_t) (adc1_in0 >> 8);
	buff[2] = (uint8_t) (adc1_in1 & 0x00ff);
	buff[3] = (uint8_t) (adc1_in1 >> 8);
	buff[4] = (uint8_t) (adc1_in11 & 0x00ff);
	buff[5] = (uint8_t) (adc1_in11 >> 8);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, 3);

	MatLab_Send_Response((uint8_t) MATLAB_CMD_REPLY, buff, 6);
}

void Set_PWM(uint8_t *data)
{
	Motor_Forward_Drive((uint16_t) data[0]);
	PWM_Set_Duty((uint16_t) data[0]);
	MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, (uint8_t *)MSG_REPLY_ACK, 1);
}

void MatLab_Send_Param_Handler(uint8_t *data, uint8_t len)
{
	Control_Motor(data, len);
	/****************************************/

	/* Respond result by calling Send_Response_Message*/

	MatLab_Send_Response((uint8_t)MATLAB_CMD_REPLY, (uint8_t *)MSG_REPLY_ACK, 1);
}


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
