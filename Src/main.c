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
#include "message_parser.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern DMA_HandleTypeDef hdma_usart2_rx;
/* Private variables ---------------------------------------------------------*/
#define RECV_BUFF_SIZE 64
#define SEND_BUFF_SIZE 64

char* bufftr = "Hello!\n\r";
uint8_t buffrec = 0;
uint8_t rxbuff[RECV_BUFF_SIZE];
uint8_t rxbuff_idx = 0;
//__IO ITStatus UartReady = RESET;
__IO ITStatus recv_msg_flag = RESET;


MatLab_Message_TypeDef matlab_msg;

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

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void PWM_Set_Duty(uint16_t);

void DMA_Init(void);
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



  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();

  DMA_Init();

  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN Init */
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);

  /* USER CODE END Init */
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  PWM_Set_Duty(duty);

  /* Receive Data register not empty interrupt */
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* Enable the UART Transmition Complete Interrupt */
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  if (HAL_UART_Receive_DMA(&huart2, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK)
  {

  }
  /* Disable Half Transfer Interrupt */
 // __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);


  /* ADC code */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)bufftr, 8);
  BSP_LED_Toggle(LED6); //TX-blue
  while (1)
  {
	  if (recv_msg_flag == SET) {
		  BSP_LED_Toggle(LED4); // green
		  HAL_UART_Transmit_IT(&huart2, data, data_len); //echo

		  recv_msg_flag = RESET;
	  }


//	  while (UartReady != SET) {
//
//	  }

	  /* Check receive buffer for processing command */


//	  duty += fade;
//	  if (duty == 400 || duty ==0) {
//		  fade=-fade;
//	  }
//	  HAL_Delay(50);
	  /*##-3- Wait for the end of the transfer ###################################*/
//	  while (UartReady != SET)
//	  {
//	  }

	  /* Reset transmission flag */
//	  UartReady = RESET;
//	  BSP_LED_Off(LED4); // green
//	  HAL_Delay(500);


//	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)bufftr, 8);

	  /*##-3- Wait for the end of the transfer ###################################*/
//	  while (UartReady != SET)
//	  {
//	  }
	  /* Reset transmission flag */
//	  UartReady = RESET;
//
//	  HAL_Delay(500);
//	  BSP_LED_Off(LED6); //TX-blue
//	  BSP_LED_Off(LED4); // RX-green
//	  HAL_Delay(500);


	  //HAL_Delay(500);



//	  BSP_LED_Off(LED5); //red

//	  BSP_LED_Off(LED3);//orange


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
	//__HAL_UART_FLUSH_DRREGISTER(&huart2);
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
  /* Set transmission flag: transfer complete */
//  UartReady = SET;
//  if (buffrec[0] == 'a' && buffrec[1] == 'b') {
//	  /* Turn LED4 on: Transfer in reception process is correct */
//	  BSP_LED_On(LED4);
//  }
//	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun
//	if (rxbuff_idx == RECV_BUFF_SIZE) {
//		rxbuff_idx = 0; //buffer overflow
//	}
//	rxbuff[rxbuff_idx] = buffrec;
//	rxbuff_idx++;

	//HAL_UART_Transmit_IT(&huart2, &buffrec, 1);

//	if (buffrec != 0) {
//			  if (buffrec == 'i' && duty < 400) {
//			     duty += fade;
//			  } else if ( buffrec == 'd' && duty > 0) {
//			  		  duty -= fade;
//			  }
//		  PWM_Set_Duty(duty);
//
//			  	  uint8_t buff[2];
//			  	  buff[0] = duty & 0xff;
//			  	  buff[1] = (duty >> 8);
//			  	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)buff, sizeof(buff));
////			  	  UartReady = RESET;
//
//	//		  	  buffrec = 0;
//		  }



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

//	uint8_t err;
//	matlab_msg = MatLab_Message_Parser(data, length, &err);
//	HAL_UART_Transmit_IT(&huart2, &(matlab_msg.len), 1);
//	HAL_UART_Transmit_IT(&huart2, matlab_msg.payload, 2);


//	HAL_UART_Transmit_IT(&huart2, data, length);
}

void PWM_Set_Duty(uint16_t duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle);
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
