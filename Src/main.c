/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComDMA/Src/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-April-2015
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          DMA transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComDMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Size of Trasmission buffer */
#define UART_TXBUFFERSIZE          (COUNTOF(UART_TxBuffer) - 1)
/* Size of Reception buffer */
#define UART_RXBUFFERSIZE          TXBUFFERSIZE

/* Size of buffer */
#define BUFFERSIZE                 (COUNTOF(aTxBufferRead) - 1)

#define EXTERNAL_AD0   0
#define EXTERNAL_AD1   1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handle declaration */
ADC_HandleTypeDef  AdcHandle;

/* SPI handle declaration */
SPI_HandleTypeDef  SpiHandle;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;

/* Variable containing ADC conversions data */
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO ITStatus UartReady = SET;
__IO uint32_t VirtualUserButtonStatus = 0;  /* set to 1 after User set a button  */

/* Buffer used for transmission */
uint8_t UART_TxBuffer[200];

/* Buffer used for reception */
//uint8_t UART_RxBuffer[UART_RXBUFFERSIZE];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void ADC_Config(void);
void LedInit(void);
void SpiInit(void);
void UartInit(UART_HandleTypeDef *UartHandle,USART_TypeDef *UARTx, uint32_t BaudRate);
void ExternalAdcEnable(uint8_t id);
void ExternalAdcInit(EX_ADC_Parameter *para);
void ExAdcReadData(uint32_t *data);
void TimerInit();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* data struct */
  struct {
      uint16_t   VDD18_HDMI_mV;
      uint16_t   VDD11_CORE_mV;
      uint16_t   VDD15_mV;
      uint16_t   VDD18_mV;
      uint16_t   VDD33_mV;
	    uint16_t   VCC_5V_mV;
	    uint16_t   VIN_28V_mV;
	    uint16_t   VDD12_SDI_mV;
	    uint16_t   VTT_mV;//0.75V
      int32_t    TEMP_STM32_DegreeCelsius ;
	    float      TEMP_ADC_Exteranl;
	    int32_t    TEMP_CPU;
   }DataCollected;
	
	 EX_ADC_Parameter ExAdcPara;
	 uint32_t ExAdcData;
	/* STM32F3xx HAL library initialization:
       - Configure the Flash prefetch
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();
	LedInit();	
	
  BSP_LED_On(LED3);
	HAL_Delay(1000);
	
  //AdcInit(&AdcHandle,&sConfig,ADC1,ADC_CHANNEL_3,ADC_REGULAR_RANK_1,CALIBRATION_ENABLE);
	ADC_Config();
	
	 /* Run the ADC calibration in single-ended mode */  
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  SpiInit();
  ExternalAdcEnable(EXTERNAL_AD1);
  ExternalAdcInit(&ExAdcPara);	
	
  /* ### - 4 - Start conversion in DMA mode ################################# */
  
	if (HAL_ADC_Start_DMA(&AdcHandle,(uint32_t *)aADCxConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  
  UartInit(&UartHandle,USART1,115200);


  /* Infinite loop */
  while (1)
  {
		/* Start ADC conversion */
    /* Since sequencer is enabled in discontinuous mode, this will perform    */
    /* the conversion of the next rank in sequencer.                          */
    /* Note: For this example, conversion is triggered by software start,     */
    /*       therefore "HAL_ADC_Start()" must be called for each conversion.  */
    /*       Since DMA transfer has been initiated previously by function     */
    /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer      */
    /*       active.                                                          */
    HAL_ADC_Start(&AdcHandle);
    HAL_Delay(100); 
    /* Wait for conversion completion before conditional check hereafter */
    //HAL_ADC_PollForConversion(&AdcHandle, 1);

		
		if (ubSequenceCompleted != RESET)
    {

      //BSP_LED_On(LED2);
      
      /* Computation of ADC conversions raw data to physical values */
      /* Note: ADC results are transferred into array "aADCxConvertedValues"  */
      /*       in the order of their rank in ADC sequencer.                   */
			DataCollected.TEMP_STM32_DegreeCelsius = COMPUTATION_TEMPERATURE(aADCxConvertedData[0]);
			DataCollected.VDD18_HDMI_mV = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[1]);
      DataCollected.VDD11_CORE_mV  = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[2]); 
			DataCollected.VIN_28V_mV    = 11*COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[3]);
			DataCollected.VCC_5V_mV     = 2*COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[4]);
			DataCollected.VDD18_mV      = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[5]);
			DataCollected.VDD15_mV      = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[6]);
			DataCollected.VDD33_mV      = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[7]);
			DataCollected.VTT_mV        = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[8]);
			DataCollected.VDD12_SDI_mV  = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedData[9]);
			
			ExAdcReadData(&ExAdcData);
			DataCollected.TEMP_ADC_Exteranl = ((-(int32_t)ExAdcData * REF_RESISTOR/ADS1220_RANGE)-RTD_VALUE)/(RTD_VALUE*TEMP_COEFFICIENT);
			

      sprintf((char *)UART_TxBuffer,"T_STM32,%4d, 1v1,%4d, 1v8_HDMI,%4d, 28V,%4d, 5V,%4d, 1v8,%4d, 1v5,%4d, 3v3,%4d, 1v2,%4d, VTT,%4d, Ex_T,%4.2f\n",
				                         DataCollected.TEMP_STM32_DegreeCelsius,DataCollected.VDD11_CORE_mV,DataCollected.VDD18_HDMI_mV,
			                           DataCollected.VIN_28V_mV,DataCollected.VCC_5V_mV,DataCollected.VDD18_mV,DataCollected.VDD15_mV,
			                           DataCollected.VDD33_mV,DataCollected.VDD12_SDI_mV,DataCollected.VTT_mV,DataCollected.TEMP_ADC_Exteranl);
	    //if(HAL_UART_Transmit(&UartHandle, (uint8_t*)UART_TxBuffer, UART_TXBUFFERSIZE, 500)!= HAL_OK)
      //{
      //  Error_Handler();   
      //}	
			
      if(UartReady == SET)		
      {
				UartReady = RESET;
				if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)UART_TxBuffer, UART_TXBUFFERSIZE)!= HAL_OK)
        {
          Error_Handler();
        }
				
			}
			
			//while (UartReady != SET)
      //{
				  /* Reset transmission flag */
       // UartReady = RESET;
      //}
			
      ubSequenceCompleted = RESET;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
		  HAL_Delay(1000);
    }
		

  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = RCC_PLL_MUL16 (16)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* HSI Oscillator already ON after system reset, activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_12)
  {
    VirtualUserButtonStatus = 1;
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
    /* Error if LED3 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(LED3); 
    HAL_Delay(200); 
  }  
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif



static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  
  
  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADC1;

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* Synchronous clock mode, input ADC clock divided by 2*/
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = ENABLE;                        /* Sequencer enabled (ADC conversion on several channels, successively, following settings below) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
	//AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  AdcHandle.Init.NbrOfConversion       = 10;                             /* Sequencer of regular group will convert the 3 first ranks: rank1, rank2, rank3 */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Sequencer of regular group will convert ranks one by one, at each conversion trig */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Trig of conversion start done manually by software, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* ADC DMA continuous request to match with DMA circular mode */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints.                                   */
  /*       For example, sampling time of temperature sensor must be higher    */
  /*       than 2.2us. Refer to device datasheet for min/typ/max values.      */
  sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel      = ADC_CHANNEL_3;//Pin PA2,VDD18_HDMI
  sConfig.Rank         = ADC_REGULAR_RANK_2;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel      = ADC_CHANNEL_2;//VDD11_CORE,Pin PA1
  sConfig.Rank         = ADC_REGULAR_RANK_3;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
	
  sConfig.Channel      = ADC_CHANNEL_6;//VIN 28V ,pin PC0
  sConfig.Rank         = ADC_REGULAR_RANK_4;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  } 
	
	sConfig.Channel      = ADC_CHANNEL_7;//VCC 5V ,Pin PC1
  sConfig.Rank         = ADC_REGULAR_RANK_5;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  } 
	
	sConfig.Channel      = ADC_CHANNEL_8;//VDD18  ,Pin PC2
  sConfig.Rank         = ADC_REGULAR_RANK_6;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
	
	sConfig.Channel      = ADC_CHANNEL_9;//VDD15  ,Pin PC3
  sConfig.Rank         = ADC_REGULAR_RANK_7;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }	
	
	sConfig.Channel      = ADC_CHANNEL_1;//VDD33  ,Pin PA0
  sConfig.Rank         = ADC_REGULAR_RANK_8;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }	
	
	sConfig.Channel      = ADC_CHANNEL_12;//VTT 0.75V  ,Pin PB1
  sConfig.Rank         = ADC_REGULAR_RANK_9;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }		
	
	sConfig.Channel      = ADC_CHANNEL_4;//VDD12_SDI  ,Pin PA3
  sConfig.Rank         = ADC_REGULAR_RANK_10;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }		
}


void LedInit(void)
{
	  GPIO_InitTypeDef  GPIO_InitStruct; 
	/* Configure LED */
  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* -2- Configure IO in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	/* configure LED over */
}


void UartInit(UART_HandleTypeDef *UartHandle,USART_TypeDef *UARTx, uint32_t BaudRate)
{
	  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle->Instance        = UARTx;

  UartHandle->Init.BaudRate   = BaudRate;
  UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle->Init.StopBits   = UART_STOPBITS_1;
  UartHandle->Init.Parity     = UART_PARITY_NONE;
  UartHandle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle->Init.Mode       = UART_MODE_TX_RX;
  UartHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}
/**
  * @}
  */
void SpiInit()
{
	

  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPI1;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;


  SpiHandle.Init.Mode = SPI_MODE_MASTER;


  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }	
}


void ExternalAdcEnable(uint8_t id)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  //__HAL_RCC_GPIOC_CLK_ENABLE();

  /* -2- Configure IO in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	if(id==EXTERNAL_AD1)//SPI bus switch to AD1
	{
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //AD1 enable
		GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // AD0 disable
	}else if(id==EXTERNAL_AD0)//SPI bus switch to AD0
	{
		GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // AD0 enable
		GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //AD1 disable
	}
}
void ExternalAdcInit(EX_ADC_Parameter *para)
{
	//uint8_t TxBuffer[] = {0x43,0x00,0x00,0x46,0x80,0x00};
	uint8_t RxBuffer[] = {0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t aTxBufferRead[] = {0x23,0x00,0x00,0x00,0x0,0x0};
	para->command = ADS1220_CMD_WRITE_ALL_REG;
	para->reg02   = ADS1220_REG02_VREF_EX_REFP|ADS1220_REG02_IDAC_1000UA;
	para->reg03   = ADS1220_REG03_I1MUX_IDAC1_AIN3;
	
	switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)para, (uint8_t *)RxBuffer, 5, 100))
  {
    case HAL_OK:
      /* Communication is completed ___________________________________________ */
		
      /* Turn LED3 on: Transfer in transmission/Reception process is correct */
      //BSP_LED_On(LED3);
      break;

    case HAL_TIMEOUT:
      /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
      /* Call Timeout Handler */
      Error_Handler();
      break;
    default:
      break;
  }	
	switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBufferRead, (uint8_t *)RxBuffer, 5, 100))
  {
    case HAL_OK:
      /* Communication is completed ___________________________________________ */
		
      /* Turn LED3 on: Transfer in transmission/Reception process is correct */
      //BSP_LED_On(LED3);
      break;

    case HAL_TIMEOUT:
      /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
      /* Call Timeout Handler */
      Error_Handler();
      break;
    default:
      break;
  }	
}

void ExAdcReadData(uint32_t *data)
{
	uint8_t TxBuffer[] = {ADS1220_CMD_READ_DATA,0x00,0x00,0x00,0x00};
	uint8_t RxBuffer[] = {0x00,0x00,0x00,0x00,0x00};
	uint8_t TxBufferStart[]={ADS1220_CMD_START,0x00};
	switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)TxBufferStart, (uint8_t *)RxBuffer, 2, 100))
  {
    case HAL_OK:
      /* Communication is completed ___________________________________________ */
      // *data= (int)(0xff000000|(RxBuffer[1]<<16)|(RxBuffer[2]<<8)|(RxBuffer[3]))*1650/8388608;
		  //*data= 0xff000000|(RxBuffer[1]<<16)|(RxBuffer[2]<<8)|(RxBuffer[3]);
      /* Turn LED3 on: Transfer in transmission/Reception process is correct */
      //BSP_LED_On(LED3);
      break;

    case HAL_TIMEOUT:
      /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
      /* Call Timeout Handler */
      Error_Handler();
      break;
    default:
      break;
  }
	
	switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)TxBuffer, (uint8_t *)RxBuffer, 4, 100))
  {
    case HAL_OK:
      /* Communication is completed ___________________________________________ */
      // *data= (int)(0xff000000|(RxBuffer[1]<<16)|(RxBuffer[2]<<8)|(RxBuffer[3]))*1650/8388608;
		  *data= 0xff000000|(RxBuffer[1]<<16)|(RxBuffer[2]<<8)|(RxBuffer[3]);
      /* Turn LED3 on: Transfer in transmission/Reception process is correct */
      //BSP_LED_On(LED3);
      break;

    case HAL_TIMEOUT:
      /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
      /* Call Timeout Handler */
      Error_Handler();
      break;
    default:
      break;
  }
}

void TimerInit(void)
{
	  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM2 input clock (TIM2CLK)  is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1.
      TIM2CLK = 2 * PCLK1
      PCLK1 = HCLK / 2
      => TIM2CLK = HCLK = SystemCoreClock (Hz)
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f3xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIM2;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 10000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
