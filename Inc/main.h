/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComDMA/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-April-2015
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_nucleo_32.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor ADCx instance used and associated
   resources */
   
/* Definition for ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC12_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define DMAx_CHANNELx_CLK_ENABLE()      __HAL_RCC_DMA1_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC12_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC12_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
//#define ADCx_CHANNEL_PIN_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
//#define ADCx_CHANNEL_PIN                GPIO_PIN_4
//#define ADCx_CHANNEL_GPIO_PORT          GPIOA
#define ADCx_CHANNEL_PIN_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNEL_PIN                GPIO_PIN_2
#define ADCx_CHANNEL_GPIO_PORT          GPIOA
/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_12

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel4
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Channel5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel4_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Channel5_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler




/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1


/* Definition for TIMx clock resources */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()


/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM2_IRQn
#define TIMx_IRQHandler                TIM2_IRQHandler

/* Private define ------------------------------------------------------------*/
#define VDD_APPLI                      ((uint32_t)3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t)4095)   /* Max value for a full range of 12 bits */

/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */
/* For more accurate values, device should be calibrated on offset and slope  */
/* for application temperature range.                                         */
#define INTERNAL_TEMPSENSOR_V25        ((uint32_t)1430)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((uint32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t)0x1FFFF7B8))  /* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t)0x1FFFF7C2))  /* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */

/* Internal voltage reference */
#define VREFINT_CAL       ((uint16_t*) ((uint32_t)0x1FFFF7BA))  /* Internal temperature sensor, parameter VREFINT_CAL: Raw data acquired at a temperature of 30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
                                                                /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor calibration values done
  *         in production.
  *         Computation formula:
  *         Temperature = (TS_ADC_DATA - TS_CAL1)/Avg_Slope + 30
  *         with TS_ADC_DATA = temperature sensor raw data measured by ADC
  *              Avg_Slope = (TS_CAL2 - TS_CAL1) / (110 - 30)
  *              TS_CAL1 = TS_ADC_DATA @30degC (calibrated in factory, vdda 3.3V)
  *              TS_CAL2 = TS_ADC_DATA @110degC (calibrated in factory, vdda 3.3V)
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE(TS_ADC_DATA)                                   \
  (((( ((int32_t)(TS_ADC_DATA) - (int32_t) *TEMP30_CAL_ADDR)                   \
     ) * (int32_t)(110 - 30)                                                   \
    ) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + 30                                                                      \
  )

/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor standard parameters (refer
  *         to device datasheet).
  *         Computation formula:
  *         Temperature = (VTS - V25)/Avg_Slope + 25
  *         with VTS = temperature sensor voltage
  *              Avg_Slope = temperature sensor slope (unit: uV/DegCelsius)
  *              V25 = temperature sensor @25degC and Vdda 3.3V (unit: mV)
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE_STD_PARAMS(TS_ADC_DATA)                        \
  ((((INTERNAL_TEMPSENSOR_V25 - (((TS_ADC_DATA) * VDD_APPLI) / RANGE_12BITS)   \
     ) * 1000                                                                  \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )

/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)                        \
  ( (ADC_DATA) * VDD_APPLI / RANGE_12BITS)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

  /* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  10)   /* Size of array aADCxConvertedData[] */
#define CALIBRATION_ENABLE ((uint8_t)  1)
#define CALIBRATION_DISABLE ((uint8_t)  0)
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
	
/*  external ADC ADS1220 definetions -----------------------------------------*/
typedef struct 
{
	uint8_t command; 
	uint8_t reg00;
	uint8_t reg01;
	uint8_t reg02;
	uint8_t reg03;	
}EX_ADC_Parameter;
#define TEMP_COEFFICIENT           (0.003850)    //ppm/K
#define RTD_VALUE        100
#define REF_RESISTOR     ((uint32_t)1650)    //1.65k
#define ADS1220_RANGE    ((uint32_t)8388608) //24bit adc ,bipolar
#define ADS1220_CMD_RESET         0x06
#define ADS1220_CMD_START         0x08
#define ADS1220_CMD_PWR_DOWN      0x02
#define ADS1220_CMD_READ_DATA     0x10
#define ADS1220_CMD_READ_ALL_REG  0x23 
#define ADS1220_CMD_WRITE_ALL_REG 0x43

#define ADS1220_REG00_MUX_MASK    4
#define ADS1220_REG00_MUX_AIN0_AIN1    ((uint8_t) 0<<ADS1220_REG00_MUX_MASK)
#define ADS1220_REG00_GAIN_MASK   1
#define ADS1220_REG00_GAIN_1      ((uint8_t) 0<<#define ADS1220_REG00_GAIN_MASK)
#define ADS1220_REG00_PGA_ENABLE   (uint8_t 0)
#define ADS1220_REG00_PGA_DISABLE  (uint8_t 1)


#define ADS1220_REG01_DR_MASK    5
#define ADS1220_REG01_DR_NORMAL_20SPS  ((uint8_t) 0<<ADS1220_REG01_DR_MASK)
#define ADS1220_REG01_DR_NORMAL_45SPS  ((uint8_t) 1<<ADS1220_REG01_DR_MASK)
#define ADS1220_REG01_MODE_MASK  3
#define ADS1220_REG01_MODE_NORMAL      ((uint8_t) 0<<ADS1220_REG01_MODE_MASK) //256Khz modulator clock
#define ADS1220_REG01_CM_MASK    2
#define ADS1220_REG01_CM_SINGLE_SHOT   ((uint8_t) 0<<ADS1220_REG01_CM_MASK)
#define ADS1220_REG01_CM_CONTINUOUS    ((uint8_t) 1<<ADS1220_REG01_CM_MASK)
#define ADS1220_REG01_TS_MASK    1
#define ADS1220_REG01_TS_ENABLE         ((uint8_t) 1<<ADS1220_REG01_TS_MASK)
#define ADS1220_REG01_TS_DISABLE        ((uint8_t) 0<<ADS1220_REG01_TS_MASK)
#define ADS1220_REG01_BCS_OFF           uint8_t 0

#define ADS1220_REG02_VREF_MASK  6
#define ADS1220_REG02_VREF_INTERNAL     ((uint8_t) 0<<ADS1220_REG02_VREF_MASK)
#define ADS1220_REG02_VREF_EX_REFP      ((uint8_t) 1<<ADS1220_REG02_VREF_MASK)
#define ADS1220_REG02_50_60_MASK 4
#define ADS1220_REG02_50_60_NO   0      ((uint8_t) 0<<ADS1220_REG02_50_60_MASK)
#define ADS1220_REG02_PSW_MASK   3   
#define ADS1220_REG02_PSW_ALWAYS_OPEN   ((uint8_t)0x00<<ADS1220_REG02_PSW_MASK)
#define ADS1220_REG02_IDAC_1000UA       (uint8_t)0x06
#define ADS1220_REG02_IDAC_1500UA       (uint8_t)0x07

#define ADS1220_REG03_I1MUX_MASK  5
#define ADS1220_REG03_I1MUX_IDAC1_DISABLE   ((uint8_t)0x00<<ADS1220_REG03_I1MUX_MASK)
#define ADS1220_REG03_I1MUX_IDAC1_AIN3      ((uint8_t)0x04<<ADS1220_REG03_I1MUX_MASK)//IDAC1 connected to AIN3
#define ADS1220_REG03_I2MUX_MASK  2
#define ADS1220_REG03_I2MUX_IDAC2_DISABLE   ((uint8_t)0x00<<ADS1220_REG03_I2MUX_MASK)
#define ADS1220_REG03_I2MUX_IDAC2_AIN2      ((uint8_t)0x03<<ADS1220_REG03_I2MUX_MASK)//IDAC2 connected to AIN2
#define ADS1220_REG03_DRDYM_MASK  1
#define ADS1220_REG03_DRDYM_DISABLE         ((uint8_t)0x00<<ADS1220_REG03_DRDYM_MASK)//pin /DRDY is used to indicate data ready
#define ADS1220_REG03_DRDYM_ENABLE          ((uint8_t)0x01<<ADS1220_REG03_DRDYM_MASK)//data ready is indicated simultaneously on Dout and DRDY

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
