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
* COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_dac.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_bus.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DELAY_20US	1600
#define CALIB_DELAY	116

#define ADCBUFSIZE 32
#define DACBUFSIZE 32

const uint16_t dacbuf[DACBUFSIZE] = {
	2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
	3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
	599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647 };

uint16_t adcbuf[ADCBUFSIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */
	/* GPIO LL configuration */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA); // enable clock to the GPIOA peripheral
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG); // configure GPIOA pin1 (ADC1 Channel6) into analog mode
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG); // configure GPIOA pin4 (DAC1 output1) into analog mode
	LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_1); // connect GPIO analog switch to ADC1 input for PA1

	/* DAC LL configuration */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1); // enable clock to the DAC1 peripheral
	LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_1, LL_DAC_TRIG_EXT_TIM2_TRGO); // select trigger source (TRGO signal from TIM2)
	LL_DAC_ConfigOutput(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_MODE_NORMAL, LL_DAC_OUTPUT_BUFFER_ENABLE, LL_DAC_OUTPUT_CONNECT_GPIO);
	// configure DAC1 output for Channel1 in normal mode with buffer enable and connection to GPIO (PA4 in our case)
	LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_1); // enable DMA request for DAC1, Channel1

	/* ADC LL configuration */
	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK); // select system clock as clock source for ADC
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC); // enable clock to the ADC peripheral
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV2); // input clock for ADC configuration PCLK/2
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM2_CH2); // select trigger for ADDC1 regular conversions (capture compare TIM2 CH2)
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING); // configure trigger signal edge for rising
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE); // configure single conversion per trigger event
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); // configure DMA data transfer to unlimited mode
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE); // configure length of regular sequence for 1
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6); // configure sequencer ranks for each channel (only channel6)
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	// configure sampling time for each channel (only channel16 for 12.5 clk cycles)
	LL_ADC_DisableDeepPowerDown(ADC1); // disable deep power down mode (ADC is in this mode after reset by default)
	LL_ADC_EnableInternalRegulator(ADC1); // enable internal voltage regulator
	// wait 20us for internal regulator stabilization
	for (uint16_t ii = 0; ii<DELAY_20US; ii++) ii++;

	/* DMA LL configuration */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	// Channel3 in DMA1 configuration
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_CIRCULAR |
		LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
		LL_DMA_PRIORITY_HIGH);
	// MEM -> PERIPH, circular mode, read data in halfwords with incrementation, write data halfwords without incrementation
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_6); // assign channel4 of DMA1 to DAC1 request (table 39 in Referece Manual
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&dacbuf,
		LL_DAC_DMA_GetRegAddr(DAC1, LL_DAC_CHANNEL_1, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	// set transfer addresses of source and destination, at destination DAC1 data register for 12bit data aligned to right
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DACBUFSIZE); // configure DMA transfer size
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	// Channel1 in DMA1 configuration
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR |
		LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
		LL_DMA_PRIORITY_HIGH);
	// MEM -> PERIPH, circular mode, read data in halfwords with incrementation, write data halfwords without incrementation
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0); // assign channel4 of DMA1 to DAC1 request (table 39 in Referece Manual)
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		(uint32_t)&adcbuf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	// set transfer addresses of source and destination, at source ADC1 data register for regula channels (12 bits)
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADCBUFSIZE); // configure DMA transfer size
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	/* TIM LL configuration */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // enable clock to TIM2 peripheral
	LL_TIM_SetPrescaler(TIM2, 39999); // set TIM2 clk for 2 kHz (39999 period value)
	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP); // make counter count upwards
	LL_TIM_SetAutoReload(TIM2, 399); // set auto-reload value (-> TRGO)
	LL_TIM_SetRepetitionCounter(TIM2, 0); // disable repetition counter
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE); // connect TRGO to update event
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE); // set output compare mode to toggle (toggling TIM2 CH2 output on compare match)
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // set output channel polarity to OC active high
	LL_TIM_OC_SetCompareCH2(TIM2, 200); // set value for TIM2 CH2 for 200 (set pulse value, duty cycle 50%)
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2); // enable CH2 of TIM2



	/* DAC activation */
	LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_1); // enable DMA for channel1 of DAC1
	LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1); // enable TIM2 TRGO trigger for channel1 of DAC1
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1); // enable channel1 of DAC1

	/* ADC activation */
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED); // start calibration of ADC1 for single ended conversions
														//necessary 116 ADC clk delay
	for (uint16_t ii = 0; ii<CALIB_DELAY; ii++) ii++;
	LL_ADC_Enable(ADC1);
	LL_ADC_REG_StartConversion(ADC1); // start regular conversion after next HW trigger

	/* TIM2 activation */
	LL_TIM_EnableCounter(TIM2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	*/
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
	while (1)
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
