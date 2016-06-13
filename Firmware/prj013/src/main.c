/*
******************************************************************************
070253 - Circuitos Microprocessados
Prof. Lucio Rene Prade
Exemplo  - ADC
******************************************************************************
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include <HD44780LIB.h>


#define  PERIOD_VALUE       (uint32_t)(1000 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef	TimHandlePwm;
/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
GPIO_InitTypeDef  GPIO_InitStruct;

/* ADC configuration structure declaration */
ADC_HandleTypeDef    AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef   sConfig;

/* Struct de configuração da saida do Timer */
TIM_OC_InitTypeDef sConfigPwm;

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef    TimHandle;

/* Declaração da variável Prescaler */
uint32_t uwPrescalerValue = 0;

/* Variavel de confguração do prescaler */
//uint32_t uhPrescalerValue = 0;

/* Buffer de Transmissão  */
uint8_t Buffer[50] = "\n **** Exemplo de ADC ****\n\r ";

uint32_t g_ADCValue1;
uint32_t g_ADCValue2;
uint32_t g_ADCValue3;
uint32_t g_Flag;

#define ZERO_AMP	3400//3040
float g_ConvAmp;
float g_ConvVolt;
float g_ConvTemp;
uint8_t statusString[100];
uint8_t tempString[30];

static void SystemClock_Config(void);
static void Error_Handler(void);
void sendSerial(uint8_t *pData);
char readSerial();
void limpaTela(uint8_t n);
void ConcatFloat(float value, uint8_t *pData, uint8_t nCasas);
void pwmConfig();
void pwmByPercent(uint8_t percent,uint32_t channel);

int main(void)
{
	/* Inicializa as bibliotecas HAL */
	HAL_Init();

	/* Configura o clock do sistema */
	SystemClock_Config();

	/* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
	__GPIOA_CLK_ENABLE();

	/* Configura  o pino do led como output push-pull */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pin 	= GPIO_PIN_5;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
	__GPIOA_CLK_ENABLE();


	/* ADC1 Channel8 GPIO pin configuration */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configura a variável prescaler com valor de contagem  para 10000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	/* Configura TIM1 */
	TimHandle.Instance = TIM3;
	TimHandle.Init.Period            = 1000 - 1;//5000 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TimHandle);

	/* Configura a geração de interrupção para o timer 1 */
	HAL_TIM_Base_Start_IT(&TimHandle);


	/*Configura os pinos GPIOs  para a função alternativa de RX e TX da USART2*/
	GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	/* habilita o clock da  USART2 */
	__USART2_CLK_ENABLE();


	/*Configuração do Periférico USART
	- modo assincrono (UART Mode)
	- Word  = 8 Bits
	- Stop Bit = One Stop bit
	- Parity = None
	- BaudRate = 9600 baud
	- Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance        = USART2;
	UartHandle.Init.BaudRate   = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	HAL_UART_Init(&UartHandle);


	/* Habilita o clock do ADC 1*/
	__ADC1_CLK_ENABLE();

	/* Configuração  do periférico ADC */
	/*
	*  Instance                  = ADC1.
	*  ClockPrescaler            = PCLK divided by 4.
	*  LowPowerAutoWait          = Disabled
	*  LowPowerAutoPowerOff      = Disabled
	*  Resolution                = 12 bit (increased to 16 bit with oversampler)
	*  ScanConvMode              = ADC_SCAN_ENABLE
	*  DataAlign                 = Right
	*  ContinuousConvMode        = Enabled
	*  DiscontinuousConvMode     = Enabled
	*  ExternalTrigConv          = ADC_SOFTWARE_START
	*  ExternalTrigConvEdge      = None (Software start)
	*  EOCSelection              = End Of Conversion event
	*  DMAContinuousRequests     = Disabled
	*/

	AdcHandle.Instance = ADC1;

	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC;
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;
	AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
	AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ContinuousConvMode    = DISABLE;
	AdcHandle.Init.DiscontinuousConvMode = ENABLE;
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
	AdcHandle.Init.DMAContinuousRequests = DISABLE;
	AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
	AdcHandle.NbrOfConversionRank = 3;

	/* Inicilaiza  o ADC  com as configurações*/
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}


	/*Calibra o ADC com as configurações */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}


	/* Seleciona o canal analogico (Channel 0) */
	sConfig.Channel      = ADC_CHANNEL_0;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;  // AUMENTA O TEMPO DE AMOSTRAGEM, RESOLVE O PROBLEMA DE INSTABILIDADE
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Seleciona o canal analogico (Channel 0) */
	sConfig.Channel      =  ADC_CHANNEL_1;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Seleciona o canal analogico (Channel 0) */
	sConfig.Channel      =  ADC_CHANNEL_4;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	/* Envia a mensagem de inicio */
	HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 50, 100);


	pwmConfig();
	/* Infinite Loop */
	while (1)
	{
		if(g_Flag == 1)
		{
			g_Flag = 0;

			HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do primeiro canal
			if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
			{
				// Sensor de Corrente.
				g_ADCValue1 = HAL_ADC_GetValue(&AdcHandle);
				// Converte ADC para Corrente
				//g_ConvAmp = (float)((g_ADCValue1/* - 3103 */) * (3.3/4096) *5);
				if(g_ADCValue1 < ZERO_AMP) g_ConvAmp = 0;
				else g_ConvAmp = (float)((g_ADCValue1 - ZERO_AMP) *0.005);

				HAL_Delay(10);
			}

			HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do segundo canal
			if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
			{
				// Leitura da Tensão
				g_ADCValue2 = HAL_ADC_GetValue(&AdcHandle);
				// Tensão da Bateria, Razão do divisor de tensão
				g_ConvVolt = (float)(g_ADCValue2 * (3.3/4096) *( (22+5.6)/5.6 ) );

				HAL_Delay(10);
			}

			HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do teceiro canal
			if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
			{
				// Leitura da Temperatura
				g_ADCValue3 = HAL_ADC_GetValue(&AdcHandle);
				// Temperatura 0 mV + 10.0 mV/°C
				g_ConvTemp = (float)(g_ADCValue3 * ((3.3/4096)*100));

				HAL_Delay(10);
			}
			HAL_ADC_Stop(&AdcHandle);

			//Transmissão de dados
			{
#if 0
				//Limpa a Tela
				limpaTela(20);

				//Inicializa a tela de Status
				strcpy(statusString,"Leitura do Sistema:\r\n");

				//Concatena Valores de Corrente
				strcat(statusString,"Corrente:\t");
				ConcatFloat(g_ConvAmp, statusString, 4);
				strcat(statusString,"A\r\n");

				//Concatena Valores de Tensão
				strcat(statusString,"Tensao:\t");
				ConcatFloat(g_ConvVolt, statusString, 2);
				strcat(statusString,"V\r\n");

				//Concatena Valores de Temperatura
				strcat(statusString,"Temp:\t");
				ConcatFloat(g_ConvTemp, statusString, 2);
				strcat(statusString,"oC\r\n");
#endif
#if 1
				//Concatena Valores de Corrente
				strcpy(statusString,"ADC:\t");
				ConcatFloat(g_ADCValue1-2000, statusString, 0);
				strcat(statusString,"\r\n");
#endif
				//Imprime na Serial
				sendSerial((uint8_t*)statusString);
			}
		}
		HAL_Delay(10);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	g_Flag = 1;
}

static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
	{
		Error_Handler();
	}
}


/* Função chamada no caso de erro na configuração */
static void Error_Handler(void)
{
	while(1);
}

/******************************************************************************/

/******************************************************************************/
void sendSerial(uint8_t *pData)
{
	HAL_UART_Transmit(&UartHandle, pData, strlen(pData), 50);
}
/******************************************************************************/

/******************************************************************************/
char readSerial()
{
	uint8_t rxChar;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&rxChar, 1, 50);
	return rxChar;
}
/******************************************************************************/

/******************************************************************************/
void limpaTela(uint8_t n)
{
	uint8_t i;
	for(i=0 ; i<n ; i++) sendSerial("\r\n");
}
/******************************************************************************/

/******************************************************************************/
void ConcatFloat(float value, uint8_t *pData, uint8_t nCasas)
{
	uint8_t tString[30];
	uint16_t base = 1,j;
	float lsbValue;

	for(j=0;j<nCasas;j++) base *= 10;

	itoa((int)value,tString,10);
	strcat(pData,tString);

	lsbValue = (value - (int)value)*base;

	if(nCasas > 0)
	{
		strcat(pData,".");

		for(nCasas ; nCasas > 1 ; nCasas-- )
		{
			base = base/10;
			if((int)lsbValue < base) strcat(pData,"0");
		}
		itoa((int)lsbValue, (uint8_t*)tString, 10);
		strcat(pData,tString);
	}

}
/******************************************************************************/

/******************************************************************************/
void pwmConfig()
{
	/* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;


	/* Configuração  do periférico TIM3 como PWM
		+ Prescaler = (SystemCoreClock / 16000000) - 1
		+ Period = (666 - 1)
		+ ClockDivision = 0
		+ Counter direction = Up
	*/
	//TimHandle.Instance = TIM3;
	TimHandlePwm.Instance = TIM16;

	TimHandlePwm.Init.Prescaler         = uwPrescalerValue;
	TimHandlePwm.Init.Period            = PERIOD_VALUE;
	TimHandlePwm.Init.ClockDivision     = 0;
	TimHandlePwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandlePwm.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&TimHandlePwm);


	/* Configure the PWM channels*/
	sConfigPwm.OCMode       = TIM_OCMODE_PWM1;
	sConfigPwm.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfigPwm.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfigPwm.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfigPwm.OCIdleState  = TIM_OCIDLESTATE_RESET;
	sConfigPwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	//TIM3
	/* Set the pulse value for channel 1  PB4*/
	//pwmByPercent(DutyControl[0],TIM_CHANNEL_1);
	/* Set the pulse value for channel 2 PB5 - pag 34 doc DM00088500.pdf*/
	//pwmByPercent(DutyControl[1],TIM_CHANNEL_2);

	//TIM15
	/* Set the pulse value for channel 1  PB8*/
	sConfigPwm.Pulse = 500;

	HAL_TIM_PWM_ConfigChannel(&TimHandlePwm, &sConfigPwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandlePwm, TIM_CHANNEL_1);


	//pwmByPercent(DutyControl[0],TIM_CHANNEL_1);
}
/******************************************************************************/
void pwmByPercent(uint8_t percent,uint32_t channel)
{
	sConfigPwm.Pulse = (PERIOD_VALUE+1)*percent/100;

	HAL_TIM_PWM_ConfigChannel(&TimHandlePwm, &sConfigPwm, channel);
	HAL_TIM_PWM_Start(&TimHandlePwm, channel);
}
/******************************************************************************/
