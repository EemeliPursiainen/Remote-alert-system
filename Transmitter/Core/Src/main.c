/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Eemeli Pursiainen.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RTC_WUT_TIME ((uint32_t)30) // time of standby after SYS_WKUP1 pin rising edge (seconds)
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1) // Init variable out of expected ADC conversion data range

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint16_t lostPackageCounter = 0;
/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */
__IO uint8_t ubAdcGrpRegularUnitaryConvStatus = 2; /* Variable set into ADC interruption callback */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void uartTransmit(void);

void readVbat(void);

void EnterStandbyModeRTC(void);
void EnterStandbyModeWKUP(void);

void activateADC(void);
void deactivateADC(void);

void AdcGrpRegularUnitaryConvComplete_Callback();
void ConversionStartPoll_ADC_GrpRegular(void);
void AdcGrpRegularOverrunError_Callback(void);

void sendMessage(char message[10]);
bool receiveAck(char message[10]);

void reverse(char str[], int length);
char* citoa(uint16_t num, char *str, int base);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Uart buffers
//uint8_t buffer[50] = "";
//bufInfo bi; // UART buffer
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN1);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	//MX_USART2_UART_Init();
	MX_ADC_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	while (!loraBegin()) {
	}

	loraSetSpreadingFactor(_sf);
	loraSetSignalBandwidth(_bw);
	loraDisableCrc();
	char message[10] = "";

	if (LL_RTC_IsActiveFlag_WUT(RTC) == 1) {
		LL_RTC_ClearFlag_WUT(RTC);
		LL_RTC_DisableIT_WUT(RTC);

		loraSleep();

		EnterStandbyModeWKUP();
	}

	LL_PWR_ClearFlag_SB();
	LL_PWR_ClearFlag_WU();

	activateADC();
	readVbat();
	deactivateADC();

	LL_mDelay(100);

	uint16_t vbat = 10000 + ((uhADCxConvertedData * 6772) / 4095); // addition of 10000 moves other digits 1 to the right to make message forming easier
	//memcpy(message, citoa(vbat, message, 10), strlen(message));
	citoa(vbat, message, 10);
	message[0] = 'H';
	sendMessage(message);

	int nackCounter = 0;
	while (!receiveAck(message) && nackCounter <= 20) {
		if(nackCounter > 0){
			message[5] = 'L';
		}
		loraSleep();
		LL_mDelay(500);

		sendMessage(message);
		nackCounter++;
	}

	loraSleep();
	LL_mDelay(100);

	EnterStandbyModeRTC();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	while (LL_PWR_IsActiveFlag_VOS() != 0) {
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_LSI_Enable();

	/* Wait till LSI is ready */
	while (LL_RCC_LSI_IsReady() != 1) {

	}
	LL_PWR_EnableBkUpAccess();
	if (LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI) {
		LL_RCC_ForceBackupDomainReset();
		LL_RCC_ReleaseBackupDomainReset();
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
	}
	LL_RCC_EnableRTC();
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {

	}
	LL_SetSystemCoreClock(16000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
	LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**ADC GPIO Configuration
	 PA2   ------> ADC_IN2
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* ADC interrupt Init */
	NVIC_SetPriority(ADC1_IRQn, 0);
	NVIC_EnableIRQ(ADC1_IRQn);

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);

	/** Common config
	 */
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
	LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1),
	LL_ADC_CLOCK_FREQ_MODE_LOW);
	LL_ADC_DisableIT_EOC(ADC1);
	LL_ADC_DisableIT_EOS(ADC1);
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_ASYNC);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
	LL_ADC_CLOCK_ASYNC_DIV16);

	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADC1);
	/* Delay for ADC internal voltage regulator stabilization. */
	/* Compute number of CPU cycles to wait for, from delay in us. */
	/* Note: Variable divided by 2 to compensate partially */
	/* CPU processing cycles (depends on compilation optimization). */
	/* Note: If system core clock frequency is below 200kHz, wait time */
	/* is only a few CPU processing cycles. */
	uint32_t wait_loop_index;
	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / (100000 * 2))) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	/* USER CODE END RTC_Init 0 */

	LL_RTC_InitTypeDef RTC_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_RCC_EnableRTC();

	/* RTC interrupt Init */
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* USER CODE BEGIN RTC_Init 1 */
	while (LL_RTC_IsActiveFlag_WUTW(RTC) != 1) {
	}
	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC and set the Time and Date
	 */
	RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
	RTC_InitStruct.AsynchPrescaler = 127;
	RTC_InitStruct.SynchPrescaler = 255;
	LL_RTC_Init(RTC, &RTC_InitStruct);

	/** Initialize RTC and set the Time and Date
	 */

	/** Enable the WakeUp
	 */
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	/* USER CODE BEGIN RTC_Init 2 */
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);

	LL_RTC_WAKEUP_SetAutoReload(RTC, RTC_WUT_TIME);
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	 PA9   ------> USART2_TX
	 PA10   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 interrupt Init */
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART2);
	LL_USART_Enable(USART2);
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void uartTransmit(void) {
//	bi.ptr = &buffer[1]; // start from the second byte in the interrupt
//	bi.size = sizeof(buffer);
//	bi.counter = 0;
//	LL_USART_TransmitData8(USART2, buffer[0]);
//	LL_USART_EnableIT_TXE(USART2);
//}
void activateADC(void) {
	__IO uint32_t wait_loop_index = 0;

	if (LL_ADC_IsEnabled(ADC1) == 0) {
		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC1);

		while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
//			char message[10] = "";
//			message[0] = 'E';
//			message[1] = '1';
//			sendMessage(message);
//			LL_mDelay(100);
		}
	}
	wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	/* Enable ADC */
	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) {
//		char message[10] = "";
//		message[0] = 'E';
//		message[1] = '2';
//		sendMessage(message);
//		LL_mDelay(100);
	}
}

void deactivateADC(void) {
	__IO uint32_t wait_loop_index = 0;
	wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_Disable(ADC1);
	LL_ADC_ClearFlag_ADRDY(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 1) {
//		char message[10] = "";
//		message[0] = 'E';
//		message[1] = '3';
//		sendMessage(message);
//		LL_mDelay(100);
	}
}

void EnterStandbyModeRTC(void) {
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);

	LL_RTC_EnableWriteProtection(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
	LL_LPM_EnableDeepSleep();

	/* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif

	__WFI();
}

void EnterStandbyModeWKUP(void) {
	LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN1);
	LL_PWR_ClearFlag_WU();
	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN1);
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
	LL_LPM_EnableDeepSleep();

	/* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif

	__WFI();
}

void readVbat(void) {
	if (ubAdcGrpRegularUnitaryConvStatus != 0) {
		ubAdcGrpRegularUnitaryConvStatus = 0;
	} else {
		/* Error: Previous action (ADC conversion not yet completed).           */
		//buffer = "ADC previous action not yet completed";
		Error_Handler();
	}

	/* Init variable containing ADC conversion data */
	uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;

	/* Perform ADC group regular conversion start, poll for conversion        */
	/* completion.                                                            */
	ConversionStartPoll_ADC_GrpRegular();

	/* Retrieve ADC conversion data */
	/* (data scale corresponds to ADC resolution: 12 bits) */
	uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC1);
	ubAdcGrpRegularUnitaryConvStatus = 1;
	return;
}

void ConversionStartPoll_ADC_GrpRegular(void) {

	/* Start ADC group regular conversion */
	/* Note: Hardware constraint (refer to description of the function          */
	/*       below):                                                            */
	/*       On this STM32 series, setting of this feature is conditioned to     */
	/*       ADC state:                                                         */
	/*       ADC must be enabled without conversion on going on group regular,  */
	/*       without ADC disable command on going.                              */
	/* Note: In this example, all these checks are not necessary but are        */
	/*       implemented anyway to show the best practice usages                */
	/*       corresponding to reference manual procedure.                       */
	/*       Software can be optimized by removing some of these checks, if     */
	/*       they are not relevant considering previous settings and actions    */
	/*       in user application.                                               */
	if ((LL_ADC_IsEnabled(ADC1) == 1) && (LL_ADC_IsDisableOngoing(ADC1) == 0)
			&& (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)) {
		LL_ADC_REG_StartConversion(ADC1);
	} else {
		/* Error: ADC conversion start could not be performed */
	}

	while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {
//		char message[10] = "";
//		message[0] = 'E';
//		message[1] = '4';
//		sendMessage(message);
//		LL_mDelay(100);
	}

	/* Clear flag ADC group regular end of unitary conversion */
	/* Note: This action is not needed here, because flag ADC group regular   */
	/*       end of unitary conversion is cleared automatically when          */
	/*       software reads conversion data from ADC data register.           */
	/*       Nevertheless, this action is done anyway to show how to clear    */
	/*       this flag, needed if conversion data is not always read          */
	/*       or if group injected end of unitary conversion is used (for      */
	/*       devices with group injected available).                          */
	LL_ADC_ClearFlag_EOC(ADC1);

}

void AdcGrpRegularOverrunError_Callback(void) {
	/* Note: Disable ADC interruption that caused this error before entering in */
	/*       infinite loop below.                                               */

	/* Disable ADC group regular overrun interruption */
	LL_ADC_DisableIT_OVR(ADC1);

	/* Error from ADC */

}

void sendMessage(char message[10]) {
	loraBeginPacket();
	uint8_t *messageStr = (uint8_t*) message;
	loraWriteBuf(messageStr, strlen(message));
	loraEndPacket(false);
}

bool receiveAck(char message[10]) {
	uint8_t k = 0;
	//unsigned long entry = expTime_getTick();
	//while (stat == false && HAL_GetTick() - entry < 2000) {
	while (1) {
		if (loraParsePacket()) {
			char ack[10] = "";
			uint8_t ackIterator = 0;
			while (loraAvailable()) {
				ack[ackIterator] = (unsigned char) loraRead();
				ackIterator++;
			}
			if (strlen(message) != strlen(ack)) {
				return false;
			}

			for (uint8_t j = 0; j < strlen(message); j++) {
				if (ack[j] != message[j]) {
					return false;
				}
			}
			return true;
		}
		LL_mDelay(10);
		k++;
		if (k > 50) {
			return false;
		}

	}
}

void reverse(char str[], int length) {
	int start = 0;
	int end = length - 1;
	while (start < end) {
		char temp = str[start];
		str[start] = str[end];
		str[end] = temp;
		end--;
		start++;
	}
}
// Implementation of citoa()
char* citoa(uint16_t num, char *str, int base) {
	int i = 0;

	/* Handle 0 explicitly, otherwise empty string is
	 * printed for 0 */
	if (num == 0) {
		str[i++] = '0';
		str[i] = '\0';
		return str;
	}
	// Process individual digits
	while (num != 0) {
		int rem = num % base;
		str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
		num = num / base;
	}

	str[i] = '\0'; // Append string terminator

	// Reverse the string
	reverse(str, i);

	return str;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		char message[10] = "";
		message[0] = 'E';
		message[1] = '7';
		sendMessage(message);
		LL_mDelay(1000);
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
