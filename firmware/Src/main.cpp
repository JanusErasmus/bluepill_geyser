#include "main.h"

#include <math.h>
#include <time.h>

#include "Utils/cli.h"
#include "Utils/crc.h"
#include "Utils/terminal_serial.h"
#include "stm32f1xx_hal.h"

#include "Utils/terminal.h"
#include "Utils/utils.h"
#include "usb_device.h"
#include "sonoff_pipe.h"

#define STREET_NODE_ADDRESS     0x00
#define UPS_NODE_ADDRESS        0x01
#define UPS12V_NODE_ADDRESS     0x02
#define FERMENTER_NODE_ADDRESS  0x03
#define HOUSE_NODE_ADDRESS      0x04
#define GARAGE_NODE_ADDRESS     0x05
#define WATER_NODE_ADDRESS      0x06
#define GEYSER_NODE_ADDRESS      0x07

#define NODE_ADDRESS WATER_NODE_ADDRESS
#define MINIMUM_REPORT_RATE 600000// 1800000

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef espUartHandle;
RTC_HandleTypeDef hrtc;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
SonoffPipe *pipe;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
}

void init_espUSART(void);

uint32_t last_report = 0;

/* Private function prototypes -----------------------------------------------*/
enum nodeFrameType_e
{
	DATA = 0,
	COMMAND = 1,
	ACKNOWLEDGE = 2
};

//Node send 32 bytes of data, with the last byte being the 8-bit CRC
typedef struct {
	uint8_t nodeAddress;	//1
	uint8_t frameType;		//1
	uint32_t timestamp;		//4  6
	uint8_t inputs;			//1  7
	uint8_t outputs;		//1  8
	uint16_t voltages[4];	//8  16
	uint16_t temperature;	//2  18
	uint8_t reserved[13]; 	//13 31
	uint8_t crc;			//1  32
}__attribute__((packed, aligned(4))) nodeData_s;


int adc_idx = 0;
uint16_t adc_raw[4];

uint16_t adc_ch0[80];
uint16_t adc_ch1[80];
int adc_ch0_idx = 0;
int adc_ch1_idx = 0;
bool adc_ch0_sampled = false;
bool adc_ch1_sampled = false;
int adc_trigger_sample = 0;
int adc_samples_ready = 0;
//
//void sampleRaw(
//		uint32_t &adc0,
//		uint32_t &adc1,
//		uint32_t &adc2,
//		uint32_t &adc3
//	)
//{
//	adc_idx = 0;
//	HAL_ADC_Start_IT(&hadc1);
//	HAL_Delay(10); //wait for conversion
//
//	adc0 = adc_raw[0];
//	adc1 = adc_raw[1];
//	adc2 = adc_raw[2];
//	adc3 = adc_raw[3];
//}

bool waitSamples(double &rms)
{
	if(adc_samples_ready)
	{
		adc_samples_ready = 0;

		double sum = 0;
		//printf("collected 80 samples\n");
		for (int k = 0; k < 80; ++k)
		{
			//printf("%04X %04X\n", adc_ch0[k], adc_ch1[k]);
			double raw_voltage = 1.02 - ((double)adc_ch0[k] * 0.000806);
			//printf(" %02d: %0.3f\n", k, raw_voltage);
			sum += raw_voltage * raw_voltage;
		}

		//printf("SUM: %f\n", sum);
		double mean = sum / 80.0;
		//printf("M  : %f\n", mean);
		rms = sqrt(mean) * 29.806259;
		//printf("RMS:%f A\n", rms);

		return true;
	}

	return false;
}

void sampleAnalog(
		double &temperature,
		double &tempExt,
		double &current_rms)
{
	uint32_t adc2_sum = 0;
	//uint32_t adc3_sum = 0;


	HAL_ADCEx_Calibration_Start(&hadc1);
	adc_trigger_sample = 1;

	while(!waitSamples(current_rms))
		HAL_Delay(20);
	HAL_ADC_Stop(&hadc1);

	if(current_rms < 0.26)
		current_rms = 0;

	for (int k = 0; k < 16; ++k)
	{
		//uint32_t adc0 = 0, adc1 = 0, adc2 = 0, adc3 = 0;
		//sampleRaw(adc0,adc1,adc2,adc3);

		//adc0_sum += adc_raw[0];
		//adc1_sum += adc_raw[1];
		adc2_sum += adc_ch1[k];
		//adc3_sum += adc_ch0[k];

		HAL_Delay(20);
	}

	adc2_sum >>= 4;

//	printf("ADC0: %5ld " , (int)adc_raw[0]);
//	printf("ADC1: %5ld " , (int)adc_raw[1]);
//	printf("ADC2: %5ld " , (int)adc_raw[2]);
//	printf("ADC3: %5ld\n", (int)adc_raw[3]);

	//this amount of steps measure 1.2V
	double step = 1.2 / adc_raw[0];
//	printf(" s	%0.3f\n", step);
	double voltage = ((double)adc_raw[1] * step);
	voltage = 1.43 - voltage;
//	printf(" -	%0.3f\n", voltage);
	voltage /= 0.0043;
//	printf(" /	%0.3f\n", voltage);
	temperature = (25.0 + voltage) - 11;

	//measure raw voltage
	double voltage0 = (((double)adc2_sum * step) + 0.01);

	tempExt = (voltage0 * 100) - 273.0;
}



extern "C" {
void esp_handle_byte(uint8_t byte)
{
	if(pipe)
		pipe->handleByte(byte);
}
}

int esp_transmit(uint8_t *buf, int len)
{
	//the sonoff is not fast enough to receive the whole message at once, delay each byte
	for (int k = 0; k < len; ++k)
	{
		if(HAL_UART_Transmit(&espUartHandle, &buf[k], 1, 300)  != HAL_OK)
			return -1;

		HAL_Delay(100);
	}

	return len;
}

bool report(const char *msg)
{
	if(!msg)
	{
		double tempInt;
		double tempExt;
		double current;

		sampleAnalog(tempInt, tempExt, current);

		char json[128];
		sprintf(json, "{\"uptime\":%d,"
				"\"temp\":%0.3f,"
				"\"voltages\":[%0.3f,%ld,%ld,%ld]"
				"}",
				(int)HAL_GetTick(),
				tempExt,
				current,
				(uint32_t)0,
				(uint32_t)0,
				(uint32_t)0
		);
		msg = (const char*)json;
	}

	printf("Reporting: %s\n", msg);

	if(!pipe->publish(msg))
	{
		printf("Report failed\n");
		HAL_Delay(500);
		return false;
	}
	last_report = (HAL_GetTick() + MINIMUM_REPORT_RATE);

	return true;
}


void handleMessage(const char* line)
{
	printf("MQTT: %s\n", line);

}

int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	HAL_Delay(1000);

	MX_USB_DEVICE_Init();
	sTerminalInterface_t usb = {
			MX_USB_DEVICE_ready,
			MX_USB_DEVICE_transmit
	};

	terminal_serial_Init();
	sTerminalInterface_t serial = {
			terminal_serial_ready,
			terminal_serial_transmit
	};

	sTerminalInterface_t *interfaces[] = {
			&serial,
			&usb,
			0
	};

	terminal_init((sTerminalInterface_t **)&interfaces);

	MX_ADC1_Init();
	init_espUSART();
	MX_TIM2_Init();

	pipe = new SonoffPipe(esp_transmit);
	pipe->setReceivedCB(handleMessage);

	printf("Bluepill Geyser @ %dHz\n", (int)HAL_RCC_GetSysClockFreq());
	MX_RTC_Init();

	//  report(netAddress);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	while (1)
	{
		terminal_run();

		pipe->run();

		if(last_report < HAL_GetTick())
		{
			report(0);
		}

		HAL_Delay(100);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

/* RTC init function */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

//  RCC->APB1ENR |= (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.State = HAL_RTC_STATE_RESET;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
//  RTC_TamperTypeDef tamper;
//  tamper.Tamper = RTC_TAMPER_1;
//  tamper.Trigger = RTC_TAMPERTRIGGER_HIGHLEVEL;
//  HAL_RTCEx_SetTamper(&hrtc, &tamper);
  HAL_RTCEx_DeactivateTamper(&hrtc, RTC_TAMPER_1);

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x32F2)
  {
	  printf(GREEN("RTC: "));

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  printf("RTC date");
	  printf(" - %04d-%02d-%02d ", 2000 + sDate.Year, sDate.Month, sDate.Date);
	  printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
  }
  else
  {
	  printf(RED("RTC: Not set\n"));
	  sTime.Hours = 0;
	  sTime.Minutes = 0;
	  sTime.Seconds = 0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Year = 0;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = WATER_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WATER_OUT_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(WATER_OUT_Port, WATER_OUT_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = WATER_HOLD_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WATER_HOLD_OUT_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(WATER_HOLD_OUT_Port, WATER_HOLD_OUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : ADC12_IN0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC12_IN1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*Init ESP UART */
void init_espUSART()
{
    espUartHandle.Instance        = ESP_USART;

    espUartHandle.Init.BaudRate   = 115200;
    espUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    espUartHandle.Init.StopBits   = UART_STOPBITS_1;
    espUartHandle.Init.Parity     = UART_PARITY_NONE;
    espUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    espUartHandle.Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_Init(&espUartHandle) != HAL_OK)
    {
        /* Initialization Error */
        _Error_Handler(__FILE__, __LINE__);
    }

    ESP_USART->CR1 |= USART_CR1_RXNEIE;

    HAL_NVIC_SetPriority(ESP_USART_IRQn, 0x1, 0);
    HAL_NVIC_EnableIRQ(ESP_USART_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
	hadc1.Instance = ADC1;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE	;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(adc_trigger_sample)
	{
		adc_idx = 0;
		HAL_ADC_Start_IT(&hadc1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_raw[adc_idx] = HAL_ADC_GetValue(&hadc1);

	//continue to sample the rest of the inputs
	if(adc_idx <= 4)
	{
		ADC_ChannelConfTypeDef sConfig;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		switch(adc_idx)
		{
		case 0:
			sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
			break;
		case 1:
			sConfig.Channel = ADC_CHANNEL_0;
			break;
		case 2:
			adc_ch0[adc_ch0_idx++] = adc_raw[2];
			if(adc_ch0_idx >= 80)
			{
				adc_ch0_sampled = true;
				adc_ch0_idx = 0;
			}
			sConfig.Channel = ADC_CHANNEL_1;
			break;
		case 3:
			adc_ch1[adc_ch1_idx++] = adc_raw[3];
			if(adc_ch1_idx >= 80)
			{
				adc_ch1_sampled = true;
				adc_ch1_idx = 0;
			}
			sConfig.Channel = ADC_CHANNEL_VREFINT;
			break;
		}
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		HAL_ADC_Start_IT(&hadc1);
	}


	if(++adc_idx >= 4)
	{
		adc_idx = 0;
		HAL_ADC_Stop_IT(&hadc1);
	}

	if(adc_ch0_sampled && adc_ch1_sampled)
	{
		adc_ch0_sampled = false;
		adc_ch1_sampled = false;
		adc_trigger_sample = 0;
		adc_samples_ready = 1;
	}
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	//printf("APB1 @ %dHz\n", (int)HAL_RCC_GetPCLK1Freq());
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4500; //input clock is 8kHz (36 000 000 / 4 500)
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 3;   // trigger every 4 cycles, gives 1kHz (25ms) tick
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
}

const char *getDayName(int week_day)
{
	switch(week_day)
	{
	case RTC_WEEKDAY_MONDAY:
		return "Monday";
	case RTC_WEEKDAY_TUESDAY:
		return "Tuesday";
	case RTC_WEEKDAY_WEDNESDAY:
		return "Wednesday";
	case RTC_WEEKDAY_THURSDAY:
		return "Thursday";
	case RTC_WEEKDAY_FRIDAY:
		return "Friday";
	case RTC_WEEKDAY_SATURDAY:
		return "Saturday";
	case RTC_WEEKDAY_SUNDAY:
		return "Sunday";
	}

	return 0;
}

#ifdef __cplusplus
 extern "C" {
#endif

void mqtt(uint8_t argc, char **argv)
{
	printf("Report over MQTT\n");

	if(argc > 1)
	{
		printf(" - custom msg: %s\n", argv[1]);
		report(argv[1]);
	}
	else
		report(0);
}

void exit_py(uint8_t argc, char **argv)
{
	printf ("Exit Python script\n");
	char json[] = {"exit\n\r"};
	esp_transmit((uint8_t*)json, strlen(json));
}

void reset_sonoff(uint8_t argc, char **argv)
{
	printf("Soft Reseting Sonoff\n");

	pipe->resetSonoff();
}

void adc(uint8_t argc, char **argv)
{
	double tempInt;
	double tempExt;
	double current;

	sampleAnalog(tempInt, tempExt, current);

	printf("ADC: %0.3f, %0.3f, %0.3f\n", tempInt, tempExt, current);

//	adc_trigger_sample = 1;

//	adc_trigger_sample = 1;
//	HAL_ADC_Start_IT(&hadc1);
//	double current;
//	HAL_ADCEx_Calibration_Start(&hadc1);
//	for (int k = 0; k < 16; ++k)
//	{
//		uint32_t adc0 = 0, adc1 = 0, adc2 = 0, adc3 = 0;
//		sampleRaw(adc0,adc1,adc2,adc3);
//		adc0_sum += adc0;
//		adc1_sum += adc1;
//		adc2_sum += adc2;
//		adc3_sum += adc3;
//
//		HAL_Delay(20);
//	}
//	HAL_ADC_Stop(&hadc1);
}

void rtc_debug(uint8_t argc, char **argv)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(argc > 5)
	{
		printf("Setting date %d\n", atoi(argv[5]));

		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Year = atoi(argv[1]) - 2000;
		sDate.Month = atoi(argv[2]);
		sDate.Date = atoi(argv[3]);
		sTime.Hours = atoi(argv[4]);
		sTime.Minutes = atoi(argv[5]);
		sTime.Seconds = 0;

		RCC->APB1ENR |= (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
		//PWR->CR |= PWR_CR_DBP;
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	printf("RTC date: %s %d\n", getDayName(sDate.WeekDay), (int)HAL_RTC_SecondsSinceEpoch(sDate, sTime));
	printf(" - %04d-%02d-%02d ", 2000 +sDate.Year, sDate.Month, sDate.Date);
	printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

#ifdef __cplusplus
 }
#endif


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

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

