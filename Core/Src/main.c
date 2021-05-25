/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include <stdio.h>
#include <string.h>

#include "EEPROM.h"

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

/* USER CODE BEGIN PV */
float h=0.0, t=0.0;
uint8_t step = 0;
HAL_StatusTypeDef status;
int timerForUART = 0;

volatile uint32_t adc_val = 0; // Light Sensor

char str[50];
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];

int record = 0;
int topLightBuffer = 0;
int topTempBuffer = 0;
int topHumidBuffer = 0;
int lightBuffer[20];
float tempBuffer[20];
float humidBuffer[20];
int lightValue = 0;


DisplayInfo lcdInfo; // DisplayInfo details can be found at main.h
Timer timer; // Timer details can be found at main.h


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int LCDDisplay(DisplayInfo* info, Timer* timer);
void timerController(DisplayInfo* info,Timer* timer);

void adcLightSensor(uint32_t);
void i2cAM2320Sensor(void );
void printRecord(void );

uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
	uint16_t crc = 0xffff;
	uint8_t s = 0x00;
	
	while (length--) {
		crc ^= *ptr++;
		for (s = 0; s < 8; s++) {
			if ((crc & 0x01) != 0) {
				crc >>= 1;
				crc ^= 0xa001;
			} else crc >>= 1;
		}
	}
	return crc;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void init_eeprom()
{
	for (int i = 0; i < 512; i++) {
		EEPROM_PageErase(i);
	}
}

void write_to_eeprom(uint8_t light, uint8_t temp, uint8_t humid)
{
	// delete data in eeprom
	for (int i = 0; i < 0; i++) {
		EEPROM_PageErase(i);
	}
	
	// write data to eeprom
	uint8_t tmp[] = {light, temp, humid};
	EEPROM_Write(0, 0, tmp, 3);
}

void read_from_eeprom(uint8_t dataLoad[])
{
	EEPROM_Read(0,0, dataLoad, 3);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_RNG_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	ILI9341_Init();
	
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	
	
	lcdInfo.temperature = 0;
	lcdInfo.humidity = 0;
	lcdInfo.light = 99;
	lcdInfo.page = 0;
	lcdInfo.updateScreen = 1;
	
	lcdInfo.timeHr = 0;
	lcdInfo.timeMin = 0;
	lcdInfo.timeSec = 0;
	
	uint8_t eepromData[3];
	read_from_eeprom(eepromData);
	lcdInfo.avgLight = eepromData[0];
	lcdInfo.avgTemp = eepromData[1];
	lcdInfo.avgHumid = eepromData[2];
	lcdInfo.updateAvg = 0;
	lcdInfo.avgHighlightTimer = 0;
	
	
	
	timer.isActive = 0;
	timer.timeLeftSec = 0;
	timer.tick = 0;
	timer.timerDone = 0;
	
	
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim1);
	
	sprintf(str, "\n\rAM2320 I2C DEMO Starting . . .\n\r");
	HAL_UART_Transmit(&huart3, (uint8_t*)&str, strlen(str), 1000);
	
	cmdBuffer[0] = 0x03;
	cmdBuffer[1] = 0x00;
	cmdBuffer[2] = 0x04;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
		printRecord();
		timerController(&lcdInfo,&timer);
		
		LCDDisplay(&lcdInfo,&timer);
		
		/*
		if(TP_Touchpad_Pressed()){
			uint16_t x_pos = 0;
			uint16_t y_pos = 0;
			
			uint16_t position_array[2];
			
			if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
				x_pos = position_array[0];
				y_pos = position_array[1];
				
				char coord[20];
				sprintf(coord,"%.3d %.3d %.5d",x_pos,y_pos,lcdInfo.avgHighlightTimer);
				ILI9341_Draw_Text(coord, 20, 230, WHITE, 1, BLACK);
			}
		}
		*/
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void changeToPage(DisplayInfo* info, int pageNo){
	info -> page = pageNo;
	ILI9341_Fill_Screen(BLACK);
	info -> updateScreen = 1;
	HAL_Delay(100);
}

void mainPage(DisplayInfo* info, Timer* timer){
	
	uint8_t currTemp = (uint8_t) t;
	uint8_t currHumid = (uint8_t) h;
	uint8_t currLight = (uint8_t) lightValue;
	
	if(info -> updateScreen || info -> temperature != currTemp){
		info -> temperature = currTemp;
		char temperatureStr[10];
		if(info -> temperature < 10) sprintf(temperatureStr," %d", info -> temperature);
		else sprintf(temperatureStr,"%d", info -> temperature);
	
		ILI9341_Draw_Text("TEMP", 70, 60, WHITE, 1, BLACK);
		ILI9341_Draw_Text(temperatureStr, 115, 65, WHITE, 7, BLACK);
		ILI9341_Draw_Text("C", 210, 85, WHITE, 3, BLACK);
	}
	
	if(info -> updateScreen || info -> humidity != currHumid){
		info -> humidity = currHumid;
		char humidityStr[10];
		ILI9341_Draw_Text("HUMID", 70, 145, WHITE, 1, BLACK);
		sprintf(humidityStr,"%d%%",info -> humidity);
		ILI9341_Draw_Text(humidityStr, 70, 155, WHITE, 4, BLACK);
	}
	
	if(info -> updateScreen || info -> light != currLight){
		info -> light = currLight;
		char lightStr[10];
		ILI9341_Draw_Text("LIGHT", 190, 145, WHITE, 1, BLACK);
		if(info -> light < 10) sprintf(lightStr," %d%%",info -> light);
		else sprintf(lightStr,"%d%%",info -> light);
		ILI9341_Draw_Text(lightStr, 190, 155, WHITE, 4, BLACK);
	}
	uint8_t currentTimerStatus = timer -> isActive;
	if(timer -> timerDone) currentTimerStatus = 2;
	const uint8_t isTimerStatusChange = currentTimerStatus != info -> timerActive;
	const uint8_t isTimeChange = timer -> timeLeftSec != (info -> timeHr*3600 + info -> timeMin*60 + info->timeSec);
	
	if(info -> updateScreen || isTimerStatusChange || isTimeChange){
		if(info -> updateScreen) ILI9341_Draw_Text("TIMER", 10, 13, WHITE, 1, BLACK);
		if(timer -> isActive){
			info -> timerActive = timer -> isActive;
			if(isTimerStatusChange || isTimeChange || info -> updateScreen){
				info -> timeHr = (timer -> timeLeftSec) / 3600.0;
				info -> timeMin = ((timer -> timeLeftSec) - (info -> timeHr*3600))/60.0;
				info -> timeSec = (timer -> timeLeftSec) - (info -> timeHr*3600) - (info -> timeMin*60);
				char timerStr[20];
				sprintf(timerStr,"%.2d:%.2d:%.2d",info -> timeHr, info -> timeMin, info -> timeSec);
				ILI9341_Draw_Text(timerStr, 10, 23, WHITE, 2, BLACK);
			}
		}
		else{
			if(isTimerStatusChange && timer -> timerDone){
				info -> timerActive = 2;
				ILI9341_Draw_Text("         ", 10, 23,  WHITE, 2, BLACK);
				ILI9341_Draw_Text(" DONE ", 10, 23,  BLACK, 2, WHITE);
			}
			else if(isTimerStatusChange || info -> updateScreen) ILI9341_Draw_Text("OFF      ", 10, 23,  0x9492, 2, BLACK);
		}
	}
	
	if(info -> updateScreen){
		ILI9341_Draw_Text("ABOUT", 250, 220, BLACK, 2, WHITE);
	}
	
	if(info -> updateScreen || info -> updateAvg){
		uint16_t textColor = WHITE;
		uint16_t bgColor = BLACK;
		if(info -> avgHighlightTimer > 0){
			textColor = BLACK;
			bgColor = WHITE;
		}
		
		char avgTempStr[10];
		sprintf(avgTempStr,"AVG %.2d C",info -> avgTemp);
		ILI9341_Draw_Text(avgTempStr, 70, 120, textColor, 1, bgColor);
		
		char avgHumidStr[10];
		sprintf(avgHumidStr,"AVG %.3d%%",info -> avgHumid);
		ILI9341_Draw_Text(avgHumidStr, 70, 190, textColor, 1, bgColor);
		
		char avgLightStr[10];
		sprintf(avgLightStr,"AVG %.3d%%",info -> avgLight);
		ILI9341_Draw_Text(avgLightStr, 190, 190, textColor, 1, bgColor);
		info -> updateAvg = 0;
	}
	
	if(info -> avgHighlightTimer > 1000){
		info -> avgHighlightTimer = 0;
		info -> updateAvg = 1;
	}
	
	info -> updateScreen = 0;
	
	if(TP_Touchpad_Pressed()){
			uint16_t x_pos = 0;
			uint16_t y_pos = 0;
			
			uint16_t position_array[2];
			
			if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
				x_pos = position_array[0];
				y_pos = position_array[1];
				
				if(y_pos >= 1 && y_pos <= 100 && x_pos >= 200 && x_pos <= 233){
					if(info -> timerActive == 2){
						info -> timerActive = 0;
						timer -> timerDone = 0;
					}
					changeToPage(info,1);
				}
				if(y_pos >= 250 && y_pos <= 315 && x_pos >= 5 && x_pos <= 23){
					changeToPage(info,2);
			}
		}
	}

	return;
}

void timerPage(DisplayInfo* info, Timer* timer){
	
	if(timer -> isActive && timer -> timeLeftSec != (info -> timeHr*3600 + info -> timeMin*60 + info->timeSec)){
		info -> timeHr = (timer -> timeLeftSec) / 3600.0;
		info -> timeMin = ((timer -> timeLeftSec) - (info -> timeHr*3600.0))/60.0;
		info -> timeSec = (timer -> timeLeftSec) - (info -> timeHr*3600.0) - (info -> timeMin*60.0);
		
		info -> updateScreen = 1;
	}
	
	if(timer -> timerDone){
		timer -> timerDone = 0;
		info -> timerActive = 0;
	}
	
	if(info -> updateScreen){
			char strHo[10],strM[10],strS[10];
			uint16_t m = 0,ho = 0 ,s = 0;
			ho = info -> timeHr;
			m = info -> timeMin;
			s = info -> timeSec;
	
			sprintf(strHo ,"%02d:",ho);
			sprintf(strM ,"%02d:",m);
			sprintf(strS ,"%02d",s);
			ILI9341_Draw_Text(strHo, 45, 50, WHITE, 6, BLACK);
			ILI9341_Draw_Text(strM, 130, 50, WHITE, 6, BLACK);
			ILI9341_Draw_Text(strS, 215, 50, WHITE, 6, BLACK);
			ILI9341_Draw_Filled_Rectangle_Coord(45, 100, 110, 135, BLUE);
			ILI9341_Draw_Filled_Rectangle_Coord(130, 100, 195, 135, BLUE);
			ILI9341_Draw_Filled_Rectangle_Coord(215, 100, 275, 135, BLUE);
			ILI9341_Draw_Filled_Circle(155, 180, 30, RED);
		
			ILI9341_Draw_Text("TIMER", 45,5, WHITE, 2, BLACK);
			ILI9341_Draw_Text("<<", 10,5, WHITE, 2, BLACK);
		
			info -> updateScreen = 0;
	}

	if(TP_Touchpad_Pressed()){
			info -> updateScreen = 1;
			uint16_t x_pos = 0;
			uint16_t y_pos = 0;
					
      uint16_t position_array[2];

					
			if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
					x_pos = position_array[0];
					y_pos = position_array[1];
				
				
					if(y_pos > 45 && y_pos < 110 && x_pos > 100 && x_pos < 135) {
						info -> timeHr += 1;
						info -> timeHr %= 99;
					}
					if(y_pos > 130 && y_pos < 195 && x_pos > 100 && x_pos < 135) {
						info -> timeMin += 1;
						info -> timeMin %= 60;
					}
					if(y_pos > 215 && y_pos < 275 && x_pos > 100 && x_pos < 135) {
						info -> timeSec += 10;
						info -> timeSec %= 60;
					}
					if(y_pos > 125 && y_pos < 185 && x_pos > 30 && x_pos < 90) { 
						timer -> isActive = 1;
						timer -> timeLeftSec = (info -> timeHr*3600) + (info -> timeMin*60) + (info -> timeSec);
						timer -> timerDone = 0;
					}
					if(y_pos > 0 && y_pos < 30 && x_pos > 220 && x_pos < 244) { 
						changeToPage(info,0);
					}
        }
	}
}

void aboutUsPage(DisplayInfo* info){
	if(info -> updateScreen){
		ILI9341_Draw_Text("ABOUT US", 45,5, WHITE, 2, BLACK);
		ILI9341_Draw_Text("<<", 10,5, WHITE, 2, BLACK);
		
		ILI9341_Draw_Text("CREATED BY", 45, 35, WHITE, 2, BLACK);
		ILI9341_Draw_Text("61010216", 45, 50, WHITE, 2, BLACK);
		ILI9341_Draw_Text("61010279", 45, 65, WHITE, 2, BLACK);
		ILI9341_Draw_Text("61010588", 45, 80, WHITE, 2, BLACK);
		ILI9341_Draw_Text("61010627", 45, 95, WHITE, 2, BLACK);
		
		info -> updateScreen = 0;
	}
	if(TP_Touchpad_Pressed()){
			info -> updateScreen = 1;
			uint16_t x_pos = 0;
			uint16_t y_pos = 0;
					
      uint16_t position_array[2];

					
			if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
					x_pos = position_array[0];
					y_pos = position_array[1];
				
					if(y_pos > 0 && y_pos < 30 && x_pos > 220 && x_pos < 244) { 
						changeToPage(info,0);
					}
        }
		}
}

int LCDDisplay(DisplayInfo* info, Timer* timer){

	switch(info -> page){
		case 0:
			mainPage(info, timer);
			break;
		case 1:
			timerPage(info, timer);
			break;
		case 2:
			aboutUsPage(info);
			break;
	}
	
	
	return 1;
}

void timerController(DisplayInfo* info,Timer* timer){
	if(timer -> timerDone){
		// when timer is done
		if(info -> timerActive != 2) info -> updateScreen = 1;
		info -> timeHr = info -> timeMin = info -> timeSec = 0;
	}
}


// UART

void adcLightSensor(uint32_t adc_val){
	while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {}
	adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_UART_Transmit(&huart3, (uint8_t*) &"Light  ", 7, 1000);
	
	char tmp[20];
	sprintf(tmp, "%d", adc_val * 100 / 4096);
	HAL_UART_Transmit(&huart3, (uint8_t*) &tmp, strlen(tmp), 1000);
	HAL_UART_Transmit(&huart3, (uint8_t*) &" %\n\r", 6, 1000);
		
	lightValue = (int) adc_val * 100 / 4096.0;
	
	
	if(topLightBuffer == 19) // Checking Array is full or not. 
	{ 
		for(int i=0;i<19;i++){
			lightBuffer[i] = lightBuffer[i+1];
		}
		lightBuffer[topLightBuffer] = adc_val * 100 / 4096;
	} 
	else 
	{ 
		lightBuffer[topLightBuffer] = adc_val * 100 / 4096;
		topLightBuffer += 1; // top = top + 1 
		 
	} 
	
}


void i2cAM2320Sensor(){
	sprintf(str, "Temp   %4.1f C\n\r", t);
	while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET) {}
	HAL_UART_Transmit(&huart3, (uint8_t*) &str, strlen(str), 200);
			
	sprintf(str, "Humid  %4.1f ", h);
	while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET) {}
	HAL_UART_Transmit(&huart3, (uint8_t*) &str, strlen(str), 200);
	HAL_UART_Transmit(&huart3, (uint8_t*) &"%\n\r\n", 4, 1000);
		
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);
		
	HAL_I2C_Master_Receive(&hi2c1, 0x5c<<1, dataBuffer, 8, 200);
			
	uint16_t Rcrc = dataBuffer[7] << 8;
	Rcrc += dataBuffer[6];
	if (Rcrc == CRC16_2(dataBuffer, 6)) {
		uint16_t temperature = ((dataBuffer[4] & 0x7f) << 8) + dataBuffer[5];
		t = temperature / 10.0;
		t = (((dataBuffer[4] & 0x80) >> 7) == 1) ? (t * (-1)) : t;
			
		uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
		h = humidity / 10.0;
	}
	
	
	if(topTempBuffer == 19) // Checking Array is full or not. 
	{ 
		for(int i=0;i<19;i++){
			tempBuffer[i] = tempBuffer[i+1];
		}
		tempBuffer[topTempBuffer] = t;
	} 
	else 
	{ 
		tempBuffer[topTempBuffer] = t;
		topTempBuffer += 1; // top = top + 1 
		 
	} 
	
	
	if(topHumidBuffer == 19) // Checking Array is full or not. 
	{ 
		for(int i=0;i<19;i++){
			humidBuffer[i] = humidBuffer[i+1];
		}
		humidBuffer[topHumidBuffer] = h;
	} 
	else 
	{ 
		humidBuffer[topHumidBuffer] = h;
		topHumidBuffer += 1; // top = top + 1 
		 
	} 
}


void printRecord(){
	if(timerForUART < 500) return;
	timerForUART = 0;
	record ++;
	sprintf(str, "Record %05d\n\r", record);
	while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET) {}
	HAL_UART_Transmit(&huart3, (uint8_t*) &str, strlen(str), 200);
			
	adcLightSensor(adc_val);
		
	i2cAM2320Sensor();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		HAL_UART_Transmit(&huart3, (uint8_t *) "Average 10 s Period\n\r", 21, 100);
		int light = 0;
		float t = 0.0;
		float h = 0.0;
		for (int i = 0; i < 20; i++)
		{
			light += lightBuffer[i];
			t += tempBuffer[i];
			h += humidBuffer[i];
			
		}
		light = light/20;
		t = t/20;
		h = h/20;
		
		HAL_UART_Transmit(&huart3, (uint8_t*) &"Light  ", 7, 1000);
		char tmp[20];
		sprintf(tmp, "%d", light);
		HAL_UART_Transmit(&huart3, (uint8_t*) &tmp, strlen(tmp), 1000);
		HAL_UART_Transmit(&huart3, (uint8_t*) &" %\n\r", 6, 1000);
		
		sprintf(str, "Temp   %4.1f C\n\r", t);
		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET) {}
		HAL_UART_Transmit(&huart3, (uint8_t*) &str, strlen(str), 200);
				
		sprintf(str, "Humid  %4.1f ", h);
		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)==RESET) {}
		HAL_UART_Transmit(&huart3, (uint8_t*) &str, strlen(str), 200);
		HAL_UART_Transmit(&huart3, (uint8_t*) &"%\n\r\n", 4, 1000);
			
		// Save the data to EEPROM
		write_to_eeprom(light,t,h);
		lcdInfo.avgTemp = t;
		lcdInfo.avgHumid = h;
		lcdInfo.avgLight = light;
		lcdInfo.updateAvg = 1;
		lcdInfo.avgHighlightTimer = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
