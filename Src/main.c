/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "math.h"
#include "imu.h"
#include "string.h"
#include "IRremote.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "ssd1306.h"
int numsc = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define num_masters 6
#define DOF 3
#define sample_size 50
int master[num_masters][DOF][sample_size];       //array to store master gesture
int16_t temp_values[DOF][sample_size]; //array to store temp. values from take_reading function
int16_t reading[DOF][sample_size];
uint16_t devAddr = (0x50 << 1); // HAL expects address to be shifted one bit to the left
uint16_t memAddr = 0;
uint8_t Menupos = 0;
uint8_t Menupos2 = 0;
uint8_t Start = 0;
HAL_StatusTypeDef status;
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
uint32_t Mcounter, count;
uint32_t cod_but = 0;
// Поскольку я всё делаю в суперцикле тактированием таймера а не делей, всёё делается вот этими флагами
// по очереди, счётчики положения прямоу, вверх, вниз.
//startmov - старт анализа(по переполнению 50), ждёт первого резкого движения, ждёт когда считает 50 циклов.

uint8_t calcforw, calcdown, calcup, startmov, redywait, redyread,take_reading_master_guesture;
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
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
uint8_t i;
uint8_t i2cBuff[8];
uint16_t ax, ay, az;
float fAX_Cal, fAY_Cal, fAZ_Cal;
float fAX, fAY, fAZ;
#define mpu6050Address 0xd0
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float op_time;
int line = 0, Send = 1;
int var;
int x = 0;
int8_t giro_step = 0;
int TIM_Pulse = 0;
int Giro_Pulse = 0;
uint16_t Diod_Pulse = 0;
unsigned long CODE_TO_SEND;
unsigned long CODE_TO_REC;
char trans_str[96] = { 0, };
int DTW_score[num_masters], min_score, location;
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */
	/* MCU Configuration--------------------------------- -----------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */
	/* Configure the system clock */
	SystemClock_Config();
	MX_TIM2_Init();
	NVIC_SetPriority(TIM2_IRQn, 15);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */
	MX_GPIO_Init();
	MX_I2C2_Init();
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(2, 0);
	ssd1306_WriteString("Start init mpu. ", Font_7x10, White);
	ssd1306_UpdateScreen();
	MPU6050_Init();
	MPU6050_Calibrate();
	MX_TIM4_Init();
	MX_TIM1_Init();
	HAL_Delay(600);
	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	ssd1306_Fill(Black);
	ssd1306_SetCursor(2, 0);
	ssd1306_WriteString("Init MPU complit", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(600);
	my_enableIRIn();
	HAL_TIM_Base_Start_IT(&htim1);

	static char *decode_str[] = { "UNUSED", "UNKNOWN", "RC5", "RC6", "NEC",
			"SONY", "PANASONIC", "JVC", "SAMSUNG", "WHYNTER", "AIWA_RC_T501",
			"LG", "SANYO", "MITSUBISHI", "DISH", "SHARP", "DENON", "PRONTO" };
	int16_t mpu6050data[6];
	int calctime;
	int calctime2;
	// ШИМ-
	Diod_Pulse=600;

	calctime2 = 0;

	 // enable the use DWT
	*DEMCR = *DEMCR | 0x01000000;

	// Reset cycle counter
	*DWT_CYCCNT = 0;

	// enable cycle counter
	*DWT_CONTROL = *DWT_CONTROL | 1 ;

	// читаем из ипрама в рам таблицы движений. что-бы оперативно их сравнивать. читать в полёте не выходит. долго
	ssd1306_Fill(Black);
			ssd1306_SetCursor(2, 0);
			ssd1306_WriteString("Read master guesture from EEPROM", Font_7x10, White);
			ssd1306_UpdateScreen();
	for (int i = 0; i < num_masters; i++) {
		EEPROM_read(i);
		Diod_Pulse+=200;
	}

		Diod_Pulse=100;
			ssd1306_Fill(Black);
			ssd1306_SetCursor(2, 0);
			ssd1306_WriteString("Init complite", Font_7x10, White);
			ssd1306_UpdateScreen();
	HAL_Delay(500);
	lcd_menu(1);
	while (1) {

		// модуль обработки нажатия кнопок. останется только на мастер палочке.
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
			lcd_menu(1);
			HAL_Delay(50);
		} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET) {
			lcd_menu(2);
			HAL_Delay(50);
		}
		if (Start == 1) {
			if (Giro_Pulse >= 3)
				{
				MPU6050_GetAllData(mpu6050data);
				fAX += mpu6050data[0];
				fAY += mpu6050data[1];
				fAZ += mpu6050data[2];
				Giro_Pulse = 0;
				giro_step++;
				}
			if (giro_step == 4) {
				// получили усреднённое значение. Делю его на количество считываний и беру это зхначение в массив для сравнения
				// работаю в этом условии, так как тут у нас актуальные фильтрованные данные по прерываниям, времени и.т.д
				// обновляется примерно кадные 0.022с (20 мс)
				fAX /= 4;
				fAY /= 4;
				fAZ /= 4;
				for (int j = 0; j < sample_size - 1; j++) {
					reading[0][j] = temp_values[0][j + 1];
					reading[1][j] = temp_values[1][j + 1];
					reading[2][j] = temp_values[2][j + 1];
				}
				// добираем новое значение
				reading[0][sample_size - 1] = fAX - fAX_Cal;
				reading[1][sample_size - 1] = fAY - fAY_Cal;
				reading[2][sample_size - 1] = fAZ - fAZ_Cal;
				for (int i = 0; i < DOF; i++) {
					for (int j = 0; j < sample_size; j++) {
						temp_values[i][j] = reading[i][j];
					}
				}
				fAX = 0;
				fAY = 0;
				fAZ = 0;
				giro_step = 0;
				calctime++;
				// 0.0016s read guesture 1
				// этот кусочек для отладки
				if (calctime >= 50) {
						// флаг выставляется ниже, что мы готовы читать именно жест. тут мы просто считаем 50 считываний
						// скидываем флаг redyread в ноль и ставим что можно читать жест. вуа-ля! ну порнуха же (
					if (redyread >= 1)
					{

						if (take_reading_master_guesture>=1 )
						{
							take_reading_master_guesture=0; //считали жест для памяти
							Start=0;
							redyread = 0;
							count=*DWT_CYCCNT-Mcounter;
							float op_time=count/72000000.0f;// count/F_CPU

						}
						else
						{
							startmov = 1; // считали жест для анализа
							redyread = 0;
						}
					}
					// отладка
					/*
					 ssd1306_Fill(Black);
					 snprintf(trans_str, 20, "nom calc: %i\n", calctime2);
					 ssd1306_SetCursor(0,0);
					 ssd1306_WriteString(trans_str, Font_7x10, White);
					 snprintf(trans_str, 96, "x:%d   %s\n",temp_values[0][48],"   ");
					 ssd1306_SetCursor(0, 16);
					 ssd1306_WriteString(trans_str, Font_7x10, White);
					 snprintf(trans_str, 96,  "u:%d   %s\n",temp_values[1][48],"   ");
					 ssd1306_SetCursor(0, 26);
					 ssd1306_WriteString(trans_str, Font_7x10, White);
					 snprintf(trans_str, 96, "z:%d   %s\n",temp_values[2][48],"   ");
					 ssd1306_SetCursor(0, 36);
					 ssd1306_WriteString(trans_str, Font_7x10, White);
					 ssd1306_UpdateScreen();*/
					calctime2++;
					calctime = 0;

				}
				// тут проверяем на 3 условия
				// 1. мы замерли в горизонтальной плоскости паралельно полу. Это движение 1. для считывания с палочки заклинания комнаты
				// тут самые сложные жесты. Подсказки на них будут в книгах.
				// 2. мы замерли с паолчкой вверх перпендикулярно полу. движения 2. дуэльный клуб. Загружаем в палочку из памяти боевые жесты
				// тут будут относительно сложные жесты. 6 жестов. 3 блока 3 атаки.
				// 3. Палочка вниз перпендикулярно полу. движение 3. аркадный режим комнаты. для бития монстров
				// самые простые жесты. но с реалтайм чтением. буквально сделаю 3-4 жеста. но что-бы можно было в реалтайме максимально быстро следить

				// модуль первого условия. замерли горизонтально. и так стоим секунду. После этого взводим сигнал на готовность к заклинанию.
				// после того как по любой координате прошло ускорение больше +-350 едениц (сделать переменную чувствительности?) начинаем чтение
				// 50 координат. потом сразу после читаем, похоже это на то что есть в памяти ( 0.11 мс). если не оно, ждём чего нить дальше.

				if ((((temp_values[0][sample_size - 1] >= -100)&& (temp_values[0][sample_size - 1] <= 100))
				  & ((temp_values[1][sample_size - 1] >= -100)&& (temp_values[1][sample_size - 1] <= 100))
				  & ((temp_values[2][sample_size - 1] >= -70 )&& (temp_values[2][sample_size - 1] <= 70)))
				  &	(redyread==0))
					//x  +-70 //y  +-100 //z +-50
				{
					calcforw++;
					if (Diod_Pulse<=6000)
					{Diod_Pulse+=60;}
					if (calcforw >= 20)	// ждём порядка 700 мc. это 35 тиков.
					{
						redywait = 1; // я знаю что палочка зависла выставляю флаг ждать жест. с первых резких движений - чтаем жест
						if (Diod_Pulse<=6000)
						{Diod_Pulse+=600;}
					}
				} else {
					calcforw = 0;
					Diod_Pulse=0;

				} // Сбросили таймер
				//тут продолжим...
			}

			// И вот тут НАКОНЕЦ мы выставляем флаг чтения жеста. выставляем его по любому резкому движению. первой резкой координате больще 150;
			if (  ((temp_values[0][sample_size - 1] <= -150)|| (temp_values[0][sample_size - 1] >= 150))
				& ((temp_values[1][sample_size - 1] <= -150)|| (temp_values[1][sample_size - 1] >= 150))
				& ((temp_values[2][sample_size - 1] <= -150)|| (temp_values[2][sample_size - 1] >= 150))
				& (redywait == 1))
				//x  +-150 //y  +-150 //z +-150
			{
				redyread = 1;
				redywait = 0;
				calctime = 0; // А это мой счётчик. он считает до 50 и выводит либо на экран, либо передаёт обработку на чтение
				Mcounter=*DWT_CYCCNT;
			}
			if (startmov == 1) {
				startmov = 0;
				for (i = 0; i < num_masters; i++) //calculate DTW_score for each master
				{
					DTW_score[i] = calc_DTW_score(i);
					min_score = DTW_score[0];  //finding minimum of DTW_scores
					//0,10s цикл длиться столько. буду считать это тактированием. сука, как долго ((((
				}
				for (i = 1; i < num_masters; i++) {
					if (DTW_score[i] < min_score) {
						min_score = DTW_score[i];
					}
				}
				for (i = 0; i < num_masters; ++i) {
					if (min_score == DTW_score[i]) {
						if (i != 0) { // resting position

							if (i==1)
							{
								cod_but = 0x11111110; // РєРѕРґ РєРЅРѕРїРєРё РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё
								sendSAMSUNG(cod_but, 32); // РїСЂРѕС‚РѕРєРѕР» РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё, РєРѕРґ РєРЅРѕРїРєРё Рё РґР»РёРЅР° РїР°РєРµС‚Р° РІ Р±РёС‚Р°С… (Cod: 0x707048b7 | Type: SAMSUNG | Bits: 32)
								my_enableIRIn();

							}
							else if (i==2)
							{
								cod_but = 0x11111101; // РєРѕРґ РєРЅРѕРїРєРё РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё
								sendSAMSUNG(cod_but, 32); // РїСЂРѕС‚РѕРєРѕР» РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё, РєРѕРґ РєРЅРѕРїРєРё Рё РґР»РёРЅР° РїР°РєРµС‚Р° РІ Р±РёС‚Р°С… (Cod: 0x707048b7 | Type: SAMSUNG | Bits: 32)
								my_enableIRIn();


							}
							else if (i==3)
							{
							cod_but = 0x11111011; // РєРѕРґ РєРЅРѕРїРєРё РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё
							sendSAMSUNG(cod_but, 32); // РїСЂРѕС‚РѕРєРѕР» РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё, РєРѕРґ РєРЅРѕРїРєРё Рё РґР»РёРЅР° РїР°РєРµС‚Р° РІ Р±РёС‚Р°С… (Cod: 0x707048b7 | Type: SAMSUNG | Bits: 32)
							my_enableIRIn();
							}


							snprintf(trans_str, 20, "nom matr:  %i\n", i);
							ssd1306_Fill(Black);
							ssd1306_SetCursor(3, 10);
							ssd1306_WriteString(trans_str, Font_7x10, White);
							ssd1306_UpdateScreen();
							HAL_Delay(500);
							break;
						}
					}
				}
			}
		}

		// модуль приёма ИК сигнала. сука очень важный и требовательный
		if (my_decode(&results)) {
			////////// РІС‹РІРѕРґ РґРµРєРѕРґРёСЂРѕРІР°РЅС‹С… РґР°РЅРЅС‹С… ///////////////
			snprintf(trans_str, 96, "Cod: %p | Type: %s | Bits: %d\n",
					(void*) results.value, decode_str[results.decode_type + 1],
					results.bits);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(3, 10);
			ssd1306_WriteString(trans_str, Font_7x10, White);
			ssd1306_UpdateScreen();
			my_resume();
			HAL_Delay(20);

			//Модуль отправки ИК сигнала. ещё более важный и требовательный

			cod_but = 0x11111111; // РєРѕРґ РєРЅРѕРїРєРё РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё
			sendSAMSUNG(cod_but, 32); // РїСЂРѕС‚РѕРєРѕР» РїРѕР»СѓС‡РµРЅС‹Р№ РїСЂРё РґРµРєРѕРґРёСЂРѕРІР°РЅРёРё, РєРѕРґ РєРЅРѕРїРєРё Рё РґР»РёРЅР° РїР°РєРµС‚Р° РІ Р±РёС‚Р°С… (Cod: 0x707048b7 | Type: SAMSUNG | Bits: 32)
			my_enableIRIn();

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

void lcd_menu(int key) { //copy readings from temp_values to selected master
	//Menupos
	// snprintf(trans_str, 20, "cyc: %i\n", cycles_count);
	// Первая страница
	int nm = -1;
	if (Menupos == 0) {
		ssd1306_Fill(Black);
		ssd1306_SetCursor(7, 0);
		ssd1306_WriteString("Start guesture", Font_7x10, White);
		ssd1306_SetCursor(7, 12);
		ssd1306_WriteString("Record  guesture", Font_7x10, White);
		ssd1306_SetCursor(7, 24);
		ssd1306_WriteString("save to EEEPROM ", Font_7x10, White);
		ssd1306_SetCursor(7, 36);
		ssd1306_WriteString("Print eeeprom", Font_7x10, White);
		ssd1306_UpdateScreen();
		Menupos = 1;
	}

	//движение вперёд звёздочки
	if (key == 1) {
		ssd1306_SetCursor(0, Menupos2);
		ssd1306_WriteString("*", Font_7x10, White);
		ssd1306_SetCursor(0, Menupos2 - 12);
		ssd1306_WriteString(" ", Font_7x10, White);

		HAL_Delay(200);
		if (Menupos2 > 48) {
			ssd1306_SetCursor(0, Menupos2 - 12);
			ssd1306_WriteString(" ", Font_7x10, White);
			Menupos2 = 0;
			ssd1306_SetCursor(0, Menupos2);
			ssd1306_WriteString("*", Font_7x10, White);
		}
		ssd1306_UpdateScreen();
		Menupos2 += 12;
	}

	if ((key == 2) && (Menupos2 == 12))	// анализируем жест
	{
		HAL_Delay(70);
		Start = 1;
	}
	if ((key == 2) && (Menupos2 == 24))		    //пишем жест
	{
		HAL_Delay(110);
		take_reading_master_guesture=1;
		Start=1;
		Menupos = 0;
		Menupos2 = 0;
	}
	if ((key == 2) && (Menupos2 == 36))		    //пишем в память
	{		    // надо получить номер матрицы
		HAL_Delay(300);
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) != GPIO_PIN_SET) {
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
				nm++;
				ssd1306_Fill(Black);
				snprintf(trans_str, 20, "nom matr: %i\n", nm);
				ssd1306_SetCursor(3, Menupos2);
				ssd1306_WriteString("               ", Font_7x10, White);
				ssd1306_SetCursor(3, Menupos2);
				ssd1306_WriteString(trans_str, Font_7x10, White);
				ssd1306_UpdateScreen();
				HAL_Delay(200);
			}
		}
		EEPROM_write(nm);
		Menupos2 = 0;

	}
	if ((key == 2) && (Menupos2 == 48))		    //читаем жест из памяти
	{
		HAL_Delay(300);
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) != GPIO_PIN_SET) {
			HAL_Delay(30);
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
				nm++;
				ssd1306_Fill(Black);
				snprintf(trans_str, 20, "nom matr: %i\n", nm);
				ssd1306_SetCursor(3, Menupos2);
				ssd1306_WriteString("               ", Font_7x10, White);
				ssd1306_SetCursor(3, Menupos2);
				ssd1306_WriteString(trans_str, Font_7x10, White);
				ssd1306_UpdateScreen();
				HAL_Delay(200);
			}
		}

		EEPROM_read(nm);
		ssd1306_Fill(Black);
		ssd1306_SetCursor(3, 10);
		ssd1306_WriteString("Read complyte !", Font_7x10, White);
		ssd1306_UpdateScreen();
		HAL_Delay(200);
		Menupos = 0;
		lcd_menu(1);
		Menupos2 = 0;
	}
}

void EEPROM_write(int master_select) { //store master gestures in EEPROM of arduino

	int addr;
	uint8_t raw[2];
	uint8_t rmsg[2];
	int16_t data;
	int16_t temp1, temp2;
	// master_select - номер массива в котором храниться жест/заклинание
	addr = (master_select * sample_size * DOF) * 2; // умножаем на 2. потому что пишем по 2 бита ( 16 байт)

	for (;;) { // wait...
		status = HAL_I2C_IsDeviceReady(&hi2c2, devAddr, 1, HAL_MAX_DELAY);
		if (status == HAL_OK)
			break;
	}

	for (int i = 0; i < DOF; i++) {
		for (int j = 0; j < sample_size; j++) {
			// тут как-то краво читается
			// rebild fo byit
			temp1 = (temp_values[i][j] & 0xFF);
			temp2 = ((temp_values[i][j] >> 8) & 0xFFFF);
			//raw[0]=(temp_values[i][j]& 0xFF);
			//raw[1]=((temp_values[i][j]>> 8) & 0xFFFF );
			raw[0] = temp1;
			raw[1] = temp2;

			HAL_I2C_Mem_Write(&hi2c2, devAddr, addr, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*) raw, sizeof(raw), HAL_MAX_DELAY);
			HAL_Delay(20);
			HAL_I2C_Mem_Read(&hi2c2, devAddr, addr, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*) rmsg, sizeof(raw), HAL_MAX_DELAY);
			data = (int16_t*) ((rmsg[1] << 8) | rmsg[0]);

			if (temp_values[i][j] != data) {
				snprintf(trans_str, 40, "temp_values: %i\n", temp_values[i][j]);
				ssd1306_Fill(Black);
				ssd1306_SetCursor(3, 2);
				ssd1306_WriteString(trans_str, Font_7x10, White);
				snprintf(trans_str, 40, "data: %i\n", data);
				ssd1306_SetCursor(3, 10);
				ssd1306_WriteString(trans_str, Font_7x10, White);
				snprintf(trans_str, 96, "i: %p | j: %s \n", i, j);
				ssd1306_SetCursor(8, 20);
				ssd1306_WriteString(trans_str, Font_7x10, White);
				ssd1306_SetCursor(8, 30);
				ssd1306_WriteString("Alarm!", Font_7x10, White);
				ssd1306_UpdateScreen();
				HAL_Delay(20000);
			}

			addr += 2;

			// маленькая проверка памяти
			// тут мы пробуем систему записи в еепром. пищем по 2 бита в массив. читаем так-же. по 2 бита. адрес сразу щёлкаем на 2 вперёд
		}
	}
	HAL_Delay(200);
	Menupos = 0;
	lcd_menu(1);
}

void EEPROM_read(int master_select) { //retrieve master gestures from EEPROM
	int i, j, addr;
	int16_t data;
	int8_t rmsg[2];
	//	for (;;) { // wait...
	//	status = HAL_I2C_IsDeviceReady(&hi2c2, devAddr, 1, HAL_MAX_DELAY);
	//		if (status == HAL_OK)
	//		break;
	//	}
	addr = (master_select * sample_size * DOF) * 2;
	for (i = 0; i < DOF; i++) {
		for (j = 0; j < sample_size; j++) {
			HAL_I2C_Mem_Read(&hi2c2, devAddr, addr, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*) rmsg, 2, HAL_MAX_DELAY);
			data = ((rmsg[1] << 8) | rmsg[0]);
			addr += 2;
			master[master_select][i][j] = data;
		}
	}

}


int abs_sub(int a, int b) { //finds the absolute subtraction (difference) of 2 entered numbers
	int c = a - b;
	if (c < 0)
		c = -c;
	return c;
}

int Min(int a, int b, int c) { //finds the minimum of 3 numbers entered
	if (a < b && a < c)
		return a;
	else if (b < a && b < c)
		return b;
	else
		return c;
}

int calc_DTW_score(int master_select) { //calculates DTW score between 2 time series
	int print_DTW = 0;       //1 to print DTW matrix, else 0
	int i;
	//creating variables to store input parameters
	int matrix_size = sample_size + 1;
	unsigned int a[matrix_size][matrix_size];    //matrix of DTW
	unsigned int DTW_score[DOF];
	unsigned int final_DTW_score = 0;

	for (i = 0; i < DOF; i++) {
		int x = 0, y = 0;

		//moving input series to DTW matrix
		for (x = 0; x < sample_size; x++)
			a[x + 1][0] = master[master_select][i][x];
		for (y = 0; y < sample_size; y++)
			a[0][y + 1] = temp_values[i][y];

		a[0][0] = 0;
		a[1][1] = abs_sub(a[1][0], a[0][1]);  //as no previously computed values
		a[2][1] = abs_sub(a[2][0], a[0][1]);
		a[1][2] = abs_sub(a[1][0], a[0][2]);

		x = 1;
		for (y = 2; y < matrix_size; y++) {
			a[x][y] = abs_sub(a[x][0], a[0][y]) + a[x][y - 1];
		}

		y = 1;
		for (x = 2; x < matrix_size; x++) {
			a[x][y] = abs_sub(a[x][0], a[0][y]) + a[x - 1][y];
		}

		for (x = 2; x < matrix_size; x++) {
			for (y = 2; y < matrix_size; y++) {
				a[x][y] = abs_sub(a[x][0], a[0][y])
								+ Min(a[x][y - 1], a[x - 1][y], a[x - 1][y - 1]);
			}

		}

		//**calculating DTW score**

		x = sample_size;
		y = sample_size;
		DTW_score[i] = a[x][y];

		while (x != 0 && y != 0) {
			if (a[x - 1][y - 1] <= a[x][y - 1]
										&& a[x - 1][y - 1] <= a[x - 1][y]) {
				DTW_score[i] = DTW_score[i] + a[x - 1][y - 1];
				x--;
				y--;
			}

			else if (a[x][y - 1] <= a[x - 1][y - 1]
											 && a[x][y - 1] <= a[x - 1][y]) {
				DTW_score[i] = DTW_score[i] + a[x][y - 1];
				y--;
			} else {
				DTW_score[i] = DTW_score[i] + a[x - 1][y];
				x--;
			}
		}
	}
	for (i = 0; i < DOF; i++)
		final_DTW_score = final_DTW_score + DTW_score[i];
	final_DTW_score = final_DTW_score / DOF;
	return final_DTW_score;
}

void TIM1_UP_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	TIM_Pulse++;
	Giro_Pulse++;
	TIM2->CCR1 = Diod_Pulse;
		//TIM2->CCR2 = 666;
	/* USER CODE END WHILE */

	/* USER CODE END TIM3_IRQn 1 */
}

void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer,
		uint8_t TXBUFFERSIZE) {
	while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) I2C_ADDRESS << 1,
			(uint8_t*) aTxBuffer, (uint16_t) TXBUFFERSIZE, (uint32_t) 1000)
			!= HAL_OK) {
		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {
			//   _Error_Handler(__FILE__, aTxBuffer[0]);
		}

	}

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
	}
}

void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer,
		uint8_t RXBUFFERSIZE) {

	I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);

	while (HAL_I2C_Master_Receive(&hi2c2, (uint16_t) I2C_ADDRESS << 1,
			aRxBuffer, (uint16_t) RXBUFFERSIZE, (uint32_t) 1000) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {
			//  _Error_Handler(__FILE__, __LINE__);
		}
	}

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
	}
}

void MPU6050_Init(void) {

	uint8_t buffer[7];

	// включение/побудка модуля
	buffer[0] = MPU6050_RA_PWR_MGMT_1;
	buffer[1] = 0x00;
	I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);

	// конфиг гироскопа на ±500°/с
	buffer[0] = MPU6050_RA_GYRO_CONFIG;
	buffer[1] = 0x8;
	I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);

	// конфиг акселерометра на ±8g
	buffer[0] = MPU6050_RA_ACCEL_CONFIG;
	buffer[1] = 0x10;
	I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);
}

void MPU6050_Calibrate(void) {

	int16_t mpu6050data[6];
	uint16_t iNumCM = 500;
	for (int i = 0; i < iNumCM; i++) {
		MPU6050_GetAllData(mpu6050data);
		fAX_Cal += mpu6050data[0];
		fAY_Cal += mpu6050data[1];
		fAZ_Cal += mpu6050data[2];
		HAL_Delay(3); // 3 сек на калибровку
	}
	fAX_Cal /= iNumCM;
	fAY_Cal /= iNumCM;
	fAZ_Cal /= iNumCM;

}

void MPU6050_GetAllData(int16_t *Data) {

	uint8_t accelbuffer[14];

	// с 0x3B 14 следующих регистров содержат данные измерения модуля
	I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H,
			accelbuffer, 14);

	/* Registers 59 to 64 – Accelerometer Measurements */
	for (int i = 0; i < 3; i++)
		Data[i] = (((int16_t) ((uint16_t) accelbuffer[2 * i] << 8)
				+ accelbuffer[2 * i + 1])) / 10;

	/* Registers 65 and 66 – Temperature Measurement */
	//пока пропускаем Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	/* Registers 67 to 72 – Gyroscope Measurements */
	for (int i = 4; i < 7; i++)
		Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8)
				+ accelbuffer[2 * i + 1]);

}

void IMU_CalculateAll_Data(void) {

	int16_t mpu6050data[6];
	// РїРµСЂРµРјРµС‰Р°РµРј РІ РјР°СЃСЃРёРІ mpu6050data РІСЃРµ РґР°РЅРЅС‹Рµ СЃ РґР°С‚С‡РёРєР°
	MPU6050_GetAllData(mpu6050data);

	// char msg[40];
	// uint16_t len = sprintf(msg, "X:%i Y:%i Z:%i\r\n",(int16_t)(mpu6050data[3]- fGX_Cal), (int16_t)(mpu6050data[4]- fGY_Cal), (int16_t)(mpu6050data[5] - fGZ_Cal));
	// HAL_UART_Transmit(&huart1, (uint8_t*) msg, len, HAL_MAX_DELAY);

	// Р РђР‘РћРўРђ РЎ Р“Р�Р РћРЎРљРћРџРћРњ
	//РєР°Рє РїРµСЂРµРІРµСЃС‚Рё Р®РЅРёС‚С‹ РІ СЂРµР°Р»СЊРЅС‹Рµ РіСЂР°РґСѓСЃС‹ СЂР°Р·РІРµСЂРЅСѓС‚Рѕ РЅР° РїСЂРёРјРµСЂРµ Roll
	float Roll = mpu6050data[3] - fGX_Cal; // РѕС‚РЅРѕСЃРёС‚РµР»СЊРЅРѕ "РЅСѓР»СЏ"
	Roll = Roll / 65.5 / 50;
	MPU6050_Data.aRoll += Roll;

	//СЃРѕРєСЂР°С‰РµРЅРЅР°СЏ Р·Р°РїРёСЃСЊ РґР»СЏ Pitch
	MPU6050_Data.aPitch += (mpu6050data[4] - fGY_Cal) / 65.5 / 50;

	float Yaw = (mpu6050data[5] - fGZ_Cal) / 65.5 / 50;
	MPU6050_Data.aYaw += Yaw;
	//СѓС‡РёС‚С‹РІР°РµРј РїР°СЂР°РјРµС‚СЂ РїРѕ Z РµСЃР»Рё РїРѕ РЅРµРјСѓ РµСЃС‚СЊ РґРІРёР¶РµРЅРёРµ
	if (abs(Yaw) > 1) {
		float _Y = sin(Yaw * 3.1415 / 180);
		MPU6050_Data.aPitch += MPU6050_Data.aRoll * _Y;
		MPU6050_Data.aRoll -= MPU6050_Data.aPitch * _Y;
	}

	// Р РђР‘РћРўРђ РЎ РђРљРЎР•Р›Р•Р РћРњР•РўР РћРњ
	float roll4macc = atan2(mpu6050data[1],
			(sqrt( SQR(mpu6050data[0]) + SQR(mpu6050data[2])))) * R2DEG;
	float pitch4macc = atan2(mpu6050data[0],
			(sqrt(SQR(mpu6050data[1]) + SQR(mpu6050data[2])))) * R2DEG;

	// РєРѕСЂСЂРµРєС†РёСЏ РґСЂРµР№С„Р° РЅСѓР»СЏ РіРёСЂРѕСЃРєРѕРїР°
	// Р”Р»СЏ РєРѕСЂСЂРµРєС‚РёСЂРѕРІРєРё СѓРіР»РѕРІ РІРѕСЃРїРѕР»СЊР·СѓРµРјСЃСЏ РєРѕРјРїР»РµРјРµРЅС‚Р°СЂРЅС‹Рј С„РёР»СЊС‚СЂРѕРј
	// A = (1-K)*Ag + K*Ac
	MPU6050_Data.aPitch = MPU6050_Data.aPitch
			* (1 - MPU6050_KOEF_COMPL)+ pitch4macc * MPU6050_KOEF_COMPL;
	MPU6050_Data.aRoll = MPU6050_Data.aRoll
			* (1 - MPU6050_KOEF_COMPL)+ roll4macc * MPU6050_KOEF_COMPL;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 720000;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 6000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 30000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : recive_IR_Pin */
	GPIO_InitStruct.Pin = recive_IR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(recive_IR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
