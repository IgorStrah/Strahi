/* USER CODE BEGIN Header */
/**
 *
 * NWE!
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

int numsc = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define gesture_massive 3
#define num_masters 12
#define DOF 3
#define sample_size 50
int master[num_masters][DOF][sample_size];       //array to store master gesture
int16_t temp_values[DOF][sample_size]; //array to store temp. values from take_reading function
int16_t reading[DOF][sample_size];
int16_t velocity[2][2]; //старое новое значение ускорения/ Х и  У
int16_t position[2][2]; //старое новое значение позиции / Х и  У
uint16_t devAddr = (0x51 << 1); // HAL expects address to be shifted one bit to the left
uint16_t memAddr = 0;
uint8_t Menupos = 0; // позиция  в меню. по ней определяется так-же что я выбрал кнопками
uint8_t Start = 0; // начало анализа "жеста". Без этого флага не читается акселерометр
uint8_t debug = 0; // флаг дебаша. показывает координаты акселерометра. выбирается кнопкой
uint8_t pos = 0; // флаг счётчика счёта светодиода. 1-крче.0-темнее.
uint8_t temp0;
//Mode
uint8_t mode=2; // Режим чтения. 0- ничего, 1-аркада. 2-магия. 3-дуэль
uint8_t arcademode=0; // Жест аркады. 0 = пусто. 1 - лево,2 - право. 3 верх. 4 низ. 5 перы. после отправки сброс на 0
uint8_t fadingmode=0; // Режим скидывания счётчика шим. 1- гасим шим аркады, 2-шим магии. 3-шим дуэли. 4ы-шим жеста
int8_t nm; // номер ячейки пимяти куда пишем жест
HAL_StatusTypeDef status;
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
uint32_t Mcounter, count;
uint32_t cod_but = 0;
//startmov - СЃС‚Р°СЂС‚ Р°РЅР°Р»РёР·Р°(РїРѕ РїРµСЂРµРїРѕР»РЅРµРЅРёСЋ 5debug0), Р¶РґС‘С‚ РїРµСЂРІРѕРіРѕ СЂРµР·РєРѕРіРѕ РґРІРёР¶РµРЅРёСЏ, Р¶РґС‘С‚ РєРѕРіРґР° СЃС‡РёС‚Р°РµС‚ 50 С†РёРєР»РѕРІ.

uint8_t calcforw,calcforwA,calcforwM, calcdown, calcup, startmov, redywait, redyread,take_reading_master_guesture;
int velocityX,velocityY,velocityZ;

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
uint16_t Diod_Pulse1 = 0;
uint16_t Diod_Pulse2 = 0;
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
	HAL_Init();
	SystemClock_Config();
	MX_TIM2_Init();
	NVIC_SetPriority(TIM2_IRQn, 15);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

	TIM2->CCR1 = 50;
	TIM2->CCR2 = 50;
	TIM2->CCR3 = 500;

	MX_GPIO_Init();
	MX_I2C2_Init();

	HAL_StatusTypeDef result;
		for (i=1; i<128; i++)
		 	{
		 	  result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 2, 2);
		 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		 	  {
		 		  printf("."); // No ACK received at that address
		 	  }
		 	  if (result == HAL_OK)
		 	  {
		 		snprintf(trans_str, 20, "0x%X", i);
		 		printf(trans_str);
		 	  }
		 	}

	MPU6050_Init();
	MPU6050_Calibrate();

	MX_TIM4_Init();
	MX_TIM1_Init();
	HAL_Delay(600);
		//Scan I2C
	int8_t strlcd=9;

	HAL_Delay(1000);
	HAL_TIM_Base_Start_IT(&htim1);
	my_enableIRIn();
	static char *decode_str[] = { "UNUSED", "UNKNOWN", "RC5", "RC6", "NEC",
			"SONY", "PANASONIC", "JVC", "SAMSUNG", "WHYNTER", "AIWA_RC_T501",
			"LG", "SANYO", "MITSUBISHI", "DISH", "SHARP", "DENON", "PRONTO" };
	int16_t mpu6050data[6];
	int calctime;
	int calctime2;

	Diod_Pulse=600;

	calctime2 = 0;

	 // enable the use DWT
	*DEMCR = *DEMCR | 0x01000000;
	// Reset cycle counter
	*DWT_CYCCNT = 0;
	// enable cycle counter
	*DWT_CONTROL = *DWT_CONTROL | 1 ;


	HAL_Delay(500);
	for (int i = 0; i < num_masters; i++) {
		EEPROM_read(i);
		Diod_Pulse+=600;
	}
			Diod_Pulse=10;
			HAL_Delay(500);



// Если кнопка нашажа - запускаемся с ожиданием меню. иначе сразу херачим читать жесты

if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) != GPIO_PIN_RESET) {
Start=1;
HAL_Delay(100);
}

	while (1) {

		// модуль обработки нажатия кнопок. останется только на мастер палочке.
        // первая кнопка  - шаг. вторая - выбор. При загрузке с зажатой первой кнопкой - входим в меню программирования

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
			menu(1);
			HAL_Delay(50);
		} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {
			menu(2);
			HAL_Delay(50);
		}


		if (Start == 1) {
			//Читаем гироскоп
			if (Giro_Pulse >= 3)
				{
				MPU6050_GetAllData(mpu6050data);
				fAX += mpu6050data[0];
				fAY += mpu6050data[1];
				fAZ += mpu6050data[2];
				Giro_Pulse = 0;
				giro_step++;
				}
			//Приводим данные и пишим в массив
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

				// 0.0016s read guesture 1
				// чтения положения палочки. делается постоянно. дабы менять режим что мы делаем
				// пригот овились к аркаде, магиии, или дуэли

				// expect_arcade(); // читаем положение палочки  для аркады
				// mode =1 включает режим магии
				 expect_magic(); // читаем для положения магии ( вернуться к магии
				// тут у меня есть  данные флага mode 1-аркада. 2 магия.

				// Ещё будет дуэль



				if (mode==2)
				{
					// актуально только в режиме магии
				 	expect_fading();
				 // Жду что-бы палочка замерла. Если она замерла разгорается светодиод и выставляется флаг готовности
									// чтения заклинания. Потому что заклинание нужно читать с первого резкого движения....
				calctime++;
					if (calctime >= sample_size) {
						// Запускается чтение 50 значенией которые потом анализируются как жест
						// флаг выставляется ниже, что мы готовы читать именно жест.  тут мы просто считаем 50 считываний
						// скидываем флаг redyread в ноль и ставим что можно читать жест. вуа-ля! ну порнуха же (
							if (redyread >= 1)
							{
								//Если пишем жест для памяти - работает этот кусок. он обрубает чтение после
								//первых 50 значений которые считали после первого резкого движения
								if (take_reading_master_guesture>=1 )
								{ // модуль читает жест для записи в память. после его чтения флаг работы скидывается и больше
									//  с гироскопа данные не читаются
									take_reading_master_guesture=0; //считали жест для памяти
									Start=0;
									redyread = 0;

								}
								else
								{
									startmov = 1; // считали жест для анализа. 50 значений
									redyread = 0; // Готов считать именно  жест а не набираю просто информацию
								}
							}

							calctime = 0;
						}
				}
				else if (mode==1)
				{

					CODE_TO_SEND=0;
					CODE_TO_SEND=mapcalc(mpu6050data[0],-600,600,0,99);
					CODE_TO_SEND*=1000000;
					CODE_TO_SEND+=(mapcalc(mpu6050data[1],-500,500,0,99))*1000;

					HAL_Delay(40);
					enableIROut(36);
	     	 		sendSAMSUNG(CODE_TO_SEND, 32);
		   	 	    my_enableIRIn();HAL_Delay(150);
		   	 		cod_but = 0;
			 		temp0=0;



			 		// плавно растём и гаснем
			 		if (Diod_Pulse>=6000)
					{pos=0;}
					else if (Diod_Pulse<=1)
			 		{pos=1;}
			 		if (pos==1)
			 		{Diod_Pulse+=100;Diod_Pulse1+=100;Diod_Pulse2+=100;} // мигаем неким цветом мол всё верно
			 		else
			 		{Diod_Pulse-=100;Diod_Pulse1-=100;Diod_Pulse2-=100;}



				};

			    }

			if (expect_gesture()==1)
			{calctime=0;}

			// Сравнение жестов из набора жестов в памяти
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
					if ((min_score == DTW_score[i]))//&& (min_score<600000)) {
					{

					//	CODE_TO_SEND = min_score;
					//	sendSAMSUNG(CODE_TO_SEND, 32);
					//	my_enableIRIn();
					//	HAL_Delay(1500);
					//	CODE_TO_SEND = min_score;
					//	sendSAMSUNG(CODE_TO_SEND, 32);
					//	my_enableIRIn();
					//	HAL_Delay(150);
						if (i != 0) { // resting position

							if (i==1)
							{
								CODE_TO_SEND = 1111000001;

								mode=1; // включаем режим  управления // транслируем  сигнал ИК кода в формате акселерометра.
								}
							else if (i==2)
								{CODE_TO_SEND = 1111000002;}
							else if (i==3)
								{CODE_TO_SEND = 1111000003;}
							else if (i==4)
								{CODE_TO_SEND = 1111000004;}
							else if (i==5)
								{CODE_TO_SEND = 1111000005;}
							else if (i==6)
								{CODE_TO_SEND = 1111000006;}
							else if (i==7)
								{CODE_TO_SEND = 1111000007;}
							else if (i==8)
								{CODE_TO_SEND = 1111000008;}
							else if (i==9)
								{CODE_TO_SEND = 1111000009;}
							else if (i==10)
								{CODE_TO_SEND = 1111000010;}
							else if (i==11)
								{CODE_TO_SEND = 1111000011;}
							else if (i==12)
							{CODE_TO_SEND = 1100000012;
														break;
						}
					enableIROut(36);
					sendSAMSUNG(CODE_TO_SEND, 32);
					my_enableIRIn();
					HAL_Delay(250);
					enableIROut(36);
					sendSAMSUNG(CODE_TO_SEND, 32);
					my_enableIRIn();
					HAL_Delay(250);
						}
				}

			}
		}
		}
		// РјРѕРґСѓР»СЊ РїСЂРёС‘РјР° Р�Рљ СЃРёРіРЅР°Р»Р°. СЃСѓРєР° РѕС‡РµРЅСЊ РІР°Р¶РЅС‹Р№ Рё С‚СЂРµР±РѕРІР°С‚РµР»СЊРЅС‹Р№
		if (my_decode(&results)) {
			////////// Р Р†РЎвЂ№Р Р†Р С•Р Т‘ Р Т‘Р ВµР С”Р С•Р Т‘Р С‘РЎР‚Р С•Р Р†Р В°Р Р…РЎвЂ№РЎвЂ¦ Р Т‘Р В°Р Р…Р Р…РЎвЂ№РЎвЂ¦ ///////////////
			snprintf(trans_str, 96, "Cod: %p | Type: %s | Bits: %d\n",
					(void*) results.value, decode_str[results.decode_type + 1],
					results.bits);

			// 16744575 - 1
			// 16728255 - 2
			// 16760895 - 3
			// 16720095 - 4
			// 16752735 - 5
			// 16736415 - 6
			// 16769055 - 7
			// 16716015 - 8
			// 16748655 - 9
			// 16711935 - 0
			// 16749165 - ok
			// 16765485 - l

			int a;

			switch (results.value) {
			    case 16753245:
			    	  a=1;
			    	  break;
			    case 16736925:
			    	  a=2;
			    	  break;
			    case 16769565:
			    	  a=3;
			      break;
			    case 16720605:
			    	  a=4;
			      break;
			    case 16712445:
			    	  a=5;
			      break;
			    case 16761405:
			          a=6;
			      break;
			    case 16769055:
			    	  a=7;
			      break;
			    case 16754775:
			    	  a=8;
			      break;
			    case 16748655:
			    	  a=9;
			      break;
			    case 16750695:
			    	  a=0;
			      break;
			    case 16726215:
			 		 a=1000001; //ok
			      break;
			    case 16730805:
		    	  a=101;        // down
			     break;
			    default:
			    	 a=000111;
			      }

			if (a<=9)
			{
				if ((nm>=10)||(nm==0))
				nm=a;
				else if (nm>=1)
				nm=nm*10+a;
				else
				nm=a;

			}

				if (a==1000001)		    //пишем жест
				{
					take_reading_master_guesture=1;
					Start=1;
					Menupos = 2;
					HAL_Delay(1000);
				}
					// после выполнения этого мы помним что стоим на позиции 2 и в мастере у нас жест для памяти


				if (a==101)
				{
					EEPROM_write(nm);
					CODE_TO_SEND = nm;
					sendSAMSUNG(CODE_TO_SEND, 32);
					my_enableIRIn();
					HAL_Delay(1000);
					CODE_TO_SEND = 10000001;
					sendSAMSUNG(CODE_TO_SEND, 32);
					my_enableIRIn();
					HAL_Delay(1000);
				}


			CODE_TO_SEND = nm;
			sendSAMSUNG(CODE_TO_SEND, 32);
			my_enableIRIn();
			HAL_Delay(150);
			CODE_TO_SEND = nm;
			sendSAMSUNG(CODE_TO_SEND, 32);
			my_enableIRIn();
			HAL_Delay(150);


			my_resume();
			HAL_Delay(20);



		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}
}
//Предусмотреть защиту от блокировки. что-бы при проверке альетнативы не скидывался актуальный флаг.!!!


int  expect_arcade() { // жду что палочка опущена вниз. Это будет включать режим <i>.
	// когда палочка опущена, нужно подождать скажем 300 циклов, тогда включется режим аркады и будут мониторится
	// резкие движерния лево право вверх вниз и пыр )


	if ((((temp_values[0][sample_size - 1] >= -100)&& (temp_values[0][sample_size - 1] <= 100))
			& ((temp_values[1][sample_size - 1] >= 300)&& (temp_values[1][sample_size - 1] <= 500))
			& ((temp_values[2][sample_size - 1] >= -400 )&& (temp_values[2][sample_size - 1] <= -200)))
			& (redyread==0))
			{
							fadingmode=1;
							calcforwA++;
							arcademode=1;
							if (Diod_Pulse<=6000)  {Diod_Pulse+=100;} // мигаем неким цветом мол всё верно
							if (calcforwA >= 60) // ждём порядка 700 мc. это 35 тиков.
							{
								mode=1;// Arcade
								if (Diod_Pulse<=6000)
								{Diod_Pulse+=1000;mode=1;return 1;}

							}
						}
						else {
							// тут нужно редусмотреть баг. мы можем поменять положение палочки, но жест уже ждётся

							calcforwA = 0;
							if (fadingmode==1)
							{Diod_Pulse=0;}

						} // Сбросили таймер //тут продолжим...
	return 0;
}


void expect_fading() { // жду что палочка замерла. в режиме магии значит сейчас будет заклинание

	 if ((((temp_values[0][sample_size - 1] >= -100)&& (temp_values[0][sample_size - 1] <= 100))
		& ((temp_values[1][sample_size - 1] >= -100)&& (temp_values[1][sample_size - 1] <= 100))
		& ((temp_values[2][sample_size - 1] >= -70 )&& (temp_values[2][sample_size - 1] <= 70)))
		& (redyread==0))
						//x  +-70 //y  +-100 //z +-50
					{
		 	 	 	 	fadingmode=4;
						calcforw++;
						if (Diod_Pulse2<=6000)  {Diod_Pulse2+=100;} // мигаем неким цветом мол всё верно
						if (calcforw >= 20) // ждём порядка 700 мc. это 35 тиков.
						{
							redywait = 1; // я знаю что палочка зависла выставляю флаг ждать жест. с первых резких движений - чтаем жест
							if (Diod_Pulse2<=6000)
							{Diod_Pulse2+=1000;}
						}
					}
					else {
						// тут нужно редусмотреть баг. мы можем поменять положение палочки, но жест уже ждётся
						calcforw = 0;
						if (fadingmode==4)
	 					{Diod_Pulse2=0;}}

				    }


void expect_magic() { // жду что палочка в горизонтале перевёрнута вверхногами. Это включения режима магии
	//ждём сколько нибуть и ставим флаг что работаем с магием ( и загружаем набор мастерма магии
		if ((     ((temp_values[0][sample_size - 1] >= 2)&& (temp_values[0][sample_size - 1] <= 200))
				   & ((temp_values[1][sample_size - 1] >= -250)&& (temp_values[1][sample_size - 1] <= 200))
	 	 	    & ((temp_values[2][sample_size - 1] >=-900 )&& (temp_values[2][sample_size - 1] <= -300))))
							//x  +- 100 //y  +-300 //z +-150
						{
							fadingmode=2;
							calcforwM++;
							if (Diod_Pulse1<=6000)  {Diod_Pulse1+=100;} // мигаем неким цветом мол всё верно
							if (calcforwM >= 5) // ждём порядка 700 мc. это 35 тиков.
							{
													Diod_Pulse = 0;

													Diod_Pulse2 =0;
								mode=2;// magic
								if (Diod_Pulse1<=6000)
								{Diod_Pulse1+=1000;mode=2;} // выставляю что это опять магия.
							}
						}
						else {
							// тут нужно редусмотреть баг. мы можем поменять положение палочки, но жест уже ждётся
							calcforwM = 0;
							Diod_Pulse1=0;
							if (fadingmode==2)
							{Diod_Pulse2=0;}
						} // Сбросили таймер //тут продолжим...



}


int expect_gesture() {

	// Это работает если произошло замирание.
	//При первом резком движении включается чтение 50 значений для жеста
	//Нужно что-бы точно понимать когда начался жест
	if (  ((temp_values[0][sample_size - 1] <= -150)|| (temp_values[0][sample_size - 1] >= 150))
					& ((temp_values[1][sample_size - 1] <= -150)|| (temp_values[1][sample_size - 1] >= 150))
					& ((temp_values[2][sample_size - 1] <= -150)|| (temp_values[2][sample_size - 1] >= 150))
					& (redywait == 1))
					//x  +-150 //y  +-150 //z +-150
				{
					redyread = 1;
					redywait = 0;
					return 1;
				}

						return 0;
	}

void menu(int key) { //copy readings from temp_values to selected master

	if (key == 1)
	{
		Menupos = 1;
		Diod_Pulse=600;
		Diod_Pulse1=600;
		Diod_Pulse2=600;
		HAL_Delay(1000);
    }

	if ((key == 2) && (Menupos == 1))		    //пишем жест
	{

		take_reading_master_guesture=1;
		Start=1;
		key=0;
		nm=0;
		Menupos = 2;
		HAL_Delay(1000);
	}
		// после выполнения этого мы помним что стоим на позиции 2 и в мастере у нас жест для памяти


		//Пишем в память
		if ((key == 2) && (Menupos == 2))		    //пишем жест
			{
							Diod_Pulse = 0;
							Diod_Pulse1 =0;
							Diod_Pulse2 =0;
		// надо получить номер матрицы. Пока не нажмётся вторая кнопка, считываем данные по первой.
		// там-же внутри сделаю флешер светодиодом для понятности. как это прсото закрытые циклы.
		HAL_Delay(300);
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) != GPIO_PIN_RESET) {
			HAL_Delay(20);

			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
			{
				nm++;
				HAL_Delay(200);


				for (int i = 0; i < nm; i++)

				{
				HAL_Delay(500);
				Diod_Pulse = 500;
				Diod_Pulse1 = 500;
				Diod_Pulse2 = 500;
				HAL_Delay(500);

			    Diod_Pulse = 0;
				Diod_Pulse1 =0;
				Diod_Pulse2 =0;
				}


			}
		}
		EEPROM_write(nm);
		Menupos = 0;
	    Diod_Pulse = 0;
		Diod_Pulse1 =0;
		Diod_Pulse2 =0;
		HAL_Delay(200);
		Diod_Pulse1 =600;
		HAL_Delay(500);
		Diod_Pulse1 =0;
			}


}

void EEPROM_write(int master_select) { //store master gestures in EEPROM of arduino

	int addr;
	uint8_t raw[2];
	uint8_t rmsg[2];
	int16_t data;
	int16_t temp1, temp2;
	// master_select - номер массива в котором храниться жест/заклинание
	addr = (master_select * sample_size * DOF) * 2;

	for (;;) { // wait...
		status = HAL_I2C_IsDeviceReady(&hi2c2, devAddr, 1, HAL_MAX_DELAY);
		if (status == HAL_OK)
			break;
	}

	for (int i = 0; i < DOF; i++) {
		for (int j = 0; j < sample_size; j++) {
			// С‚СѓС‚ РєР°Рє-С‚Рѕ РєСЂР°РІРѕ С‡РёС‚Р°РµС‚СЃСЏ
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
							CODE_TO_SEND = 010101001;
							sendSAMSUNG(CODE_TO_SEND, 32);
							my_enableIRIn();
							HAL_Delay(150);
							CODE_TO_SEND = 000110011;
							sendSAMSUNG(CODE_TO_SEND, 32);
							my_enableIRIn();
							HAL_Delay(150);
				HAL_Delay(20000);
			}

			addr += 2;

			// РјР°Р»РµРЅСЊРєР°СЏ РїСЂРѕРІРµСЂРєР° РїР°РјСЏС‚Рё
			// С‚СѓС‚ РјС‹ РїСЂРѕР±СѓРµРј СЃРёСЃС‚РµРјСѓ Р·Р°РїРёСЃРё РІ РµРµРїСЂРѕРј. РїРёС‰РµРј РїРѕ 2 Р±РёС‚Р° РІ РјР°СЃСЃРёРІ. С‡РёС‚Р°РµРј С‚Р°Рє-Р¶Рµ. РїРѕ 2 Р±РёС‚Р°. Р°РґСЂРµСЃ СЃСЂР°Р·Сѓ С‰С‘Р»РєР°РµРј РЅР° 2 РІРїРµСЂС‘Рґ
		}
	}
	HAL_Delay(200);


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
	TIM2->CCR2 = Diod_Pulse1;
	TIM2->CCR3 = Diod_Pulse2;
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

	// РІРєР»СЋС‡РµРЅРёРµ/РїРѕР±СѓРґРєР° РјРѕРґСѓР»СЏ
	buffer[0] = MPU6050_RA_PWR_MGMT_1;
	buffer[1] = 0x00;
	I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);

	// РєРѕРЅС„РёРі РіРёСЂРѕСЃРєРѕРїР° РЅР° В±500В°/СЃ
	buffer[0] = MPU6050_RA_GYRO_CONFIG;
	buffer[1] = 0x8;
	I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);

	// РєРѕРЅС„РёРі Р°РєСЃРµР»РµСЂРѕРјРµС‚СЂР° РЅР° В±8g
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
		HAL_Delay(3); // 3 СЃРµРє РЅР° РєР°Р»РёР±СЂРѕРІРєСѓ
	}
	fAX_Cal /= iNumCM;
	fAY_Cal /= iNumCM;
	fAZ_Cal /= iNumCM;

}

void MPU6050_GetAllData(int16_t *Data) {

	uint8_t accelbuffer[14];

	// СЃ 0x3B 14 СЃР»РµРґСѓСЋС‰РёС… СЂРµРіРёСЃС‚СЂРѕРІ СЃРѕРґРµСЂР¶Р°С‚ РґР°РЅРЅС‹Рµ РёР·РјРµСЂРµРЅРёСЏ РјРѕРґСѓР»СЏ
	I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H,
			accelbuffer, 14);

	/* Registers 59 to 64 вЂ“ Accelerometer Measurements */
	for (int i = 0; i < 3; i++)
		Data[i] = (((int16_t) ((uint16_t) accelbuffer[2 * i] << 8)
				+ accelbuffer[2 * i + 1])) / 10;

	/* Registers 65 and 66 вЂ“ Temperature Measurement */
	//РїРѕРєР° РїСЂРѕРїСѓСЃРєР°РµРј Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	/* Registers 67 to 72 вЂ“ Gyroscope Measurements */
	for (int i = 4; i < 7; i++)
		Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8)
				+ accelbuffer[2 * i + 1]);

}

void IMU_CalculateAll_Data(void) {

	int16_t mpu6050data[6];
	// Р С—Р ВµРЎР‚Р ВµР С�Р ВµРЎвЂ°Р В°Р ВµР С� Р Р† Р С�Р В°РЎРѓРЎРѓР С‘Р Р† mpu6050data Р Р†РЎРѓР Вµ Р Т‘Р В°Р Р…Р Р…РЎвЂ№Р Вµ РЎРѓ Р Т‘Р В°РЎвЂљРЎвЂЎР С‘Р С”Р В°
	MPU6050_GetAllData(mpu6050data);

	// char msg[40];
	// uint16_t len = sprintf(msg, "X:%i Y:%i Z:%i\r\n",(int16_t)(mpu6050data[3]- fGX_Cal), (int16_t)(mpu6050data[4]- fGY_Cal), (int16_t)(mpu6050data[5] - fGZ_Cal));
	// HAL_UART_Transmit(&huart1, (uint8_t*) msg, len, HAL_MAX_DELAY);

	// Р В Р С’Р вЂ�Р С›Р СћР С’ Р РЋ Р вЂњР пїЅР В Р С›Р РЋР С™Р С›Р СџР С›Р Сљ
	//Р С”Р В°Р С” Р С—Р ВµРЎР‚Р ВµР Р†Р ВµРЎРѓРЎвЂљР С‘ Р В®Р Р…Р С‘РЎвЂљРЎвЂ№ Р Р† РЎР‚Р ВµР В°Р В»РЎРЉР Р…РЎвЂ№Р Вµ Р С–РЎР‚Р В°Р Т‘РЎС“РЎРѓРЎвЂ№ РЎР‚Р В°Р В·Р Р†Р ВµРЎР‚Р Р…РЎС“РЎвЂљР С• Р Р…Р В° Р С—РЎР‚Р С‘Р С�Р ВµРЎР‚Р Вµ Roll
	float Roll = mpu6050data[3] - fGX_Cal; // Р С•РЎвЂљР Р…Р С•РЎРѓР С‘РЎвЂљР ВµР В»РЎРЉР Р…Р С• "Р Р…РЎС“Р В»РЎРЏ"
	Roll = Roll / 65.5 / 50;
	MPU6050_Data.aRoll += Roll;

	//РЎРѓР С•Р С”РЎР‚Р В°РЎвЂ°Р ВµР Р…Р Р…Р В°РЎРЏ Р В·Р В°Р С—Р С‘РЎРѓРЎРЉ Р Т‘Р В»РЎРЏ Pitch
	MPU6050_Data.aPitch += (mpu6050data[4] - fGY_Cal) / 65.5 / 50;

	float Yaw = (mpu6050data[5] - fGZ_Cal) / 65.5 / 50;
	MPU6050_Data.aYaw += Yaw;
	//РЎС“РЎвЂЎР С‘РЎвЂљРЎвЂ№Р Р†Р В°Р ВµР С� Р С—Р В°РЎР‚Р В°Р С�Р ВµРЎвЂљРЎР‚ Р С—Р С• Z Р ВµРЎРѓР В»Р С‘ Р С—Р С• Р Р…Р ВµР С�РЎС“ Р ВµРЎРѓРЎвЂљРЎРЉ Р Т‘Р Р†Р С‘Р В¶Р ВµР Р…Р С‘Р Вµ
	if (abs(Yaw) > 1) {
		float _Y = sin(Yaw * 3.1415 / 180);
		MPU6050_Data.aPitch += MPU6050_Data.aRoll * _Y;
		MPU6050_Data.aRoll -= MPU6050_Data.aPitch * _Y;
	}

	// Р В Р С’Р вЂ�Р С›Р СћР С’ Р РЋ Р С’Р С™Р РЋР вЂўР вЂєР вЂўР В Р С›Р СљР вЂўР СћР В Р С›Р Сљ
	float roll4macc = atan2(mpu6050data[1],
			(sqrt( SQR(mpu6050data[0]) + SQR(mpu6050data[2])))) * R2DEG;
	float pitch4macc = atan2(mpu6050data[0],
			(sqrt(SQR(mpu6050data[1]) + SQR(mpu6050data[2])))) * R2DEG;

	// Р С”Р С•РЎР‚РЎР‚Р ВµР С”РЎвЂ Р С‘РЎРЏ Р Т‘РЎР‚Р ВµР в„–РЎвЂћР В° Р Р…РЎС“Р В»РЎРЏ Р С–Р С‘РЎР‚Р С•РЎРѓР С”Р С•Р С—Р В°
	// Р вЂќР В»РЎРЏ Р С”Р С•РЎР‚РЎР‚Р ВµР С”РЎвЂљР С‘РЎР‚Р С•Р Р†Р С”Р С‘ РЎС“Р С–Р В»Р С•Р Р† Р Р†Р С•РЎРѓР С—Р С•Р В»РЎРЉР В·РЎС“Р ВµР С�РЎРѓРЎРЏ Р С”Р С•Р С�Р С—Р В»Р ВµР С�Р ВµР Р…РЎвЂљР В°РЎР‚Р Р…РЎвЂ№Р С� РЎвЂћР С‘Р В»РЎРЉРЎвЂљРЎР‚Р С•Р С�
	// A = (1-K)*Ag + K*Ac
	MPU6050_Data.aPitch = MPU6050_Data.aPitch
			* (1 - MPU6050_KOEF_COMPL)+ pitch4macc * MPU6050_KOEF_COMPL;
	MPU6050_Data.aRoll = MPU6050_Data.aRoll
			* (1 - MPU6050_KOEF_COMPL)+ roll4macc * MPU6050_KOEF_COMPL;
}



int mapcalc(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
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
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
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







/*  кусок который считает расстояние
			 		// Аркада. смотрю по жестам на текущий момент. лавлю сильный жест лево - право. и шлю сигнал
					// надо переписать. так как тут есть 3 важных этапа. Начало движение движение и конец движения
					// только если по 3 пунктам совпал алгоритм - значит это жест.
					// а не эпелептический припадок
					// Точно. я буду считать какое расстояниеи от "0" пройдено. В лево право вверх низ.
					// и сверять с эталоном. скажем если больше 10 едениц - по Х с "-" - значит в лево
					// буду считать тут. каждый рыз считая что первая точка в массиве - моя точка старда движения пути

					//velocityX=temp_values[0][sample_size-1]/10 + ((temp_values[0][sample_size-2]/10 + temp_values[0][sample_size-1]/10)) / 2;
					//velocityY=temp_values[1][sample_size-1]/10 + ((temp_values[1][sample_size-2]/10 + temp_values[1][sample_size-1]/10)) / 2;
					//velocityZ=temp_values[2][sample_size-1]/10 + ((temp_values[2][sample_size-2]/10 + temp_values[2][sample_size-1]/10)) / 2;

					velocityX=temp_values[0][sample_size-2]/10;
					velocityY=temp_values[1][sample_size-2]/10;
					velocityZ=temp_values[2][sample_size-2]/10;



						if ((abs(velocityX)>100)||(abs(velocityY)>100)||(abs(velocityZ)>100)){
							// get the velocity
						 velocity[0][1] = velocity[0][0] + velocityX;
						 velocity[1][1] = velocity[1][0] + velocityZ;
							// get the distance
						 position[0][1] = position[0][0] + velocity[0][1] ;
						 position[1][1] = position[1][0] + velocity[1][1] ;
						    // get ready for the next set of readings

					     velocity[0][0] = velocity[0][1];
					     position[0][0] = position[0][1];
					     velocity[1][0] = velocity[1][1];
					     position[1][0] = position[1][1];

					     //position[0][1] порог срабатывания <5000 >-5000
					     //position[1][1] порог срабатывания <5000 >-5000
					     temp0=1;
						}
						else
							{

			     	 	 	 if (temp0==1){


			     	 	 	if      (position[0][0]<-3500) {snprintf(trans_str, 20, "LEFT   : %d\n", position[0][0] );	CODE_TO_SEND = 1000000001;}
			     	 	 	else if (position[0][0]> 3500) {snprintf(trans_str, 20, "RIGHT : %d\n", position[0][0] );	CODE_TO_SEND = 1000000002;}
			     	 	 	else if (position[1][0]<-3500) {snprintf(trans_str, 20, "UP : %d\n", position[1][0] );	    CODE_TO_SEND = 1000000003;}
			     	 	 	else if (position[1][0]> 3500) {snprintf(trans_str, 20, "DOWN: %d\n", position[1][0] );	    CODE_TO_SEND = 1000000004;}

			     	 		HAL_Delay(150);
			     	 		sendSAMSUNG(CODE_TO_SEND, 32);
			     	 	    my_enableIRIn();HAL_Delay(350);
			     	 		cod_but = 0;
			     	 		sendSAMSUNG(CODE_TO_SEND, 32);
							my_enableIRIn();HAL_Delay(350);
							 temp0=0;
			     	 	 	 }





							velocity[0][0] = 0;
							position[0][0] = 0;
							velocity[1][0] = 0;
							position[1][0] = 0;

							velocity[0][1] = 0;
							position[0][1] = 0;
							velocity[1][1] = 0;
							position[1][1] = 0;
							}



*/
