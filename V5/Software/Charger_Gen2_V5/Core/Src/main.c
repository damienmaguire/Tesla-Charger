/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *
  *   Tesla Gen 2 Charger Control Program
  *	2017-2019
  *T de Bree
  *D.Maguire
  *Additional work by C. Kidder
  *Runs on OpenSource Logic board V5 in Gen2 charger. Commands all modules.

 	 *D.Maguire 2019 Mods :
*	-Stop sending power module can messages when charger not running - Working.
*	-Correct reading of charger Fault and Enable feedback signals - Working.
*	-Correct AC present flag so only sets if more than 70V AC is on each module - Working.
*	-Reset charger on detection of power module fault - Testing.
*	-Shutdown on exceeding a preset HV Battery voltage - Working.
*	-Evse read routine now in 500ms loop to prevent false triggering -Working.
*	-Added counter/timer to autoshutdown to prevent false triggering on transients -Working.
*	-Added manual control mode for use of charger without EVSE. Digital one in when brought to +12v commands the charger to start
*	and when brought low commands charger off. This mode also control HVDC via digital out 1 and AC mains via a contactor via Digital out 2.-Working.
  *
  * Ported to the STM32F103 based V5 board in 2020 by Köksal Kurt.
  *
  * Serial Console on USART1
  *
  * WiFi access via USART3 and ESP8266 module
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"
#include "config.h"
#include "stdbool.h"
#include "stdint.h"
#include "eeprom.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"


/* Private typedef -----------------------------------------------------------*/
CAN_TxHeaderTypeDef myTxMessage; //CAN Tx message header

CAN_RxHeaderTypeDef myRxMessage; //CAN Rx message header

uint32_t              TxMailbox; //CAN Tx Mailbox ID


/* Private define ------------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

char wifistr[75];
uint8_t CanRxData[8];
uint8_t CanTxData[8];

int watchdogTime = 8000;

char* daynames[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};


//*********GENERAL VARIABLE   DATA ******************
int evsedebug = 1; // 1 = show Proximity status and Pilot current limits
int debug = 1; // 1 = show phase module CAN feedback


uint16_t curset = 0;
int  setting = 1;
uint8_t incomingByte = 0;
int state;
unsigned long slavetimeout, tlast, tcan,tpause, tboot = 0;
bool bChargerEnabled;
uint8_t Uart_Rx[5];



//*********EVSE VARIABLE   DATA ******************
uint8_t Proximity = 0;
uint16_t ACvoltIN = 240; // AC input voltage 240VAC for EU/UK and 110VAC for US
//proximity status values for type 1
#define Unconnected 0 // 3.3V
#define Buttonpress 1 // 2.3V
#define Connected 2 // 1.35V

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;

uint16_t cablelim = 0; // Type 2 cable current limit
uint16_t val=0;
//*********Single or Three Phase Config VARIABLE   DATA ******************

//proximity status values
#define Singlephase 0 // all parrallel on one phase Type 1
#define Threephase 1 // one module per phase Type 2

//*********Charger Control VARIABLE   DATA ******************
bool Vlimmode = true; // Set charges to voltage limit mode
uint16_t modulelimcur, dcaclim = 0;
uint16_t maxaccur = 16000; //maximum AC current in mA
uint16_t maxdccur = 45000; //max DC current outputin mA
int activemodules, slavechargerenable = 0;



//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt[3] = {0, 0, 0};//1 = 1V
uint16_t dccur[3] = {0, 0, 0};
uint16_t totdccur = 0;//1 = 0.005Amp
uint16_t acvolt[3] = {0, 0, 0};//1 = 1V
uint16_t accur[3] = {0, 0, 0};//1 = 0.06666 Amp
long acpower = 0;
uint8_t inlettarg [3] = {0, 0, 0}; //inlet target temperatures, should be used to command cooling.
uint8_t curtemplim [3] = {0, 0, 0};//current limit due to temperature
uint8_t templeg[2][3] = {{0, 0, 0}, {0, 0, 0}}; //temperatures reported back
bool ACpres [3] = {0, 0, 0}; //AC present detection on the modules
bool ModEn [3] = {0, 0, 0}; //Module enable feedback on the modules
bool ModFlt [3] = {0, 0, 0}; //module fault feedback
bool LockOut = false; //lockout on termination voltage reached. Reset by evse plug recycle.
uint8_t ModStat [3] = {0, 0, 0};//Module Status
int newframe = 0;

ChargerParams parameters;

//*********DCDC Messages VARIABLE   DATA ******************
bool dcdcenable = 1; // turn on can messages for the DCDC.

//*********Charger Messages VARIABLE   DATA ******************
int ControlID = 0x300;
int StatusID = 0x410;
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;

uint16_t LockOutCnt=0;// lockout counter

int candebug = 0;
int menuload = 0;
int mescycle = 0;

uint16_t VirtAddVarTab[14]={20777,20778,20779,20780,20781,20782,20783,20784,20785,20786,20787,20788,20789,20790};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void EEPROM_Read(ChargerParams *tempparams);
void EEPROM_Write(ChargerParams *tempparams);
bool CAN_FilterConfig(void);
bool Can_Init(void);
void candecode(CAN_RxHeaderTypeDef *tempRx,uint8_t *tempCanData);
void menu(void);
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t dly);
void Charger_msgs(void);
void resetFaults(void);
void evseread(void);
void autoShutdown(void);

void manualMode(void);
void DCcurrentlimit(void);
void ACcurrentlimit(void);


int main(void)
{

	dcaclim = maxaccur;
	bChargerEnabled = false; //are we supposed to command the charger to charge?

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  //MX_ADC2_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  EE_Init();
  EEPROM_Read(&parameters);
  if (parameters.version != EEPROM_VERSION)
  {
	  parameters.version = EEPROM_VERSION;
	  parameters.currReq = 20000; //max input limit per module 1500 = 1A
	  parameters.enabledChargers = 123; // enable per phase - 123 is all phases - 3 is just phase 3
	  parameters.can0Speed = 500000;
	  parameters.mainsRelay = 48;
	  parameters.voltSet = 40000; //1 = 0.01V
	  parameters.tVolt = 40000;//1 = 0.01V
	  parameters.autoEnableCharger = 1; //disable auto start, proximity and pilot control
	  parameters.canControl = 1; //0 disabled can control, 1 master, 3 slave
	  parameters.dcdcsetpoint = 14000; //voltage setpoint for dcdc in mv
	  parameters.phaseconfig = Singlephase; //AC input configuration
	  parameters.type = 1; //Socket type1 or 2
	  EEPROM_Write(&parameters);
  }
  if(Can_Init()){
	  printf("Using CAN - initialization completed.\r\n");
  }
  else{
	  printf("CAN initialization (sync) ERROR\r\n");
  }
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim4);
  /* Infinite loop */
  while (1)
  {
	if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) >= 1)
	{
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &myRxMessage, CanRxData);
		candecode(&myRxMessage,CanRxData);
	}
	if(HAL_UART_Receive(&huart1, &incomingByte, 1, 5)==HAL_OK){
		menu();
	}
	if (parameters.canControl > 1)
	{
		if (state != 0)
	    {
			if (millis() - tcan > 2000)
			{
				state = 0;
				printf("CAN time-out\r\n");
	      }
	    }
	}
	if(HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_RESET){
		state = 0;
	}
	switch (state)
	{
		case 0: //Charger off
			//Timer3.detachInterrupt(); // stop sending charger power module CAN messages
			HAL_TIM_Base_Stop_IT(&htim3);
			if (bChargerEnabled == true)
			{
			bChargerEnabled = false;
			}
			HAL_GPIO_WritePin(DIG_OUT_1_GPIO_Port, DIG_OUT_1_Pin, GPIO_PIN_RESET);//HV OFF
			HAL_GPIO_WritePin(EVSE_ACTIVATE_GPIO_Port, EVSE_ACTIVATE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_RESET);//AC OFF when in manual mode.
			HAL_GPIO_WritePin(CHARGER1_ACTIVATE_GPIO_Port, CHARGER1_ACTIVATE_Pin, GPIO_PIN_RESET);//chargeph1 deactivate
			HAL_GPIO_WritePin(CHARGER2_ACTIVATE_GPIO_Port, CHARGER2_ACTIVATE_Pin, GPIO_PIN_RESET);//chargeph2 deactivate
			HAL_GPIO_WritePin(CHARGER3_ACTIVATE_GPIO_Port, CHARGER3_ACTIVATE_Pin, GPIO_PIN_RESET);//chargeph3 deactivate
			HAL_GPIO_WritePin(CHARGER1_ENABLE_GPIO_Port, CHARGER1_ENABLE_Pin, GPIO_PIN_RESET);//disable phase 1 power module
			HAL_GPIO_WritePin(CHARGER2_ENABLE_GPIO_Port, CHARGER2_ENABLE_Pin, GPIO_PIN_RESET);//disable phase 2 power module
			HAL_GPIO_WritePin(CHARGER3_ENABLE_GPIO_Port, CHARGER3_ENABLE_Pin, GPIO_PIN_RESET);//disable phase 3 power module
			break;
	    case 1://Charger on
	    	if(HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_SET)
	    	{
	            if (bChargerEnabled == false)
	            {
	              bChargerEnabled = true;
	              switch (parameters.enabledChargers)
	              {
	                case 1:
	                	HAL_GPIO_WritePin(CHARGER1_ACTIVATE_GPIO_Port, CHARGER1_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 1;
	                	break;
	                case 2:
	                	HAL_GPIO_WritePin(CHARGER2_ACTIVATE_GPIO_Port, CHARGER2_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 1;
	                	break;
	                case 3:
	                	HAL_GPIO_WritePin(CHARGER3_ACTIVATE_GPIO_Port, CHARGER3_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 1;
	                 	break;
	                case 12:
	                	HAL_GPIO_WritePin(CHARGER1_ACTIVATE_GPIO_Port, CHARGER1_ACTIVATE_Pin, GPIO_PIN_SET);
	                	HAL_GPIO_WritePin(CHARGER2_ACTIVATE_GPIO_Port, CHARGER2_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 2;
	                	break;
	                case 13:
	                	HAL_GPIO_WritePin(CHARGER1_ACTIVATE_GPIO_Port, CHARGER1_ACTIVATE_Pin, GPIO_PIN_SET);
	                	HAL_GPIO_WritePin(CHARGER3_ACTIVATE_GPIO_Port, CHARGER3_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 2;
	                	break;
	                case 123:
	                	HAL_GPIO_WritePin(CHARGER1_ACTIVATE_GPIO_Port, CHARGER1_ACTIVATE_Pin, GPIO_PIN_SET);
	                	HAL_GPIO_WritePin(CHARGER2_ACTIVATE_GPIO_Port, CHARGER2_ACTIVATE_Pin, GPIO_PIN_SET);
	                	HAL_GPIO_WritePin(CHARGER3_ACTIVATE_GPIO_Port, CHARGER3_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 3;
	                	break;
	                case 23:
	                	HAL_GPIO_WritePin(CHARGER2_ACTIVATE_GPIO_Port, CHARGER2_ACTIVATE_Pin, GPIO_PIN_SET);
	                	HAL_GPIO_WritePin(CHARGER3_ACTIVATE_GPIO_Port, CHARGER3_ACTIVATE_Pin, GPIO_PIN_SET);
	                	activemodules = 2;
	                	break;
	                default:
	                  // if nothing else matches, do the default
	                  // default is optional
	                	break;
	              }

	              delay(100);
	              HAL_GPIO_WritePin(EVSE_ACTIVATE_GPIO_Port, EVSE_ACTIVATE_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_IN_2_Pin, GPIO_PIN_SET);
	            }
	    	}
	    	else
	    	{
	    		bChargerEnabled = false;
	            state = 0;
	    	}
		  break;
	    case 2:
            HAL_GPIO_WritePin(DIG_OUT_1_GPIO_Port, DIG_OUT_1_Pin, GPIO_PIN_SET);//HV ON
            HAL_TIM_Base_Start_IT(&htim3);
            //Timer3.attachInterrupt(Charger_msgs).start(90000); // start sending charger power module CAN messages
	        switch (parameters.enabledChargers)
	        {
	            case 1:
                	HAL_GPIO_WritePin(CHARGER1_ENABLE_GPIO_Port, CHARGER1_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 2:
                	HAL_GPIO_WritePin(CHARGER2_ENABLE_GPIO_Port, CHARGER2_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 3:
                	HAL_GPIO_WritePin(CHARGER3_ENABLE_GPIO_Port, CHARGER3_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 12:
                	HAL_GPIO_WritePin(CHARGER1_ENABLE_GPIO_Port, CHARGER1_ENABLE_Pin, GPIO_PIN_SET);
                	HAL_GPIO_WritePin(CHARGER2_ENABLE_GPIO_Port, CHARGER2_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 13:
                	HAL_GPIO_WritePin(CHARGER1_ENABLE_GPIO_Port, CHARGER1_ENABLE_Pin, GPIO_PIN_SET);
                	HAL_GPIO_WritePin(CHARGER3_ENABLE_GPIO_Port, CHARGER3_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 123:
                	HAL_GPIO_WritePin(CHARGER1_ENABLE_GPIO_Port, CHARGER1_ENABLE_Pin, GPIO_PIN_SET);
                	HAL_GPIO_WritePin(CHARGER2_ENABLE_GPIO_Port, CHARGER2_ENABLE_Pin, GPIO_PIN_SET);
                	HAL_GPIO_WritePin(CHARGER3_ENABLE_GPIO_Port, CHARGER3_ENABLE_Pin, GPIO_PIN_SET);
                	break;
	            case 23:
                	HAL_GPIO_WritePin(CHARGER2_ENABLE_GPIO_Port, CHARGER2_ENABLE_Pin, GPIO_PIN_SET);
                	HAL_GPIO_WritePin(CHARGER3_ENABLE_GPIO_Port, CHARGER3_ENABLE_Pin, GPIO_PIN_SET);
                	break;

	            default:
	              // if nothing else matches, do the default
	            	break;
	        }
	        if (tboot <  (millis() - 10)) //delay in ms before moving to state 1.
	        {
	            state = 1;
	        }
	        break;
	        default:
	          // if nothing else matches, do the default
	        	break;
	  }
	  if (tlast <  (millis() - 500))
	  {
	    tlast = millis();

	    evseread();
	    autoShutdown();

	    manualMode();

	    if (debug != 0)
	    {
	      printf("\r\n");
	      printf("%ld",millis());
	      printf(" State: ");
	      printf("%d",state);
	      printf(" Phases : ");
	      printf("%d",parameters.phaseconfig);
	      printf(" Modules Avtive : ");
	      printf("%d",activemodules);
	      if (bChargerEnabled)
	      {
	        printf(" ON  ");
	      }
	      else
	      {
	        printf(" OFF ");
	      }
	      if (HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_SET)
	      {
	        printf(" D1 H");
	      }
	      else
	      {
	        printf(" D1 L");
	      }


	      if (bChargerEnabled)
	      {
	        printf("\r\n");
	        for (int x = 0; x < 3; x++)
	        {
	          printf("  Phase ");
	          printf("%d",x + 1);
	          printf(" Feebback //  AC present: ");
	          printf("%d",ACpres[x]);
	          printf("  AC volt: ");
	          printf("%d",acvolt[x]);
	          printf("  AC cur: ");
	          printf("%.2f",accur[x] * 0.06666f);
	          //printf("  ");
	          ///printf(accur[x], HEX); ///not needed since current fixed
	          printf("  DC volt: ");
	          printf("%d",dcvolt[x]);
	          printf("  DC cur: ");
	          printf("%0.2f",dccur[x] * 0.000839233f);
	          printf("  Inlet Targ: ");
	          printf("%d",inlettarg[x]);
	          printf("  Temp Lim Cur: ");
	          printf("%d",curtemplim[x]);
	          printf("  ");
	          printf("%d",templeg[0][x]);
	          printf("  ");
	          printf("%d",templeg[1][x]);
	          printf(" EN:");
	          printf("%d",ModEn[x]);
	          printf(" Flt:");
	          printf("%d",ModFlt[x]);
	          printf(" Stat:");
	          printf("%d",ModStat[x]);
	          printf("\r\n");
	        }
	      }
	      else
	      {
	        printf("\r\n");
	        printf("Modules Turned OFF");
	        printf("\r\n");
	      }
	    }
	    if (debug == 1)
	    {
	      if (evsedebug != 0)
	      {
	        printf("\r\n");
	        printf("  Proximity Status : ");
	        switch (Proximity)
	        {
	          case Unconnected:
	            printf("Unconnected");
	            break;
	          case Buttonpress:
	            printf("Button Pressed");
	            break;
	          case Connected:
	            printf("Connected");
	            break;

	        }
	        printf("  /ADC Raw: ");
	        printf("%d",val);
	        printf(" AC limit : ");
	        printf("%d",accurlim);
	        printf(" /Cable Limit: ");
	        printf("%d",cablelim);
	        printf(" /Module Cur Request: ");
	        printf("%.0f",modulelimcur / 1.5f);
	        printf(" /DC total Cur:");
	        printf("%.2f",totdccur * 0.005f);
	        printf(" /DC Setpoint:");
	        printf("%.0f",parameters.voltSet * 0.01f);
	        printf(" /DC tVolt:");
	        printf("%.0f",parameters.tVolt*0.01f);
	        printf(" /DC driven AC Cur Lim: ");
	        printf("%d",dcaclim);
	      }
	    }
	  }
	  DCcurrentlimit();
	  ACcurrentlimit();
	  resetFaults();
	  if (parameters.autoEnableCharger == 1)
	  {
	    if ((Proximity == Connected)&&(LockOut==false)) //check if plugged in and not locked out
	    {
	      //digitalWrite(EVSE_ACTIVATE, HIGH);//pull pilot low to indicate ready - NOT WORKING freezes PWM reading
	      if (accurlim > 1400) // one amp or more active modules
	      {
	        if (state == 0)
	        {
	          if (HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_SET)
	          {
	            state = 2;// initialize modules
	            tboot = millis();
	          }
	        }
	      }
	      HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_SET); //enable AC present indication
	    }
	    else // unplugged or buton pressed stop charging
	    {
	      state = 0;
	      HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_RESET); //disable AC present indication
	      HAL_GPIO_WritePin(EVSE_ACTIVATE_GPIO_Port, EVSE_ACTIVATE_Pin, GPIO_PIN_SET);
	    }
	  }

  }
/************END OF LOOP****************/
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


void resetFaults(void){
if ((bChargerEnabled == true) && (ACpres[0] == true) && (ModFlt[0] ==true) && ((parameters.enabledChargers == 1) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 123)))
    {
      //if these conditions are met then phase one is enabled, has ac present and has entered a fault state so we want to reset.
		state = 0;
		HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_RESET); //disable AC present indication;
          //  digitalWrite(EVSE_ACTIVATE, LOW);
    }
  if ((bChargerEnabled == true) && (ACpres[1] == true) && (ModFlt[1] ==true) && ((parameters.enabledChargers == 2) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123)))
    {
      //if these conditions are met then phase two is enabled, has ac present and has entered a fault state so we want to reset.
		state = 0;
		HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_RESET); //disable AC present indication;
            //digitalWrite(EVSE_ACTIVATE, LOW);
    }
 if ((bChargerEnabled == true) && (ACpres[2] == true) && (ModFlt[2] ==true) && ((parameters.enabledChargers == 3) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123)))
    {
      //if these conditions are met then phase three is enabled, has ac present and has entered a fault state so we want to reset.
		state = 0;
		HAL_GPIO_WritePin(DIG_OUT_2_GPIO_Port, DIG_OUT_2_Pin, GPIO_PIN_RESET); //disable AC present indication;
            //digitalWrite(EVSE_ACTIVATE, LOW);
    }
 }
//If the HV voltage exceeds the tVolt setpoint we want to shut down the charger and not re enable until the charge plug
//is removed and re connected. For now we just read the voltage on phase module one.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoShutdown(void){
  if ((bChargerEnabled ==true)&&(LockOut==false)) //if charger is running and we are not locked out ...
  {
    if (dcvolt[0]>(parameters.tVolt*0.01f)) //and if we exceed tVolt...
    {
        LockOutCnt++; //increment the lockout counter
//      LockOut=true; //lockout and shutdown
  //    state=0;
  }
  else
  {
    LockOutCnt=0; //other wise we reset the lockout counter
  }

  }
  if (Proximity == Unconnected&&(parameters.autoEnableCharger == 1)) LockOut=false;  //re set the lockout flag when the evse plug is pulled only if we are in evse mode.

  if (LockOutCnt>10)
  {
  state=0; //if we are above our shutdown targer for 10 consecutive counts we lockout
  LockOut=true; //lockout and shutdown
  LockOutCnt=0;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Manual control mode via Digital input 1. Special case for A.Rish.
/////////////////////////////////////////////////////////////////////////////////////
void manualMode(void)
{

  if (parameters.autoEnableCharger == 0)//if we are not in auto mode ...
  {
	if (state == 0)//....and if we are currently turned off....
	{
	  if ((HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_SET)&&(LockOut==false))//...and if digital one is high....
	  {
		state = 2;// initialize modules. Fire up the charger.
		tboot = millis();
	  }
	}
	if (HAL_GPIO_ReadPin(DIG_IN_1_GPIO_Port, DIG_IN_1_Pin)==GPIO_PIN_RESET)//...if brought low then we shutoff the charger.
	{
	state = 0;//
	LockOut=false;//release lockout when dig 1 in is brought low.
	}
  }
}
void evseread(){
/*{
 uint16_t val=0;

  if(HAL_ADC_PollForConversion(&hadc1, 5)){
	  val=HAL_ADC_GetValue(&hadc1);
  }*/

  HAL_ADC_PollForConversion(&hadc1, 5);
  val = (HAL_ADC_GetValue(&hadc1)/4);

  if ( parameters.type == 2)
  {
    if ( val > 950)
    {
      Proximity = Unconnected;
    }
    else
    {
      Proximity = Connected;
      if ( val < 950 && val > 800)
      {
        cablelim = 13000;
      }
      if ( val < 800 && val > 700)
      {
        cablelim = 20000;
      }
      if ( val < 600 && val > 450)
      {
        cablelim = 32000;
      }
      if ( val < 400 && val > 250)
      {
        cablelim = 63000;
      }
    }
  }

  if ( parameters.type == 1)
  {
    if ( val > 800)
    {
      Proximity = Unconnected;
    }
    else
    {
      if ( val > 550)
      {
        Proximity = Buttonpress;
      }
      else
      {
        Proximity = Connected;
      }
    }
  }


}
void EEPROM_Read(ChargerParams *tempparams){
	HAL_FLASH_Unlock();
	EE_ReadVariable(VirtAddVarTab[0], (uint16_t*)&tempparams->version);
	EE_ReadVariable(VirtAddVarTab[1], (uint16_t*)&tempparams->enabledChargers);
	EE_ReadVariable(VirtAddVarTab[2], (uint16_t*)&tempparams->mainsRelay);
	EE_ReadVariable(VirtAddVarTab[3], (uint16_t*)&tempparams->autoEnableCharger);
	EE_ReadVariable(VirtAddVarTab[4], (uint16_t*)&tempparams->canControl);
	EE_ReadVariable(VirtAddVarTab[5], (uint16_t*)&tempparams->type);
	EE_ReadVariable(VirtAddVarTab[6], (uint16_t*)&tempparams->phaseconfig);
	EE_ReadVariable(VirtAddVarTab[7], &tempparams->voltSet);
	EE_ReadVariable(VirtAddVarTab[8], &tempparams->tVolt);
	EE_ReadVariable(VirtAddVarTab[9], &tempparams->currReq);
	uint16_t can0Speedpart1;
	uint16_t can0Speedpart2;
	EE_ReadVariable(VirtAddVarTab[10], &can0Speedpart1);
	EE_ReadVariable(VirtAddVarTab[11], &can0Speedpart2);
	EE_ReadVariable(VirtAddVarTab[12], &tempparams->dcdcsetpoint);
	tempparams->can0Speed=((uint32_t)can0Speedpart1<<16)|(uint32_t)can0Speedpart2;
	HAL_FLASH_Lock();
}
void EEPROM_Write(ChargerParams *tempparams){
	uint16_t can0Speedpart1 = ((tempparams->can0Speed)>>16);
	uint16_t can0Speedpart2 = (uint16_t)((tempparams->can0Speed)&&0xFFFF);
	HAL_FLASH_Unlock();
	EE_WriteVariable(VirtAddVarTab[0], tempparams->version);
	EE_WriteVariable(VirtAddVarTab[1], tempparams->enabledChargers);
	EE_WriteVariable(VirtAddVarTab[2], tempparams->mainsRelay);
	EE_WriteVariable(VirtAddVarTab[3], tempparams->autoEnableCharger);
	EE_WriteVariable(VirtAddVarTab[4], tempparams->canControl);
	EE_WriteVariable(VirtAddVarTab[5], tempparams->type);
	EE_WriteVariable(VirtAddVarTab[6], tempparams->phaseconfig);
	EE_WriteVariable(VirtAddVarTab[7], tempparams->voltSet);
	EE_WriteVariable(VirtAddVarTab[8], tempparams->tVolt);
	EE_WriteVariable(VirtAddVarTab[9], tempparams->currReq);
	EE_WriteVariable(VirtAddVarTab[10], can0Speedpart1);
	EE_WriteVariable(VirtAddVarTab[11], can0Speedpart2);
	EE_WriteVariable(VirtAddVarTab[12], tempparams->dcdcsetpoint);
	HAL_FLASH_Lock();
}
bool Can_Init(void){
	if(!CAN_FilterConfig()){
		return false;
		Error_Handler();
	}
	HAL_Delay(2);
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
	    return false;
	    Error_Handler();
	}
	//if (CAN_IT_Enabled){
	//}
	myTxMessage.RTR = CAN_RTR_DATA;
	myTxMessage.IDE = CAN_ID_STD;
	myTxMessage.DLC = 8;
	myTxMessage.TransmitGlobalTime = DISABLE;
	return true;
}
bool CAN_FilterConfig(void){
	CAN_FilterTypeDef myFilterConfig;
	myFilterConfig.FilterBank = 0;
	myFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	myFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	myFilterConfig.FilterIdHigh = 0x000;
	myFilterConfig.FilterIdLow = 0x0000;
	myFilterConfig.FilterMaskIdHigh = 0x0000; //0xFFE0
	myFilterConfig.FilterMaskIdLow = 0x0000;
	myFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	myFilterConfig.FilterActivation = ENABLE;
	myFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan, &myFilterConfig) != HAL_OK)
	{
	  return false;
	}
	return true;
}
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void candecode(CAN_RxHeaderTypeDef *tempRx,uint8_t *tempCanData){

	int x = 0;
	  switch (tempRx->StdId)
	  {
	    case 0x217: //phase 1 Status message
	      ModStat[0] = tempCanData[0];
	      break;

	    case 0x219: //phase 2 Status message
	      ModStat[1] = tempCanData[0];
	      break;

	    case 0x21B: //phase 3 Status message
	      ModStat[2] = tempCanData[0];
	      break;

	    case 0x24B: //phase 3 temp message 2
	      curtemplim[2] = tempCanData[0] * 0.234375;
	      newframe = newframe | 1;
	      break;

	    case 0x23B: //phase 3 temp message 1
	      templeg[0][2] = tempCanData[0] - 40;
	      templeg[1][2] = tempCanData[1] - 40;
	      inlettarg[2] = tempCanData[5] - 40;
	      newframe = newframe | 1;
	      break;

	    case 0x239: //phase 2 temp message 1
	      templeg[0][1] = tempCanData[0] - 40;
	      templeg[1][1] = tempCanData[1] - 40;
	      inlettarg[1] = tempCanData[5] - 40;
	      newframe = newframe | 1;
	      break;
	    case 0x249: //phase 2 temp message 2
	      curtemplim[1] = tempCanData[0] * 0.234375;
	      newframe = newframe | 1;
	      break;

	    case 0x237: //phase 1 temp message 1
	      templeg[0][0] = tempCanData[0] - 40;
	      templeg[1][0] = tempCanData[1] - 40;
	      inlettarg[0] = tempCanData[5] - 40;
	      newframe = newframe | 1;
	      break;
	    case 0x247: //phase 2 temp message 2
	      curtemplim[0] = tempCanData[0] * 0.234375;
	      newframe = newframe | 1;
	      break;

	    case 0x207: //phase 2 msg 0x209. phase 3 msg 0x20B
	      acvolt[0] = tempCanData[1];
	      accur[0] = (uint16_t)((tempCanData[6] & 0x03) * 256 + tempCanData[5]) >> 1 ;
	      x = tempCanData[1];// & 12;
	      if (x > 0x46) //say 0x46 = 70V
	      {
	        ACpres[0] = true;
	      }
	      else
	      {
	        ACpres[0] = false;
	      }
	      x = tempCanData[2] & 0x02; //was 40
	      if (x != 0)
	      {
	        ModEn[0] = true;
	      }
	      else
	      {
	        ModEn[0] = false;
	      }
	      x = tempCanData[2] & 0x04; //was 20
	      if (x != 0)
	      {
	        ModFlt[0] = true;
	      }
	      else
	      {
	        ModFlt[0] = false;
	      }
	      newframe = newframe | 1;
	      break;
	    case 0x209: //phase 2 msg 0x209. phase 3 msg 0x20B
	      acvolt[1] = tempCanData[1];
	      accur[1] = (uint16_t)((tempCanData[6] & 0x03) * 256 + tempCanData[5]) >> 1 ;
	      x = tempCanData[1];// & 12;
	      if (x > 0x46) //say 0x46 = 70V)
	      {
	        ACpres[1] = true;
	      }
	      else
	      {
	        ACpres[1] = false;
	      }
	      x = tempCanData[2] & 0x02; //was 40
	      if (x != 0)
	      {
	        ModEn[1] = true;
	      }
	      else
	      {
	        ModEn[1] = false;
	      }
	      x = tempCanData[2] & 0x04; //was 20
	      if (x != 0)
	      {
	        ModFlt[1] = true;
	      }
	      else
	      {
	        ModFlt[1] = false;
	      }
	      newframe = newframe | 1;
	      break;
	    case 0x20B: //phase 2 msg 0x209. phase 3 msg 0x20B
	      acvolt[2] = tempCanData[1];
	      accur[2] = (uint16_t)((tempCanData[6] & 0x03) * 256 + tempCanData[5]) >> 1 ;
	      x = tempCanData[1];// & 12;
	      if (x > 0x46) //say 0x46 = 70V)
	      {
	        ACpres[2] = true;
	      }
	      else
	      {
	        ACpres[2] = false;
	      }
	      x = tempCanData[2] & 0x02; //was 40
	      if (x != 0)
	      {
	        ModEn[2] = true;
	      }
	      else
	      {
	        ModEn[2] = false;
	      }
	      x = tempCanData[2] & 0x04; //was 20
	      if (x != 0)
	      {
	        ModFlt[2] = true;
	      }
	      else
	      {
	        ModFlt[2] = false;
	      }
	      newframe = newframe | 1;
	      break;
	    case 0x227: //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
	      dccur[0] = ((tempCanData[5] << 8) + tempCanData[4]) ;//* 0.000839233 convert in rest of code
	      dcvolt[0] = ((tempCanData[3] << 8) + tempCanData[2]) * 0.0105286; //we left shift 8 bits to make a 16bit uint.
	      newframe = newframe | 2;
	      break;
	    case 0x229: //dc feedback. Phase 2 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
	      dccur[1] = ((tempCanData[5]) << tempCanData[4]) ;//* 0.000839233 convert in rest of code
	      dcvolt[1] = ((tempCanData[3] << 8) + tempCanData[2]) * 0.0105286; //we left shift 8 bits to make a 16bit uint.
	      newframe = newframe | 2;
	      break;
	    case 0x22B: //dc feedback. Phase 3 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
	      dccur[2] = ((tempCanData[5] << 8) + tempCanData[4]) ;//* 0.000839233 convert in rest of code
	      dcvolt[2] = ((tempCanData[3] << 8) + tempCanData[2]) * 0.01052856; //we left shift 8 bits to make a 16bit uint.
	      newframe = newframe | 2;
	      break;

	    default:
	      // if nothing else matches, do the default
	      break;
	}
}

void menu(void)
{
  if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 'q': //q for quit
        debug = 1;
        menuload = 0;
        break;

      case 'a'://a for auto enable
        candebug ++;
        if (candebug > 1)
        {
          candebug = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;

      case 'b'://a for auto enable
        evsedebug ++;
        if (evsedebug > 1)
        {
          evsedebug = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;


      case '1'://a for auto enable
        parameters.autoEnableCharger ++;
        if (parameters.autoEnableCharger > 1)
        {
          parameters.autoEnableCharger = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;

      case '2'://e for enabling chargers followed by numbers to indicate which ones to run

          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
				parameters.enabledChargers=atoi((const char *)Uart_Rx);
				menuload = 0;
				incomingByte = 'd';
		  }
        break;

      case '3'://a for can control enable
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
				parameters.canControl=atoi((const char *)Uart_Rx);
		          if (parameters.canControl > 3)
		          {
		            parameters.canControl = 0;
		          }
		          menuload = 0;
		          incomingByte = 'd';
		  }

        break;

      case '4'://t for type
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  parameters.type=atoi((const char *)Uart_Rx);
			  if (parameters.type > 2)
			  {
				parameters.type = 2;
			  }
			  if (parameters.type == 0)
			  {
				parameters.type = 2;
			  }
			  menuload = 0;
			  incomingByte = 'd';
        }
        break;

      case '5'://a for can control enable
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  parameters.phaseconfig=atoi((const char *)Uart_Rx)-1;
			  if ( parameters.phaseconfig == 2)
			  {
				parameters.phaseconfig = 1;
			  }
			  if (parameters.phaseconfig == 0)
			  {
				parameters.phaseconfig = 0;
			  }
			  menuload = 0;
			  incomingByte = 'd';
		  }
        break;
      case '6'://v for voltage setting in whole numbers
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  parameters.voltSet=atoi((const char *)Uart_Rx)*100;
			  menuload = 0;
			  incomingByte = 'd';
		  }
        break;

      case '7': //c for current setting in whole numbers
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  parameters.currReq=atoi((const char *)Uart_Rx)*1500;
			  menuload = 0;
			  incomingByte = 'd';
		  }
        break;

      case '9': //c for current setting in whole numbers
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  menuload = 0;
			  incomingByte = 'd';
		  }
        break;
      case 't'://t for termaintion voltage setting in whole numbers
          memset(Uart_Rx,0,5);
          HAL_UART_Receive(&huart1, Uart_Rx, 5, 5);
		  if(Uart_Rx[0]!=0){
			  parameters.tVolt=atoi((const char *)Uart_Rx)*100;
			  menuload = 0;
			  incomingByte = 'd';
		  }
        break;
    }
  }

  if (menuload == 0)
  {
    switch (incomingByte)
    {
      case 's'://s for start AND stop
    	  	  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
              state = 2;// initialize modules
              tboot = millis();
        break;

     case 'o':
            if (state > 0)
            {
            	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
            	state = 0;// initialize modules
            }
         break;

      case 'q': //q for quit
        EEPROM_Write(&parameters);
        debug = 1;
        menuload = 0;
        break;

      case 'd'://d for display
        debug = 0;
        menuload = 1;
        printf(" ");
        printf(" ");
        printf(" ");
        printf(" ");
        printf("Settings Menu\r\n");
        printf("1 - Auto Enable : ");
        if (parameters.autoEnableCharger == 1)
        {
          printf("ON\r\n");
        }
        else
        {
          printf("OFF\r\n");
        }
        printf("2 - Modules Enabled : ");
        printf("%d\r\n",parameters.enabledChargers);
        printf("3 - Can Mode : ");
        if (parameters.canControl == 0)
        {
          printf(" Off \r\n");
        }
        if (parameters.canControl == 1)
        {
          printf(" Master \r\n");
        }
        if (parameters.canControl == 2)
        {
          printf(" Master Elcon \r\n");
        }
        if (parameters.canControl == 3)
        {
          printf(" Slave \r\n");
        }
        printf("4 - Port Type : ");
        printf("%d\r\n",parameters.type);
        printf("5 - Phase Wiring : ");
        printf("%d\r\n",parameters.phaseconfig + 1);
        printf("6 - DC Charge Voltage : ");
        printf("%f %d",parameters.voltSet * 0.01f, 0);
        printf("V\r\n");
        printf("7 - AC Current Limit : ");
        printf("%d",parameters.currReq / 1500);
        printf("A\r\n");
        printf("8 - CAN0 Speed : ");
        printf("%f %d\r\n",parameters.can0Speed * 0.001f, 0);
        printf("a - Can Debug : ");
        if (candebug == 1)
        {
          printf("ON\r\n");
        }
        else
        {
          printf("OFF\r\n");
        }
        printf("b - EVSE Debug : ");
        if (evsedebug == 1)
        {
          printf("ON\r\n");
        }
        else
        {
          printf("OFF\r\n");
        }
        printf("t - termination voltage : ");
        printf("%f %d",parameters.tVolt * 0.01f, 0);
        printf("V\r\n");
        printf("q - To Quit Menu\r\n");
        break;
    }
  }
}
uint32_t millis(void){
	return HAL_GetTick();
}
uint32_t micros(void){ // returns microsecond
	uint32_t temp = HAL_GetTick();
	uint32_t microvalue= (temp*1000)+(1000-SysTick->VAL/168);
	return microvalue;
}
void delay(uint32_t dly){
	HAL_Delay(dly);
}

void Charger_msgs1(void){

	  /////////////////////This msg addresses all modules/////////////////////////////////////////////////
	  myTxMessage.StdId = 0x045c;            // Set our transmission address ID
	  myTxMessage.DLC = 8;            // Data payload 8 bytes

	  CanTxData[0] = (uint8_t)((parameters.voltSet)&0xFF);  //Voltage setpoint
	  CanTxData[1] = (uint8_t)((parameters.voltSet)>>8);//Voltage setpoint
	  CanTxData[2] = 0x14;
	  if (bChargerEnabled)
	  {
	    CanTxData[3] = 0x2e;
	  }
	  else {CanTxData[3] = 0x0e;}
	  CanTxData[4] = 0x00;
	  CanTxData[5] = 0x00;
	  CanTxData[6] = 0x90;
	  CanTxData[7] = 0x8c;
	  HAL_CAN_AddTxMessage(&hcan, &myTxMessage, CanTxData, &TxMailbox);
	  //////////////////////////////////////////////////////////////////////////////////////////////////////

	  //////////////////////////////////////Phase 1 command message////////////////////////////////////////
	  myTxMessage.StdId = 0x042c;            // Set our transmission address ID
	  myTxMessage.DLC = 8;            // Data payload 8 bytes

	  CanTxData[0] = 0x42;
	  CanTxData[2] = (uint8_t)(modulelimcur&0xFF); //AC Current setpoint
	  CanTxData[3] = (uint8_t)(modulelimcur>>8); //AC Current setpoint
	  if (bChargerEnabled)
	  {
	    CanTxData[1] = 0xBB;
	    CanTxData[4] = 0xFE; //FE dont clear faults. FF do clear faults.
	  }
	  else
	  {
	    CanTxData[1] = (uint8_t)(((uint16_t)(ACvoltIN / 1.2))&0xFF);
	    CanTxData[4] = 0x64;
	  }
	  CanTxData[5] = 0x00;
	  CanTxData[6] = 0x00;
	  CanTxData[7] = 0x00;
	  HAL_CAN_AddTxMessage(&hcan, &myTxMessage, CanTxData, &TxMailbox);


}
void Charger_msgs2(void){


	  //////////////////////////////Phase 2 command message/////////////////////////////////////////////*/

	  myTxMessage.StdId = 0x43c;        //phase 2 and 3 are copies of phase 1 so no need to set them up again
	  HAL_CAN_AddTxMessage(&hcan, &myTxMessage, CanTxData, &TxMailbox);
	  ///////////////////////////////Phase 3 command message/////////////////////////////////////////////

	  myTxMessage.StdId = 0x44c;
	  HAL_CAN_AddTxMessage(&hcan, &myTxMessage, CanTxData, &TxMailbox);

	  ///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////

	  myTxMessage.StdId = 0x368;            // Set our transmission address ID

	  CanTxData[0] = 0x03;
	  CanTxData[1] = 0x49;
	  CanTxData[2] = 0x29;
	  CanTxData[3] = 0x11;
	  CanTxData[4] = 0x00;
	  CanTxData[5] = 0x0c;
	  CanTxData[6] = 0x40;
	  CanTxData[7] = 0xff;
	  HAL_CAN_AddTxMessage(&hcan, &myTxMessage, CanTxData, &TxMailbox);

}

void Pilotcalc()
{
  if (HAL_GPIO_ReadPin(EVSE_PILOT_GPIO_Port, EVSE_PILOT_Pin) == GPIO_PIN_SET)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    accurlim = (micros() - pilottimer) * 100 / duration * 600; //Calculate the duty cycle then multiply by 600 to get mA current limit
    //printf( "\r\n");
    //printf("Pilot curlim: ");
    //printf(accurlim);
  }
}

void ACcurrentlimit()
{
  if (parameters.autoEnableCharger == 1)
  {
    if (micros() - pilottimer > 1200) //too big a gap in pilot signal kills means signal error or disconnected so no current allowed.
    {
      accurlim = 0;
    }
    if (parameters.phaseconfig == 0)
    {
      modulelimcur = (accurlim / 3) * 1.5 ; // all module parallel, sharing AC input current
    }
    else
    {
      modulelimcur = accurlim * 1.5; // one module per phase, EVSE current limit is per phase
      if (modulelimcur > (cablelim * 1.5))
      {
        modulelimcur = cablelim * 1.5;
      }
    }
  }
  else
  {
    if (parameters.phaseconfig == 0)
    {
      modulelimcur = (parameters.currReq / 3); // all module parallel, sharing AC input current
    }
    else
    {
      modulelimcur = parameters.currReq;
    }
  }
  if ((parameters.canControl == 1 )| (parameters.canControl == 2))
  {
    if (accurlim * 1.5 > (16000 * 1.5)) //enable second charger if current available >15A
    {
      modulelimcur = modulelimcur * 0.5;
      slavechargerenable = 1;

    }
    else
    {
      slavechargerenable = 0;
    }
  }

  if (parameters.phaseconfig == 1)
  {
    if (modulelimcur > (dcaclim * 1.5)) //if more current then max per module or limited by DC output current
    {
      // modulelimcur = (dcaclim * 1.5);
    }
    if (modulelimcur > parameters.currReq) //if evse allows more current then set in parameters limit it
    {
      modulelimcur = parameters.currReq;
    }
  }
  if (parameters.phaseconfig == 0)
  {
    if (modulelimcur > (dcaclim * 0.5)) //if more current then max per module or limited by DC output current
    {
      //modulelimcur = (dcaclim * 0.5);
    }
    if (modulelimcur > (parameters.currReq / activemodules)) //if evse allows more current then set in parameters limit it
    {
      modulelimcur = (parameters.currReq / activemodules);
    }
  }
  if (parameters.phaseconfig != 0 && parameters.phaseconfig != 1)
  {
    modulelimcur =  0;
  }
}

void DCcurrentlimit()
{
  totdccur = 0;
  for (int x = 0; x < 3; x++)
  {
    totdccur = totdccur + (dccur[x] * 0.1678466) ;
  }
  dcaclim = 0;
  int x = 0;
  if (totdccur > 0.2)
  {
    dcaclim = (((float)dcvolt[x] / (float)acvolt[x]) * (maxdccur * 1.2)) ; /// activemodules
  }
  else
  {
    dcaclim = 5000;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_6){
		Pilotcalc();
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
		if (mescycle > 1){
			mescycle = 0;}

		if (mescycle == 0 ){

		Charger_msgs1();
	}
		else {
			Charger_msgs2();
		}
		mescycle ++;
	}


	if(htim->Instance == TIM4){
		sprintf(wifistr,"va%03d,vb%03d,vc%03d,ia%03d,ib%03d,ic%03d,ua%03d,ub%03d,uc%03d,aa%03d,ab%03d,ac%03d*",acvolt[0],acvolt[1],acvolt[2]
																											   ,accur[0],accur[1],accur[2]
																												,dcvolt[0],dcvolt[1],dcvolt[2]
																												,dccur[0],dccur[1],dccur[2]);
		HAL_UART_Transmit(&huart3, (uint8_t*)wifistr, 73, 5);
	}
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */

}

