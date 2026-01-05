/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <clarke.h>
#include <vsd.h>
#include <ivsd.h>
#include <ipark.h>
#include <park.h>
#include <park_xy.h>
#include <ipark_xy.h>
#include <ramp_gen.h>
#include <rampctrl.h>
#include <svgen.h>
#include <Vhz_profile.h>
#include <pid.h>
#include <piq.h>
#include <pi.h>
#include <pi_spd.h>
#include <PhaseVoltage.h>
#include <DataLogger.h>
#include <fluxObs.h>
#include <fluxObs_const.h>
#include <speed_est.h>
#include <speed_const.h>
#include "PFC_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TBPRD_PWM 21250                       // 10 kHz        Up-down count mode for TIM1 with controlling the PWM.
#define HALF_TBPRD_PWM 10625                  // 10 kHz
#define Fs 4000                             // Switching frequency
#define T 0.00025                             // Sampling Period
#define PI 3.14159265358979
#define Oneby2PI 0.15915494



#define VBASE 		380                           // Base Voltage : Vdc/sqrt(3)
#define IBASE 		10                            // Base Current
#define FBASE 		100                           // Base Frequency
#define INV_FBASE 	0.01                           // 1/FBASE
#define INV_IBASE   0.1                           // 1/IBASE

#define RS 			1.15                               // Stator resistance
#define RR 			1.20                               // Rotor Resistance
#define LS 			0.114                             // Stator Inductance
#define LR 			0.114                             // Rotor Inductance
#define LM 			0.105                             // Magnetizing Inductance
#define POLES   	6

//#define RS 			2.5                               // Stator resistance
//#define RR 			1.31                               // Rotor Resistance
//#define LS 			0.265                             // Stator Inductance
//#define LR 			0.265                             // Rotor Inductance
//#define LM 			0.256                             // Magnetizing Inductance
//#define POLES   	4


#define IBUFF_SIZE 1000
#define FBUFF_SIZE 1000


#define MODE_1 1                 // Double Scalar Control
#define MODE_2 2                 // VSD Scalar Control + XY Compensation
#define MODE_3 3                 // Dual Vector Control
#define MODE_4 4                 // VSD Control
#define MODE_5 5                 // Dual I/F Control
#define MODE_6 6                 // VSD IF Control


#define MODE MODE_2

#ifndef MODE
#error Define MODE OF OPERATION!!!
#endif


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t ADC2_buffer[4];   // Motor Phase Currents

volatile uint16_t Inj_A1 = 0;
volatile uint16_t Inj_B1 = 0;
volatile uint16_t Inj_A2 = 0;
volatile uint16_t Inj_B2 = 0;
volatile uint16_t Inj_C1 = 0;
volatile uint16_t Inj_C2 = 0;


DATA_LOG_4CH_INT dLogInt;
DATA_LOG_4CH_FLOAT dLogFloat;
int16_t iBuff1[IBUFF_SIZE];
int16_t iBuff2[IBUFF_SIZE];
int16_t iBuff3[IBUFF_SIZE];
int16_t iBuff4[IBUFF_SIZE];
float fBuff1[FBUFF_SIZE];
float fBuff2[FBUFF_SIZE];
float fBuff3[FBUFF_SIZE];
float fBuff4[FBUFF_SIZE];
uint16_t clear_screen, print_log;


// User defined Variables
uint16_t Enable = 0;
uint16_t counter = 0;
uint16_t mode = 1;
uint16_t Activate = 0;
uint16_t SpeedLoopCount = 1;
uint16_t ClosedCurrent = 0;
uint16_t ClosedSpeed = 0;

float user_freq = 10;
float user_current = 0.5;
float mod_index = 0.1;
float mod = 1;
float Vd = 0;
float Vq = 0;
float Valpha = 0;
float Vbeta = 0;
float freq_pu = 0.0;
float F_rate_up = 0.02;
float phase_shift = PI/6;

const float TwoPI = 2.0f*M_PI;

float Trig = 1.0;

// Sensor Offsets and Gains (Calibrated Using Experiments)
float Three3VBy4095 = 0.000805 ;
float V_Offset = -1.5;
float Slope_A1 = 0.1206f ;
float Slope_B1 = 0.1218f;
float Slope_A2 = 0.1222f;
float Slope_B2 = 0.1199f;

float VDC_Slope = - 0.4032;
float VDC_Intercept = 690.37;


// Control Variables
uint32_t pwm_cnt =1;
uint32_t ctr_cnt = 1;
uint32_t pfc_cnt = 1;
uint16_t pfc_control_run=0;
uint32_t counter_TIM1 = 0;
uint32_t counter_TIM8 = 0;
uint16_t enable_voltage_closed_loop=0;
uint16_t SpeedLoopPrescaler = 10;
float A1_INT,B1_INT,A2_INT,B2_INT;
float A1,B1,C1,A2,B2,C2;
float Vdc;
float Angle,Angle1,Angle2,Angle6;
float Alpha, Beta,X,Y;

float IdRef = 0.3;                            // 30 percent of nominal current
float IqRef = 0.0;


// Controller Gains
float Kpp = 0.018;  //0.001;
float Kii = 0.05;  //0.05;

float Kpx = 0.01;
float Kix = 0.05;

float Kpw = 1.0;
float Kiw = 0.0005;


//Per unit Values
float Z_base,Phi_base,L_base,W_base,R_pu,Ls_pu;


uint32_t cnt = 1;

//PFC variables
//junk

//**********************//
float Vac_filtered, Vac_RMS, VacPK_inv,Vdc_filtered, Vburst_H, Vburst_L, Vac_RMS_real;
float Vdc,IL, Vac,IA,IB,IC,NTC_temp;
float VdcRef, VdcTarget = 120, ev, DemandCurrent,DemandCurrentMAX = 1500,dVref = 0.000302265f, Vary_Vc =150.0;  // 1 corresponds to 3.7 V/s
float NTC_Precharge, Vdc_Trip, IL_Trip, IL_ref;
float ei, pwm_cmd, pwm_cmd_debug, Vc, duty_PFC;

const float IL_calibration = 1.0f;
const float Vac_calibration = 1.0f; //0.823f;
const float Vdc_calibration = 1.0f;

volatile uint16_t Vdc_INT,Vac_INT,IL_INT,IA_INT,IB_INT,IC_INT, Temp_INT, Uu_INT, Uv_INT, Uw_INT, OverCurrent_INT;
volatile uint16_t ADC4_buffer[ADC_BUF_SIZE];
volatile uint32_t ADC_buffer2[ADC_BUF_SIZE-1];

uint16_t enable_closed_loop=0;
uint16_t enable_PFC_burst_mode = 0;
uint16_t pfc_DC_control_run=0;
uint16_t pwm_cmd_uint;

int enable_OCD = 0, CurrProtection_ON = 0, VolProtection_ON = 0, prot_var = 0, NTC_locked = 0;

//*********************** PFC Controlelr gains ***************
#define Vac_b0     0.000009807882855712//0.00006092
#define Vac_b1     0.000019615765711424//0.00012184
#define Vac_b2     0.000009807882855712//0.00006092
#define Vac_a1    -1.987472984166555//-1.9751
#define Vac_a2     0.987512215697978//0.9753

// current compensator gains, NEW Current compensator
#define CI_b0      0.044365534154009
#define CI_b1      -0.018039365595791
#define CI_b2      0.001176812183666
#define CI_a1      -0.451535753739813
#define CI_a2      -0.543225583261733

//voltage compensator gains
#define CV_b0     0.107379952051721  //0.099376577815668
#define CV_b1     0.000364078028194  //0.000311711092166
#define CV_b2     -0.107015874023527 //-0.099064866723502
#define CV_a1     -1.983161391196042 //-1.9844144453917
#define CV_a2     0.983161391196042  //0.9844144453917
// Notch filter gains (50Hz)
#define Notch_b0    0.9849830  // 0.9860932
#define Notch_b1    -1.9687630 // -1.9711512
#define Notch_b2    0.9849160  // 0.9860312
#define Notch_a1    -1.9687630 // -1.9711512
#define Notch_a2    0.9698990  // 0.9721243


//***************************protection init***********************************
int Read_EARTH_Fault, Read_SC_Fault, Read_OC_Fault, Read_OnOff_Control;
int OCfault_Flag, count_SCFault, EM_Stop, count_EarthFault ;
//**************************************************************
// Display variables
extern DATA_LOG_4CH_INT dLogInt;
extern DATA_LOG_4CH_FLOAT dLogFloat;

// Initialize MACROS

RAMPGEN rg1 = RAMPGEN_DEFAULTS;
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
VSD vsd = VSD_DEFAULTS;
IVSD ivsd = IVSD_DEFAULTS;
CLARKE clarke1 = CLARKE_DEFAULTS;
CLARKE clarke2 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
PARK   park2   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;
IPARK  ipark2  = IPARK_DEFAULTS;
PARK_XY park_xy = PARK_XY_DEFAULTS;
IPARK_XY ipark_xy = IPARK_XY_DEFAULTS;
SVGEN svgen1 = SVGEN_DEFAULTS;
SVGEN svgen2 = SVGEN_DEFAULTS;
VHZPROF vhz1 = VHZPROF_DEFAULTS;
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;
PHASEVOLTAGE volt2 = PHASEVOLTAGE_DEFAULTS;
PI_CONTROLLER pi_id1 = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq1 = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id2 = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq2 = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_ix = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iy = PI_CONTROLLER_DEFAULTS;
PI_SPD_CONTROLLER pi_spd = PI_SPD_CONTROLLER_DEFAULTS;
FLUX_OBS flux1 = FLUXOBS_DEFAULTS;
FLUXOBS_CONST flux_const1 = FLUXOBS_CONST_DEFAULTS;
SPEEDEST speed1 = SPEEDEST_DEFAULTS;
SPEED_CONST speed_const1 = SPEED_CONST_DEFAULTS;
DF22 testDF22filter;
DF22 NotchFilter;
DF22 CurrentCompensator;
DF22 DCVoltageCompensator;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC4_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void PWM_start(void);
void PWM_stop(void);
void PFC_PWM_start(void);
void PFC_PWM_stop(void);
void UARTCommands(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buf[12];
uint8_t msg[32];
uint8_t num[6];
uint8_t TXbuffer[50];
uint8_t received;
unsigned char* msg2;
uint8_t buffer[10];
unsigned char* display_number;
float PWM_PFC_duty, duty;
uint16_t ramp_duty=0, no_fault = 0, OC_var = 1, IsRelayOn = 0, PFC_PWM = 0;
int16_t trigger=0;
float trigger_float=0;
float EM_SD_Status;
void PrintScreen(void);
void printDataLog(int16_t *DataBuffer1, int16_t *DataBuffer2, int16_t *DataBuffer3, int16_t *DataBuffer4, uint16_t dataBufferLength, uint16_t *start);
void runVDCramp(void);
void Check_OCFault(void) ;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	vhz1.VoltMax = 1.0;
	vhz1.VoltMin = 0.01;
	vhz1.HighFreq = 0.9;
	vhz1.LowFreq = 0.05;
	vhz1.FreqMax = 1.0;

	rc1.Tstep = T*rc1.RampDelayMax*F_rate_up;
	rg1.StepAngleMax = FBASE*T;

// Initialize motor parameters for flux observer
	flux_const1.Rs = RS;
	flux_const1.Rr = RR;
	flux_const1.Ls = LS;
	flux_const1.Lr = LR;
	flux_const1.Lm = LM;
	flux_const1.Vb = VBASE;
	flux_const1.Ib = IBASE;
	flux_const1.Ts = T;
	FLUXOBS_CONST_MACRO(flux_const1)

// 	Initialize constants for flux observer
	flux1.K1 = flux_const1.K1;
	flux1.K2 = flux_const1.K2;
	flux1.K3 = flux_const1.K3;
	flux1.K4 = flux_const1.K4;
	flux1.K5 = flux_const1.K5;
	flux1.K6 = flux_const1.K6;
	flux1.K7 = flux_const1.K7;
	flux1.K8 = flux_const1.K8;
	flux1.Kp = 0.05;
	flux1.Ki = 0.1;

	// 	Initialize constants for speed estimation

	speed_const1.Rr = RR;
	speed_const1.Lr = LR;
	speed_const1.fb = FBASE;
	speed_const1.fc = 0.5;
	speed_const1.Ts = T;
	SPEED_CONST_MACRO(speed_const1)

	// Initialize parameters for speed estimation

	speed1.K1 = speed_const1.K1;
	speed1.K2 = speed_const1.K2;
	speed1.K3 = speed_const1.K3;
	speed1.K4 = speed_const1.K4;
	speed1.BaseRpm = 120*FBASE/POLES;


	pi_spd.Ki = Kiw;
	pi_spd.Kp = Kpw;
	pi_spd.Umax = 0.8;
	pi_spd.Umin = -0.8;


	pi_id1.Ki = Kii;
	pi_id1.Kp = Kpp;
	pi_id1.Umax = 0.8;
    pi_id1.Umin = -0.8;

    pi_iq1.Ki = Kii;
    pi_iq1.Kp = Kpp;
    pi_iq1.Umax = 0.8;
    pi_iq1.Umin = -0.8;

	pi_id2.Ki = Kii;
	pi_id2.Kp = Kpp;
	pi_id2.Umax = 0.8;
	pi_id2.Umin = -0.8;

	pi_iq2.Ki = Kii;
	pi_iq2.Kp = Kpp;
	pi_iq2.Umax = 0.8;
	pi_iq2.Umin = -0.8;

	pi_id.Ki = Kii;
	pi_id.Kp = Kpp;
	pi_id.Umax = 0.8;
    pi_id.Umin = -0.8;

    pi_iq.Ki = Kii;
    pi_iq.Kp = Kpp;
    pi_iq.Umax = 0.8;
    pi_iq.Umin = -0.8;

    pi_ix.Ki = Kix;
	pi_ix.Kp = Kpx;
	pi_ix.Umax = 0.3;
	pi_ix.Umin = -0.3;

	pi_iy.Ki = Kix;
	pi_iy.Kp = Kpx;
	pi_iy.Umax = 0.3;
	pi_iy.Umin = -0.3;


//	W_base = 2*PI*FBASE;
//	Z_base = VBASE/IBASE;
//	Phi_base = VBASE/W_base;
//	L_base = Z_base/W_base;
//	Ls_pu = Ls/L_base;
//	R_pu = Rs/Z_base;

//	pi_id1.Ki = R_pu/Ls_pu;
//	pi_id1.Kp = 0.1*Ls_pu*2*PI*Fs/15/W_base;



  // Initialize the PI module for Id
  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  DATA_LOG_4CH_FLOAT_config(&dLogFloat,
                                     &ivsd.As1,
                                     &pi_id.Fbk,
                                     &pi_spd.Out,
                                     &pi_iq.Fbk,
   									fBuff1,
   									fBuff2,
   									fBuff3,
   									fBuff4,
   									FBUFF_SIZE,
                                       0.5,    //trigger
                                       1    // preScalar
                                       );

 DATA_LOG_4CH_INT_config(&dLogInt,
								&ivsd.As1,
								&pi_id.Fbk,
								&pi_spd.Out,
								&pi_iq.Fbk,
								iBuff1,
								iBuff2,
								iBuff3,
								iBuff4,
								IBUFF_SIZE,
								   1, //trigger
								   1 // preScalar
								   );

 //PFC controller variables
 InitFilterDF22(&testDF22filter,Vac_b0,Vac_b1,Vac_b2,Vac_a1,Vac_a2);
 InitFilterDF22(&CurrentCompensator,CI_b0,CI_b1,CI_b2,CI_a1,CI_a2);
 InitFilterDF22(&NotchFilter,Notch_b0,Notch_b1,Notch_b2,Notch_a1,Notch_a2);
 InitFilterDF22(&DCVoltageCompensator,CV_b0,CV_b1,CV_b2,CV_a1,CV_a2);


  clear_screen=1;


  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim1));                   // Keep Compare register == ARR
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);                                                       // Clear Flags

//  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim8));                   // Keep Compare register == ARR
//  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC4);                                                       // Clear Flags




  HAL_TIM_Base_Start(&htim1);                                                                       // Start TIM1

  HAL_TIM_Base_Start_IT(&htim8);                                                                   // Start TIM8 first (slave waits for trigger)

  //HAL_TIM_Base_Start_IT(&htim8);
 // HAL_TIM_Base_Start_IT(&htim15);
//  HAL_TIM_Base_Start_IT(&htim4);

  //HAL_ADC_Start_DMA(&hadc2, ADC2_buffer, 4);
 // HAL_ADC_Start_DMA(&hadc4, ADC4_buffer, 3); //PFC

  if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED)) Error_Handler();
  if(HAL_OK != HAL_ADC_Start_DMA(&hadc4, (uint32_t*)ADC4_buffer, ADC_BUF_SIZE)) Error_Handler();//Start ADC4 with DMA

  HAL_ADCEx_InjectedStart_IT(&hadc2); // Start Motor ADC Conversion (Injected Channel, Tim1-PWM-Triggered, Interrupt Mode)

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                    // Start PWMs TIM 1 - Channels 1 - 3
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);                     // Start Compare Path  - Channel 4

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);


  /*-------------- PFC init begin----------------------------*/
  Vc=Vary_Vc ;
  VdcRef=90.0f;
//  VdcTarget_init=120 ; //380.0f;
  IL_Trip = -30;
  //dVref=1.0f; // VDC ramp rate
 // DemandCurrentMAX=1500;        //9500.0;
  NTC_Precharge = 250.0; // 250dc-150V forNTC
  Vdc_Trip = 350.0;
  Vburst_H = 400.0;
  Vburst_L = 370.0;
  enable_closed_loop=0;
  enable_voltage_closed_loop=0;
  /*-------------- PFC init end----------------------------*/

  //****************** GPIO settings for protection *************************
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   //PD13- EM_STOP_SD - high for IPM to be on, if low then its fault
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);  // OC_fault
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // SC_fault
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET); // Earth_fault
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  // on off


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    pi_spd.Ki = Kiw; 	pi_spd.Kp = Kpw;

		pi_id1.Ki = Kii; 	pi_id1.Kp = Kpp;
		pi_iq1.Ki = Kii; 	pi_iq1.Kp = Kpp;

		pi_id2.Ki = Kii; 	pi_id2.Kp = Kpp;
		pi_iq2.Ki = Kii; 	pi_iq2.Kp = Kpp;

		pi_id.Ki = Kii;		pi_id.Kp = Kpp;
		pi_iq.Ki = Kii;		pi_iq.Kp = Kpp;

		pi_ix.Ki = Kix;		pi_ix.Kp = Kpx;
		pi_iy.Ki = Kix;		pi_iy.Kp = Kpp;


	  if(print_log)
	 	 {
	 	  	printDataLog(iBuff1, iBuff2, iBuff3, iBuff4, IBUFF_SIZE, &print_log);
	 	 }
	 else PrintScreen();

/*-------------- PFC init begin----------------------------*/
	//  if((Vdc>NTC_Precharge/*2376*/) && (NTC_locked == 0)) // relay threshold for bypassing relay in PFC
	  if(IsRelayOn == 1)
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); //PE3 - input for NTC, LED off
		  ramp_duty=1;
		//  HAL_Delay(20);
		  NTC_locked = 1;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); //LED ON
	  }
	 // else if (Vdc<150.0f/*1944*/)
//	  {
//		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//		//  duty=0;   // uncomment this for real proto
//		  ramp_duty=0;
//	  }

	  PWM_PFC_duty = (duty*2297.0f);// use constant instead of 2125.0f
	  if(enable_closed_loop==0) __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_PFC_duty);

//	  if(PFC_PWM == 1)
//	  {
//		  PFC_PWM_start();
//	  }
//	  else PFC_PWM_stop();

	  /*-------------- protections ----------------------------*/
	   Check_OCFault() ; //OC fault detection

////PD10 for on off control
	  Read_OnOff_Control = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) ; // 1- on off switch is off, 0-on off switch is on

//	  if (Read_OnOff_Control == 1)
//	  {
//		  PWM_stop();  //MCU stops PWM
//	  }

	  //enable this to test fault
	  if (EM_Stop)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //IPM_stops here
	  }

	  /*-------------- protection end ----------------------------*/

	  UARTCommands();

	  if(Vac_RMS>30.0) VacPK_inv= 0.5/Vac_RMS;//0.707106781/Vac_RMS;
/*-------------- PFC init end----------------------------*/

	 HAL_Delay(300);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 3;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 21250;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 21250;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 170;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2297;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2297;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period = 21250;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 170;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Reset_Controllers(void){
	pi_id1.ui = 0; pi_id1.i1 = 0; pi_id1.Out = 0;
	pi_iq1.ui = 0; pi_iq1.i1 = 0; pi_iq1.Out = 0;
	pi_id2.ui = 0; pi_id2.i1 = 0; pi_id2.Out = 0;
	pi_id2.ui = 0; pi_id2.i1 = 0; pi_iq2.Out = 0;
	pi_id.ui  = 0; pi_id.i1 = 0; pi_id.Out = 0;
	pi_iq.ui  = 0; pi_iq.i1 = 0; pi_iq.Out = 0;
	pi_ix.ui  = 0; pi_ix.i1 = 0; pi_ix.Out = 0;
	pi_iy.ui  = 0; pi_iy.i1 = 0; pi_iy.Out = 0;

}

void PWM_start(void){

	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}


void PWM_stop(void){
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
}


void PrintScreen(void)
{
//    msg = "\033[2J\0";
	if(clear_screen)
	{
		strcpy((char*)msg,"\033[2J\0");
		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
		clear_screen=0;
	}

	strcpy((char*)msg,"\033[2;20HABB USRC SPACER Project PFC\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);

// Line 2
    strcpy((char*)msg,"\033[5;1HFrequency=\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	sprintf(msg,"%f    ",freq_pu);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    strcpy((char*)msg,"\033[5;25HIdRef=\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	sprintf(msg,"%f    ",IdRef);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//    strcpy((char*)msg,"\033[5;50HIL=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",IL);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	//Line 3
//    strcpy((char*)msg,"\033[7;1HIsRelayOn=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	//sprintf(msg,"%hu\r\n",Vdc_INT);
//	sprintf(msg,"%f    ",IsRelayOn);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//    strcpy((char*)msg,"\033[7;25HPFC_duty=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",duty);
//	//snprintf(num,4,"%u",Vac);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//    strcpy((char*)msg,"\033[7;50HonOff status=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%u    ",Read_OnOff_Control);
//	//snprintf(num,4,"%u",Vac);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	//line 4
    strcpy((char*)msg,"\033[9;1HEARTH_Fault=\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	//sprintf(msg,"%hu\r\n",Vdc_INT);
	sprintf(msg,"%u    ",Read_EARTH_Fault);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    strcpy((char*)msg,"\033[9;25HOC_Fault=\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	sprintf(msg,"%u    ",Read_OC_Fault);
	//snprintf(num,4,"%u",Vac);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    strcpy((char*)msg,"\033[9;50HSC_Fault=\0");
    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	sprintf(msg,"%u    ",Read_SC_Fault);
	//snprintf(num,4,"%u",Vac);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	//line 5
//    strcpy((char*)msg,"\033[11;1HIL_ref=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	//sprintf(msg,"%hu\r\n",Vdc_INT);
//	sprintf(msg,"%f    ",IL_ref);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    strcpy((char*)msg,"\033[11;25HVc=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",Vary_Vc);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    strcpy((char*)msg,"\033[11;50HVac=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",Vac);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//	//line 6
//    strcpy((char*)msg,"\033[13;1HDmndCurr=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",DemandCurrent);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    strcpy((char*)msg,"\033[13;25HPFC I-protection=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%u    ",CurrProtection_ON);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    strcpy((char*)msg,"\033[13;50HVdc_target=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",VdcTarget);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//	//Line 7
//    strcpy((char*)msg,"\033[15;1HPFC V-protection=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%u    ",VolProtection_ON);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    strcpy((char*)msg,"\033[15;25HDmdCurrMax=\0");
//    HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
//	sprintf(msg,"%f    ",DemandCurrentMAX);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

}

void printDataLog(int16_t *DataBuffer1, int16_t *DataBuffer2, int16_t *DataBuffer3, int16_t *DataBuffer4, uint16_t dataBufferLength, uint16_t *start)
{
    //unsigned char *u;
    int i=0;

    // Clear screen first
	strcpy((char*)msg,"\033[2J\033[1;1H\0");
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);


    for(i=0;i<dataBufferLength;i++)
    {

     	sprintf(msg,"%f\t",fBuff1[i]);
     	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

     	sprintf(msg,"%f\t",fBuff2[i]);
     	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

     	sprintf(msg,"%f\t",fBuff3[i]);
     	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

     	sprintf(msg,"%f\r\n",fBuff4[i]);
     	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);

    }

	strcpy((char*)msg,"\033[2J\0");
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);


    *start=0;
    clear_screen=1;
    trigger=0;
    trigger_float=0;

}

///* ---------------------- Callbacks ---------------------- */
//// One-shot: CH4 occurs at CNT==ARR (overflow). Use it to phase-lock UIE to subsequent UNDERFLOW.
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//    {
//        // Stop CC4 IRQ (we only needed the first overflow)
//        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
//
//        // Clear any pending Update flag just in case
//        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
//
//        // Enable Update interrupt now; with RCR=1, next UIE will be at UNDERFLOW and then every UNDERFLOW
//        __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
//    }
//}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//pwm_cnt++;

//	  Read_SC_Fault = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14)    ;
//	  Read_EARTH_Fault = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15) ;
//
//  if ((Read_SC_Fault == 0) || (Read_EARTH_Fault == 0))
//      {
//	      PWM_stop();  //MCU stops PWM
//      }

	ctr_cnt++;
	counter_TIM1 = __HAL_TIM_GET_COUNTER(&htim1);
	counter_TIM8 = __HAL_TIM_GET_COUNTER(&htim8);

#if (MODE == MODE_1)
 {
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
    Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // PC0
    Inj_B1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // PA6
    Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); // PC1
    Inj_B2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4); // PA7

    //1.5V is 0 value for ADC --> 1861, not 1.65V
	A1 = ((Inj_A1+66)*Three3VBy4095 + V_Offset)/Slope_A1 ;
	B1 = ((Inj_B1+66)*Three3VBy4095 + V_Offset)/Slope_B1;
	C1 = -(A1+B1);

	A2 = ((Inj_A2+66)*Three3VBy4095 + V_Offset)/Slope_A2 ;
	B2 = ((Inj_B2+66)*Three3VBy4095 + V_Offset)/Slope_B2;
	C2 = -(A2+B2);

	if (Activate == 1)
	{

		rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		// define inputs for inverse park transform
		Angle1  = rg1.Out*2*PI ;
		Angle2  = fmod((Angle1 - phase_shift),TwoPI);

		vhz1.Freq = rc1.SetpointValue;
		VHZ_PROF_MACRO(vhz1);

		Vq = 0;
		Vd = vhz1.VoltOut;

		ipark1.Sine   = sin(Angle1);
		ipark1.Cosine = cos(Angle1);
		ipark2.Sine   = sin(Angle2);
		ipark2.Cosine = cos(Angle2);

		ipark1.Ds = Vd;
		ipark1.Qs = Vq;
		ipark2.Ds = Vd;
		ipark2.Qs = Vq;

		IPARK_MACRO(ipark1);
		IPARK_MACRO(ipark2);

//		clarke1.As = -A1*INV_IBASE;
//		clarke1.Bs = -B1*INV_IBASE;
//		CLARKE_MACRO(clarke1);

//		volt1.DcBusVolt = 1;
//		volt1.MfuncV1 = svgen1.Ta;
//		volt1.MfuncV2 = svgen1.Tb;
//		volt1.MfuncV3 = svgen1.Tc;
//		PHASEVOLT_MACRO(volt1);
//
//		flux1.UDsS = volt1.Valpha;
//		flux1.UQsS = volt1.Vbeta;
//		flux1.IDsS = clarke1.Alpha;
//		flux1.IQsS = clarke1.Beta;
//		FLUX_OBS_MACRO(flux1);
//
//		speed1.IDsS = clarke1.Alpha;
//		speed1.IQsS = clarke1.Beta;
//		speed1.PsiDrS = flux1.PsiDrS;
//		speed1.PsiQrS = flux1.PsiQrS;
//		speed1.ThetaFlux = flux1.ThetaFlux*Oneby2PI;
//		SPEEDEST_MACRO(speed1);

		// define inputs for Space vector Generation
		svgen1.Ualpha = ipark1.Alpha;
		svgen1.Ubeta  = ipark1.Beta;
		svgen2.Ualpha = ipark2.Alpha;
		svgen2.Ubeta  = ipark2.Beta;

		SVGENDQ_MACRO(svgen1);
		SVGENDQ_MACRO(svgen2);

	}
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
 }
#endif


#if (MODE == MODE_2)
 {
	 //	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
		 Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // PC0
		 Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); // PC1
		 Inj_C1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // PA1
		 Inj_C2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4); // PB2

		 //1.5V is 0 value for ADC --> 1861, not 1.65V
		A1 = ((Inj_A1+66)*Three3VBy4095 + V_Offset)/Slope_A1 ;
		C1 = ((Inj_C1+66)*Three3VBy4095 + V_Offset)/Slope_A1;
		B1 = -(A1+C1);

		A2 = ((Inj_A2+66)*Three3VBy4095 + V_Offset)/Slope_A2 ;
		C2 = ((Inj_C2+66)*Three3VBy4095 + V_Offset)/Slope_A2;
		B2 = -(A2+C2);

	if (Enable == 1)
	{
		rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		// define inputs for inverse park transform
		Angle  = rg1.Out*TwoPI ;
		Angle6 = fmod(Angle,TwoPI);

		vhz1.Freq = rc1.SetpointValue;
		VHZ_PROF_MACRO(vhz1);

		Valpha = vhz1.VoltOut*cos(Angle);
		Vbeta = vhz1.VoltOut*sin(Angle);

		// Angle for XY subspace transformation
		ipark_xy.Sine   = sin(Angle);
		park_xy.Sine = ipark_xy.Sine;

		ipark_xy.Cosine = cos(Angle);
		park_xy.Cosine = ipark_xy.Cosine;

		// VSD Transform for current
		vsd.As1 = -A1*INV_IBASE;
		vsd.Bs1 = -B1*INV_IBASE;
		vsd.Cs1 = -C1*INV_IBASE;
		vsd.As2 = -A2*INV_IBASE;
		vsd.Bs2 = -B2*INV_IBASE;
		vsd.Cs2 = -C2*INV_IBASE;
		VSD_MACRO(vsd);

		// Park Transform for XY
        park_xy.Alpha = vsd.X;
        park_xy.Beta = vsd.Y;
        PARK_XY_MACRO(park_xy);

        // PI Controllers - XY_frame
		pi_ix.Ref = 0;
		pi_ix.Fbk = park_xy.Ds;
		PI_MACRO(pi_ix);

		pi_iy.Ref = 0;
		pi_iy.Fbk = park_xy.Qs;
		PI_MACRO(pi_iy);


		//Park Transform
		ipark_xy.Ds =  pi_ix.Out ;
		ipark_xy.Qs =  pi_iy.Out ;
		IPARK_XY_MACRO(ipark_xy);

        // Inverse VSD Transform
		ivsd.Alpha = Valpha;
		ivsd.Beta = Vbeta;

		if (Activate == 1)
		{
			ivsd.X = ipark_xy.Alpha;
			ivsd.Y = ipark_xy.Beta;
		}
		else
		{
			ivsd.X = 0;
			ivsd.Y = 0;
		}
		IVSD_MACRO(ivsd);

//		volt1.DcBusVolt = 1;
//		volt1.MfuncV1 = ivsd.As1;
//		volt1.MfuncV2 = ivsd.Bs1;
//		volt1.MfuncV3 = ivsd.Cs1;
//		PHASEVOLT_MACRO(volt1);
//
//		flux1.UDsS = volt1.Valpha;
//		flux1.UQsS = volt1.Vbeta;
//		flux1.IDsS = vsd.Alpha;
//		flux1.IQsS = vsd.Beta;
//		FLUX_OBS_MACRO(flux1);
//
//		speed1.IDsS = vsd.Alpha;
//		speed1.IQsS = vsd.Beta;
//		speed1.PsiDrS = flux1.PsiDrS;
//		speed1.PsiQrS = flux1.PsiQrS;
//		speed1.ThetaFlux = flux1.ThetaFlux*Oneby2PI;
//		SPEEDEST_MACRO(speed1);


	}
 }
#endif

// Dual Vector control, Double SVM
#if (MODE == MODE_3)
 {
	Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	Inj_B1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
	Inj_B2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);


	A1 = Inj_A1 * SlopeA1 + A1_Intercept;
	B1 = Inj_B1 * SlopeA1 + A1_Intercept;
	C1 = -(A1+B1);

	A2 = Inj_A2 * SlopeA1 + A1_Intercept;
	B2 = Inj_B2 * SlopeA1 + A1_Intercept;
	C2 = -(A2+B2);

    if (ClosedCurrent == 1)
    {

    	rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		// define angle inputs for synchronous transformation
		if (ClosedSpeed == 0)
		{
			Angle1 = rg1.Out*TwoPI ;
		}
		else Angle1 = flux1.ThetaFlux ;
		Angle2 = fmod((Angle1 - phase_shift),TwoPI);

		ipark1.Sine   = sin(Angle1);
		park1.Sine = ipark1.Sine;

		ipark1.Cosine = cos(Angle1);
		park1.Cosine = ipark1.Cosine;

		ipark2.Sine   = sin(Angle2);
	    park2.Sine = ipark2.Sine;

		ipark2.Cosine = cos(Angle2);
		park2.Cosine = ipark2.Cosine;

		clarke1.As = -A1*INV_IBASE;
		clarke1.Bs = -B1*INV_IBASE;
		CLARKE_MACRO(clarke1);

		clarke2.As = -A2*INV_IBASE;
		clarke2.Bs = -B2*INV_IBASE;
		CLARKE_MACRO(clarke2);

		park1.Alpha = clarke1.Alpha;
		park1.Beta = clarke1.Beta;
		PARK_MACRO(park1);

		park2.Alpha = clarke2.Alpha;
		park2.Beta = clarke2.Beta;
		PARK_MACRO(park2);

		if (SpeedLoopCount == SpeedLoopPrescaler)
		{
			pi_spd.Ref = rc1.SetpointValue;
			pi_spd.Fbk = speed1.WrHat;
			PI_MACRO(pi_spd);
			SpeedLoopCount = 1;
		}
		else SpeedLoopCount++;


		if (ClosedSpeed == 0)
		{
			pi_iq1.Ref = 0;
			pi_iq2.Ref = 0;
			pi_spd.ui = 0;
			pi_spd.i1 = 0;
		}
		else
		{	pi_iq1.Ref = pi_spd.Out;
			pi_iq2.Ref = pi_spd.Out;
		}

		// PI Controller - First Winding Set
		pi_iq1.Fbk = park1.Qs;
		PI_MACRO(pi_iq1);

		pi_id1.Ref = IdRef;
		pi_id1.Fbk = park1.Ds;
		PI_MACRO(pi_id1);

		// PI Controller - Second Winding Set
		pi_iq2.Fbk = park2.Qs;
		PI_MACRO(pi_iq2);

		pi_id2.Ref = IdRef;
		pi_id2.Fbk = park2.Ds;
		PI_MACRO(pi_id2);

		// Inverse Park Transforms
		ipark1.Ds = pi_id1.Out;
		ipark1.Qs = pi_iq1.Out;
		IPARK_MACRO(ipark1);

		ipark2.Ds = pi_id2.Out;
		ipark2.Qs = pi_iq2.Out;
		IPARK_MACRO(ipark2);

		// Voltage reconstruction and speed estimation
		volt1.DcBusVolt = 1;
		volt1.MfuncV1 = svgen1.Ta;
		volt1.MfuncV2 = svgen1.Tb;
		volt1.MfuncV3 = svgen1.Tc;
		PHASEVOLT_MACRO(volt1);

		flux1.UDsS = volt1.Valpha;
		flux1.UQsS = volt1.Vbeta;
		flux1.IDsS = clarke1.Alpha;
		flux1.IQsS = clarke1.Beta;
		FLUX_OBS_MACRO(flux1);

		speed1.IDsS = clarke1.Alpha;
		speed1.IQsS = clarke1.Beta;
		speed1.PsiDrS = flux1.PsiDrS;
		speed1.PsiQrS = flux1.PsiQrS;
		speed1.ThetaFlux = flux1.ThetaFlux*Oneby2PI;
		SPEEDEST_MACRO(speed1);

		// define inputs for Space vector Generation
		svgen1.Ualpha = ipark1.Alpha;
		svgen1.Ubeta  = ipark1.Beta;
		SVGENDQ_MACRO(svgen1);

		svgen2.Ualpha = ipark2.Alpha;
		svgen2.Ubeta  = ipark2.Beta;
		SVGENDQ_MACRO(svgen2);

    }

  }
#endif

#if (MODE == MODE_4) // VSD Controller, Double PWM
{
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
	 Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // PC0
	 Inj_B1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // PA6
	 Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); // PC1
	 Inj_B2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4); // PA7

	 //1.5V is 0 value for ADC --> 1861, not 1.65V
	A1 = ((Inj_A1+66)*Three3VBy4095 + V_Offset)/Slope_A1 ;
	B1 = ((Inj_B1+66)*Three3VBy4095 + V_Offset)/Slope_B1;
	C1 = -(A1+B1);

	A2 = ((Inj_A2+66)*Three3VBy4095 + V_Offset)/Slope_A2 ;
	B2 = ((Inj_B2+66)*Three3VBy4095 + V_Offset)/Slope_B2;
	C2 = -(A2+B2);

	if (ClosedCurrent == 1)
	{
		rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		// define angle inputs for synchronous transformation
		if (ClosedSpeed == 0)
		{
			Angle = rg1.Out*TwoPI ;
		}
		else Angle = flux1.ThetaFlux ;

		ipark1.Sine   = sin(Angle);
		park1.Sine = ipark1.Sine;

		ipark1.Cosine = cos(Angle);
		park1.Cosine = ipark1.Cosine;

		ipark_xy.Sine   = sin(Angle);
		park_xy.Sine = ipark_xy.Sine;

		ipark_xy.Cosine = cos(Angle);
		park_xy.Cosine = ipark_xy.Cosine;


		// VSD Transform
		vsd1.As1 = -A1*INV_IBASE;
		vsd1.Bs1 = -B1*INV_IBASE;
		vsd1.Cs1 = -C1*INV_IBASE;
		vsd1.As2 = -A2*INV_IBASE;
		vsd1.Bs2 = -B2*INV_IBASE;
		vsd1.Cs2 = -C2*INV_IBASE;
		VSD_MACRO(vsd1);

		// Park Transforms
		park1.Alpha = vsd1.Alpha;
		park1.Beta = vsd1.Beta;
		PARK_MACRO(park1);

		park2.Alpha = vsd1.X;
		park2.Beta = vsd1.Y;
		PARK_MACRO(park2);

		if (SpeedLoopCount == SpeedLoopPrescaler)
		{
			pi_spd.Ref = rc1.SetpointValue;
			pi_spd.Fbk = speed1.WrHat;
			PI_MACRO(pi_spd);
			SpeedLoopCount = 1;
		}
		else SpeedLoopCount++;

		if (ClosedSpeed == 0)
		{
			pi_iq.Ref = 0;
			pi_spd.ui = 0;
			pi_spd.i1 = 0;
		}
		else
		{	pi_iq.Ref = pi_spd.Out;
		}


		// PI Controllers - DQ_frame
		pi_iq.Fbk = park1.Qs;
		PI_MACRO(pi_iq);

		pi_id.Ref = IdRef;
		pi_id.Fbk = park1.Ds;
		PI_MACRO(pi_id);

		// PI Controllers - XY_frame
		pi_ix.Ref = 0;
		pi_ix.Fbk = park2.Ds;
		PI_MACRO(pi_ix);

		pi_iy.Ref = 0;
		pi_iy.Fbk = park2.Qs;
		PI_MACRO(pi_iy);

		// Inverse Park Transforms
		ipark1.Ds = pi_id.Out;
		ipark1.Qs = pi_iq.Out;
		IPARK_MACRO(ipark1);

		ipark2.Ds = pi_ix.Out;
		ipark2.Qs = pi_iy.Out;
		IPARK_MACRO(ipark2);

		// Phase Voltage Reconstruction
		volt1.DcBusVolt = 1;
		volt1.MfuncV1 = ivsd1.As1;
		volt1.MfuncV2 = ivsd1.Bs1;
		volt1.MfuncV3 = ivsd1.Cs1;
		PHASEVOLT_MACRO(volt1);

		// Rotor Flux Observer
		flux1.UDsS = volt1.Valpha;
		flux1.UQsS = volt1.Vbeta;
		flux1.IDsS = vsd1.Alpha;
		flux1.IQsS = vsd1.Beta;
		FLUX_OBS_MACRO(flux1);

		// Speed Estimation
		speed1.IDsS = vsd1.Alpha;
		speed1.IQsS = vsd1.Beta;
		speed1.PsiDrS = flux1.PsiDrS;
		speed1.PsiQrS = flux1.PsiQrS;
		speed1.ThetaFlux = flux1.ThetaFlux*Oneby2PI;
		SPEEDEST_MACRO(speed1);

		// Inverse VSD Transform
		ivsd1.Alpha = ipark1.Alpha;
		ivsd1.Beta = ipark1.Beta;
		ivsd1.X = ipark2.Alpha;
		ivsd1.Y = ipark2.Beta;
		IVSD_MACRO(ivsd1);
        }

}

    #endif

#if (MODE == MODE_5)     //  Double I/F Control
 {
 //	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
	 Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // PC0
	 Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); // PC1
	 Inj_C1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // PA1
	 Inj_C2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4); // PB2

	 //1.5V is 0 value for ADC --> 1861, not 1.65V
	A1 = ((Inj_A1+66)*Three3VBy4095 + V_Offset)/Slope_A1 ;
	C1 = ((Inj_C1+66)*Three3VBy4095 + V_Offset)/Slope_A1;
	B1 = -(A1+C1);

	A2 = ((Inj_A2+66)*Three3VBy4095 + V_Offset)/Slope_A2 ;
	C2 = ((Inj_C2+66)*Three3VBy4095 + V_Offset)/Slope_A2;
	B2 = -(A2+C2);

    if (Activate == 1)
    {
    	rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		Angle1 = rg1.Out*TwoPI ;
		Angle2 = fmod((Angle1 - phase_shift),TwoPI);

		ipark1.Sine   = sin(Angle1);
		park1.Sine = ipark1.Sine;

		ipark1.Cosine = cos(Angle1);
		park1.Cosine = ipark1.Cosine;

		ipark2.Sine   = sin(Angle2);
	    park2.Sine = ipark2.Sine;

		ipark2.Cosine = cos(Angle2);
		park2.Cosine = ipark2.Cosine;

		clarke1.As = -A1*INV_IBASE;
		clarke1.Bs = -B1*INV_IBASE;
		CLARKE_MACRO(clarke1);

		clarke2.As = -A2*INV_IBASE;
		clarke2.Bs = -B2*INV_IBASE;
		CLARKE_MACRO(clarke2);

		park1.Alpha = clarke1.Alpha;
		park1.Beta = clarke1.Beta;
		PARK_MACRO(park1);

		park2.Alpha = clarke2.Alpha;
		park2.Beta = clarke2.Beta;
		PARK_MACRO(park2);

		// PI Controller - First Winding Set
		pi_iq1.Ref = 0;
		pi_iq1.Fbk = park1.Qs;
		PI_MACRO(pi_iq1);

		pi_id1.Ref = IdRef;
		pi_id1.Fbk = park1.Ds;
		PI_MACRO(pi_id1);

		// PI Controller - Second Winding Set
		pi_iq2.Ref = 0;
		pi_iq2.Fbk = park2.Qs;
		PI_MACRO(pi_iq2);

		pi_id2.Ref = IdRef;
		pi_id2.Fbk = park2.Ds;
		PI_MACRO(pi_id2);

		// Inverse Park Transforms
		ipark1.Ds = pi_id1.Out;
		ipark1.Qs = pi_iq1.Out;
		IPARK_MACRO(ipark1);

		ipark2.Ds = pi_id2.Out;
		ipark2.Qs = pi_iq2.Out;
		IPARK_MACRO(ipark2);

		// define inputs for Space vector Generation
		svgen1.Ualpha = ipark1.Alpha;
		svgen1.Ubeta  = ipark1.Beta;
		SVGENDQ_MACRO(svgen1);

		svgen2.Ualpha = ipark2.Alpha;
		svgen2.Ubeta  = ipark2.Beta;
		SVGENDQ_MACRO(svgen2);

    }

  }
#endif

#if (MODE == MODE_6) // VSD I/F Controller, Double PWM
{
	//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
		 Inj_A1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // PC0
		 Inj_A2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); // PC1
		 Inj_C1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // PA1
		 Inj_C2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4); // PB2

		 //1.5V is 0 value for ADC --> 1861, not 1.65V
		A1 = ((Inj_A1+66)*Three3VBy4095 + V_Offset)/Slope_A1 ;
		C1 = ((Inj_C1+66)*Three3VBy4095 + V_Offset)/Slope_A1;
		B1 = -(A1+C1);

		A2 = ((Inj_A2+66)*Three3VBy4095 + V_Offset)/Slope_A2 ;
		C2 = ((Inj_C2+66)*Three3VBy4095 + V_Offset)/Slope_A2;
		B2 = -(A2+C2);

	if (Activate == 1)
	{
		rc1.TargetValue = freq_pu;
		RC_MACRO(rc1);

		rg1.Freq = rc1.SetpointValue;
		RG_MACRO(rg1);

		Angle = rg1.Out*TwoPI ;

		ipark1.Sine   = sin(Angle);
		park1.Sine = ipark1.Sine;

		ipark1.Cosine = cos(Angle);
		park1.Cosine = ipark1.Cosine;

		ipark2.Sine   = sin(Angle);
		park2.Sine = ipark2.Sine;

		ipark2.Cosine = cos(Angle);
		park2.Cosine = ipark2.Cosine;

		// VSD Transform
		vsd1.As1 = -A1*INV_IBASE;
		vsd1.Bs1 = -B1*INV_IBASE;
		vsd1.Cs1 = -C1*INV_IBASE;
		vsd1.As2 = -A2*INV_IBASE;
		vsd1.Bs2 = -B2*INV_IBASE;
		vsd1.Cs2 = -C2*INV_IBASE;
		VSD_MACRO(vsd1);

		// Park Transforms
		park1.Alpha = vsd1.Alpha;
		park1.Beta = vsd1.Beta;
		PARK_MACRO(park1);

		park2.Alpha = vsd1.X;
		park2.Beta = vsd1.Y;
		PARK_MACRO(park2);

		// PI Controllers - DQ_frame
	    pi_iq.Ref = 0;
		pi_iq.Fbk = park1.Qs;
		PI_MACRO(pi_iq);

		pi_id.Ref = IdRef;
		pi_id.Fbk = park1.Ds;
		PI_MACRO(pi_id);

		// PI Controllers - XY_frame
		pi_ix.Ref = 0;
		pi_ix.Fbk = park2.Ds;
		PI_MACRO(pi_ix);

		pi_iy.Ref = 0;
		pi_iy.Fbk = park2.Qs;
		PI_MACRO(pi_iy);

		// Inverse Park Transforms
		ipark1.Ds = pi_id.Out;
		ipark1.Qs = pi_iq.Out;
		IPARK_MACRO(ipark1);

		ipark2.Ds = pi_ix.Out;
		ipark2.Qs = pi_iy.Out;
		IPARK_MACRO(ipark2);


		// Inverse VSD Transform
		ivsd1.Alpha = ipark1.Alpha;
		ivsd1.Beta = ipark1.Beta;
		ivsd1.X = ipark2.Alpha;
		ivsd1.Y = ipark2.Beta;
		IVSD_MACRO(ivsd1);
        }

}

    #endif

	 if( dLogFloat.count == FBUFF_SIZE-1 )
		{
			dLogFloat.status=0;
			dLogFloat.count=0;
			print_log=1;
		}
		DATA_LOG_4CH_FLOAT_run(&dLogFloat);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim == &htim4)                     // PFC ISR
	{
//************************** SC & and Earth Protection for Motor ************************************

//************************** SC & and Earth Protection for Motor ************************************

//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
		pfc_cnt++;
	    pfc_control_run=1;

		if(enable_voltage_closed_loop)
			{
				runVDCramp();
			}
		else VdcRef=Vdc_filtered+10.0;

//		PE14 - IL
//		PB15 - Vac
//      PE8 - Vdc

		Vac_INT=ADC4_buffer[0]; //PB15 - ADC4IN5
		Vdc_INT=ADC4_buffer[1]; //PE8 - ADC4IN
		IL_INT=ADC4_buffer[2];  //PE14 - ADC4IN

		Vac=0.101779541f*(float)Vac_INT*Vac_calibration;
		IL=-0.0084817f*(float)(3760-IL_INT/*3072*/)*IL_calibration; // from PFC
		Vdc=0.109296892*(float)Vdc_INT*Vdc_calibration;

        //uncomment to enable protection

//    	if((IL < IL_Trip) && (enable_OCD == 1))   // OC and OV protection
//		{
//			PFC_PWM_stop();
//			enable_closed_loop=0;
//			enable_voltage_closed_loop=0;
//			// DemandCurrentMAX=9500.0;
//			CurrProtection_ON = 1;
//			PWM_stop();
//		}

//    	if((enable_OCD == 1) && (Vdc_filtered > Vdc_Trip))   // OC and OV protection
//		{
//			PFC_PWM_stop();
//			enable_closed_loop=0;
//			enable_voltage_closed_loop=0;
//			// DemandCurrentMAX=9500.0;
//			VolProtection_ON = 1;
//			PWM_stop();
//		}

//		if((enable_PFC_burst_mode) && (Protection_ON == 0)) // how did we make enable_PFC_burst_mode 1 ???
//		{
//		     PFC_PWM_start();
//			 enable_closed_loop=1;
//			 enable_voltage_closed_loop=1;
//		//	uncomment this for burst mode
//	        if (Vdc_filtered > Vburst_H)
//			    {
//				    PFC_PWM_stop();
//					enable_closed_loop=0;
//					enable_voltage_closed_loop=0;
//					ResetFilterDF22(&DCVoltageCompensator);
//					ResetFilterDF22(&CurrentCompensator);
//			    }
//
//		    if (Vdc_filtered < Vburst_L)
//		        {
//				     PFC_PWM_start();
//					 enable_closed_loop=1;
//					 enable_voltage_closed_loop=1;
//			  }
//		}

		if (pfc_control_run)
		    {
			    pfc_control_run=0;
	    		Vac_filtered=1.1f*RunFilterDF22(&testDF22filter,Vac);
	    		Vac_RMS=0.1*Vac_filtered*Vac_filtered+0.9*Vac_RMS;  // Vac_RMS is actually Vac_RMS^2
	    		Vdc_filtered=RunFilterDF22(&NotchFilter,Vdc);

	    		if(enable_voltage_closed_loop) IL_ref=DemandCurrent*Vac*VacPK_inv;
	    		else
	    		{
	    			IL_ref=Vary_Vc*Vac*VacPK_inv;
	    		}

	    		ei=(IL_ref+IL)*0.5;
	    		pfc_DC_control_run++;

	    		if(enable_closed_loop)
	    		    {
	    			pwm_cmd=RunFilterDF22(&CurrentCompensator,ei);// run current control compensator
	    			pwm_cmd_debug=pwm_cmd;
	    			if(pwm_cmd>0.95f)
	    			    {
	    				pwm_cmd=0.95f;
	    				CurrentCompensator.x2=CurrentCompensator.b2*ei-CurrentCompensator.a2*pwm_cmd;  //anti-wimdup
	    				CurrentCompensator.x1=CurrentCompensator.b1*ei-CurrentCompensator.a1*pwm_cmd+CurrentCompensator.x2;
	    			    }
	    			if(pwm_cmd<0.05f)
	    			    {
	    				pwm_cmd=0.05f;
	    				CurrentCompensator.x2=CurrentCompensator.b2*ei-CurrentCompensator.a2*pwm_cmd;  //anti-wimdup
	    				CurrentCompensator.x1=CurrentCompensator.b1*ei-CurrentCompensator.a1*pwm_cmd+CurrentCompensator.x2;
	    			    }
	    			pwm_cmd_uint=(uint16_t)(pwm_cmd*2297.0f);// use constant instead of 2125.0f

	    			if((pwm_cmd_uint<2182) && (pwm_cmd_uint>112)) __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pwm_cmd_uint);
	    		}
	    		else
	    		{
	    			CurrentCompensator.x1=0.0f;
	    			CurrentCompensator.x2=0.0f;
//			    			pwm_cmd=0.05;
//			    			pwm_cmd_uint=112;
	    		}
	    		if(pfc_DC_control_run>=2)  // voltage loop
	    		{

	    			pfc_DC_control_run=0;
	    			ev=VdcRef-Vdc_filtered;

	    			if(enable_voltage_closed_loop)      // voltage loop
	    			{
	    				DemandCurrent=RunFilterDF22(&DCVoltageCompensator,ev);

						if(prot_var > 463)      //to delay the protection activation by 50ms
						{
						    enable_OCD = 1;
						    prot_var = 0;
						}
						prot_var ++ ;

	    				if(DemandCurrent>DemandCurrentMAX)
	    				{
	    					DemandCurrent=DemandCurrentMAX;
	    					DCVoltageCompensator.x2=DCVoltageCompensator.b2*ev-DCVoltageCompensator.a2*DemandCurrentMAX; //anti-windup
	    					DCVoltageCompensator.x1=DCVoltageCompensator.b1*ev-DCVoltageCompensator.a1*DemandCurrentMAX+DCVoltageCompensator.x2;
	    				}
	    				else if(DemandCurrent<100.0f)
	    				{
	    					DemandCurrent=100.0f;
	    					DCVoltageCompensator.x2=DCVoltageCompensator.b2*ev-DCVoltageCompensator.a2*100.0f;
	    					DCVoltageCompensator.x1=DCVoltageCompensator.b1*ev-DCVoltageCompensator.a1*100.0f+DCVoltageCompensator.x1;
	    				}
	    			}
	    			else
	    			{
	    			//	DCVoltageCompensator.x2=DCVoltageCompensator.b2*ev-DCVoltageCompensator.a2*Vc;
	    			//	DCVoltageCompensator.x1=DCVoltageCompensator.b1*ev-DCVoltageCompensator.a1*Vc+DCVoltageCompensator.x1;
	    			}

	    		}
	    }
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	if (htim==&htim8)
	{
		pwm_cnt++;

		if (Enable == 1)
		{
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
			#if (MODE == MODE_1 || MODE == MODE_3 ||  MODE == MODE_5)      // Double Space Vector Modulation

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((HALF_TBPRD_PWM*svgen1.Ta)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,((HALF_TBPRD_PWM*svgen1.Tb)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,((HALF_TBPRD_PWM*svgen1.Tc)+HALF_TBPRD_PWM));

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,((HALF_TBPRD_PWM*svgen2.Ta)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,((HALF_TBPRD_PWM*svgen2.Tb)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,((HALF_TBPRD_PWM*svgen2.Tc)+HALF_TBPRD_PWM));


		   #endif

			#if (MODE == MODE_2 || MODE == MODE_4 )      // VSD Modulation

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((HALF_TBPRD_PWM*ivsd.As1)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,((HALF_TBPRD_PWM*ivsd.Bs1)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,((HALF_TBPRD_PWM*ivsd.Cs1)+HALF_TBPRD_PWM));

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,((HALF_TBPRD_PWM*ivsd.As2)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,((HALF_TBPRD_PWM*ivsd.Bs2)+HALF_TBPRD_PWM));
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,((HALF_TBPRD_PWM*ivsd.Cs2)+HALF_TBPRD_PWM));
		   #endif
//			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

		}
		else

		{

			//PWM_stop();
			Reset_Controllers();
			rc1.SetpointValue = 0;


			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0.0*TBPRD_PWM);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0.0*TBPRD_PWM);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0.0*TBPRD_PWM);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0.0*TBPRD_PWM);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0.0*TBPRD_PWM);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0.0*TBPRD_PWM);

		}
		}
}

void Check_OCFault(void)
	  {

	     uint8_t i = 0, count_OCFault = 0;
		  for(i=0; i<6; i++)
		  {
			  Read_OC_Fault = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)     ;
		        if (Read_OC_Fault == 0)
		        {
		            count_OCFault++;
		        }
		        HAL_Delay(1); // optional, small delay between reads
		  }

		  if (count_OCFault >= 5)  // 5 out of 6
		      {
			      PWM_stop();
		          OCfault_Flag = 1;
		      }
		      else
		      {
		    	  OCfault_Flag = 0;
		    	  count_OCFault = 0;
		      }
	  }


void runVDCramp(void)
{

    if(VdcRef<(VdcTarget-dVref))
    {
    	VdcRef+=dVref*4;                                 // multiply by 4 due to reduction of loop frequency
    	/*if(DemandCurrentMAX<6000.0f)
    	{
    		DemandCurrentMAX+=0.07425f;
    	}*/
    }
    else if (VdcRef>(VdcTarget+dVref))
    {
    	VdcRef-=dVref;
    }
    else
    {
    	VdcRef=VdcTarget;
    	//VdcRampDone=1;
    }
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
}

void PFC_PWM_start(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void PFC_PWM_stop(void){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}


void UARTCommands()
{
	if (HAL_UART_Receive(&huart2, &received, sizeof(received), 10/*HAL_MAX_DELAY*/) == HAL_OK)
		  {
			  HAL_UART_Transmit(&huart2,&received,sizeof(received),10/*HAL_MAX_DELAY*/);

		  if(received=='e')                     // Enable PWM
		  {
			 PWM_start();
			 Enable = 1 ;
		  }
		  else if(received=='a')
		  {
			  Activate = 1.0 ;
			  Reset_Controllers();
		  }
		  else if(received=='s')  //stop PFC
		  {
			  PWM_stop();
			  user_freq = 0.0f;
			  mod_index = 0.0f;
			  PFC_PWM_stop();
			  enable_voltage_closed_loop = 0;
			  enable_closed_loop=0;
			  CurrProtection_ON = 0 ;
			  VolProtection_ON = 0 ;
			  DemandCurrentMAX = 1500;
			  Reset_Controllers();
			  Activate = 0.0 ;
		  }
//		  else if(received=='d')
//		  {
//			 dLogInt.status=1;
//			 trigger=2;
//			 dLogFloat.status=1;
//			 trigger_float=1;
//		  }
		  else if(received=='o')                     // Enable PFC PWM
		  {
			 PFC_PWM_start();
		  }
		  else if(received=='c')                     // Enable PFC PWM
		  {
			 enable_closed_loop=1;
		  }
		  else if(received == 'g')
		  {
			 enable_voltage_closed_loop = 1;
		  }
		  else if(received=='f')
		  {
			 freq_pu += 0.05; //pu
		  }
		  else if(received=='r')
		  {
			  freq_pu -= 0.05; //pu
		  }
		  else if(received=='m')
		  {
			  EM_Stop = 1;
		  }
		  else if(received=='i')
		  {
			  duty += 0.05; //pu
		  }
		  else if(received=='l')
		  {
			  duty -= 0.05; //pu
		  }
		  else if(received=='d')
		  {
			  IsRelayOn = 1; //pu
		  }
		  else if(received=='b')
		  {
			  IsRelayOn = 0; //pu
		  }
		  else if(received=='v')
		  {
			  Vary_Vc += 10.0; //pu
		  }
		  else if(received=='w')
		  {
			  Vary_Vc -= 10.0; //pu
		  }
		  else if(received=='t')
		  {
			  VdcTarget+=5.0 ;
		  }
		  else if(received=='u')
		  {
			  VdcTarget-=5.0 ;
		  }
		  else if(received=='x')
		  {
			  //DemandCurrentMAX += 100;
			  Kpx += 0.01;
		  }
		  else if(received=='y')
		  {
			  //DemandCurrentMAX -= 100;
			  Kix -= 0.01;
		  }
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
