/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
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
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1001// corespond à 10 cm.
#define Seuil_Dist_3 490
#define Seuil_Dist_1 755
#define Seuil_Dist_2 979
#define DISTANCE_50cm 5000 // 5034 Trouvé avec la formule
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S 1000 //( pwm période = 2 ms )
#define T_50_MS 25
#define T_200_MS 100
#define T_2000_MS 1000
#define CKp_D 100  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50

#define ANGLE0 2410
#define ANGLE90 4250
#define ANGLEn90 850

#define T_5S_AVANCE_X_50 100
#define T_3S_TOURNER_AH_90 58
#define T_3S_TOURNER_H_90 56
#define T_5S_AVANCE_Z 100
#define T_5S_AVANCE_X 100

enum CMDE {
	START,
	STOP,
	AVANT,
	ARRIERE,
	DROITE,
	GAUCHE,
	PARK,
	ATTENTE_PARK
};
volatile enum CMDE CMDE;
enum MODE {
	SLEEP, ACTIF, MODE_PARK, MODE_ATTENTE_PARK
};
volatile enum MODE Mode;
volatile unsigned char New_CMDE = 0;
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4;
volatile unsigned int Time = 0;
volatile unsigned int Tech = 0;
uint16_t adc_buffer[10];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX;

uint16_t _DirG, _DirD, CVitG, CVitD, DirD, DirG;
uint16_t _CVitD = 0;
uint16_t _CVitG = 0;
uint16_t VitD, VitG;
int16_t DistD, DistG;
int16_t DistD_old = 0;
int16_t DistG_old = 0;
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
uint32_t Dist_Obst_;
uint32_t Dist_Obst_cm;
uint32_t Dist;
uint8_t UNE_FOIS = 1;
uint32_t OV = 0;

int cntr = 0;
int cntrBis = 0;
volatile int compteur_machine_etat = 0;
volatile int compteur_machine_etat_park = 0;
volatile int compteur_machine_etat_attente_park = 0;
volatile int compteur_avancer_X_50 = 0;
volatile int compteur_tourner_AH_90 = 0;
volatile int compteur_avancer_Z = 0;
volatile int compteur_tourner_H_90 = 0;
volatile int compteur_avancer_X = 0;
int compteur_waiting = 0;
uint32_t time_to_X0 = 0;
uint32_t time_to_Z0 = 0;
int reculer = 0;

volatile uint16_t PositionX;
volatile uint16_t PositionZ;
volatile uint16_t ZIGBEE_RX[3];
int adresse_recue = 0;
int reception_demande_adresse = 0;
int adresse_et_position_recue = 0;
volatile uint16_t robot_0[3];
volatile uint16_t robot_1[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);

void Move_Park(uint16_t position_to_go_x, uint16_t position_to_go_z);
void Park(void);
void Attente_Park(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Dist_Obst = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	  	HAL_SuspendTick(); // suppresion des Tick interrupt pour le mode sleep.

    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM motor
    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    	CMDE = STOP;
    	New_CMDE = 1;
    	HAL_TIM_Base_Start_IT(&htim2);  // Start IT sur font montant PWM
    	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    	HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);
    	HAL_UART_Receive_IT(&huart1, &ZIGBEE_RX, 6);
    	//HAL_ADC_Start_IT(&hadc1);
      	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10, GPIO_PIN_SET);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ANGLE0);
    	// Bloque la lecture IR
    	// HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Gestion_Commandes();
	  controle();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void Gestion_Commandes(void) {
	enum ETAT {
		VEILLE,
		ARRET,
		AV1,
		AV2,
		AV3,
		RV1,
		RV2,
		RV3,
		DV1,
		DV2,
		DV3,
		GV1,
		GV2,
		GV3
	};
	static enum ETAT Etat = VEILLE;

if (New_CMDE) {
		New_CMDE = 0;
	switch (CMDE) {
	case PARK: {
		_CVitG = 0;
		_CVitD = 0;
		Etat = ARRET;
		Mode = MODE_PARK;
		break;
			}
	case ATTENTE_PARK: {
		_CVitG = 0;
		_CVitD = 0;
		Etat = VEILLE;
		Mode = MODE_ATTENTE_PARK;
		break;
	}
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , réveil via IT BP1
			Etat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// réveil sytème grace à l'IT BP1
			Etat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case ARRIERE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case DROITE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case GAUCHE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			}
			break;

		}
	}
}
}
void controle(void) {

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}

}

void ACS(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF )
			Etat = ACTIF;
		else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIF: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

void Calcul_Vit(void) {

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF)
			Etat = ACTIF;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIF: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);

		}
		break;
	}
	}
}
/* Décide de park ou attente park */
void PARKASSIST(void)
{
	switch (Mode)
	{
		case MODE_PARK:
		{
			Park();
			break;
		}
		case MODE_ATTENTE_PARK:
		{
			Attente_Park();
			break;
		}
		default:
			break;
	}
}

void Park(void) {
	enum ETAT {
		REPOS,
		Ask_ADDRESS,
		Recep_ADDRESS,
		Pos_SONAR_X,
		Lancement_Sonar_X, // On n'utilise que les axes X et Z
		Mesure_Distance_X,
		Pos_SONAR_Z,
		Lancement_Sonar_Z,
		Mesure_Distance_Z,
		Send_ADDRESS,
	};
	static enum ETAT Etat = REPOS; //Le Robot commence à l'état de repos
	if (compteur_machine_etat_park >= T_50_MS) // On rentre dans la machine d'état toutes les 50 ms
	{
		compteur_machine_etat_park = 0;
		switch (Etat) {
			case REPOS: {
				if (Mode == MODE_PARK)
					Etat = Ask_ADDRESS;
				break;
			}
			case Ask_ADDRESS: { // Demande d'adresse en broadcast en ZigBee sur 0000 0000 0001 qui n'est jamais utilisé
				ZIGBEE_RX[0] = 0xFFFF;
				ZIGBEE_RX[1] = 0xFFFF;
				ZIGBEE_RX[2] = 0xFFFF;
				HAL_UART_Transmit(&huart1, ZIGBEE_RX, 6, HAL_MAX_DELAY);
				HAL_UART_Receive_IT(&huart1, &ZIGBEE_RX, 6);
				Etat = Recep_ADDRESS;
				break;
			}
			case Recep_ADDRESS: { // Reception d'une adresse d'un robot en attente de se garer
				if (adresse_recue == 1)
				{
					//adresse_recue = 0;
					Etat = Pos_SONAR_X;
				}
				break;
			}
			case Pos_SONAR_X: { // GESTION DE L'ORIENTATION DU SONAR
				if (cntr++ == 40) {
					cntr = 0;
					Etat = Lancement_Sonar_X;
				}
				else {
					// On place le Sonar à 0° pour aller mesurer X
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ANGLE0);
				}
				break;
			}
			case Lancement_Sonar_X: { // DECLANCHEMENT D'UNE MESURE
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10 , GPIO_PIN_SET);
				Etat = Mesure_Distance_X;
				break;
			}
			case Mesure_Distance_X: { // Mesure selon l'axe X
				robot_0[1] = Dist_Obst_;
				Etat = Pos_SONAR_Z;
				break;
			}
			case Pos_SONAR_Z: { // GESTION DE L'ORIENTATION DU SONAR
				if (cntr++ == 40) {
					cntr = 0;
					Etat = Lancement_Sonar_Z;
				}
				else {
					// On place le Sonar à 90° pour aller mesurer Z
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ANGLE90);
				}
				break;
			}
			case Lancement_Sonar_Z: { // DECLANCHEMENT D'UNE MESURE
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10, GPIO_PIN_SET);
				Etat = Mesure_Distance_Z;
				break;
			}
			case Mesure_Distance_Z: { // Mesure selon l'axe Z
				robot_0[2] = Dist_Obst_;
				Etat = Send_ADDRESS;
				break;
			}
			case Send_ADDRESS: { // Envoi au robot qui a envoyé son adresse, les positions du robot garé
				ZIGBEE_RX[0] = robot_1[0];
				ZIGBEE_RX[1] = robot_0[1];
				ZIGBEE_RX[2] = robot_0[2];
				HAL_UART_Transmit(&huart1, ZIGBEE_RX, 6, HAL_MAX_DELAY);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ANGLE0);
				Mode = SLEEP;
				break;
			}

		}
	}
}

void Attente_Park(void) {
	enum ETAT {
		REPOS,
		ATTENTE_Ask_ADDRESS,
		TemporisationBis,
		Send_ADDRESS,
		Get_ADDRESS_AND_POS,
		TemporisationBis2,
		DECISION,
		MOV_PARK
	};
	static enum ETAT Etat = REPOS;
	if (compteur_machine_etat_attente_park >= T_50_MS) // On rentre dans la machine d'état toutes les 50 ms
	{
		compteur_machine_etat_attente_park = 0;
		switch (Etat) {
			case REPOS: {
				if (Mode == MODE_ATTENTE_PARK)
					HAL_UART_Receive_IT(&huart1, &ZIGBEE_RX, 6);
					Etat = ATTENTE_Ask_ADDRESS;
				break;
			}
			case ATTENTE_Ask_ADDRESS: { // Attente de la demande d'adresse
				if (reception_demande_adresse == 1)
				{
					reception_demande_adresse = 0;
					Etat = TemporisationBis;
				}
				break;
			}


			case TemporisationBis: {
				if (cntr++ == 150) {
					cntr = 0;
					Etat = Send_ADDRESS;
				}
				break;
			}



			case Send_ADDRESS: { // Envoi de son adresse au robot garé
				ZIGBEE_RX[0] = robot_0[0];
				ZIGBEE_RX[1] = 0;
				ZIGBEE_RX[2] = 0;
				HAL_UART_Transmit(&huart1, ZIGBEE_RX, 6, HAL_MAX_DELAY);
				Etat = Get_ADDRESS_AND_POS;
				break;
			}

			case Get_ADDRESS_AND_POS: {
				HAL_UART_Receive_IT(&huart1, &ZIGBEE_RX, 6);
				if (adresse_et_position_recue == 1) // Reception de l'adresse et position du robot garé
				{
					PositionX = ZIGBEE_RX[1];
					PositionZ = ZIGBEE_RX[2];
					adresse_et_position_recue = 0;
					Etat = DECISION;
				}
				break;
			}

			/*
			case TemporisationBis2: {
				if (cntr++ == 50) {
					cntr = 0;
					Etat = DECISION;
				}
				break;
			}
			*/


			case DECISION: {
				if (robot_1[0] == robot_0[0])
				{
					Etat = MOV_PARK; // Déplacement du Robot à la position du robot garé (z+50cm)
				}
				else
				{
					Etat = ATTENTE_Ask_ADDRESS;
				}
				break;
			}
			case MOV_PARK: {
				Move_Park(PositionX, PositionZ);
				break;
			}
		}
	}
}

void Move_Park(uint16_t position_to_go_x, uint16_t position_to_go_z) {
	enum ETAT {
		REPOS,
		Temporisation0, // On ajoute des temporisations avant/après les rotations afin de diminuer les erreurs d'angle du à l'inertie
		Temporisation1,
		Temporisation2,
		Mesure_X,
		Lancement_Sonar_X,
		Lancement_Sonar_Z,
		Avancer_X_50cm,
		TemporisationX0,
		Tourner_Droite_90,
		Mouvement_Z,
		Mouvement_X,
		Tourner_Gauche_90,
		Temporisation3,
		Parked
	};
	static enum ETAT Etat = REPOS;
	if (compteur_machine_etat >= T_50_MS) // On rentre dans la machine d'état toutes les 50 ms
	{
		compteur_machine_etat = 0;
		switch (Etat) {
			case REPOS: {
				if (Mode == MODE_ATTENTE_PARK)
					Etat = Temporisation0;
				break;
			}
			case Temporisation0: {
				if (cntr++ == 30) {
					cntr = 0;
					Etat = Avancer_X_50cm;
				}
				break;
			}

			case Avancer_X_50cm: {
				if (compteur_avancer_X_50++ >= T_5S_AVANCE_X_50)
				{
					//compteur_avancer_X_50 = 0;
					_CVitG = 0;
					_CVitD = 0;
					Etat = TemporisationX0;
				}
				else
				{
					_DirG = AVANCE;
					_DirD = AVANCE;
					_CVitG = V1;
					_CVitD = V1;
					Etat = Avancer_X_50cm;
				}
				break;
			}

			case TemporisationX0: {
				if (cntr++ == 20) {
					cntr = 0;
					Etat = Tourner_Gauche_90;
				}
				break;
			}

			case Tourner_Gauche_90: {
				if (compteur_tourner_AH_90++ >= T_3S_TOURNER_AH_90)
				{
					compteur_tourner_AH_90 = 0;
					_CVitG = 0;
					_CVitD = 0;
					Etat = Temporisation1;
				}
				else
				{
					_DirG = RECULE;
					_DirD = AVANCE;
					_CVitG = V1;
					_CVitD = V1;
					Etat = Tourner_Gauche_90;
				}
				break;
			}
			case Temporisation1: {
				if (cntr++ == 14) {
					cntr = 0;
					Etat = Mouvement_Z;
					}
				break;
			}

			case Mouvement_Z: {
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10 , GPIO_PIN_SET);
				if (Dist_Obst_ <= position_to_go_z - 3000)
				{
					_CVitG = 0;
					_CVitD = 0;
					Etat = Tourner_Droite_90;
				}
				else
				{
					_DirG = AVANCE;
					_DirD = AVANCE;
					_CVitG = V1;
					_CVitD = V1;
					Etat = Mouvement_Z;
				}
				break;
			}
			case Tourner_Droite_90: {
				if (compteur_tourner_H_90++ >= T_3S_TOURNER_H_90)
				{
					//compteur_tourner_H_90 = 0;
					_CVitG = 0;
					_CVitD = 0;
					Etat = Temporisation2;
				}
				else
				{
					_DirG = AVANCE;
					_DirD = RECULE;
					_CVitG = V1;
					_CVitD = V1;
					Etat = Tourner_Droite_90;
				}
				break;
			}
			case Temporisation2: {
				if (cntr++ == 15)
				{
					cntr = 0;
					Etat = Mouvement_X;
				}
				break;
			}
			case Mouvement_X: {
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10, GPIO_PIN_SET);
				if (Dist_Obst_ <= position_to_go_x)
				{
					_CVitG = 0;
					_CVitD = 0;
					Etat = Temporisation3;
				}
				else
				{
					_DirG = AVANCE;
					_DirD = AVANCE;
					_CVitG = V1;
					_CVitD = V1;
					Etat = Mouvement_X;
				}
				break;
			}
			case Temporisation3: {
				if (cntr++ == 15)
				{
					cntr = 0;
					Etat = Parked;
				}
			break;
			}
			case Parked: {
				ZIGBEE_RX[0] = 0xFFFF;
				ZIGBEE_RX[1] = 0xFFFF;
				ZIGBEE_RX[2] = 0xFFFF;
				HAL_UART_Transmit(&huart1, ZIGBEE_RX, 6, HAL_MAX_DELAY);
				Mode = MODE_PARK;
			break;
			}
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F': {
			CMDE = AVANT;
			//New_CMDE = 1;
			break;
		}

		case 'B': {
			CMDE = ARRIERE;
			//New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			//New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			//New_CMDE = 1;
			break;
		}

		/* On ajoute les 2 touches de l'application nécessaire au PARKASSIST */
		case 'W': {
			CMDE = PARK;
			New_CMDE = 1;
			break;
		}

		case 'X': {
			CMDE = ATTENTE_PARK;
			New_CMDE = 1;
			break;
		}

		case 'D':{
			// disconnect bluetooth
			break;
		}

		default:
			New_CMDE = 1;
		}

		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

	}
	/* Communication Zigbee */
		if (huart->Instance == USART1) { //ZigBee
			if ((ZIGBEE_RX[0] != 0) && (ZIGBEE_RX[1] == 0) && (ZIGBEE_RX[2] == 0) && (adresse_recue == 0 )) // Reception d'une adresse
			{
				robot_1[0] = ZIGBEE_RX[0];
				adresse_recue = 1;
			}
			else if ((ZIGBEE_RX[0] == 0xFFFF) && (ZIGBEE_RX[1] == 0xFFFF) && (ZIGBEE_RX[2] == 0xFFFF)) // Reception d'une demande d'adresse
			{
				reception_demande_adresse = 1;
			}
			else if ((ZIGBEE_RX[0] == robot_0[0]) && ((ZIGBEE_RX[1] != 0) || (ZIGBEE_RX[2] != 0))) // Reception d'une commande pour se garer (adresse+position)
			{
				robot_1[0] = ZIGBEE_RX[0];
				robot_1[1] = ZIGBEE_RX[1];
				robot_1[2] = ZIGBEE_RX[2];
				adresse_et_position_recue = 1;
				HAL_UART_Transmit_IT(&huart1, &ZIGBEE_RX, 6);
			}
		}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//modification des indices après avoir ajouté la surveillance batterie
	Dist_ACS_3 = adc_buffer[0] - adc_buffer[5];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[8];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[6];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[7];
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if ( htim->Instance == TIM2) {
		cpt++;
		Time++;
		Tech++;
		compteur_machine_etat++;

		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE)
		CMDE = STOP;
	else
		CMDE = START;
	TOGGLE = ~TOGGLE;
	New_CMDE = 1;
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	Dist_Obst_ = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 , GPIO_PIN_RESET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
