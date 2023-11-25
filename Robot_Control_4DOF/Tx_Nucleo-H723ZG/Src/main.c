/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "BNO055.h"
#include "Flash.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Tx_Data[5];
uint8_t Control_Tx;
uint32_t Date_Memorie_Flash[11];
float Yaw_Rotire_Baza_Old, Yaw_Rotire_Baza_New,Yaw_Rotire_Baza_M;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == IRQ_Tx_Pin)
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		NRF24L01_Check_IRQ(Tx_Data);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BNO055_Data BNO055_Data;
	uint8_t Buffer[45];

	float Roll_Rotire_Gheara_New,Roll_Rotire_Gheara_Old;
	float Yaw_Rotire_Baza_Old, Yaw_Rotire_Baza_New;
	float Pitch_Balansare_Baza_New, Pitch_Balansare_Baza_Old;
	float Pitch_Balansare_Gheara_New,Pitch_Balansare_Gheara_Old;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  for(int i = 0; i < 5; i++)
  {
	  Tx_Data[i] = 0;
  }
  BNO055_Test(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS);
  HAL_Delay(500);
  BNO055_Initialise(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS);
  HAL_Delay(500);
  Flash_Read_Data(0x08020000,Date_Memorie_Flash,11); // Citire Memorie Flash!
  BNO055_Read_Calib_Data_From_Flash(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  /*
   * Aceasta Functie Stocheaza Datele primite de la memoria Flash in Struct!
   */
  BNO055_Transfer_Calib_Data(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  HAL_Delay(100);
  /*
   * Aceasta Functie Incarca Datele Calibrarii in senzor, dar fiindca Procesul de
   * Calibrare are loc in permanenta, si datele ar trebui incarcate periodic!
   */

  /*
   * Magnetometrul este foarte sensibil si de aceea va trebui calibrat la fiecare utilizare!!!
   */

  BNO055_Test(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS);
  HAL_Delay(500);
  BNO055_Initialise(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS);
  HAL_Delay(500);
  Flash_Read_Data(0x0802A000,Date_Memorie_Flash,11); // Citire Memorie Flash!
  BNO055_Read_Calib_Data_From_Flash(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);

  BNO055_Transfer_Calib_Data(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  HAL_Delay(100);

  BNO055_Read_Calib_Stat(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  HAL_Delay(5);

  int k;
  int Test_Logic_Buton_Miscare;
  int Test_Logic_FTS;
  Yaw_Rotire_Baza_Old = 0;
  Pitch_Balansare_Gheara_Old = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while((BNO055_Data.cal_state_sys != 3) | (BNO055_Data.cal_state_gyr != 3) | (BNO055_Data.cal_state_acc != 3) | (BNO055_Data.cal_state_mag != 3))
	  {
	  	  BNO055_Read_Calib_Stat(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
	  	  sprintf((char*)Buffer,"Sys:%d	Mag:%d	Gyr:%d	Acc:%d -> Rotire Baza\n\r", BNO055_Data.cal_state_sys,BNO055_Data.cal_state_mag,BNO055_Data.cal_state_gyr,BNO055_Data.cal_state_acc);
	  	  HAL_UART_Transmit(&Serial_Plot_UART,Buffer,strlen((char*)Buffer),HAL_MAX_DELAY);
	  	  BNO055_Set_Offset_Yaw(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
	  	  Roll_Rotire_Gheara_New = BNO055_Data.Roll;
	  	  Yaw_Rotire_Baza_New = BNO055_Data.Yaw;
	  	  Pitch_Balansare_Gheara_New = BNO055_Data.Pitch;
	  	  Pitch_Balansare_Baza_New = 0;
	  }
	  k = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(k == 1)
	  {
		  k = 0;
		  NRF24L01_Init_TxAA(&htim1);
		  NRF24L01_Transmit (Tx_Data);
	  }
	  Test_Logic_Buton_Miscare = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
	  Tx_Data[4] = Test_Logic_Buton_Miscare;
	  switch (Test_Logic_Buton_Miscare)
	  {
	  case 0:
		  Test_Logic_Buton_Miscare = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

		  Test_Logic_FTS = 0;
		  while(Test_Logic_FTS != 1)
		  {
			  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
			  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
			  if(((Roll_Rotire_Gheara_New + 15) <= BNO055_Data.Roll) | ((Roll_Rotire_Gheara_New - 15) >= BNO055_Data.Roll))
			  {
				  while(abs(Roll_Rotire_Gheara_New - BNO055_Data.Roll) >= 0.5)
				  {
					  Roll_Rotire_Gheara_New = 0.05 * BNO055_Data.Roll + 0.95 * Roll_Rotire_Gheara_Old;
					  Tx_Data[0] = (uint8_t)Roll_Rotire_Gheara_New;
					  Roll_Rotire_Gheara_Old = Roll_Rotire_Gheara_New;
					  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
					  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
				  }
			  }

			  BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
			  BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
			  if(((Pitch_Balansare_Baza_New + 15) <= BNO055_Data.Pitch) | ((Pitch_Balansare_Baza_New - 15) >= BNO055_Data.Pitch))
			  {
				  while(abs(Pitch_Balansare_Baza_New - BNO055_Data.Pitch) >= 0.5)
				  {
					  Pitch_Balansare_Baza_New = 0.05 * BNO055_Data.Pitch + 0.95 * Pitch_Balansare_Baza_Old;
					  Tx_Data[3] = (uint8_t)((Pitch_Balansare_Baza_New - 35)/(90-35)*180);
					  Pitch_Balansare_Baza_Old = Pitch_Balansare_Baza_New;
					  BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
					  BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
				  }
			  }
			  Test_Logic_FTS = 1;
		  }
		  break;

	  case 1:
		  Test_Logic_Buton_Miscare = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

		  Test_Logic_FTS = 0;
		  while(Test_Logic_FTS != 1)
		  {
			  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
			  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
			  if(((Yaw_Rotire_Baza_New + 20) <= BNO055_Data.Yaw) | ((Yaw_Rotire_Baza_New - 20) >= BNO055_Data.Yaw))
			  {
				  while(abs(Yaw_Rotire_Baza_New - BNO055_Data.Yaw) >= 0.5)
				  {
					  Yaw_Rotire_Baza_New = 0.05 * BNO055_Data.Yaw + 0.95 * Yaw_Rotire_Baza_Old;
					  Tx_Data[1] = (uint8_t)Yaw_Rotire_Baza_New;
					  Yaw_Rotire_Baza_Old = Yaw_Rotire_Baza_New;
					  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
					  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
				  }
			  }
			  if(((Pitch_Balansare_Gheara_New + 20) <= BNO055_Data.Pitch) | ((Pitch_Balansare_Gheara_New - 20) >= BNO055_Data.Pitch))
			  {
				  while(abs(Pitch_Balansare_Gheara_New - BNO055_Data.Pitch) >= 0.5)
				  {
				  Pitch_Balansare_Gheara_New = 0.05 * BNO055_Data.Pitch + 0.95 * Pitch_Balansare_Gheara_Old;
				  Tx_Data[2] = (uint8_t)((Pitch_Balansare_Gheara_New - 20)/(160-20)*180);
				  //Tx_Data[2] = (uint8_t)Pitch_Balansare_Gheara_New;
				  Pitch_Balansare_Gheara_Old = Pitch_Balansare_Gheara_New;
				  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
				  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
				  }
			  }
			  Test_Logic_FTS = 1;
		  }
		  break;
	  }
	/*  Test_Logic_FTS = 0;
  	  while(Test_Logic_FTS != 1)
  	  {
  		  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  		  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  		  while(((Roll_Rotire_Gheara + 10) <= BNO055_Data.Roll) | ((Roll_Rotire_Gheara - 10) >= BNO055_Data.Roll))
  		  {
  			  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  Roll_Rotire_Gheara = BNO055_Data.Roll;
  			  Tx_Data[0] = BNO055_Data.Roll;
  		  }
  		  while(((Yaw_Rotire_Baza + 15) <= BNO055_Data.Yaw) | ((Yaw_Rotire_Baza - 15) >= BNO055_Data.Yaw))
  		  {
  			  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  Yaw_Rotire_Baza = BNO055_Data.Yaw;
  		  	  Tx_Data[1] = BNO055_Data.Yaw;
  		  }
  		  while(((Pitch_Balansare_Gheara + 15) <= BNO055_Data.Pitch) | ((Pitch_Balansare_Gheara - 15) >= BNO055_Data.Pitch))
  		  {
  			  BNO055_Read_Quaternion_WXYZ(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  BNO055_Convert_Quat_to_Euler(&IMU_3_4_I2C,&Serial_Plot_UART,BNO055_MODULE_3_ADDRESS,&BNO055_Data);
  			  Pitch_Balansare_Gheara = BNO055_Data.Pitch;
  			  Tx_Data[2] = (uint8_t)((BNO055_Data.Pitch - 20)/(160-20)*180);
  		  }

  		  BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  		  BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  		  while(((Pitch_Balansare_Baza + 5) <= BNO055_Data.Pitch) | ((Pitch_Balansare_Baza - 5) >= BNO055_Data.Pitch))
  		  {
  			  BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  			  BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,&BNO055_Data);
  			  Pitch_Balansare_Baza = BNO055_Data.Pitch;
  			  Tx_Data[3] = (uint8_t)30*log(BNO055_Data.Pitch - 40);
  		  }
  		  Test_Logic_FTS = 1;
  	  }*/
	  //Tx_Data[2] = BNO055_Data.Pitch;


	  //Tx_Data[3] =	(uint8_t)(40)*log(BNO055_Data.Pitch - 40);
	  //Tx_Data[4] = BNO055_Data.Roll;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60404E72;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Tx_Pin */
  GPIO_InitStruct.Pin = IRQ_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_Tx_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin PD0 */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Movement_Pin */
  GPIO_InitStruct.Pin = Movement_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Movement_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
