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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	FDCAN_FilterTypeDef sFilterConfig;
	FDCAN_TxHeaderTypeDef TxHeader;
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData_Node3_To_Node1[8];
	uint8_t TxData_Node3_To_Node2[8];
	uint8_t RxData_From_Node2[8];
	uint8_t RxData_From_Node1[8];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int indx = 8;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
	  //?��기서?�� 먼�? RX FIFO0?�� ?��?�� ?��보�?? RxHeader�??????? 복사?���??????? ?��?��?���??????? RxData 배열�??????? 복사?��?��?��.
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData_From_Node1) != HAL_OK)
    {
    /* Reception Error */
    Error_Handler();
    }
//    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData_From_Node2) != HAL_OK)
//        {
//        /* Reception Error */
//        Error_Handler();
//        }
    //그런 ?��?�� ?�� 메시�????????�� ???�� ?��림을 ?��?�� ?��?��?��?��?��?��.
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
    //FDCAN2 콜백�??????? ?���??????? ?��기서?�� ?��?��?���??????? ?��?��?���??????? ?��?��?��?��. while 루프?��?�� FDCAN1?�� ?��?�� 매초 ?��?��?���??????? ?��?��?���??????? ?��문입?��?��.
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
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Configure Tx buffer message */
       // Ignore because FDCAN_NO_TX_EVENTS

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3,3,0,1);
  if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
      Error_Handler();
    }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  TxHeader.Identifier = 0x33;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x0;


      /* Configure standard ID reception filter to Rx buffer 0 */



        /* Start the FDCAN module */


        /* Send Tx buffer message */


        /* Polling for transmission complete on buffer index 0 */

        /* Polling for reception complete on buffer index 0 */
        /* Retrieve message from Rx buffer 0. Rec msg from Node 2 */

//	       int q;
//	       sprintf(q,"%d%d%d",a,b,c);
//	       HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  int a,b,c,d,e,f,g,h;
//
//	          a=RxData_From_Node2[0]-'0';
//	          b=RxData_From_Node2[1]-'0';
//	          c=RxData_From_Node2[2]-'0';
//	          d=RxData_From_Node2[3]-'0';
//	          e=RxData_From_Node2[4]-'0';
//	          f=RxData_From_Node2[5]-'0';
//	          g=RxData_From_Node2[6]-'0';
//	          h=RxData_From_Node2[7]-'0';
//
//
//		       int q=31323;
//
////	       printf("%d %d %d %d %d %d %d %d\r\n",a,b,c,d,e,f,g,h);
//	          HAL_Delay(1000);
//
//	  sprintf ((char *)TxData_Node3_To_Node1, "%d",q);//while 루프 ?��?��?�� FDCAN1?? 1초마?�� ?��?��?���??????? ?��?��?��?��?��.
//
//	  //TxData?�� indx �????????��?�� 증�??�� 값과 ?���??????? 문자?��(?��?��?���??????? FDCAN1?��?�� ?��?�� 것을 ?��???��)?�� ?��?��?��?��?��.
//  	   if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData_Node3_To_Node1)!= HAL_OK)//그런 ?��?�� ?��?�� HAL_FDCAN_AddMessageToTxFifoQ?�� 메시�???????�??????? FIFO ??기열?�� 추�??��?��?��.
//	  	   {
//	  	    Error_Handler();
//	  	   }
//  	 HAL_Delay (500);
// 	  sprintf ((char *)TxData_Node3_To_Node2, "%d",q);//while 루프 ?��?��?�� FDCAN1?? 1초마?�� ?��?��?���??????? ?��?��?��?��?��.
//
// 	  //TxData?�� indx �????????��?�� 증�??�� 값과 ?���??????? 문자?��(?��?��?���??????? FDCAN1?��?�� ?��?�� 것을 ?��???��)?�� ?��?��?��?��?��.
//   	   if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData_Node3_To_Node2)!= HAL_OK)//그런 ?��?�� ?��?�� HAL_FDCAN_AddMessageToTxFifoQ?�� 메시�???????�??????? FIFO ??기열?�� 추�??��?��?��.
// 	  	   {
// 	  	    Error_Handler();
// 	  	   }
//   	HAL_Delay (500);
	  	   //메시�???????�??????? FIFO ??기열?�� 추�??���??????? 메시�???????�??????? CAN 버스�??????? ?��?��?��?��?��.



	   if (RxData_From_Node2[0]=='3')
		   {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET);
	   	   HAL_Delay (1000);
	   	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
     	   HAL_Delay (1000);}
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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

