
#include "main.h"


#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>

#include"dcm_rdbi.h"
#include"dcm_seca.h"
#include"dcm_wdbi.h"
#include"dcm.h"
#include"stm32f4xx_hal_conf.h"
#include"stm32f4xx_it.h"


#define DID 0x0123
#define READ_SID 0x22
#define WRITE_SID 0x2E
#define SESSION_SID 0x27
#define NRC 0x7F

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
//TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

//TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3;
uint8_t uart3_receive;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN1_pHeader;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeader;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;
uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;

uint16_t NumBytesReq = 0;
uint8_t  REQ_BUFFER  [4096];
uint8_t  REQ_1BYTE_DATA;

uint8_t CAN1_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00};
uint8_t CAN1_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00};
uint8_t CAN2_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00};
uint8_t CAN2_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00};

uint8_t KEY[4] = { 0x00, 0x00, 0x00, 0x00 };
uint8_t Seed[4] ={ 0x00, 0x00, 0x00, 0x00 };

uint16_t Num_Consecutive_Tester;
uint8_t  Flg_Consecutive = 0;

uint8_t MessageCounter = 0;
uint8_t flag = 0;

uint32_t newStdID;
volatile uint8_t SecurityUnlocked = 0;
uint8_t SeedProvided = 0;
volatile uint8_t check;
volatile uint8_t CAN2_flag=0;
volatile uint8_t CAN1_flag=0;


unsigned int TimeStamp;
// maximum characters send out via UART is 30
char bufsend[30]="XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup(uint32_t CANID);//uint32_t CANID
void MX_CAN2_Setup(uint32_t CANID);//uint32_t CANID
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);
void SID_22_Practice(uint8_t *TX_BUFFER, uint8_t *RX_BUFFER);
void SID_2E_Practice(uint8_t *TX_BUFFER, uint8_t *RX_BUFFER);
void SID_27_Practice(uint8_t *TX_BUFFER, uint8_t *RX_BUFFER);
void delay(uint16_t delay);

//void dcm_rdbi(Dcm_Msg_Info* MsgInfor);
//void dcm_seca(Dcm_Msg_Info* MsgInfor);
//void dcm_wdbi(Dcm_Msg_Info* MsgInfor);
//void CAN_TP2DCM(uint32_t CANID, uint8_t TransBuffer[]);
//void CAN_DCM2TP();
//void CAN2_RX0_IRQHandler(void);
//void Dcm_Seca_Gen_Keys();

//
//void CAN2CommSetup();
//void CAN1_Send();
//void CAN2_Send();
//void Read_DATA_BUFFER(uint8_t * REQ_BUFFER, uint8_t length, uint8_t * DATA_TX);
//uint16_t compare_Key(uint8_t * key1, uint8_t * key2, uint8_t length);
//void Calculate_Key(uint8_t * Seed, uint8_t * Key);
/* USER CODE END PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN1_TX();
void CAN2_TX();

//void CAN2CommSetup(uint32_t CANID);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


int main(void)
{


	uint16_t i,j = 0;
	uint16_t Consecutive_Cntr = 0;

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  MX_CAN1_Setup(ProtocolI_RX);
  MX_CAN2_Setup(ProtocolI_TX);

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);


  while (1)
  {

	  CAN2_TX();
	  delay(2000);
	  CAN1_TX();
	  delay(2000);
////    if(!BtnU) /*IG OFF->ON stimulation*/
////    {
////      delay(20);
////      USART3_SendString((uint8_t *)"IG OFF \n");
////      while(!BtnU){
////      CAN2_TX();
////      delay(2000);
////      CAN1_TX();
////      delay(2000);
////      }
////      while(!BtnU);
//////      MX_CAN1_Setup(ProtocolI_RX);
//////      MX_CAN2_Setup(ProtocolI_TX);
////      USART3_SendString((uint8_t *)"-> IG ON \n");
////      delay(20);
////    }



//	    if (NumBytesReq != 0) {
//			delay(100);
//			Read_DATA_BUFFER(REQ_BUFFER, NumBytesReq, CAN1_DATA_TX);
//			CAN1_Send();
//			delay(100);
//			NumBytesReq = 0;
//		}
//
//		if (CAN2_flag) {
//				switch (REQ_BUFFER[0]) {
//				case 0x27:
//					SID_27_Practice(CAN2_DATA_TX, CAN2_DATA_RX);
//					//dcm_seca(CAN2_DATA_RX);
//					break;
//				case 0x22:
//					SID_22_Practice(CAN2_DATA_TX, CAN2_DATA_RX);
//					//dcm_rdbi(CAN2_DATA_RX);
//					break;
//				case 0x2E:
//					//dcm_wdbi(CAN2_DATA_RX);
//					SID_2E_Practice(CAN2_DATA_TX, CAN2_DATA_RX);
//					break;
//				default:
//					USART3_SendString((uint8_t*) "Service not support\n");
//					break;
//				}
//				CAN2_flag = 0;
//			}
//		if (CAN1_flag) {
//			USART3_SendString((uint8_t*) "Response: ");
//			PrintCANLog(CAN1_pHeaderRx.StdId, CAN1_DATA_RX);
//			CAN1_flag = 0;
//		}
//		if (!BtnU) /*IG OFF->ON stimulation*/
//		{
//			delay(20);
//			USART3_SendString((uint8_t*) "IG OFF ");
//			while (!BtnU);
//			CAN1_pHeader.StdId = newStdID;
//			CAN2CommSetup();
//			MX_CAN1_Setup(ProtocolI_RX);
//			MX_CAN2_Setup(ProtocolI_TX);
//			USART3_SendString((uint8_t*) "-> IG ON \n");
//			delay(20);
//		}
//	}

//    if(REQ_BUFFER[0] != 0)
//    {
//      delay(50);
//      USART3_SendString((uint8_t*)" \n");
//      if(NumBytesReq <= 7)
//      {
//        CAN1_DATA_TX[0] = NumBytesReq;
//        for(i=1; i < 8;i++)
//        {
//          CAN1_DATA_TX[i] = REQ_BUFFER[i-1];
//        }
//
//      PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
//      HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
//      }
//      else
//      {
//		  j = 0;
//		  Flg_Consecutive = 0;
//		  Num_Consecutive_Tester = 0;
//		  Consecutive_Cntr = 0;
//
//		  Num_Consecutive_Tester = NumBytesReq - 6;
//		  if(Num_Consecutive_Tester % 7 == 0)
//		  {
//			Num_Consecutive_Tester /=7 ;
//		  }
//		  else
//		  {
//			Num_Consecutive_Tester /=7;
//			Num_Consecutive_Tester +=1;
//		  }
//
//		  CAN1_DATA_TX[0] = ((NumBytesReq >> 8) + 0x10) &0x1F;
//		  CAN1_DATA_TX[1] = NumBytesReq;
//		  CAN1_DATA_TX[2] = REQ_BUFFER[0];
//		  CAN1_DATA_TX[3] = REQ_BUFFER[1];
//		  CAN1_DATA_TX[4] = REQ_BUFFER[2];
//		  CAN1_DATA_TX[5] = REQ_BUFFER[3];
//		  CAN1_DATA_TX[6] = REQ_BUFFER[4];
//		  CAN1_DATA_TX[7] = REQ_BUFFER[5];
//
//		  PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
//		  HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
//
//		  j = 6;
//
//		  while(Num_Consecutive_Tester > 0)
//		  {
//		  if(Flg_Consecutive == 0x01)
//		  {
//          Flg_Consecutive = 0;
//          CAN1_DATA_TX[0] = (Consecutive_Cntr & 0x0F) + 0x20;
//          for(i=0;i<7;i++)
//          {
//            CAN1_DATA_TX[i-1] = REQ_BUFFER[i+j];
//          }
//
//
//          j +=7;
//          PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
//          HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
//          Num_Consecutive_Tester --;
//          Consecutive_Cntr ++;
//		  }
//	   }
		  //CAN_TP2DCM(0x012,CAN2_DATA_TX);
		  //CAN2_DCM2TP();
		  //Gen_Random_Seed();

//      /*debug*/
//      delay(500);
//      char c[2]="";
//      USART3_SendString((uint8_t*)"Data_Of_Dummy_Ram: ");
//      for(i=0;i<100;i++)
//      {
//        dummy_eeprom_buffer[i] = Dcm_Msg_Info_s.dataBuff[i];
//        if(i==0) dummy_eeprom_buffer[i] = Dcm_Msg_Info_s.Sid;
//        sprintf(c,"%02X",dummy_eeprom_buffer[i]);
//
//        sprintf(c, "%02X",Dcm_Msg_Info_s.dataBuff[i]);
//
//        USART3_SendString((uint8_t*)c);
//        USART3_SendString((uint8_t*)" ");
//      }
//      USART3_SendString((uint8_t*)" \n");
//    }

//   }

  memset(&REQ_BUFFER,0x00,4096);
  NumBytesReq = 0;

}
}
//}
//}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;//168
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 7999;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 9999;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//}

static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }


  CAN1_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
  CAN1_sFilterConfig.SlaveStartFilterBank = 13;
  CAN1_sFilterConfig.FilterBank = 8;
  CAN1_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN1_sFilterConfig.FilterIdHigh = 0x0123 << 5;//CANID
  CAN1_sFilterConfig.FilterIdLow = 0x0000;
  CAN1_sFilterConfig.FilterMaskIdHigh = 0x0000; //(0x7FF << 5) ; //0xFFE0
  CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
  CAN1_sFilterConfig.FilterActivation = ENABLE;

  /* USER CODE END CAN1_Init 2 */

}

static void MX_CAN2_Init(void)
{


  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 1;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN2_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
  CAN2_sFilterConfig.SlaveStartFilterBank=13;
  CAN2_sFilterConfig.FilterBank=19;
  CAN2_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN2_sFilterConfig.FilterIdHigh = 0x0123 << 5;
  CAN2_sFilterConfig.FilterIdLow = 0x0000;
  CAN2_sFilterConfig.FilterMaskIdHigh = 0x0000; //0xFFE0
  CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
  CAN2_sFilterConfig.FilterActivation = ENABLE;
  /* USER CODE END CAN2_Init 2 */

}

static void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

void MX_CAN1_Setup(uint32_t CANID)
{
	CAN1_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
	CAN1_sFilterConfig.SlaveStartFilterBank = 14;
	CAN1_sFilterConfig.FilterBank = 8;
	CAN1_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN1_sFilterConfig.FilterIdHigh = CANID << 5;//CANID
	CAN1_sFilterConfig.FilterIdLow = 0x0000;
	CAN1_sFilterConfig.FilterMaskIdHigh = 0x0000; //(0x7FF << 5) ; //0xFFE0
	CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
	CAN1_sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	CAN1_pHeader.DLC = 8;
	CAN1_pHeader.IDE = CAN_ID_STD;
	CAN1_pHeader.RTR = CAN_RTR_DATA;
	CAN1_pHeader.StdId = CANID;

}

void MX_CAN2_Setup(uint32_t CANID)
{
    CAN2_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
    CAN2_sFilterConfig.SlaveStartFilterBank=14;
    CAN2_sFilterConfig.FilterBank=19;
    CAN2_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
    CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN2_sFilterConfig.FilterIdHigh = CANID <<5;
    CAN2_sFilterConfig.FilterIdLow = 0x0000;
    CAN2_sFilterConfig.FilterMaskIdHigh = 0x0000; //0xFFE0
    CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
    CAN2_sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	CAN2_pHeader.DLC = 8;
	CAN2_pHeader.IDE = CAN_ID_STD;
	CAN2_pHeader.RTR = CAN_RTR_DATA;
	CAN2_pHeader.StdId = CANID;
}

void USART3_SendString(uint8_t *ch)
{
   while(*ch!=0)
   {
      HAL_UART_Transmit(&huart3, ch, 1,HAL_MAX_DELAY);
      ch++;
   }
}

void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame)
{
	uint16_t loopIndx = 0;
	char bufID[3] = "   ";
	char bufDat[2] = "  ";
	char bufTime [8]="        ";

	sprintf(bufTime,"%d",TimeStamp);
	USART3_SendString((uint8_t*)bufTime);
	USART3_SendString((uint8_t*)" ");

	sprintf(bufID,"%03X",CANID);
	for(loopIndx = 0; loopIndx < 3; loopIndx ++)
	{
		bufsend[loopIndx]  = bufID[loopIndx];
	}
	bufsend[3] = ':';
	bufsend[4] = ' ';


	for(loopIndx = 0; loopIndx < 8; loopIndx ++ )
	{
		sprintf(bufDat,"%02X",CAN_Frame[loopIndx]);
		bufsend[loopIndx*3 + 5] = bufDat[0];
		bufsend[loopIndx*3 + 6] = bufDat[1];
		bufsend[loopIndx*3 + 7] = ' ';
	}
	bufsend[29] = '\n';
	USART3_SendString((unsigned char*)bufsend);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
	NumBytesReq++;
	//REQ_BUFFER[7] = NumBytesReq;
}

void delay(uint16_t delay)

{
	HAL_Delay(delay);
}





//void SID_22_Practice(uint8_t * TX_BUFFER, uint8_t *RX_BUFFER) {
//	delay(100);
//	if (RX_BUFFER[0] != 3) //Invalid length
//			{
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = NRC;
//		TX_BUFFER[2] = READ_SID;
//		TX_BUFFER[3] = NRC13_INVALID_LEN;
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	} else if (((RX_BUFFER[2] << 8) | RX_BUFFER[3]) != DID) //INVALID DID
//	{
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = NRC;
//		TX_BUFFER[2] = READ_SID;
//		TX_BUFFER[3] = NRC31_DID_NOTSUPPORT ;
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	} else //CAN2 Receive Correct message
//	{
//		TX_BUFFER[0] = 0x05;
//		TX_BUFFER[1] = RX_BUFFER[1] + 0x40;
//		TX_BUFFER[2] = RX_BUFFER[2];
//		TX_BUFFER[3] = RX_BUFFER[3];
//		TX_BUFFER[4] = (CAN1_pHeader.StdId) >> 8;
//		TX_BUFFER[5] = CAN1_pHeader.StdId & 0xFF;
//		TX_BUFFER[6] = 0x55;
//		TX_BUFFER[7] = 0x55;
//	}
//	if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, TX_BUFFER, &CAN2_pTxMailbox) != HAL_OK) {
//		Error_Handler();
//	}
//	delay(100);
//}
//void SID_27_Practice(uint8_t * TX_BUFFER, uint8_t *RX_BUFFER) {
//	delay(100);
//	switch (RX_BUFFER[2]) {
//	case 0x01:
//		if (RX_BUFFER[0] != 2) {
//			TX_BUFFER[0] = 0x03;
//			TX_BUFFER[1] = NRC;
//			TX_BUFFER[2] = SESSION_SID;
//			TX_BUFFER[3] = NRC13_INVALID_LEN ;
//			for (int i = 4; i < 8; i++) {
//				TX_BUFFER[i] = 0x55;
//			}
//		} else {
//			TX_BUFFER[0] = 0x06;
//			TX_BUFFER[1] = RX_BUFFER[1] + 0x40;
//			TX_BUFFER[2] = 0x01;
//			for (int i = 3; i < 7; i++) {
//			        Seed[i - 3] = rand(); // Store the random value in the seed array
//			        TX_BUFFER[i] = Seed[i - 3]; // Assign the random value to TX_BUFFER
//			    }
//			TX_BUFFER[7] = 0x55;
//			Calculate_Key(Seed, KEY);
//
//			SeedProvided = 1;
//		}
//		break;
//	case 0x02:
//		if (RX_BUFFER[0] != 6) {
//			TX_BUFFER[0] = 0x03;
//			TX_BUFFER[1] = NRC;
//			TX_BUFFER[2] = SESSION_SID;
//			TX_BUFFER[3] = NRC13_INVALID_LEN ;
//			for (int i = 4; i < 8; i++) {
//				TX_BUFFER[i] = 0x55;
//			}
//		} else if (!compare_Key(KEY, &RX_BUFFER[3], 4)) {
//			TX_BUFFER[0] = 0x03;
//			TX_BUFFER[1] = NRC;
//			TX_BUFFER[2] = SESSION_SID;
//			TX_BUFFER[3] = NRC35_INVALID_KEY;
//			for (int i = 4; i < 8; i++) {
//				TX_BUFFER[i] = 0x55;
//			}
//		} else if (SeedProvided && !SecurityUnlocked) {
//			HAL_TIM_Base_Start_IT(&htim2);
//			SecurityUnlocked = 1;
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//			USART3_SendString((uint8_t*) "Session Unlocked\n");
//
//			TX_BUFFER[0] = 0x02;
//			TX_BUFFER[1] =RX_BUFFER[1] + 0x40;
//			TX_BUFFER[2] = 0x02;
//			for (int i = 3; i < 8; i++) {
//				TX_BUFFER[i] = 0x55;
//			}
//			SeedProvided = 0;
//		} else {
//			return;
//		}
//		break;
//	default:
//		if (RX_BUFFER[0]) { // Case "01 27"
//			TX_BUFFER[0] = 0x03;
//			TX_BUFFER[1] = NRC;
//			TX_BUFFER[2] = SESSION_SID;
//			TX_BUFFER[3] = NRC13_INVALID_LEN ;
//			for (int i = 4; i < 8; i++) {
//				TX_BUFFER[i] = 0x55;
//			}
//		}
//		else
//		return;
//		break;
//	}
//	if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, TX_BUFFER, &CAN2_pTxMailbox) != HAL_OK) {
//		Error_Handler();
//	}
//	delay(100);
//
//}
////void SID_2E_Practice(uint8_t * TX_BUFFER, uint8_t * RX_BUFFER) {
//	delay(100);
//	if (!SecurityUnlocked) {
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = NRC;
//		TX_BUFFER[2] = WRITE_SID;
//		TX_BUFFER[3] = NRC33_SECURITY_ACCESS_DENIED;
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	} else if (RX_BUFFER[0] < 5) {
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = NRC;
//		TX_BUFFER[2] = WRITE_SID;
//		TX_BUFFER[3] = NRC13_INVALID_LEN;
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	} else if (((RX_BUFFER[2] << 8) | RX_BUFFER[3]) != DID) //INVALID DID
//	{
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = NRC;
//		TX_BUFFER[2] = WRITE_SID;
//		TX_BUFFER[3] = NRC31_DID_NOTSUPPORT ;
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	} else {
//		newStdID = ((((uint32_t) RX_BUFFER[4]) << 8) | (RX_BUFFER[5])) & 0x7FF;
//		TX_BUFFER[0] = 0x03;
//		TX_BUFFER[1] = RX_BUFFER[1] + 0x40;
//		TX_BUFFER[2] = RX_BUFFER[2];
//		TX_BUFFER[3] = RX_BUFFER[3];
//		for (int i = 4; i < 8; i++) {
//			TX_BUFFER[i] = 0x55;
//		}
//	}
//	if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, TX_BUFFER, &CAN2_pTxMailbox) != HAL_OK) {
//		Error_Handler();
//	}
//	delay(100);
//
//}
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  if (hcan == &hcan1)
//  {
//    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    CAN1_flag=1;
//    return;
//  }
//  else if (hcan == &hcan2)
//  {
//    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    CAN2_flag=1;
//    return;
//    }
//}


//void CAN1_Send(){
//	if(HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox) != HAL_OK){
//		Error_Handler();
//	}
//	char buffer1[50] =  "Request: ";
//	USART3_SendString((uint8_t*)buffer1 );
//	PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
//}
//
//void CAN2_Send(){
//	if(HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK){
//		Error_Handler();
//	}
//	PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
//}
//void CAN1CommSetup() {
//	 	CAN1_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
//	    CAN1_sFilterConfig.SlaveStartFilterBank=14;
//	    CAN1_sFilterConfig.FilterBank=19;
//	    CAN1_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
//	    CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	    CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	    CAN1_sFilterConfig.FilterIdHigh = 0x0123 <<5;
//	    CAN1_sFilterConfig.FilterIdLow = 0x0000;
//	    CAN1_sFilterConfig.FilterMaskIdHigh = 0x0123 <<5; //0xFFE0
//	    CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
//	    CAN1_sFilterConfig.FilterActivation = ENABLE;
//
//		HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
//		HAL_CAN_Start(&hcan1);
//		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//
//		CAN1_pHeader.DLC = 8;
//		CAN1_pHeader.IDE = CAN_ID_STD;
//		CAN1_pHeader.RTR = CAN_RTR_DATA;
//		CAN1_pHeader.StdId = 0x7FF;
//}
//void CAN2CommSetup() {
//	 	CAN2_sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
//	    CAN2_sFilterConfig.SlaveStartFilterBank=14;
//	    CAN2_sFilterConfig.FilterBank=19;
//	    CAN2_sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
//	    CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	    CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	    CAN2_sFilterConfig.FilterIdHigh = CAN1_pHeader.StdId <<5;
//	    CAN2_sFilterConfig.FilterIdLow = 0x0000;
//	    CAN2_sFilterConfig.FilterMaskIdHigh = CAN1_pHeader.StdId; //0xFFE0
//	    CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
//	    CAN2_sFilterConfig.FilterActivation = ENABLE;
//
//		HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
//		HAL_CAN_Start(&hcan2);
//		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//
//		CAN2_pHeader.DLC = 8;
//		CAN2_pHeader.IDE = CAN_ID_STD;
//		CAN2_pHeader.RTR = CAN_RTR_DATA;
//		CAN2_pHeader.StdId = 0x0123;
//}
//void Read_DATA_BUFFER(uint8_t *REQ_BUFFER, uint8_t length, uint8_t *DATA_TX) {
//	DATA_TX[0] = length;
//	uint8_t i;
//	for (i = 0; i < length; i++) {
//		DATA_TX[i + 1] = REQ_BUFFER[i];
//	}
//	while (i <= 7) {
//		i++;
//		DATA_TX[i] = 0x55;
//	}
//}
//uint16_t compare_Key(uint8_t* key1, uint8_t* key2, uint8_t length) {
//	for (uint8_t i = 0; i < length; i++) {
//		if (key1[i] != key2[i]) {
//			return 0;
//		}
//	}
//	return 1;
//}
//void Calculate_Key(uint8_t *Seed, uint8_t *Key) {
//	Key[0] = Seed[0] ^ Seed[1];
//	Key[1] = Seed[1] + Seed[2];
//	Key[2] = Seed[2] ^ Seed[3];
//	Key[3] = Seed[3] + Seed[0];
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim2) {
//		if (SecurityUnlocked == 0){
//
//			return;
//		}
//		else {
//			HAL_TIM_Base_Stop_IT(htim);
//			SecurityUnlocked = 0;
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//			USART3_SendString((uint8_t*) "Session Locked\n");
//		}
//	}
//}

/*ham nay bai1 khong xoa */uint8_t calc_SAE_J1850(uint8_t data[], uint8_t crc_len)

{
    uint8_t idx, crc, temp1, temp2, idy;
    crc = 0;
    idx = 0;
    idy = 0;
    temp1 = 0;
    temp2 = 0;
    for(idx=0;idx < crc_len+1;idx++)
    {
        if(idx == 0)
        {
            temp1 = 0;
        }
        else
        {
            temp1 = data[crc_len-idx];
        }
        crc = (crc^temp1);
        for(idy=(uint8_t)8; idy>0; idy--)
        {
            // Save the value before the top bit is shifted out.
            temp2 = crc;
            crc <<= 1;
            if (0 != (temp2 & (uint8_t)128))
                crc ^= 0x1D;
        }
    }
    return crc;
}

/*ham nay bai1 khong xoa */ void CAN1_TX(){
	CAN1_pHeader.StdId = 0x12;
	CAN1_pHeader.DLC = 8;
	CAN1_pHeader.IDE = CAN_ID_STD;
	CAN1_pHeader.RTR = CAN_RTR_DATA;
	if(CAN1_DATA_RX[7] == calc_SAE_J1850(CAN1_DATA_RX,7)){
		CAN1_DATA_TX[0] = 0x03;//0x01;
		CAN1_DATA_TX[1] = 0x04;//0x02;
		//CAN1_DATA_TX[2] = CAN1_DATA_TX[0]+CAN1_DATA_TX[1];
        CAN1_DATA_TX[6] = (MessageCounter-0x01) &0xF;
        //CAN1_DATA_TX[7] = calc_SAE_J1850(CAN1_DATA_TX,7);

        if(flag == 0){
                    CAN1_DATA_TX[7] = calc_SAE_J1850(CAN1_DATA_TX, 7); // Gửi CRC đúng
                } else {
                    CAN1_DATA_TX[7] = calc_SAE_J1850(CAN1_DATA_TX, 7) & 0x0 ; // Gửi CRC sai
                }
        }
     else{
		char buffer1[50] = "CAN1 ERROR \n";
		USART3_SendString((unsigned char *)buffer1);
	}


	char buffer2[50] = "CAN1 TX \n";
	USART3_SendString((unsigned char *)buffer2);
	PrintCANLog(0x12, CAN1_DATA_TX);
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
	MessageCounter = MessageCounter & 0xF;
}

/*ham nay bai1 khong xoa */ void CAN2_TX(){
	CAN2_pHeader.StdId = 0xA2;
	CAN2_pHeader.DLC = 8;
	CAN2_pHeader.IDE = CAN_ID_STD;
	CAN2_pHeader.RTR = CAN_RTR_DATA;


//
//	CAN2_DATA_TX[0] = 0x01;
//	CAN2_DATA_TX[1] = 0x02;
//	CAN2_DATA_TX[2] = CAN2_DATA_TX[0]+CAN2_DATA_TX[1];
//	CAN2_DATA_TX[6] = MessageCounter;
//	CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX,7);
		if(flag == 0)
		{
		if(CAN2_DATA_RX[7] == calc_SAE_J1850(CAN2_DATA_RX,7)){
				CAN2_DATA_TX[0] = 0x01;
				CAN2_DATA_TX[1] = 0x02;
				CAN2_DATA_TX[2] = CAN2_DATA_TX[0]+CAN2_DATA_TX[1];
				CAN2_DATA_TX[6] = MessageCounter;
				CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX,7);
		}else{
				CAN2_DATA_RX[7] = calc_SAE_J1850(CAN2_DATA_RX,7);
				CAN2_DATA_TX[0] = 0x01;
				CAN2_DATA_TX[1] = 0x02;
				CAN2_DATA_TX[2] = CAN2_DATA_TX[0]+CAN2_DATA_TX[1];
				CAN2_DATA_TX[6] = MessageCounter;
				CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX,7);
		}

		}
		else {
				CAN2_DATA_TX[0] = 0x00;
				CAN2_DATA_TX[1] = 0x00;
				CAN2_DATA_TX[2] = 0x00;
				CAN2_DATA_TX[3] = 0x00;
				CAN2_DATA_TX[4] = 0x00;
				CAN2_DATA_TX[5] = 0x00;
				CAN2_DATA_TX[6] = MessageCounter;
				CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX, 7);
			}

//	if(BtnU)
//	{
//	if(CAN2_DATA_RX[7] == calc_SAE_J1850(CAN2_DATA_RX,7)){
//			CAN2_DATA_TX[0] = 0x01;
//			CAN2_DATA_TX[1] = 0x02;
//			CAN2_DATA_TX[2] = CAN2_DATA_TX[0]+CAN2_DATA_TX[1];
//			CAN2_DATA_TX[6] = MessageCounter;
//			CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX,7);
//	}else{
//		USART3_SendString((unsigned char *)"CAN2 ERROR \n");
//	}
//
//	}
//	else{
//			CAN2_DATA_TX[0] = 0x00;
//			CAN2_DATA_TX[1] = 0x00;
//			CAN2_DATA_TX[2] = 0x00;
//			CAN2_DATA_TX[3] = 0x00;
//			CAN2_DATA_TX[4] = 0x00;
//			CAN2_DATA_TX[5] = 0x00;
//			CAN2_DATA_TX[6] = MessageCounter;
//			CAN2_DATA_TX[7] = calc_SAE_J1850(CAN2_DATA_TX, 7);
//		}
	char buffer4[50] = "CAN2 TX \n";
	USART3_SendString((unsigned char *)buffer4);
	PrintCANLog(0xA2, CAN2_DATA_TX);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	MessageCounter = (MessageCounter + 1) & 0xF;
	}

/*ham nay bai1 khong xoa */ void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
    char buffer5[50] = "CAN1 RX \n";
    USART3_SendString((unsigned char *)buffer5);
    PrintCANLog(0x0A2, CAN1_DATA_RX);
  }
  else if (hcan == &hcan2)
  {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX) != HAL_OK)
    {
      Error_Handler();
    }
    if(CAN2_DATA_RX[7] == calc_SAE_J1850(CAN2_DATA_RX,7)){
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX);
    char buffer6[50] = "CAN2 RX \n";
    USART3_SendString((unsigned char *)buffer6);
    PrintCANLog(0x012, CAN2_DATA_RX);
    }else
    {
    char buffer7[50] = "CAN2 RX NOT RECEIVE DUE TO WRONG CRC \n";
    USART3_SendString((unsigned char *)buffer7);
    }
}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
		{
			switch(flag)
			{
				case 0:
					flag = 1;
					char buffer8[50] = "IG OFF \n";
					USART3_SendString((uint8_t *)buffer8);
					break;
				case 1:
					flag = 0;
					char buffer9[50] = "-> IG ON \n";
					USART3_SendString((uint8_t *)buffer9);
					break;
				default:
					break;
			}
		}
	}
}

void Error_Handler(void)
{
  __disable_irq();
  USART3_SendString((unsigned char *)"Error Detected\n");
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
