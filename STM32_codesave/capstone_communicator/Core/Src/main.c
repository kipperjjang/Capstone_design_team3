/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_SOF_0      0xAA
#define UART_SOF_1      0x55

#define PC_CMD_LEN      9
#define PC_FB_LEN       25

#define STAT_OK         0x00
#define STAT_CRC_ERR    (1 << 0)
#define STAT_LEN_ERR    (1 << 1)
#define STAT_SEQ_ERR    (1 << 2)
#define STAT_TIMEOUT    (1 << 3)

#define CMD_AIM         (1 << 0)
#define CMD_SHOOT       (1 << 1)
#define CMD_STOP        (1 << 2)

#define P_MAX   20.0f
#define D_MAX   1.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t PWM_check = 0;

uint8_t shoot = 0;

uint16_t motor1 = 1000;
uint16_t motor2 = 1000;


uint8_t CAN_setting_status = 0;

uint32_t              Tx_Mailbox;
CAN_TxHeaderTypeDef   Tx_Header;
uint8_t               Tx_Data[8];
uint8_t CAN_transmit_status = 0;

CAN_FilterTypeDef  sFilterConfig;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint8_t CAN_receive_status = 0;

#define NODE0_CMD_ID   0x100U
#define NODE0_STA_ID   0x180U
#define NODE1_CMD_ID   0x101U
#define NODE1_STA_ID   0x181U

uint16_t tx_seq = 0;
uint32_t last_can_tick = 0;

uint8_t send = 0;

float yaw_P = 5.0f;
float yaw_D = 0.2f;

float pitch_P = 3.0f;
float pitch_D = 0.1f;


// UART data
uint8_t uart_rx_byte;

uint8_t pc_prev_status = STAT_OK;   // PC가 이전에 받은 중계기 프레임 상태
uint8_t stm_prev_status = STAT_OK;  // 중계기가 이전에 받은 PC 프레임 상태

uint8_t uart_tx_seq = 0;
uint8_t pc_last_seq = 0;
uint8_t pc_seq_valid = 0;

float target_yaw = 0.0f;
float target_pitch = 0.0f;
uint8_t pc_cmd = 1;

float yaw_angle = 0.0f;
float yaw_velocity = 0.0f;
float yaw_torque = 0.0f;
float pitch_angle = 0.0f;
float pitch_velocity = 0.0f;
float pitch_torque = 0.0f;
uint8_t system_error = 0;

float sweep_speed = 0.005f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline int32_t float_to_i32_scaled(float x, float scale){
    return (int32_t)(x * scale);
}

static inline int16_t float_to_i16_scaled(float x, float scale){
    return (int16_t)(x * scale);
}

static void CAN_SendCmd(uint32_t id, float pos, float P, float D, uint8_t cmd, uint8_t seq){

    int32_t pos_q = (int32_t)(pos * 1000000.0f);

    uint8_t P_q = (uint8_t)( (P / P_MAX) * 255.0f );
    uint8_t D_q = (uint8_t)( (D / D_MAX) * 255.0f );

    Tx_Data[0] = pos_q & 0xFF;
    Tx_Data[1] = (pos_q >> 8) & 0xFF;
    Tx_Data[2] = (pos_q >> 16) & 0xFF;
    Tx_Data[3] = (pos_q >> 24) & 0xFF;

    Tx_Data[4] = P_q;
    Tx_Data[5] = D_q;

    Tx_Data[6] = cmd;
    Tx_Data[7] = seq;

    Tx_Header.StdId = id;

    CAN_transmit_status =
        (HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_Data, &Tx_Mailbox) == HAL_OK);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (hcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        return;

    uint32_t id = RxHeader.StdId;

    int16_t pos_q   = (int16_t)(RxData[0] | (RxData[1] << 8));
    int16_t vel_q   = (int16_t)(RxData[2] | (RxData[3] << 8));
    int16_t torque_q= (int16_t)(RxData[4] | (RxData[5] << 8));

    float pos = (float)pos_q / 10000.0f;
    float vel = (float)vel_q / 1000.0f;
    float torque = (float)torque_q / 1000.0f;

    if(id == NODE0_STA_ID){
        yaw_angle = pos;
        yaw_velocity = vel;
        yaw_torque = torque;
    }
    else if(id == NODE1_STA_ID){
        pitch_angle = pos;
        pitch_velocity = vel;
        pitch_torque = torque;
    }
}

static uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++){
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t j = 0; j < 8; j++){
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static void UART_ProcessPCFrame(uint8_t *body){
    // body = prev_status, len, seq, data..., crc_l, crc_h
    uint8_t rx_prev_status = body[0];
    uint8_t len = body[1];
    uint8_t seq = body[2];

    pc_prev_status = rx_prev_status;

    if (len != PC_CMD_LEN){
        stm_prev_status = STAT_LEN_ERR;
        return;
    }

    uint16_t crc_rx = (uint16_t)body[3 + len] | ((uint16_t)body[4 + len] << 8);
    uint16_t crc_calc = CRC16_CCITT(body, 3 + len);

    if (crc_rx != crc_calc){
        stm_prev_status = STAT_CRC_ERR;
        return;
    }

    if (pc_seq_valid){
        uint8_t expected = pc_last_seq + 1;
        if (seq != expected){
            stm_prev_status = STAT_SEQ_ERR;
            // seq error가 있어도 CRC가 맞으면 데이터는 사용할 수 있게 둠
        }
        else{
            stm_prev_status = STAT_OK;
        }
    }
    else{
        pc_seq_valid = 1;
        stm_prev_status = STAT_OK;
    }

    pc_last_seq = seq;

    memcpy(&target_yaw,   &body[3], 4);
    memcpy(&target_pitch, &body[7], 4);
    pc_cmd = body[11];

    if (pc_cmd & CMD_STOP){
        shoot = 0;
        send = 0;
    }

    if (pc_cmd & CMD_AIM){
        send = 1;
    }

    if (pc_cmd & CMD_SHOOT){
        shoot = 1;
    }
}

static void UART_ParseByte(uint8_t b){
    static uint8_t state = 0;
    static uint8_t body[3 + PC_FB_LEN + 2];
    static uint8_t idx = 0;
    static uint8_t expected_total = 0;

    switch (state){
    case 0: // SOF0
        if (b == UART_SOF_0) state = 1;
        break;

    case 1: // SOF1
        if (b == UART_SOF_1){
            idx = 0;
            expected_total = 0;
            state = 2;
        }
        else{
            state = 0;
        }
        break;

    case 2: // body read
        body[idx++] = b;

        if (idx == 2){
            uint8_t len = body[1];

            if (len != PC_CMD_LEN){
                stm_prev_status = STAT_LEN_ERR;
                state = 0;
                idx = 0;
                return;
            }

            expected_total = 3 + len + 2;
        }

        if (expected_total > 0 && idx >= expected_total){
            UART_ProcessPCFrame(body);
            state = 0;
            idx = 0;
        }

        if (idx >= sizeof(body)){
            state = 0;
            idx = 0;
            stm_prev_status = STAT_LEN_ERR;
        }
        break;

    default:
        state = 0;
        idx = 0;
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        UART_ParseByte(uart_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

static void UART_SendFeedback(void){
    uint8_t tx[32];
    uint8_t idx = 0;

    tx[idx++] = UART_SOF_0;
    tx[idx++] = UART_SOF_1;

    tx[idx++] = stm_prev_status;
    tx[idx++] = PC_FB_LEN;
    tx[idx++] = uart_tx_seq++;

    memcpy(&tx[idx], &yaw_angle, 4);      idx += 4;
    memcpy(&tx[idx], &yaw_velocity, 4);   idx += 4;
    memcpy(&tx[idx], &yaw_torque, 4);     idx += 4;
    memcpy(&tx[idx], &pitch_angle, 4);    idx += 4;
    memcpy(&tx[idx], &pitch_velocity, 4); idx += 4;
    memcpy(&tx[idx], &pitch_torque, 4);   idx += 4;

    tx[idx++] = system_error;

    uint16_t crc = CRC16_CCITT(&tx[2], 3 + PC_FB_LEN);

    tx[idx++] = crc & 0xFF;
    tx[idx++] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart2, tx, sizeof(tx), 10);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) << 0;
  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) << 1;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;              // receive all data from all ID
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; //14 bank is each used in CAN1 (0 ~ 13) / CAN2 (14 ~ 27). this code is used to divide filterbank to each CAN bus.
  //as CAN2 sharing filterbank with CAN1 (CAN1 and CAN2 shares same bus), both CAN have 14 filterbank as default. SlaveStartFilterBank is only setting at CAN1 filter
  //WARN : do not use SlaveStartFilterBank code in CAN2
  CAN_setting_status |= (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) == HAL_OK)                      <<7; // CAN1 Rx filter enable
  CAN_setting_status |= (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK) <<6; // CAN1 Rx ISR enable
  CAN_setting_status |= (HAL_OK == HAL_CAN_Start(&hcan1))                                             <<5; //CAN1 start

  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

  Tx_Header.StdId = NODE0_CMD_ID;
  Tx_Header.ExtId = 0;
  Tx_Header.IDE   = CAN_ID_STD;
  Tx_Header.RTR   = CAN_RTR_DATA;
  Tx_Header.DLC   = 8;
  Tx_Header.TransmitGlobalTime = DISABLE;

  if(PWM_check != 0x03){
	  //_disable_irq();
	  while(1){

	  }
  }

  if(CAN_setting_status != 0xE0){
 	  //_disable_irq();
 	  while(1){

 	  }
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*if(shoot){
		  TIM1->CCR1 = 1540;
		  TIM1->CCR2 = 1560;
	  }
	  else{
		  TIM1->CCR1 = 1000;
		  TIM1->CCR2 = 1000;
	  }

	  HAL_Delay(20);*/

//	  if(send){
//		  send = 0;
//		  Tx_Header.StdId = NODE0_CMD_ID;
//		  CAN_transmit_status = (HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_Data, &Tx_Mailbox) == HAL_OK);
//	  }
//
//	  HAL_Delay(100);

	  /*if(send){
	      send = 0;

	      uint8_t seq = tx_seq++;

	      float P = 5.0f;   // 초기값
	      float D = 0.2f;

	      CAN_SendCmd(NODE0_CMD_ID, target_yaw, P, D, pc_cmd, seq);
	      CAN_SendCmd(NODE1_CMD_ID, target_pitch, P, D, pc_cmd, seq);
	  }*/

	  uint8_t seq = tx_seq++;

	  target_yaw += sweep_speed;
	  target_pitch += sweep_speed;
	  if(target_yaw >= 3.141592f / 4.0f)
		  sweep_speed = -sweep_speed;
	  if(target_yaw <= -3.141592f / 4.0f)
		  sweep_speed = -sweep_speed;

	  CAN_SendCmd(NODE0_CMD_ID, target_yaw, yaw_P, yaw_D, pc_cmd, seq);
	  CAN_SendCmd(NODE1_CMD_ID, target_pitch, pitch_P, pitch_D, pc_cmd, seq);

	  //UART_SendFeedback();

	  HAL_Delay(20);   // 50 Hz feedback

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
