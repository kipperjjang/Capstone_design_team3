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

typedef struct {
    uint8_t  temp_c;
    float    voltage_v;
    float    current_a;
    uint16_t consumption_mah;

    uint16_t erpm_raw;      // 보통 ERPM/100
    float    erpm;
    float    mech_rpm;

    uint8_t  valid;
    uint32_t last_update_ms;
    uint32_t frame_ok;
    uint32_t crc_err;
} ESC_Telemetry_t;

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

#define P_MAX   100.0f
#define D_MAX   10.0f

#define ESC_TELEM_FRAME_LEN     10
#define ESC_TELEM_DMA_BUF_LEN   32

#define ESC_L_POLE_PAIRS        7.0f   // 14 pole motor면 7
#define ESC_R_POLE_PAIRS        7.0f

#define ESC_PWM_IDLE_US             1000.0f
#define ESC_PWM_SHOOT_TARGET_US     1600.0f   // 우선 기존 테스트값 유지
#define ESC_PWM_MIN_US              1000.0f
#define ESC_PWM_MAX_US              1960.0f

#define ESC_RAMP_UP_US_PER_SEC      800.0f
#define ESC_RAMP_DOWN_US_PER_SEC    300.0f

#define SERVO_MIN_US        1000.0f
#define SERVO_MAX_US        2000.0f

#define SERVO_S_HOME_DEG    0.0f
#define SERVO_S_MOVE_DEG    60.0f

#define SERVO_R_HOME_DEG    0.0f
#define SERVO_R_MOVE_DEG    120.0f

#define shoot_start 1500
#define gap         500
#define reload_gap  1200
#define ball_gap    3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;

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

float yaw_P = 25.0f;
float yaw_D = 2.4f; //1.0

float pitch_P = 25.0f;
float pitch_D = 2.4f;


// UART data
uint8_t uart_rx_byte;

uint8_t pc_prev_status = STAT_OK;   // PC가 이전에 받은 중계기 프레임 상태
uint8_t stm_prev_status = STAT_OK;  // 중계기가 이전에 받은 PC 프레임 상태

uint8_t uart_tx_seq = 0;
uint8_t pc_last_seq = 0;
uint8_t pc_seq_valid = 0;

float target_yaw = 0.0f;
float target_pitch = 0.0f;
uint8_t pc_cmd = 1; // 1 aim on, 4 aim off

float yaw_angle = 0.0f;
float yaw_velocity = 0.0f;
float yaw_torque = 0.0f;
float pitch_angle = 0.0f;
float pitch_velocity = 0.0f;
float pitch_torque = 0.0f;
uint8_t system_error = 0;

float sweep_speed = 0.0f;

float pitch_limit = 1.0f, yaw_limit = 1.0f;
uint16_t i = 0;

uint8_t yaw_updated = 0;
uint8_t pitch_updated = 0;

uint8_t uart_send = 0;

uint8_t esc_l_dma_buf[ESC_TELEM_DMA_BUF_LEN];
uint8_t esc_r_dma_buf[ESC_TELEM_DMA_BUF_LEN];

volatile ESC_Telemetry_t esc_l_telem = {0};
volatile ESC_Telemetry_t esc_r_telem = {0};

uint8_t esc_telem_status = 0;

float esc_pwm_current_us = ESC_PWM_IDLE_US;
float esc_pwm_target_us  = ESC_PWM_IDLE_US;

uint32_t esc_ramp_last_tick = 0;
uint8_t esc_slow_start_ready = 0;


volatile uint8_t pc13_button_pressed = 0;
volatile uint32_t pc13_button_tick = 0;
uint8_t start = 0, start_init = 1;
uint32_t start_time = 0;

uint8_t send_skip = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

static void CAN_SendCmd(uint32_t id, float pos, float P, float D, uint8_t cmd, uint8_t seq);

static uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len);
static void UART_ProcessPCFrame(uint8_t *body);
static void UART_ParseByte(uint8_t b);
static void UART_SendFeedback(void);

static uint8_t ESC_UpdateCRC8(uint8_t crc, uint8_t data);
static uint8_t ESC_GetCRC8(const uint8_t *buf, uint8_t len);
static uint8_t ESC_ParseTelemetryFrame(const uint8_t *b, volatile ESC_Telemetry_t *out, float pole_pairs);
static void ESC_ProcessRxBurst(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
static void ESC_TelemetryStartDMA(void);
static void ESC_RestartRxDMA(UART_HandleTypeDef *huart);

static void ESC_SetPWMBoth(float pwm_us);
static void ESC_SlowStartUpdate(void);

static uint16_t Servo_DegToPulseUs(float deg);

static void servo_s_move(void);
static void servo_s_return(void);
static void servo_r_move(void);
static void servo_r_return(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == UART4) {
        ESC_ProcessRxBurst(huart, esc_l_dma_buf, Size);
        ESC_RestartRxDMA(&huart4);
    }
    else if (huart->Instance == UART5) {
        ESC_ProcessRxBurst(huart, esc_r_dma_buf, Size);
        ESC_RestartRxDMA(&huart5);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == UART4) {
        HAL_UART_AbortReceive(&huart4);
        ESC_RestartRxDMA(&huart4);
    }
    else if (huart->Instance == UART5) {
        HAL_UART_AbortReceive(&huart5);
        ESC_RestartRxDMA(&huart5);
    }
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
        yaw_updated = 1;
    }
    else if(id == NODE1_STA_ID){
        pitch_angle = pos;
        pitch_velocity = vel;
        pitch_torque = torque;
        pitch_updated = 1;
    }

    if(yaw_updated && pitch_updated){
    	uart_send = 1;
    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        yaw_updated = 0;
        pitch_updated = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        UART_ParseByte(uart_rx_byte);
        if(HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) == HAL_OK){

        }

    }
}

static void UART_ProcessPCFrame(uint8_t *body){
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

    float target_yaw_add;
    float target_pitch_add;

    memcpy(&target_yaw_add,   &body[3], 4);
    memcpy(&target_pitch_add, &body[7], 4);

    pc_cmd = body[11];

    if (pc_cmd == 0x00){
        // pixel PD
        target_yaw = yaw_angle + target_yaw_add;
        target_pitch = pitch_angle + target_pitch_add;


    }
    else if(pc_cmd == 0x01){
    	// absolute angle
    	target_yaw = target_yaw_add;
    	target_pitch = target_pitch_add;
    }

    if(target_yaw >= yaw_limit)
		target_yaw = yaw_limit;
	 else if(target_yaw <= -yaw_limit)
		target_yaw = -yaw_limit;

	 if(target_pitch >= pitch_limit)
		target_pitch = pitch_limit;
	 else if(target_pitch <= -pitch_limit)
		target_pitch = -pitch_limit;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (GPIO_Pin == B1_Pin) {
        uint32_t now = HAL_GetTick();

        if (now - pc13_button_tick > 50) {
            pc13_button_tick = now;
            pc13_button_pressed++;
        }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  TIM1->CCR1 = 1000;
  TIM1->CCR2 = 1000;
  TIM1->CCR3 = 1000;
  TIM1->CCR4 = 1000;

  esc_pwm_current_us = ESC_PWM_IDLE_US;
  esc_pwm_target_us  = ESC_PWM_IDLE_US;
  esc_ramp_last_tick = HAL_GetTick();
  esc_slow_start_ready = 1;

  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) << 0;
  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) << 1;
  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK) << 2;
  PWM_check |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK) << 3;

  servo_s_return();
  servo_r_return();

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

  ESC_TelemetryStartDMA();

  Tx_Header.StdId = NODE0_CMD_ID;
  Tx_Header.ExtId = 0;
  Tx_Header.IDE   = CAN_ID_STD;
  Tx_Header.RTR   = CAN_RTR_DATA;
  Tx_Header.DLC   = 8;
  Tx_Header.TransmitGlobalTime = DISABLE;

  if(PWM_check != 0x0F){
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

	  if(pc13_button_pressed >= 1){
		  if(start_init){
			  start_init = 0;
			  start_time = HAL_GetTick();
			  esc_pwm_target_us = 1600;
		  }

		  uint32_t time_now = HAL_GetTick() - start_time;

		  if((shoot_start <= time_now) && (time_now < (shoot_start + gap))){
			  servo_s_move();
			  servo_r_return();
		  }
		  else if(((shoot_start + reload_gap) <= time_now) && (time_now < (shoot_start + reload_gap + gap))){
			  servo_s_return();
			  servo_r_move();
		  }

		  if(((ball_gap + shoot_start) <= time_now) && (time_now < (ball_gap + shoot_start + gap))){
			  servo_s_move();
			  servo_r_return();
		  }
		  else if(((ball_gap + shoot_start + reload_gap) <= time_now) && (time_now < (ball_gap + shoot_start + reload_gap + gap))){
			  servo_s_return();
			  servo_r_move();
		  }

		  if(((2*ball_gap + shoot_start) <= time_now) && (time_now < (2*ball_gap + shoot_start + gap))){
			  servo_s_move();
			  servo_r_return();
		  }
		  else if(((2*ball_gap + shoot_start + reload_gap) <= time_now) && (time_now < (2*ball_gap + shoot_start + reload_gap + gap))){
			  servo_s_return();
			  servo_r_move();
		  }

		  else if((10000 <= time_now)){
			  pc13_button_pressed = 0;
			  start_init = 0;
		  }
	  }else{
		  esc_pwm_target_us = 1000;
		  servo_s_return();
		  servo_r_return();
	  }

	  /*if(shoot){
		  esc_pwm_target_us = 1600;
	  }
	  else{
		  esc_pwm_target_us = 1000;
	  }*/

	  ESC_SlowStartUpdate();

	  uint8_t seq = tx_seq++;

	  CAN_SendCmd(NODE0_CMD_ID, target_yaw,   yaw_P,   yaw_D,   pc_cmd, seq);
	  CAN_SendCmd(NODE1_CMD_ID, target_pitch, pitch_P, pitch_D, pc_cmd, seq);

	  HAL_Delay(5);

	  if(uart_send){
		  if(send_skip++ == 1){
			  UART_SendFeedback();
			  send_skip = 0;
		  }
		  uart_send = 0;
	  }

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

static uint8_t ESC_UpdateCRC8(uint8_t crc, uint8_t data){
    crc ^= data;

    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0x07;
        else
            crc = (crc << 1);
    }

    return crc;
}

static uint8_t ESC_GetCRC8(const uint8_t *buf, uint8_t len){
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        crc = ESC_UpdateCRC8(crc, buf[i]);
    }

    return crc;
}

static uint8_t ESC_ParseTelemetryFrame(const uint8_t *b, volatile ESC_Telemetry_t *out, float pole_pairs){
    uint8_t crc_calc = ESC_GetCRC8(b, 9);
    uint8_t crc_rx   = b[9];

    if (crc_calc != crc_rx) {
        out->crc_err++;
        return 0;
    }

    uint8_t temp_c = b[0];

    uint16_t voltage_cV =
        ((uint16_t)b[1] << 8) | b[2];

    uint16_t current_cA =
        ((uint16_t)b[3] << 8) | b[4];

    uint16_t consumption_mAh =
        ((uint16_t)b[5] << 8) | b[6];

    uint16_t erpm_raw =
        ((uint16_t)b[7] << 8) | b[8];

    float erpm = (float)erpm_raw * 100.0f;
    float mech_rpm = erpm / pole_pairs;

    out->temp_c          = temp_c;
    out->voltage_v       = (float)voltage_cV / 100.0f;
    out->current_a       = (float)current_cA / 100.0f;
    out->consumption_mah = consumption_mAh;
    out->erpm_raw        = erpm_raw;
    out->erpm            = erpm;
    out->mech_rpm        = mech_rpm;
    out->last_update_ms  = HAL_GetTick();
    out->valid           = 1;
    out->frame_ok++;

    return 1;
}

static void ESC_ProcessRxBurst(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size){
    if (size < ESC_TELEM_FRAME_LEN) {
        return;
    }

    if (huart->Instance == UART4) {
        for (uint16_t i = 0; i + ESC_TELEM_FRAME_LEN <= size; i++) {
            if (ESC_ParseTelemetryFrame(&buf[i], &esc_l_telem, ESC_L_POLE_PAIRS)) {
                break;
            }
        }
    }
    else if (huart->Instance == UART5) {
        for (uint16_t i = 0; i + ESC_TELEM_FRAME_LEN <= size; i++) {
            if (ESC_ParseTelemetryFrame(&buf[i], &esc_r_telem, ESC_R_POLE_PAIRS)) {
                break;
            }
        }
    }
}

static void ESC_TelemetryStartDMA(void){
    ESC_RestartRxDMA(&huart4);
    ESC_RestartRxDMA(&huart5);
}

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

    HAL_UART_Transmit(&huart2, tx, idx, 10);
}

static void ESC_RestartRxDMA(UART_HandleTypeDef *huart){
    uint8_t *buf = NULL;

    if (huart->Instance == UART4) {
        buf = esc_l_dma_buf;
        HAL_HalfDuplex_EnableReceiver(&huart4);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, buf, ESC_TELEM_DMA_BUF_LEN);

        if (huart4.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
        }
    }
    else if (huart->Instance == UART5) {
        buf = esc_r_dma_buf;
        HAL_HalfDuplex_EnableReceiver(&huart5);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buf, ESC_TELEM_DMA_BUF_LEN);

        if (huart5.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        }
    }
}

static void ESC_SetPWMBoth(float pwm_us){
    if (pwm_us < ESC_PWM_MIN_US) pwm_us = ESC_PWM_MIN_US;
    if (pwm_us > ESC_PWM_MAX_US) pwm_us = ESC_PWM_MAX_US;

    uint16_t ccr = (uint16_t)(pwm_us + 0.5f);

    TIM1->CCR1 = ccr;
    TIM1->CCR2 = ccr;
}

static void ESC_SlowStartUpdate(void){
    uint32_t now = HAL_GetTick();

    if (!esc_slow_start_ready) {
        esc_ramp_last_tick = now;
        esc_pwm_current_us = ESC_PWM_IDLE_US;
        esc_pwm_target_us  = ESC_PWM_IDLE_US;
        ESC_SetPWMBoth(ESC_PWM_IDLE_US);
        esc_slow_start_ready = 1;
        return;
    }

    uint32_t dt_ms = now - esc_ramp_last_tick;

    if (dt_ms == 0) {
        return;
    }

    esc_ramp_last_tick = now;


    float dt_s = 0.001f * (float)dt_ms;
    float rate = shoot ? ESC_RAMP_UP_US_PER_SEC : ESC_RAMP_DOWN_US_PER_SEC;
    float max_step = rate * dt_s;

    float diff = esc_pwm_target_us - esc_pwm_current_us;

    if (diff > max_step) {
        esc_pwm_current_us += max_step;
    }
    else if (diff < -max_step) {
        esc_pwm_current_us -= max_step;
    }
    else {
        esc_pwm_current_us = esc_pwm_target_us;
    }

    ESC_SetPWMBoth(esc_pwm_current_us);
}

static uint16_t Servo_DegToPulseUs(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    float pulse_us = SERVO_MIN_US +
                     (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);

    return (uint16_t)(pulse_us + 0.5f);
}

static void servo_s_move(void)
{
    // Servo S: TIM1_CH3
    TIM1->CCR3 = Servo_DegToPulseUs(SERVO_S_MOVE_DEG);
}

static void servo_s_return(void)
{
    // Servo S: TIM1_CH3
    TIM1->CCR3 = Servo_DegToPulseUs(SERVO_S_HOME_DEG);
}

static void servo_r_move(void)
{
    // Servo R: TIM1_CH4
    TIM1->CCR4 = Servo_DegToPulseUs(SERVO_R_MOVE_DEG);
}

static void servo_r_return(void)
{
    // Servo R: TIM1_CH4
    TIM1->CCR4 = Servo_DegToPulseUs(SERVO_R_HOME_DEG);
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
