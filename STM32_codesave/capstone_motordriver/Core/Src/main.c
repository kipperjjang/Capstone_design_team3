/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

//#include "DRV8323_setting.h"
#include "DRV8353_setting.h"
#include "AS5048A.h"
#include "arm_math.h"
#include "core_cm4.h"
#include "stdio.h"
#include "stm32g4xx_ll_bus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define invsqrt3 (0.57735027f)
#define SECTOR_RAD  (1.04719755f)      // π/3
#define CPU_HZ (168000000.0f)

#define ARR_TICKS   4199               // TIM1 max ARR
#define V_BUS (14.8f)                  // input bus voltage
#define Vd_limit (V_LIMIT * 0.9f)      // Vd limit voltage. 0.98 is safety factor
#define V_LIMIT (V_BUS * invsqrt3)     // real maximum limit

#define CAN_CMD_POS_SCALE       1000000.0f   // int32 = rad * 1e6
#define CAN_CMD_VEL_SCALE       1000.0f      // int16 = rad/s * 1e3

#define CAN_FB_POS_SCALE        10000.0f     // int16 = rad * 1e4
#define CAN_FB_VEL_SCALE        1000.0f      // int16 = rad/s * 1e3
#define CAN_FB_TORQUE_SCALE     1000.0f      // int16 = torque * 1e3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

#define motor0

#ifdef motor0
	#define NODE_ID              0U      // board0 0 / board1 1

	// ADC calibrated value (different for each board and chip)
	#define current_A_calibrated 2045  // board0 2045 / board1 2047
	#define current_B_calibrated 2058  // board0 2058 / board1 2056

	//mechanical zero position
	uint16_t AS5048_mech_zeropos = 12150; // board0 8040 / board1 11500

	// motor electrical angle commutation
	uint16_t AS5048_zeropos = 12803;   // board0 12803 / board1 10768

	float pos_P = 10.0f;
	float pos_I = 10.0f;
	float pos_D = 1.9f;

	float W_M_LPF_ALPHA = 0.8f;

	#define IQ_REF_LIMIT    (2.0f)

	float POS_I_ENABLE_ERR = 0.1f;   // [rad]
	float POS_I_LIMIT_RATE = 0.5f;
#endif

#ifdef motor1
	#define NODE_ID              1U      // board0 0 / board1 1

	// ADC calibrated value (different for each board and chip)
	#define current_A_calibrated 2047  // board0 2045 / board1 2047
	#define current_B_calibrated 2056  // board0 2058 / board1 2056

	//mechanical zero position
	uint16_t AS5048_mech_zeropos = 3333; // board0 8040 / board1 2550

	// motor electrical angle commutation
	uint16_t AS5048_zeropos = 3736;   // board0 12803 / board1 3736

	/*float pos_P = 30.0f;
	float pos_I = 300.0f;
	float pos_D = 2.5f;*/

	float pos_P = 10.0f;
	float pos_I = 0.0f;
	float pos_D = 1.0f;

	float W_M_LPF_ALPHA = 0.8f;

	#define IQ_REF_LIMIT    (5.0f)

	float POS_I_ENABLE_ERR = 0.1f;   // [rad]
	float POS_I_LIMIT_RATE = 0.5f;

#endif

float mech_limit = PI/2;

volatile float pos_ref_mech_rad    = 0.0f;

float P_part, I_part, D_part;



// P 25.0, D 2.4, alpha 0.002

float pos_err, pos_err_integral;

// CAN communication setting
FDCAN_FilterTypeDef sFilter;
FDCAN_RxHeaderTypeDef   Rx_Header;
FDCAN_TxHeaderTypeDef   Tx_Header;
uint8_t               Rx_Data[8];
uint8_t               Tx_Data[8];

uint8_t CAN_setting_status = 0;
uint8_t CAN_receive_status = 0;
uint8_t CAN_transmit_status = 0;

#define CAN_ID_CMD_BASE      0x100U
#define CAN_ID_STATUS_BASE   0x180U

#define CAN_ID_CMD           (CAN_ID_CMD_BASE + NODE_ID)    // command receive id
#define CAN_ID_STATUS        (CAN_ID_STATUS_BASE + NODE_ID) // status send id

// PWM start status. CH1_start / CH2_start / CH3_start / TIM1_ISR_start / ADC1_start / ADC2_start / ADC3_start / ALL_OK
uint8_t PWM_ADC_status = 0;
// DRV8353 setting status. 0x00 / 0x01 / 0x02 / 0x03 / 0x04 / 0x05 / 0x06 / ALL_OK
uint8_t DRV8353_status = 0;

uint8_t ADC_cali_status;

uint16_t current_A[1];
uint16_t current_B[1];
uint16_t current_C[1];

#define polepair 7
float ELEC_CNT = 16384.0f / (float)polepair;
float P  = 6.0f;
float I  = 0.4f;
float Ka = 0.5f;

float iq_ref = 0.0f, id_ref = 0.0f;                              // reference current

int16_t ia, ib, ic;
float electrical_angle, _sin, _cos;                              // electrical angle, sin & cos
float i_alpha, i_beta;                                           // inv park & clarke
float iq, id, iq_err, id_err, iq_err_integral, id_err_integral;  // PI control
float Vq, Vd, V_alpha, V_beta;                                   // park

uint8_t encoder_cali = 0;
float V_mag = 4.0f, V_arg = 0.0f, V_cali_speed;

float all_us;
float cali_sin, cali_cos;

#define AS5048_CPR        (16384.0f)
#define AS5048_TO_RAD     (2.0f * PI / AS5048_CPR)

#define POS_CTRL_HZ       (100)
#define POS_CTRL_DT       (1.0f / POS_CTRL_HZ)

float th_m       = 0.0f;   // mechanical angle, wrapped [-pi, pi)
float th_m_cont  = 0.0f;   // continuous mechanical angle [rad]
float w_m        = 0.0f;   // filtered mechanical speed [rad/s]
float w_m_raw    = 0.0f;   // raw mechanical speed [rad/s]

float th_m_prev    = 0.0f;
uint8_t th_m_init  = 0U;

uint8_t not_aim = 0;
uint8_t update = 0;;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

// Forward declarations
static inline uint32_t dwt_now(void);
static inline float cycles_to_us(uint32_t cyc);

static inline float wrap_pm_pi(float x);
static inline float clampf(float x, float lo, float hi);

static inline void DWT_Init(void);
void set_ccr(uint16_t a, uint16_t b, uint16_t c);

static inline float sat(float x);

void SVPWM(float, float);

static inline float AS5048_CountToMechRad(uint16_t cnt);
static inline void MechAngleSpeed_Update(void);

static inline int16_t clamp_i16_from_float(float x);
static inline int32_t get_i32_le(uint8_t *d);
static inline void put_i16_le(uint8_t *d, int16_t v);

static void Motor_SendStatus(uint8_t seq);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void PositionPID_Update(void)
{
    pos_ref_mech_rad = clampf(pos_ref_mech_rad, -mech_limit, mech_limit);

    if (not_aim) {
        iq_ref = 0.0f;
        id_ref = 0.0f;

        pos_err = 0.0f;
        pos_err_integral = 0.0f;

        P_part = 0.0f;
        I_part = 0.0f;
        D_part = 0.0f;

        return;
    }

    pos_err = pos_ref_mech_rad - th_m_cont;

    P_part = pos_P * pos_err;
    D_part = -pos_D * w_m;

    if (pos_I > 1e-6f) {
        if (fabsf(pos_err) < POS_I_ENABLE_ERR) {
            pos_err_integral += pos_err * POS_CTRL_DT;
        }

        float pos_err_integral_LIMIT = POS_I_LIMIT_RATE * (IQ_REF_LIMIT / pos_I);

        pos_err_integral = clampf(pos_err_integral,
                                  -pos_err_integral_LIMIT,
                                   pos_err_integral_LIMIT);

        I_part = pos_I * pos_err_integral;
    }
    else {
        pos_err_integral = 0.0f;
        I_part = 0.0f;
    }

    float u_unsat = P_part + I_part + D_part;
    float u_sat = clampf(u_unsat, -IQ_REF_LIMIT, IQ_REF_LIMIT);

    iq_ref = u_sat;
    id_ref = 0.0f;
}

uint32_t debug_time_start, SVPWM_end;

#define measure_limit (V_BUS * 2)  //atan2 limit. change for max voltage or max current
#define current_const (0.00805664f*2)  // 10V/V Op-amp gain
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //TIM interrupt rank has to be added in main()
	if(htim->Instance == TIM1){
		debug_time_start = dwt_now();

		AS5048A_ReadAngle();

		float delta = (float) (mech_angle - AS5048_zeropos);
		if(delta < 0.0) delta = delta + 16384.0;
		while(delta > ELEC_CNT){
			delta = delta - ELEC_CNT;
		}
		electrical_angle = delta * 360.0f / ELEC_CNT;

		if (electrical_angle > 180.0f) electrical_angle -= 360.0f;

		arm_sin_cos_f32(electrical_angle,&_sin,&_cos);

		if(update++ >= 199){
			MechAngleSpeed_Update();
			PositionPID_Update();
			update = 0;
		}

		// alpha beta current calculation
		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (+-10 LSB) (maybe hardware PCB problem. maybe charge pump?)

		// inv clarke
		i_alpha = ia * current_const;                   // i_alpha = 2/3 * (ia - (ib + ic)/2) = ia
		i_beta  = (ib - ic) * invsqrt3 * current_const; // i_beta  = 2/3 * (ib - ic) * sqrt(3)/2 = (ib - ic) * 1/sqrt(3)


		if(encoder_cali){
			V_arg = V_arg + V_cali_speed;

			if(V_arg > 180.0f)
				V_arg -= 360.0f;
			else if(V_arg < -180.0f)
				V_arg += 360.0f;

			arm_sin_cos_f32(V_arg,&cali_sin,&cali_cos);

			SVPWM(V_mag * cali_cos, V_mag * cali_sin);
			return;
		}

		// inv park, using high frequency rejected
		id =  i_alpha * _cos + i_beta * _sin;
		iq = -i_alpha * _sin + i_beta * _cos;

		//PI
		iq_err = iq_ref - iq;
		id_err = id_ref - id;
		iq_err_integral = iq_err_integral + iq_err;
		id_err_integral = id_err_integral + id_err;

		// Vq, Vd calculation
		float Vq_unsat = (P * iq_err) + (I * iq_err_integral);
		float Vd_unsat = (P * id_err) + (I * id_err_integral);

		Vd = (Vd_unsat > Vd_limit) ? Vd_limit : (Vd_unsat < -Vd_limit) ? -Vd_limit : Vd_unsat;

		float Vq_limit;
		arm_sqrt_f32((V_LIMIT*V_LIMIT) - (Vd*Vd), &Vq_limit);
		Vq = (Vq_unsat > Vq_limit) ? Vq_limit : (Vq_unsat < -Vq_limit) ? -Vq_limit : Vq_unsat;

		iq_err_integral = iq_err_integral - Ka*(Vq_unsat - Vq);
		id_err_integral = id_err_integral - Ka*(Vd_unsat - Vd);

		// park
		V_alpha = Vd * _cos - Vq * _sin;
		V_beta  = Vd * _sin + Vq * _cos;

		SVPWM(V_alpha, V_beta);

		SVPWM_end = dwt_now();

		all_us               = cycles_to_us(SVPWM_end - debug_time_start);
	}
	return;
}




/* CAN Rx ISR start */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN2)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    CAN_receive_status = (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rx_Header, Rx_Data) == HAL_OK);

    if (!CAN_receive_status)
        return;

    CAN_receive_status = 0;

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);  // green LED

    // 내 command ID가 아니면 무시
    if (Rx_Header.Identifier != CAN_ID_CMD)
        return;

    // Classic CAN 8 byte만 허용
    if (Rx_Header.DataLength != FDCAN_DLC_BYTES_8)
        return;

    int32_t pos_q = get_i32_le(&Rx_Data[0]);

    uint8_t seq = Rx_Data[7];

    pos_ref_mech_rad = ((float)pos_q) / CAN_CMD_POS_SCALE;

	if (pos_ref_mech_rad > mech_limit)
		pos_ref_mech_rad = mech_limit;
	else if (pos_ref_mech_rad < -mech_limit)
		pos_ref_mech_rad = -mech_limit;

    Motor_SendStatus(seq);
}
/* CAN Rx ISR end */


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
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init();

  // ADC calibration
  ADC_cali_status |= (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK) << 2;
  ADC_cali_status |= (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) == HAL_OK) << 1;
  ADC_cali_status |= (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) == HAL_OK);

  // TIM 1 start
  PWM_ADC_status |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) << 7;
  PWM_ADC_status |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) << 6;
  PWM_ADC_status |= (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK) << 5;
  TIM1->CCR1 = ARR_TICKS;
  TIM1->CCR2 = ARR_TICKS;
  TIM1->CCR3 = ARR_TICKS;


  // CAN filter
  sFilter.IdType       = FDCAN_STANDARD_ID;
  sFilter.FilterIndex  = 0;
  sFilter.FilterType   = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1    = CAN_ID_CMD;   // 0x100 + NODE_ID
  sFilter.FilterID2    = 0x7FF;        // 11-bit exact match mask

  CAN_setting_status |= (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter) == HAL_OK)                                                               << 7;
  CAN_setting_status |= (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE) == HAL_OK) << 6;
  CAN_setting_status |= (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) == HAL_OK)                                 << 5;
  CAN_setting_status |= (HAL_FDCAN_Start(&hfdcan2) == HAL_OK)                                                                                << 4;

  Tx_Header.Identifier          = CAN_ID_STATUS;       // 0x180 + NODE_ID
  Tx_Header.IdType              = FDCAN_STANDARD_ID;
  Tx_Header.TxFrameType         = FDCAN_DATA_FRAME;
  Tx_Header.DataLength          = FDCAN_DLC_BYTES_8;
  Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Tx_Header.BitRateSwitch       = FDCAN_BRS_OFF;
  Tx_Header.FDFormat            = FDCAN_CLASSIC_CAN;
  Tx_Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  Tx_Header.MessageMarker       = 0;


  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);  // blue LED
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);  // green LED
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // red LED

  HAL_Delay(500);

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);  // blue LED
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);  // green LED
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // red LED


  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);  // AS5048 nCS HIGH
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1); // DRV8353 enable
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); // DRV8353 nCS HIGH
  HAL_Delay(10);

  DRV8353_errorflag_reset();
  DRV8353_SPI_read();
  DRV8353_SPI_setting();
  DRV8353_SPI_read();
  DRV8353_status = DRV8353_issetuped();

  // ADC 1,2,3 start
  PWM_ADC_status |= (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)current_A, 1) == HAL_OK) << 3;
  PWM_ADC_status |= (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)current_B, 1) == HAL_OK) << 2;
  PWM_ADC_status |= (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)current_C, 1) == HAL_OK) << 1;

  if(PWM_ADC_status != 0xEE){  // red LED blink
	  __disable_irq();
	  while (1){
		  HAL_Delay(1000);
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // red LED
	  }
  }

  if(CAN_setting_status != 0xF0){  // green LED on, red LED blink
	  __disable_irq();
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);  // green LED
  	  while (1){
  		  HAL_Delay(1000);
  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // red LED
  	  }
  }

  if(DRV8353_status != 0xFF){  // blue LED on, red LED blink
	  __disable_irq();
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);  // blue LED
	  while (1){
		  HAL_Delay(1000);
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // red LED
		  DRV8353_errorflag_reset();
		  DRV8353_SPI_read();
	  }
  }

  //disable DMA IRQ
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn); // ADC1 DMA
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn); // ADC2 DMA
  HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn); // ADC3 DMA

  // pass all test
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // blue LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1); // motor HI-Z

  PWM_ADC_status |= (HAL_TIM_Base_Start_IT(&htim1) == HAL_OK) << 4;
  PWM_ADC_status |= (PWM_ADC_status == 0xFE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	pos_ref_mech_rad = -pos_ref_mech_rad;
	HAL_Delay(2000);
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 42;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 11;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 2-1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, motor_enable_Pin|motor_HI_Z_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRV_CS_Pin|LED2_green_Pin|LED3_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_red_GPIO_Port, LED1_red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motor_enable_Pin motor_HI_Z_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = motor_enable_Pin|motor_HI_Z_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_CS_Pin */
  GPIO_InitStruct.Pin = DRV_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DRV_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_fault_Pin */
  GPIO_InitStruct.Pin = DRV_fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV_fault_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_red_Pin */
  GPIO_InitStruct.Pin = LED1_red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_red_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_green_Pin LED3_blue_Pin */
  GPIO_InitStruct.Pin = LED2_green_Pin|LED3_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static inline uint32_t dwt_now(void) { return DWT->CYCCNT; }

static inline float cycles_to_us(uint32_t cyc){
    return (float)((float)cyc / (168.0f));
}

static inline float wrap_pm_pi(float x){
    while (x >= PI)  x -= 2.0f * PI;
    while (x < -PI)  x += 2.0f * PI;
    return x;
}

static inline float clampf(float x, float lo, float hi){
    if(x < lo) return lo;
    if(x > hi) return hi;
    return x;
}



static inline void DWT_Init(void)
{
    // DWT 사용 허용
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 사이클 카운터 리셋
    DWT->CYCCNT = 0;

    // 사이클 카운터 enable
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void set_ccr(uint16_t a, uint16_t b, uint16_t c){
    TIM1->CCR1 = a; TIM1->CCR2 = b; TIM1->CCR3 = c;
}

static inline float sat(float x){
    return clampf(x, -1.0f, 1.0f);
}

static inline float AS5048_CountToMechRad(uint16_t cnt){
    int32_t dcnt = (int32_t)cnt - (int32_t)AS5048_mech_zeropos;

    if (dcnt < 0) {
        dcnt += 16384;
    }

    // 0 ~ 2pi
    float th = (float)dcnt * AS5048_TO_RAD;

    // -pi ~ pi
    if (th >= PI) {
        th -= 2.0f * PI;
    }

    return th;
}

static inline void MechAngleSpeed_Update(void){
    float th_now = AS5048_CountToMechRad((uint16_t)mech_angle);

    if (!th_m_init) {
        th_m       = th_now;
        th_m_prev  = th_now;
        th_m_cont  = th_now;
        w_m_raw    = 0.0f;
        w_m        = 0.0f;
        th_m_init  = 1U;
        return;
    }

    // encoder wrap 보정된 delta angle
    float dth = wrap_pm_pi(th_now - th_m_prev);

    th_m_prev = th_now;
    th_m      = th_now;

    // multi-turn continuous angle
    th_m_cont += dth;

    // mechanical angular velocity [rad/s]
    w_m_raw = dth * POS_CTRL_HZ;

    // 간단한 1차 LPF
    w_m += W_M_LPF_ALPHA * (w_m_raw - w_m);
}


static inline int16_t clamp_i16_from_float(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static inline int32_t get_i32_le(uint8_t *d)
{
    return (int32_t)((uint32_t)d[0]
                  | ((uint32_t)d[1] << 8)
                  | ((uint32_t)d[2] << 16)
                  | ((uint32_t)d[3] << 24));
}


static inline void put_i16_le(uint8_t *d, int16_t v)
{
    d[0] = (uint8_t)(v & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
}

#define dt_ticks    	  (16.8f)
#define I_EPS             (0.05f)
#define SQRT3_BY_2        (0.8660254037844386f)

void SVPWM(float Valpha, float Vbeta){
    float Vmag;
    arm_sqrt_f32(Valpha*Valpha + Vbeta*Vbeta, &Vmag);

    if (Vmag < 1e-6f) {
        set_ccr(ARR_TICKS - (uint16_t)(ARR_TICKS/2), ARR_TICKS - (uint16_t)(ARR_TICKS/2), ARR_TICKS - (uint16_t)(ARR_TICKS/2));
        return;
    }

    float M = 1.73205080f * (Vmag / V_BUS);
    if (M > 0.99f) M = 0.99f;

    float ang = atan2f(Vbeta, Valpha);
    if (ang < 0) ang += 6.2831853f;

    uint8_t k = (uint8_t)(ang / SECTOR_RAD);
    if (k >= 6) k = 0;

    float phi = ang - k*SECTOR_RAD;

    float T1 = M * arm_sin_f32(SECTOR_RAD - phi);
    float T2 = M * arm_sin_f32(phi);

    int32_t t1 = (int32_t)(ARR_TICKS * T1);
    int32_t t2 = (int32_t)(ARR_TICKS * T2);

    int32_t t0 = (ARR_TICKS - t1 - t2) >> 1;
    if (t0 < 0) t0 = 0;

    uint16_t Ua, Ub, Uc;
    switch (k){
        case 0: Ua=t1+t2+t0; Ub=t2+t0;     Uc=t0;            break;
        case 1: Ua=t1+t0;    Ub=t1+t2+t0;  Uc=t0;            break;
        case 2: Ua=t0;       Ub=t1+t2+t0;  Uc=t2+t0;         break;
        case 3: Ua=t0;       Ub=t1+t0;     Uc=t1+t2+t0;      break;
        case 4: Ua=t2+t0;    Ub=t0;        Uc=t1+t2+t0;      break;
        default:Ua=t1+t2+t0; Ub=t0;        Uc=t1+t0;         break;
    }

    set_ccr(ARR_TICKS - Ua, ARR_TICKS - Ub, ARR_TICKS - Uc);
}

static void Motor_SendStatus(uint8_t seq){
    int16_t pos_q;
    int16_t vel_q;
    int16_t torque_q;

    pos_q = clamp_i16_from_float(th_m_cont * CAN_FB_POS_SCALE);
    vel_q = clamp_i16_from_float(w_m * CAN_FB_VEL_SCALE);
    torque_q = clamp_i16_from_float(iq * CAN_FB_TORQUE_SCALE);

    put_i16_le(&Tx_Data[0], pos_q);
    put_i16_le(&Tx_Data[2], vel_q);
    put_i16_le(&Tx_Data[4], torque_q);

    Tx_Data[6] = 0x00;
    Tx_Data[7] = seq;

    CAN_transmit_status = (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Tx_Header, Tx_Data) == HAL_OK);
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
  while (1){

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
