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
#include <pid_control.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "callbacks.h"
#include "time_utils.h"
#include "math_utils.h"
#include "comm_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



typedef enum {
    STATE_IDLE,
    STATE_SENSOR_INIT,
    STATE_SENSOR_CALIBRATION,
    STATE_READY,
	STATE_ARMING,
    STATE_RUNNING,
	STATE_DISARMING,
    STATE_ERROR
} DroneState;

typedef enum {
	MODE_TAKEOFF,
    MODE_STABILIZED,
	MODE_FREESTYLE,
	MODE_AUTONOMOUS,
	MODE_FAILSAFE,
	MODE_LANDING
} FlightMode;


typedef struct {
	float Lx;
	float Ly;
	float Rx;
	float Ry;
	uint8_t mode;
	uint8_t arm;
} ControllerCommands;

typedef struct {
	float ax;
	float ay;
	float az;
	float vx;
	float vy;
	float vz;
} Net;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU_INT_PIN GPIO_PIN_7
#define MPU6050_ADDR (0x68 << 1)
#define g 9.80571


#define MAX_ANGLE_DEG 45.0f
#define MAX_PITCH_RATE 250.0f
#define MAX_ROLL_RATE 250.0f
#define MAX_YAW_RATE 180.0f
#define MAX_ALTITUDE_RATE 3.0f


#define PID_OUTPUT_THRESHOLD  0.02f  // throttle units (adjust as needed)
#define VELOCITY_THRESHOLD    0.05f  // m/s
#define MIN_THROTTLE 0.3f
#define MAX_THROTTLE 0.7f
#define ADJUSTMENT_STEP 0.01f

#define AZ_DEADBAND 0.1f  // Tune this threshold experimentally
#define SAMPLE_WINDOW 300
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DroneState state = STATE_SENSOR_INIT;
FlightMode mode = MODE_STABILIZED;

MPU_init MPU_Init = {
	.ADDRESS = (0x68 << 1),
	.INT_LEVEL = 0,
	.INT_OPEN = 0,
	.LATCH_INT_EN = 0,
	.INT_RD_CLEAR = 1,
	.DATA_RDY_EN = 1,
	.DLPF_CFG = 1,
	.SMPLRT_DIV = 0,
	.FS_SEL = 0,
	.AFS_SEL = 0,
	.CLKSEL = 0,
	.SLEEP = 0
	};

BMP_init BMP_Init = {
	.osrs_t   = 1,  // x1
	.osrs_p   = 5,  // x16
	.mode     = 3,  // Normal mode
	.t_sb     = 0,  // 0.5ms
	.filter   = 2,  // x4 IIR
	.spi3w_en = 0   // 4-wire SPI
	};

MPU_data_raw MPU_Data_Raw = {0};
MPU_data MPU_Data = {0};
MPU_bias MPU_Bias = {0};

BMP_data_raw BMP_Data_Raw = {0};
BMP_data BMP_Data = {0};
BMP_calib BMP_Calib = {0};

uint8_t who_am_i = 0;

uint32_t last_timestamp = 0;
float data_freq_hz = 0;

static float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
static float altitude_rate = 0;

float alpha_mpu = 0.99f;
float alpha_bmp = 0.1f;

uint32_t last_time = 0;

float hover_throttle = 0.3f;

float az_buffer[SAMPLE_WINDOW];
int az_index = 0;

// pid controllers

PIDController pid_pitch = {0};
PIDController pid_roll = {0};
PIDController pid_yaw = {0};
PIDController pid_altitude = {0};

// pid coeff

// Angle mode PID coefficients (for controlling angles)
PIDCoefficients pid_pitch_angle_coeff = {0.1f, 0.001f, 0.001f};
PIDCoefficients pid_roll_angle_coeff  = {0.1f, 0.001f, 0.001f};
PIDCoefficients pid_yaw_angle_coeff   = {0.1f, 0.001f, 0.001f};
PIDCoefficients pid_altitude_coeff = {1.0f, 0.0f, 0.0f};

// Rate mode PID coefficients (for controlling angular rates)
PIDCoefficients pid_pitch_rate_coeff = {0.01f, 0.0f, 0.0f};
PIDCoefficients pid_roll_rate_coeff  = {0.01f, 0.0f, 0.0f};
PIDCoefficients pid_yaw_rate_coeff   = {0.01f, 0.0f, 0.0f};
PIDCoefficients pid_altitude_rate_coeff = {0.1f, 0.1f, 0.1f};

// Controller Commands

ControllerCommands controller_commands = {0};

// Net position w.r.t earth

Net net = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}

// Assuming net.az and net.vz are floats or similar types
void update_net_values(Net* net) {
    static float last_az = 0;  // persists between calls

    // Check for sign change in az
    if ((last_az * net->az) < 0) {  // Sign changed
        net->vz = 0;                // Reset vz
    }

    // Update last_az for next iteration
    last_az = net->az;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

float update_accel_and_check_rest(float az, float vz) {
    az_buffer[az_index] = az;
    az_index = (az_index + 1) % SAMPLE_WINDOW;

    // Check once every SAMPLE_WINDOW samples
    if (az_index == 0) {
        bool all_within_deadband = true;
        for (int i = 0; i < SAMPLE_WINDOW; i++) {
            if (fabsf(az_buffer[i]) > AZ_DEADBAND) {
                all_within_deadband = false;
                break;
            }
        }

        if (all_within_deadband) {
            vz = 0.0f;  // Reset vertical velocity if we've been idle
        }
    }

    return vz;
}

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
  HAL_I2C_DeInit(&hi2c1);   // Ensure clean shutdown
  I2C_Unstick();            // Release SDA/SCL lines
  I2C_Force_Reset();        // Reset hardware block
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  float LSB_ACC = get_acc_lsb(MPU_Init.AFS_SEL);
  float LSB_GYR = get_gyr_lsb(MPU_Init.FS_SEL);

  enable_cycle_counter();
  int n= 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      switch (state)
      {
          case STATE_SENSOR_INIT:
        	  dma_read_enabled = 0;
              if (I2C_MPU6050_Init(&hi2c1, &MPU_Init) == HAL_OK){
            	  dma_read_enabled = 1;
            	  if (SPI_BMP280_Init(&hspi1, &BMP_Init, &BMP_Calib) == 1)
            	      state = STATE_SENSOR_CALIBRATION;
            	  }
              else {
            	  printf("%d\n", state);
                  state = STATE_ERROR;
              }
              break;

          case STATE_SENSOR_CALIBRATION:
              if (calibrate_accel_gyro(&MPU_Bias, &MPU_Data_Raw, 10000, LSB_ACC) == 1){
                  state = STATE_READY;
              }
              else
                  state = STATE_ERROR;
              break;

          case STATE_ARMING:
        	  break;

          case STATE_READY:
              if (mpu_data_ready){
				  n++;
				  //
				  float dt = get_dt_us(&last_time);
				  //
				  float acc_scale = 1 / LSB_ACC;
				  float gyr_scale = 1 / LSB_GYR;
				  MPU_Data.ax = (MPU_Data_Raw.ax - MPU_Bias.ax_bias) * acc_scale;
				  MPU_Data.ay = (MPU_Data_Raw.ay - MPU_Bias.ay_bias) * acc_scale;
				  MPU_Data.az = (MPU_Data_Raw.az - MPU_Bias.az_bias) * acc_scale;

				  MPU_Data.gx = (MPU_Data_Raw.gx - MPU_Bias.gx_bias) * gyr_scale;
				  MPU_Data.gy = (MPU_Data_Raw.gy - MPU_Bias.gy_bias) * gyr_scale;
				  MPU_Data.gz = (MPU_Data_Raw.gz - MPU_Bias.gz_bias) * gyr_scale;

				  // 2. Estimate pitch/roll from accelerometer (in degrees)
				  float acc_pitch = atan2f(MPU_Data.ay, sqrtf(MPU_Data.ax * MPU_Data.ax + MPU_Data.az * MPU_Data.az)) * 180.0f / M_PI;
				  float acc_roll = atan2f(-MPU_Data.ax, sqrtf(MPU_Data.ay * MPU_Data.ay + MPU_Data.az * MPU_Data.az)) * 180.0f / M_PI;

				  // 3. Complementary filter
				  pitch = alpha_mpu * (pitch + MPU_Data.gx * dt) + (1.0f - alpha_mpu) * acc_pitch;
				  roll  = alpha_mpu * (roll  + MPU_Data.gy * dt) + (1.0f - alpha_mpu) * acc_roll;

				  // 4. Integrate yaw from gyro Z
				  yaw += MPU_Data.gz * dt;

				  // 5. Optionally wrap yaw angle to [-180, 180]
				  if (yaw > 180.0f) yaw -= 360.0f;
				  if (yaw < -180.0f) yaw += 360.0f;


				  float pitch_rad = pitch * M_PI / 180.0f;
				  float roll_rad  = roll  * M_PI / 180.0f;

				  net.az = - MPU_Data.ax * sin(pitch_rad)
				  	  	  	 + MPU_Data.ay * sin(roll_rad) * cos(pitch_rad)
							 + MPU_Data.az * cos(roll_rad) * cos(pitch_rad)
							 - 1;

				  net.vz = update_accel_and_check_rest(net.az, net.vz);
				  net.vz += g * net.az * dt;

				  if (bmp_data_ready) {

					  BMP_Data.temperature = BMP280_compensate_temperature(BMP_Data_Raw.temperature, &BMP_Calib);  // in °C
					  BMP_Data.pressure = BMP280_compensate_pressure(BMP_Data_Raw.pressure, &BMP_Calib);           // in Pa
					  update_baro_rate();
//					  printf("temp - %.2f\n", BMP_Data.temperature);
					  bmp_data_ready = 0;
				  }

				  if (bmp_rx_ready) {  // Only start if previous read finished
					  bmp280_read_registers(&hspi1, 0xF7, 6);
				  }


				  altitude_rate =  alpha_bmp * baro_rate + (1 - alpha_bmp) * net.vz;
				  altitude_rate = net.vz;
//				  printf("baro      - %.03f\n", BMP_Data.pressure);
//				  printf("baro_rate - %.03f\n\n", baro_rate);
//
//				  printf("altitude      - %.03f\n", baro_altitude);
//				  printf("altitude rate - %.03f\n\n", altitude_rate);
				  switch (mode)
				  {
				  	  case MODE_TAKEOFF:
					  	  break;
				  	  case MODE_STABILIZED:
				  		  float desired_pitch         = controller_commands.Ly * MAX_ANGLE_DEG;
				  		  float desired_roll          = controller_commands.Lx * MAX_ANGLE_DEG;
				  		  float desired_yaw_rate      = controller_commands.Rx * MAX_YAW_RATE;
				  		  float desired_altitude_rate = controller_commands.Ry * MAX_ALTITUDE_RATE;

				  		  // ----- PITCH -----
						  update_pid(&pid_pitch, &pid_pitch_angle_coeff, desired_pitch, pitch, dt);
						  pid_pitch.output = clamp(pid_pitch.output, -1.0f, 1.0f);
						  // ----- ROLL -----
						  update_pid(&pid_roll,  &pid_roll_angle_coeff,  desired_roll,  roll,  dt);
						  pid_roll.output = clamp(pid_roll.output, -1.0f, 1.0f);
					      // ----- YAW -----
						  update_pid(&pid_yaw,   &pid_yaw_rate_coeff,   desired_yaw_rate,   MPU_Data.gz,   dt);
						  pid_yaw.output = clamp(pid_yaw.output, -1.0f, 1.0f);
						  // ----- ALT -----
						  update_pid(&pid_altitude,   &pid_altitude_rate_coeff,   desired_altitude_rate,   altitude_rate,   dt);
						  pid_altitude.output = clamp(pid_altitude.output, -1.0f, 1.0f);

//						  if (fabs(pid_altitude.output) < PID_OUTPUT_THRESHOLD) {  // system is stable
//						      if (altitude_rate > VELOCITY_THRESHOLD) {
//						          hover_throttle -= ADJUSTMENT_STEP;
//						      }
//						      else if (altitude_rate < -VELOCITY_THRESHOLD) {
//						          hover_throttle += ADJUSTMENT_STEP;
//						      }
//						      hover_throttle = clamp(hover_throttle, MIN_THROTTLE, MAX_THROTTLE);
//						  }

						  float throttle = hover_throttle + pid_altitude.output;

						  float motor_1 = throttle - pid_pitch.output - pid_roll.output + pid_yaw.output;  // Front Left  (CCW)
						  float motor_2 = throttle - pid_pitch.output + pid_roll.output - pid_yaw.output;  // Front Right (CW)
						  float motor_3 = throttle + pid_pitch.output + pid_roll.output + pid_yaw.output;  // Rear Right  (CCW)
						  float motor_4 = throttle + pid_pitch.output - pid_roll.output - pid_yaw.output;  // Rear Left   (CW)

						  float motors[4] = {motor_1, motor_2, motor_3, motor_4};

						  float min_motor = motors[0];
						  float max_motor = motors[0];

						  // Find min and max
						  for (int i = 1; i < 4; i++) {
						      if (motors[i] < min_motor) min_motor = motors[i];
						      if (motors[i] > max_motor) max_motor = motors[i];
						  }

						  // Shift if min < 0
						  if (min_motor < 0) {
						      for (int i = 0; i < 4; i++) {
						          motors[i] -= min_motor;  // shift all values up so min is 0
						      }
						      max_motor -= min_motor;      // adjust max accordingly
						  }

						  // Scale if max > 1
						  if (max_motor > 1) {
						      for (int i = 0; i < 4; i++) {
						          motors[i] /= max_motor;  // scale down to max 1
						      }
						  }

						  // Assign back
						  float motor_1_real = motors[0];
						  float motor_2_real = motors[1];
						  float motor_3_real = motors[2];
						  float motor_4_real = motors[3];

					  break;
				  	  case MODE_FREESTYLE:
				  		  break;

				  	  case MODE_AUTONOMOUS:
				  		  break;

				  	  case MODE_LANDING:
				  		  break;

				  	  case MODE_FAILSAFE:
				  		  break;
				  }
				  float loop_frequency = 1.0f / dt;
				  //printf("Loop frequency: %.2f Hz\n", loop_frequency);
				  //HAL_Delay(10);
				  mpu_data_ready = 0;
              }
              break;

          case STATE_DISARMING:
        	  break;

          case STATE_ERROR:
        	  printf("error\n");
        	  HAL_Delay(1000);
        	  break;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);
  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
