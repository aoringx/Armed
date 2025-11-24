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
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NHD_W_M 0x50

#define WII_IR_MAXSIZE_REGISTER 0x06
#define WII_IR_GAIN_REGISTER 0x08
#define WII_IR_GAINLIMIT_REGISTER 0x1A
#define WII_IR_MINSIZE_REGISTER 0x1B
#define WII_IR_CONTROL_REGISTER 0x30
#define WII_IR_OUTPUT_MODE_REGISTER 0x33
#define WII_IR_READ_ADDRESS 0x37

#define CAM_WIDTH 1024.0f // .0f decimal + float, prevent integer division
#define CAM_HEIGHT 768.0f
#define CAM_CENTER_X (CAM_WIDTH / 2.0f) // 512
#define CAM_CENTER_Y (CAM_HEIGHT / 2.0f) // 384

#define CAM_FOV_H 33.0f  // horizontal FOV in degrees
#define CAM_FOV_V 23.0f  // vertical FOV in degrees
#define DEG_PER_PIX_X (CAM_FOV_H / CAM_WIDTH)   // 33.0 / 1024.0 deg/pix
#define DEG_PER_PIX_Y (CAM_FOV_V / CAM_HEIGHT)  // 23.0 / 768.0 deg/pix

#define IR_W_M 0xB0 //0x84
#define IR_R_M 0xB1 //0x85

#define TARGET_SIZE 100.0f

/*
coordinate system

origin (0,0) is at the top left corner
x: increases from left-right (0 to 1023)
y: increases from top-bottom (0 to 767)
*/

#define FOCAL_LENGTH_PIXELS 1100.0f // temp estimate wiimote val till calibration
#define IRL_LED_SPACING_MM 130.0f // sample value of 1m=1000mm temporarily
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int hapticSwitch = 0;
int laserSwitch = 0;
int soundSwitch = 0;
char distanceDisplay[20] = "";
char hapticDisplay[20] = "";
char laserDisplay[20] = "";
char soundDisplay[20] = "";
int buzzPeriod = 99;
float distance = 0;

float x_center = 0;
float y_center = 0;
float pixel_offset_x = 0;
float pixel_offset_y = 0;
float angular_error_x = 0;
float angular_error_y = 0;
float dist_pix = 0;
float distance_mm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void NHD_Command_2(uint8_t cmd) {
    uint8_t buf[2] = {0xFE, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, NHD_W_M, buf, 2, 1000);
    HAL_Delay(2);
}

void NHD_Command_3(uint8_t cmd, uint8_t val) {
    uint8_t buf[3] = {0xFE, cmd, val};
    HAL_I2C_Master_Transmit(&hi2c1, NHD_W_M, buf, 3, 1000);
    HAL_Delay(2);
}

void displayOn() {
	NHD_Command_2(0x41);
}

void displayOff() {
	NHD_Command_2(0x42);
}

void clearScreen() {
	NHD_Command_2(0x51);
}

void cursorHome() {
	NHD_Command_2(0x46);
}

void underlineOn() {
	NHD_Command_2(0x47);
}

void underlineOff() {
	NHD_Command_2(0x48);
}

void blinkOn() {
	NHD_Command_2(0x4b);
}

void blinkOff() {
	NHD_Command_2(0x4c);
}

void setCursor(uint8_t position) {
	NHD_Command_3(0x45, position);
}

void setContrast(uint8_t contrast) {
	NHD_Command_3(0x52, contrast);
}

void setBrightness(uint8_t brightness) {
	NHD_Command_3(0x53, brightness);
}

void shiftLeft() {
	NHD_Command_2(0x55);
}

void shiftRight() {
	NHD_Command_2(0x56);
}


void writeString(char* data) {
	HAL_I2C_Master_Transmit(&hi2c1, NHD_W_M, (uint8_t*)data, strlen(data), 1000);
	HAL_Delay((float)strlen(data)/10);
}

void writeStringFirstLine(char* data) {
	setCursor(0x00);
	writeString(data);
}

void writeStringSecondLine(char* data) {
	setCursor(0x40);
	writeString(data);
}

void writeStringThirdLine(char* data) {
	setCursor(0x14);
	writeString(data);
}

void writeStringFourthLine(char* data) {
	setCursor(0x54);
	writeString(data);
}

void NHDInit() {
	displayOn();
	clearScreen();
	cursorHome();
	underlineOff();
	blinkOff();
	setContrast(40);
	setBrightness(8);
}

void setDistanceDislpay(float distance, char* distanceDisplay) {
	distance = fmodf(distance, 1000.0f);
	sprintf(distanceDisplay, "Distance: %.2f m", distance);
}

void setHapticDisplay(int hapticSwitch, char* hapticDisplay) {
	sprintf(hapticDisplay, "Haptics: %s", hapticSwitch ? "On" : "Off");
}

void setLaserDisplay(int laserSwitch, char* laserDisplay) {
	sprintf(laserDisplay, "Laser: %s", laserSwitch ? "On" : "Off");
}

void setSoundDisplay(int SoundSwitch, char* SoundDisplay) {
	sprintf(SoundDisplay, "Sound: %s", SoundSwitch ? "On" : "Off");
}

void updateBuzzPeriod(int period) {
	if (period < 0) {
		return;
	}
	__HAL_TIM_SET_AUTORELOAD(&htim2, period);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (period+1)/2);
}

void makeSound(int length) {
	if (length < 0 || !soundSwitch) {
		return;
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, buzzPeriod);
	HAL_Delay(length);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (buzzPeriod+1)/2);
}

void makeHaptic(int length) {
	if (length < 0) {
		return;
	}
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_Delay(length);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void stateInit() {
	hapticSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10);
	laserSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	soundSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);

	setDistanceDislpay(distance, distanceDisplay);
	setHapticDisplay(hapticSwitch, hapticDisplay);
	setLaserDisplay(laserSwitch, laserDisplay);
	setSoundDisplay(soundSwitch, soundDisplay);
}

void refreshDisplay() {
	clearScreen();
	writeStringFirstLine(distanceDisplay);
	writeStringSecondLine(hapticDisplay);
	writeStringThirdLine(laserDisplay);
	writeStringFourthLine(soundDisplay);
}

void calculateBuzzPeriod() {
	float pixel_offset = pixel_offset_x * pixel_offset_x + pixel_offset_y * pixel_offset_y;
	float pixel_offset_log = log10(pixel_offset - 10);
	buzzPeriod = 40 * pixel_offset_log / 5;
}

// temporary test xbee code begin

/*
permanent config
AP parameter (API Enable)
Set AP = 1 (API Mode without escapes)
Click "Write" button to save to XBee
*/

// code w/o xctu
void xbee_enable_api_mode() { // one time enable
  HAL_Delay(1100);
  char enter[] = "+++";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)enter, 3, 100);
  HAL_Delay(1100);
  
  char cmd1[] = "ATAP 1\r";  // Enable API mode
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd1, strlen(cmd1), 100);
  
  char cmd2[] = "ATWR\r";    // Write to flash (permanent)
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd2, strlen(cmd2), 100);
  
  char cmd3[] = "ATCN\r";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd3, strlen(cmd3), 100);
  
  // Reboot XBee for changes to take effect
}

uint8_t calculate_checksum(uint8_t* data, int length) {
  uint16_t sum = 0;
  for(int i = 0; i < length; i++) {
      sum += data[i];
  }
  return 0xFF - (sum & 0xFF);
}

void xbee_set_high_api() {
  // Local AT Command frame (0x08) - simpler, for local XBee
  uint8_t frame[] = {
    0x7E,        // Start delimiter
    0x00, 0x05,  // Length (5 bytes)
    0x08,        // Frame Type: AT command
    0x01,        // Frame ID (1 = want response)
    'D', '0',    // AT Command: D0
    0x05,        // Parameter: 5 = digital output high
    0x00         // Checksum (will calculate)
  };
  
  // calculate checksum (from byte 3 to end-1)
  frame[8] = calculate_checksum(&frame[3], 5);
  
  // send frame
  HAL_UART_Transmit(&hlpuart1, frame, 9, 100);
}

void xbee_set_low_api() {
  uint8_t frame[] = {
    0x7E,        // start delimiter
    0x00, 0x05,  // length (5 bytes)
    0x08,        // frame Type: AT command
    0x01,        // frame ID
    'D', '0',    // AT Command: D0
    0x04,        // parameter: 4 = digital output low
    0x00         // checksum (will calculate)
  };
  
  frame[8] = calculate_checksum(&frame[3], 5);
  HAL_UART_Transmit(&hlpuart1, frame, 9, 100);
}

// temporary test xbee code end

void xbee_set(char cmd1[] = "ATD0 4\r") {
  uint8_t rxBuffer[10];

  HAL_Delay(1100);
  char enter[] = "+++";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)enter, 3, 100);
  HAL_Delay(1100);
  HAL_UART_Receive(&hlpuart1, rxBuffer, 3, 500);  // Read "OK\r" -> check this

  // D0 = 5 (digital output high)
  // D0 = 4 (digital output low)
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd1, strlen(cmd1), 100);
  // HAL_UART_Receive(&hlpuart1, rxBuffer, 3, 500);  // Read response

  // apply
  char cmd2[] = "ATAC\r";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd2, strlen(cmd2), 100);
  // HAL_UART_Receive(&hlpuart1, rxBuffer, 3, 500);  // Read response

  // exit
  char cmd3[] = "ATCN\r";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd3, strlen(cmd3), 100);
}

void topLEDOn(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void topLEDOff(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void bottomLEDOn(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

void bottomLEDOff(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}

void leftLEDOn(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

void leftLEDOff(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

void rightLEDOn(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
}

void rightLEDOff(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
}

void soundOn() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void soundOff() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void hapticOn() {
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	xbee_set_high_api();
  // xbee_set("ATD0 5\r");
}

void hapticOff() {
//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  xbee_set_low_api();
	// xbee_set(); // default low
}

void laserOn() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void laserOff() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void updateHaptic() {
	if (!hapticSwitch){
		return;
	}
	if (pixel_offset_x <= TARGET_SIZE && pixel_offset_x >= -TARGET_SIZE && pixel_offset_y <= TARGET_SIZE && pixel_offset_y >= -TARGET_SIZE) {
		hapticOn();
	} else {
		hapticOff();
	}
}

void LEDInit() {
	leftLEDOff();
	rightLEDOff();
	topLEDOff();
	bottomLEDOff();
}

void updateLED() {
	if (pixel_offset_x > TARGET_SIZE) {
		rightLEDOn();
		leftLEDOff();
	} else if (pixel_offset_x < -TARGET_SIZE) {
		leftLEDOn();
		rightLEDOff();
	} else {
		leftLEDOff();
		rightLEDOff();
	}
	if (pixel_offset_y > TARGET_SIZE) {
		topLEDOn();
		bottomLEDOff();
	} else if (pixel_offset_y < -TARGET_SIZE) {
		bottomLEDOn();
		topLEDOff();
	} else {
		bottomLEDOff();
		topLEDOff();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == GPIO_PIN_10) {
		hapticSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10);
		setHapticDisplay(hapticSwitch, hapticDisplay);
	} else if (pin == GPIO_PIN_12) {
		laserSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
		setLaserDisplay(laserSwitch, laserDisplay);
		if (laserSwitch) {
			laserOn();
		} else {
			laserOff();
		}
	} else if (pin == GPIO_PIN_14) {
		soundSwitch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
		setSoundDisplay(soundSwitch, soundDisplay);
		if (soundSwitch) {
			soundOn();
		} else {
			soundOff();
		}
	}
	refreshDisplay();
}

void wiiCameraInit(){
	uint8_t buf[14] = {WII_IR_CONTROL_REGISTER};
	buf[1] = 0x01; // enable
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Configure Output Format
	buf[0] = WII_IR_OUTPUT_MODE_REGISTER;
	buf[1] = 0x33; // Medium Output Mode
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Configure MAXSIZE Register
	buf[0] = WII_IR_MAXSIZE_REGISTER;
	buf[1] = 0x90; // Ranges from 0x00-0xFF, Higher means more sensitive to IR light (but more noise
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Configure GAIN Register
	buf[0] = WII_IR_GAIN_REGISTER;
	buf[1] = 0xC0; // Ranges from 0x00-0xFF (Increasing means decreasing IR filter)
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Configure GAINLIMIT Register
	buf[0] = WII_IR_GAINLIMIT_REGISTER;
	buf[1] = 0x40; // Ranges from 0x00-0xFF (Increasing means decreasing sensitivity and false positives)
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Configure MINSIZE Register
	buf[0] = WII_IR_MINSIZE_REGISTER;
	buf[1] = 0x03; // Ranges from 0x00-0xFF (Decides minimum blobsize: typically valued 3-5)
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);

	// Start Wii Camera Data Collection
	buf[0] = 0x30;
	buf[1] = 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 2, 1000);
}

void wiiCameraRead(){
	  uint8_t buf[14];
	  buf[0] = WII_IR_READ_ADDRESS; // Address to read output data
	  HAL_I2C_Master_Transmit(&hi2c1, IR_W_M, &buf[0], 1, 1000);
	  uint8_t data[12];
	  HAL_I2C_Master_Receive(&hi2c1, IR_R_M, &data[0], 12, 1000);

	  int16_t coord[8]; // Pairs of two are the x and y coordinate of each point

	  for(int i = 0; i < 4; i++) {
		// max 10 bits but with a 4:3 aspect ratio,
		// assuming horizontal resolution is 1024,
		// vertical resolution is 1024 * 3/4 = 768
		// 128 x 96 -> scaled up to 1024 x 768
		  coord[0+2*i] = ((data[2+3*i] & 0x30) << 4) | data[0+3*i];
		  coord[1+2*i] = ((data[2+3*i] & 0xC0) << 2) | data[1+3*i];
	  }

	  uint8_t valid_blobs = 0;
	  if(coord[0] < 1023) valid_blobs++;
	  if(coord[1] < 1023) valid_blobs++;
	  if(coord[2] < 1023) valid_blobs++;
	  if(coord[3] < 1023) valid_blobs++;

	  if (valid_blobs == 4){
			// CALCULATIONS START
			// calculate centroid (x_center, y_center)
			x_center = (coord[0] + coord[2] + coord[4] + coord[6]) / 4.0f; // floating pnt division
			y_center = (coord[1] + coord[3] + coord[5] + coord[7]) / 4.0f;

			// calculate angular offset
			pixel_offset_x = x_center - CAM_CENTER_X;
			pixel_offset_y = y_center - CAM_CENTER_Y;
			// (y_center - CAM_CENTER_Y) means positive Y is down
			// (CAM_CENTER_Y - y_center) means positive Y is up
			// depends on orientation
			angular_error_x = pixel_offset_x * DEG_PER_PIX_X;
			angular_error_y = pixel_offset_y * DEG_PER_PIX_Y;

			// calculate distance/depth
			// arbitrarily chosen to be between blob0 and 1
			// fabsf() is floating-point absolute value from math.h
			dist_pix = fabsf((float)coord[2] - (float)coord[0]);

			if (dist_pix > 0) // div by zero prevent
			{
			distance_mm = (FOCAL_LENGTH_PIXELS * IRL_LED_SPACING_MM) / dist_pix;
			}

			// send to LPUART1/Xbee (connect to wherever ig)
//			printf("X:%.2f,Y:%.2f,D:%.1f\r\n", angular_error_x, angular_error_y, distance_mm);

			distance = distance_mm / 1000;


		  // PROBABLY want an if/else for when no target is detected
		  // have to test what the output is if not detected
		  // also have to consider cases for <4 blobs detected?

		  // CALCULATIONS DONE
	  } else {
//		  printf("No Target\r\n");
	  }

	  for(int i = 0; i < 4; i++) {
//		  printf("Coordinate %d: (%d, %d) \r\n", i, coord[0+2*i], coord[1+2*i]);
	  }

	  /* Wii IR Data Read Competed */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		wiiCameraRead();
		setDistanceDislpay(distance, distanceDisplay);
		calculateBuzzPeriod();
		updateBuzzPeriod(buzzPeriod);
		updateLED();
		updateHaptic();
		refreshDisplay();
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  NHDInit();
  stateInit();
  refreshDisplay();
  updateBuzzPeriod(buzzPeriod);
  HAL_TIM_Base_Start_IT(&htim3);
  wiiCameraInit();
  LEDInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(10000);
//	  makeSound(2000);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00100E3B;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE11
                           PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
