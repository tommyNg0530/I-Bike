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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t x_gyr_MSB,y_gyr_MSB, z_gyr_MSB, x_gyr_LSB, y_gyr_LSB, z_gyr_LSB;
uint8_t x_acc_MSB,y_acc_MSB, z_acc_MSB, x_acc_LSB, y_acc_LSB, z_acc_LSB;
int16_t gyr_x,gyr_y,gyr_z;
int16_t acc_x,acc_y,acc_z;
uint8_t Acc = 0x11;
uint8_t Gyr = 0x15;
char gyr_x_display[6];
char gyr_y_display[6];
char gyr_z_display[6];
char acc_x_display[6];
char acc_y_display[6];
char acc_z_display[6];
char speed_display[6];
char distance_display[6];
char pwm_value[10] = "Initial";
int motor_level_counter = 0;	   // User can set the initial level (plz check the size )
int derailuer_init_direction = 0;  // 0 for ascending level init, 1 for descending init

uint8_t rxData = 1;
volatile uint32_t last_interrupt_time = 0;
volatile uint32_t time_between_interrupts = 0;
volatile uint32_t rpm = 0;
volatile uint32_t rotation_count = 0;
float distance_traveled = 0;
const float wheel_radius = 0.3;
float previous_distance = 0;
char message[50] = "Speed: 0 km/h     Distance: 0 m\r\n";
int msgCounter = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Check if the received data is '0' or '1'
  if(rxData == '1')
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn B0 on
  }
  else if(rxData == '0')
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn B0 off
  }

  // Start the next receive operation
  HAL_UART_Receive_IT(&huart2, &rxData, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7)
  {
	// Get the current time and calculate the time between this interrupt and the last one
    uint32_t current_time = HAL_GetTick();
    time_between_interrupts = current_time - last_interrupt_time;

    // Update the last_interrupt_time to the current time for the next interrupt
    last_interrupt_time = current_time;

    // Calculate the RPM based on the time between interrupts
    rpm = 60000 / time_between_interrupts; // RPM = 60,000 / time_between_interrupts

    // Increment the rotation count
    rotation_count++;
  }
}

void check_bike_stopped()
{
  static uint32_t last_rotation_count = 0;
  static uint32_t last_check_time = 0;

  uint32_t current_time = HAL_GetTick();

  if (current_time - last_check_time > 2000) // Check every 3 seconds
  {
    if (rotation_count == last_rotation_count) // If rotation count has not changed, bike has stopped
    {
      rpm = 0;
      sprintf(message, "Speed: 0 km/h     Distance: %d m\r\n", (int)distance_traveled);
    }

    last_rotation_count = rotation_count;
    last_check_time = current_time;
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
  MX_FSMC_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  LCD_INIT();
  max_init(8);  //Initialize max7219 led control module, 15 as brightness

  /*
  //For BMI160 gyroscope Dev address= 0x68 (connected to GND)
  struct bmi160_dev bmi160;
  struct bmi160_sensor_data accel;
  struct bmi160_sensor_data gyro;
  */

  //bmi160_soft_reset(&bmi160);
  //bmi160_init(&bmi160);

  HAL_UART_Receive_IT(&huart2,&rxData,1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //Display of gyroscope x y z
	  LCD_DrawString(20, 30, "Gyr_x:");
	  LCD_DrawString(20, 60, "Gyr_y:");
	  LCD_DrawString(20, 90, "Gyr_z:");
	  LCD_DrawString(20, 120, "Acc_x:");
	  LCD_DrawString(20, 150, "Acc_y:");
	  LCD_DrawString(20, 180, "Acc_z:");
	  LCD_DrawString(20, 180, "Acc_z:");
	  LCD_DrawString(20, 210, "Ride lv:");
	  //LCD_DrawString(20, 240, "Turn:");
	  LCD_DrawString(20, 240, "Motor:");



	  //gyroscope
	  HAL_I2C_Mem_Write(&hi2c2, 0x69<<1, 0x7e, 1, &Gyr, 1 ,100);      //activate gyroscope
	  HAL_Delay(100);

	  //For debug purpose (Check work mode of bmi160)
	  uint8_t work_mode = 0;
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X03, 1, &work_mode, 1 ,100);

	  uint8_t gyr_value_setting = 0;
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0x43, 1, &gyr_value_setting, 1 ,100); //Read  gyr_value_setting

	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X0c, 1, &x_gyr_LSB, 1 ,100); //Read gyro x
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X0d, 1, &x_gyr_MSB, 1 ,100);
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X0e, 1, &y_gyr_LSB, 1 ,100); //Read gyro y
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X0f, 1, &y_gyr_MSB, 1 ,100);
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X10, 1, &z_gyr_LSB, 1 ,100); //Read gyro z
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X11, 1, &z_gyr_MSB, 1 ,100);


	  uint16_t whole_gyr_x = ((x_gyr_MSB | 0x0000) << 8) | (x_gyr_LSB & 0xffff);
	  uint16_t whole_gyr_y = ((y_gyr_MSB | 0x0000) << 8) | (y_gyr_LSB & 0xffff);
	  uint16_t whole_gyr_z = ((z_gyr_MSB | 0x0000) << 8) | (z_gyr_LSB & 0xffff);

	  gyr_x = (signed short)whole_gyr_x*3.14/180.0;
      gyr_y = (signed short)whole_gyr_y*3.14/180.0;
      gyr_z = (signed short)whole_gyr_z*3.14/180.0;

	  //Accelermeter
	  HAL_I2C_Mem_Write(&hi2c2, 0x69<<1, 0x7e, 1, &Acc, 1 ,100); //activate accelermeter
	  HAL_Delay(100);



	  /*
	   * HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X03, 1, &work_mode, 1 ,100);
	  uint8_t acc_value_setting = 0;
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0x43, 1, &acc_value_setting, 1 ,100); //Read  acc_value_setting
	  */
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X12, 1, &x_acc_LSB, 1 ,100); //Read acc x
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X13, 1, &x_acc_MSB, 1 ,100);
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X14, 1, &y_acc_LSB, 1 ,100); //Read acc y
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X15, 1, &y_acc_MSB, 1 ,100);
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X16, 1, &z_acc_LSB, 1 ,100); //Read acc z
	  HAL_I2C_Mem_Read(&hi2c2, 0x69<<1, 0X17, 1, &z_acc_MSB, 1 ,100);

	  uint16_t whole_acc_x = ((x_acc_MSB | 0x0000) << 8) | (x_acc_LSB & 0xffff);
	  uint16_t whole_acc_y = ((y_acc_MSB | 0x0000) << 8) | (y_acc_LSB & 0xffff);
	  uint16_t whole_acc_z = ((z_acc_MSB | 0x0000) << 8) | (z_acc_LSB & 0xffff);

	  //Accelermeter data precision (-10 to +10)
	  acc_x = (signed short)whole_acc_x/1638.40;
	  acc_y = (signed short)whole_acc_y/1638.40;
	  acc_z = (signed short)whole_acc_z/1638.40;


	  sprintf(gyr_x_display, "%d", gyr_x);	//Convert all
	  sprintf(gyr_y_display, "%d", gyr_y);
	  sprintf(gyr_z_display, "%d", gyr_z);
	  sprintf(acc_x_display, "%d", acc_x);
	  sprintf(acc_y_display, "%d", acc_y);
	  sprintf(acc_z_display, "%d", acc_z);
	  LCD_Clear(90,30, 50,  16, 0xffff);
	  LCD_Clear(90,60, 50,  16, 0xffff);
	  LCD_Clear(90,90, 50,  16, 0xffff);
	  LCD_Clear(90,120, 50, 16, 0xffff);
	  LCD_Clear(90,150, 50, 16, 0xffff);
	  LCD_Clear(90,180, 50, 16, 0xffff);
	  LCD_DrawString(90, 30,  gyr_x_display);
	  LCD_DrawString(90, 60,  gyr_y_display);
	  LCD_DrawString(90, 90,  gyr_z_display);
	  LCD_DrawString(90, 120, acc_x_display);
	  LCD_DrawString(90, 150, acc_y_display);
	  LCD_DrawString(90, 180, acc_z_display);

	  //Determine turn left (-), right(+), none
	  char turn_direction[10];
	  int threshold_acc_z = 20; // Only trigger when greater than threshold value
	  if (gyr_z > threshold_acc_z){
		  strcpy( turn_direction, "Right");
		  //write_char(26,1);
		  scroll_char(26,500,'R');
		  max_clear();




	  }else if(gyr_z < -threshold_acc_z && gyr_z != 0){
		  strcpy( turn_direction, "Left");
		  //write_char(27,1);
		  scroll_char (27,500,'L');
		  max_clear();


	  }else{
		  strcpy( turn_direction, "None");
		  max_clear();



	  }

	  //Determine riding situation to switch derailleur [Based on acc's z-axis and x-axis]
	  //Installation of the bmi160 doesn't affect this part, just need to confirm x is pointing front of the bicycle

	  char incline_lv[10];
	  //  Non-Horizontal level
	  if (abs(acc_z) < 10 && acc_x != 0 ){

		  // +ve inclination level (down hill)
		  if (acc_x > 0){
			  switch(acc_x){
				  case 1:
					  strcpy( incline_lv, "DH 1");
					  break;
				  case 2:
					  strcpy( incline_lv, "DH 2");
					  break;
				  case 3:
					  strcpy( incline_lv, "DH 3");
					  break;
				  case 4:
					  strcpy( incline_lv, "DH 4");
					  break;
				  case 5:
					  strcpy( incline_lv, "DH 5");
					  break;
				  case 6:
					  strcpy( incline_lv, "DH 6");
					  break;
				  case 7:
					  strcpy( incline_lv, "DH 7");
					  break;
			  }
		  }
		  // -ve inclination  (up hill)
		  else{
			  switch(abs(acc_x)){
				  case 1:
					  strcpy( incline_lv, "UH 1");
					  break;
				  case 2:
					  strcpy( incline_lv, "UH 2");
					  break;
				  case 3:
					  strcpy( incline_lv, "UH 3");
					  break;
				  case 4:
					  strcpy( incline_lv, "UH 4");
					  break;
				  case 5:
					  strcpy( incline_lv, "UH 5");
					  break;
				  case 6:
					  strcpy( incline_lv, "UH 6");
					  break;
				  case 7:
					  strcpy( incline_lv, "UH 7");
					  break;
			  }
		  }
	  }
	  //  Horizontal level
	  else{

		  //Check derailleur, if is not in horizontal level, change it to horizontal level suitable level slowly
		  /*
		  if (){

		  }
		  */
		  strcpy( incline_lv, "Horizontal");

	  }
	  LCD_Clear(90,210, 100, 16, 0xffff);
	  LCD_DrawString(90, 210, incline_lv);

	  //LCD_DrawString(90, 240, turn_direction);



	  //Motor control testing
	  GPIO_PinState PA_0;
	  PA_0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  int pwm_value_list[] = {195,190,185,180,175,170,165,160,155,150,145,140,135,130,125,120,115,110,105,100,95,90,85,80,75,70};
	  int num_level =  sizeof(pwm_value_list)/sizeof(pwm_value_list[0]);
	  if (  PA_0 == GPIO_PIN_SET){

		  //check counter
		  if (motor_level_counter < num_level && motor_level_counter >= 0){

			  	char int_str[20];
			  	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm_value_list[motor_level_counter]);
			  	sprintf(int_str, "%d", pwm_value_list[motor_level_counter]);
			  	strcpy(pwm_value, int_str);
			  	HAL_Delay(100);


			  //Update level counter [For level change]
			  if (derailuer_init_direction == 0){
				motor_level_counter++;
			  }else{
				motor_level_counter--;
			  }
		  }
		  //If reach maximum level reset the direction [Counter reset and reverse direction]
		  else{
			  if (derailuer_init_direction == 0){
				  derailuer_init_direction = 1;
				  motor_level_counter = num_level-2;  //set the motor level to max level - 1
			  }else{
				  derailuer_init_direction = 0;
				  motor_level_counter = 1;  //set the motor level to lowest level + 1
			  }
		  }
	  }
	  LCD_Clear(90,240, 100, 16, 0xffff);
	  LCD_DrawString(90, 240, pwm_value);

	  // speed and distance calculation
	  float wheel_circumference = 2 * 3.14159 * wheel_radius; // Replace 0.5 with the actual radius of your wheel
	  float speed = rpm * wheel_circumference / 60;
	  float speed_km_h = speed * 3.6;
	  distance_traveled = wheel_circumference * rotation_count;

//	  if (msgCounter>=5){
	  /* Update the message to send over UART if the distance or speed has changed */
	  if (distance_traveled > previous_distance && (int)speed_km_h!=0 ) {
		  sprintf(message, "Speed: %d km/h     Distance: %d m\r\n", (int)speed_km_h, (int)distance_traveled);
		  LCD_DrawString(20, 270, "RPM: ");
		  LCD_DrawString(20, 290, "Distance: ");
		  sprintf(speed_display, "%d", (int)speed_km_h);
		  sprintf(distance_display, "%d", (int)distance_traveled);
		  LCD_Clear(90,270, 50, 16, 0xffff);
		  LCD_Clear(90,290, 50, 16, 0xffff);
		  LCD_DrawString(90, 270, speed_display);
		  LCD_DrawString(90, 290, distance_display);
	  }

	  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
	  msgCounter=0;
//	  }
	  previous_distance = distance_traveled;


	  check_bike_stopped();
//	  msgCounter += 1;
//	  HAL_delay(500);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 449;
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
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Max_din_Pin|Max_cs_Pin|Max_clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Max_din_Pin Max_cs_Pin Max_clk_Pin */
  GPIO_InitStruct.Pin = Max_din_Pin|Max_cs_Pin|Max_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
