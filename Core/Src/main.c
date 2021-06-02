 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_uros/options.h>

#include <std_msgs/msg/header.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/point_stamped.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STRING_BUFFER_LEN 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // micro-ROS configuration
	  void handle_encoders(int *enc_r, int *enc_l, double right_wheel_vel, double left_wheel_vel){
			  *enc_r = (int) right_wheel_vel * 360;
			  *enc_l = (int) left_wheel_vel * 360;
	  }

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart3,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rcl_publisher_t odom_publisher;
	  rcl_publisher_t tf_publisher;
	  rcl_subscription_t cmd_vel_subscriber;

	  geometry_msgs__msg__Twist incoming_cmd_vel;
	  nav_msgs__msg__Odometry outcoming_odom;
	  geometry_msgs__msg__PointStamped outcoming_tf;

	  rcl_allocator_t allocator = rcl_get_default_allocator();
	  rclc_support_t support;
	  rcl_node_t node;

	  int seq_no = 0;
	  int seq_no_prev = 0;

	  double pos_x = 0.0;
	  double pos_y = 0.0;
	  double pos_z = 0.0;

	  double transform_x = 0.0;
	  double transform_y = 0.0;
	  double transform_z = 0.0;

	  double transform_phi = 0.0;

	  double or_x = 1.0;
	  double or_y = 0.0;
	  double or_z = 0.0;
	  double or_w = 1.0;

	  double lin_vel_x = 0.0;
	  double lin_vel_y = 0.0;
	  double lin_vel_z = 0.0;

	  double ang_vel_x = 0.0;
	  double ang_vel_y = 0.0;
	  double ang_vel_z = 0.0;

	  double left_wheel_vel = 0.0;
	  double right_wheel_vel = 0.0;

	  double radius = 0.5;
	  double width = 1.0;

	  int enc_l = 0;
	  int enc_r = 0;


	  void odom_tf_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
	  {
	  	(void) last_call_time;

	  	if (timer != NULL) {

	  		transform_phi = (enc_r / 360 - enc_l / 360) * radius / width;
	  		transform_x = (enc_r / 360 + enc_l / 360) / 2 * radius * cos(transform_phi);
	  		transform_y = (enc_r / 360 + enc_l / 360) / 2 * radius * sin(transform_phi);

	  		pos_x += transform_x;
	  		pos_y += transform_y;

	  		or_x = or_x * cos(ang_vel_z) - or_y * sin(ang_vel_z);
	  		or_y = or_x * sin(ang_vel_z) + or_y * cos(ang_vel_z);

	  		seq_no_prev = seq_no;
	  		outcoming_odom.child_frame_id.data = sprintf(outcoming_odom.child_frame_id.data, "%d", seq_no_prev);
	  	    outcoming_odom.child_frame_id.size = strlen(outcoming_odom.child_frame_id.data);

	  		seq_no++;
	  		outcoming_odom.header.frame_id.data = sprintf(outcoming_odom.header.frame_id.data, "%d", seq_no);
	  		outcoming_odom.header.frame_id.size = strlen(outcoming_odom.header.frame_id.data);

	  		struct timespec ts;
	  		clock_gettime(CLOCK_REALTIME, &ts);
	  		outcoming_odom.header.stamp.sec = ts.tv_sec;
	  		outcoming_odom.header.stamp.nanosec = ts.tv_nsec;

	  		outcoming_odom.twist.twist.linear.x = lin_vel_x;
	  		outcoming_odom.twist.twist.linear.y = lin_vel_y;
	  		outcoming_odom.twist.twist.linear.z = lin_vel_z;

	  		outcoming_odom.twist.twist.angular.x = ang_vel_x;
	  		outcoming_odom.twist.twist.angular.y = ang_vel_y;
	  		outcoming_odom.twist.twist.angular.z = ang_vel_z;

	  		outcoming_odom.pose.pose.orientation.x = or_x;
	  		outcoming_odom.pose.pose.orientation.y = or_y;
		  	outcoming_odom.pose.pose.orientation.z = 0.0;
		  	outcoming_odom.pose.pose.orientation.w = 1.0;

	  		outcoming_odom.pose.pose.position.x = pos_x;
	  		outcoming_odom.pose.pose.position.y = pos_y;
	  	    outcoming_odom.pose.pose.position.z = 0.0;

	  		for (int i =0; i<36; ++i) {
	  			outcoming_odom.pose.covariance[i] = 0;
	  			outcoming_odom.twist.covariance[i] = 0;
	  		}

	  		outcoming_tf.header.frame_id.data = sprintf(outcoming_odom.header.frame_id.data, "%d", seq_no);
	  	    outcoming_tf.header.frame_id.size = strlen(outcoming_odom.header.frame_id.data);
	  	    outcoming_tf.header.stamp.sec = ts.tv_sec;
	  	  	outcoming_tf.header.stamp.nanosec = ts.tv_nsec;
	  	    outcoming_tf.point.x = transform_x;
	  	    outcoming_tf.point.y = transform_y;
	  	    outcoming_tf.point.z = 0.0;

	  		enc_l = enc_r = lin_vel_x = ang_vel_z = 0.0;
	  		rcl_publish(&odom_publisher, (const void*)&outcoming_odom, NULL);
	  		rcl_publish(&tf_publisher, (const void*)&outcoming_tf, NULL);
	  	}
	  }

	  void cmd_vel_subscription_callback(const void * msgin)
	  {
	  	geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;

	  	lin_vel_x = msg->linear.x;
	  	lin_vel_y = msg->linear.y;
	  	lin_vel_z = msg->linear.z;

	  	ang_vel_x = msg->angular.x;
	  	ang_vel_y = msg->angular.y;
	  	ang_vel_z = msg->angular.z;

	  	right_wheel_vel = (lin_vel_x + ang_vel_z * width / 2) / radius;
	  	left_wheel_vel = (lin_vel_x - ang_vel_z * width / 2) / radius;

	  	handle_encoders(&enc_r, &enc_l, right_wheel_vel, left_wheel_vel);
	  }

	  rclc_support_init(&support, 0, NULL, &allocator);
	  rclc_node_init_default(&node, "project_node", "", &support);

	  rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
	  rclc_publisher_init_default(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped), "/tf");
	  rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

	  rcl_timer_t timer = rcl_get_zero_initialized_timer();
	  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), odom_tf_timer_callback);

	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  rclc_executor_init(&executor, &support.context, 3, &allocator);

	  unsigned int rcl_wait_timeout = 1000;   // in ms
	  rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
	  rclc_executor_add_timer(&executor, &timer);
	  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &incoming_cmd_vel, &cmd_vel_subscription_callback, ON_NEW_DATA);

	  char outcoming_odom_header_buffer[STRING_BUFFER_LEN];
	  outcoming_odom.header.frame_id.data = outcoming_odom_header_buffer;
	  outcoming_odom.header.frame_id.capacity = STRING_BUFFER_LEN;

	  char outcoming_odom_child_buffer[STRING_BUFFER_LEN];
	  outcoming_odom.child_frame_id.data = outcoming_odom_child_buffer;
	  outcoming_odom.child_frame_id.capacity = STRING_BUFFER_LEN;

	  char outcoming_tf_buffer[STRING_BUFFER_LEN];
	  outcoming_tf.header.frame_id.data = outcoming_tf_buffer;
	  outcoming_tf.header.frame_id.capacity = STRING_BUFFER_LEN;

	  for(;;)
	  {
		  rclc_executor_spin(&executor);
	  }

	  rcl_publisher_fini(&odom_publisher, &node);
	  rcl_publisher_fini(&tf_publisher, &node);
	  rcl_subscription_fini(&cmd_vel_subscriber, &node);
	  rcl_node_fini(&node);

  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
