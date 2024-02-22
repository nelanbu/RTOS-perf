/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdbool.h>
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TAKE_MEASUREMENT(x)     (x = DWT->CYCCNT)
#define NUM_SAMPLES             (100000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId initHandle;
osThreadId task1Handle;
osThreadId task2Handle;
osSemaphoreId BinarySemHandle;
/* USER CODE BEGIN PV */
bool shuffle = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void initTask(void const * argument);
void StartTask(void const * argument);

/* USER CODE BEGIN PFP */
void DWT_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
 /* Implement your write code here, this is used by puts and printf for example */
    for(int i = 0 ; i < len ; ++i)
    {
        ITM_SendChar((*ptr++));
    }

    return len;
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
  DWT_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem */
  osSemaphoreDef(BinarySem);
  BinarySemHandle = osSemaphoreCreate(osSemaphore(BinarySem), 1);

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
  /* definition and creation of init */
  osThreadDef(init, initTask, osPriorityHigh, 0, 256);
  initHandle = osThreadCreate(osThread(init), NULL);

  /* definition and creation of task1 */
  osThreadDef(task1, StartTask, osPriorityNormal, 0, 256);
  task1Handle = osThreadCreate(osThread(task1), NULL);

  /* definition and creation of task2 */
  osThreadDef(task2, StartTask, osPriorityNormal, 0, 256);
  task2Handle = osThreadCreate(osThread(task2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void DWT_Init()
{
    CoreDebug->DEMCR    |= CoreDebug_DEMCR_TRCENA_Msk; 
    DWT->CYCCNT         = 0;	
    DWT->CTRL           |= DWT_CTRL_CYCCNTENA_Msk; 
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_initTask */
/**
  * @brief  Function implementing the init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_initTask */
void initTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    while (true)
    {
        /* Store current priorities */
        UBaseType_t priority_init    = uxTaskPriorityGet(NULL);
        UBaseType_t priority_shuffle = uxTaskPriorityGet(task1Handle);

        /* Lower priority of "init" task to run task1 and task2 and measure the
         * overhead */
        uint32_t overhead_start = 0;
        uint32_t overhead_stop  = 0;
        shuffle = false;

        TAKE_MEASUREMENT(overhead_start);
        vTaskPrioritySet(NULL, priority_shuffle - 1);
        TAKE_MEASUREMENT(overhead_stop);

        /* Restore initial priority */
        vTaskPrioritySet(NULL, priority_init);

        /* Create new tasks that will shuffle semaphores */
        xTaskCreate((TaskFunction_t)StartTask, "task1", 256, NULL, priority_shuffle, &task1Handle);
        xTaskCreate((TaskFunction_t)StartTask, "task2", 256, NULL, priority_shuffle, &task2Handle);

        /* Run test with shuffling */
        uint32_t shuffle_start = 0;
        uint32_t shuffle_stop  = 0;
        shuffle = true;

        TAKE_MEASUREMENT(shuffle_start);
        vTaskPrioritySet(NULL, priority_shuffle - 1);
        TAKE_MEASUREMENT(shuffle_stop);

        uint32_t shuffle_avg = ((shuffle_stop - shuffle_start) -
                                (overhead_stop - overhead_start)) / NUM_SAMPLES;

        /* Print results */
        printf("------------ Semaphore shuffling test ------------\n");
        printf("Shuffle time total:\t\t %lu\n", shuffle_stop - shuffle_start);
        printf("Overhead total:\t\t\t %lu\n", overhead_stop - overhead_start);
        printf("Shuffle time (avg):\t\t %lu\n", shuffle_avg);
        printf("Done.\n");

        while (true)
        {
            /* do nothing */
        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask */
/**
* @brief Function implementing the task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask */
void StartTask(void const * argument)
{
  /* USER CODE BEGIN StartTask */
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        if (shuffle)
        {
            xSemaphoreTake(BinarySemHandle, portMAX_DELAY);
        }
        taskYIELD();

        if (shuffle)
        {
            xSemaphoreGive(BinarySemHandle);
        }
        taskYIELD();
    }

    vTaskDelete(NULL);
  /* USER CODE END StartTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
