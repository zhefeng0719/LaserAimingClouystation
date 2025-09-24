/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @author ?????
  * @date 2025-06-25
  * @attention
  * 2025/6/25 ??????????FOC?????CPU???????1ms??2ms????Msg??????BUG 
  * 
  *
  * 
  * 
  * 
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define S1 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)
#define S2 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)
#define S3 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
float angle;
float speed;
float maxangle;
float minangle;
}FocAngle;
FocAngle FOC1,FOC2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern pid angle1,angle2,speed1;
extern MyUartGroup uartGroup;
extern coordinate aim;
extern cloudpid Cloud1,Cloud2;
extern unsigned int FOCOK;
extern int scan;
extern int target_lost_flag;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mytask1 */
osThreadId_t mytask1Handle;
const osThreadAttr_t mytask1_attributes = {
  .name = "mytask1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void InitPrc(void *argument);
void MsgPrc(void *argument);
void Foc1Prc(void *argument);
void Foc2Prc(void *argument);
void CtrlPrc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* creation of mytask1 */
  mytask1Handle = osThreadNew(InitPrc, NULL, &mytask1_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(MsgPrc, NULL, &myTask02_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(Foc1Prc, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(Foc2Prc, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(CtrlPrc, NULL, &myTask06_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_InitPrc */
/**
* @brief ?????????????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitPrc */
void InitPrc(void *argument)
{
  /* USER CODE BEGIN InitPrc */
HAL_TIM_Base_Start(&htim3);
HAL_TIM_Base_Start(&htim4);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);	
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
MyUartGroup_Init(&uartGroup);
HAL_NVIC_EnableIRQ(UART5_IRQn);
// HAL_NVIC_EnableIRQ(USART1_IRQn);
HAL_UART_Receive_DMA(&huart5, uartGroup.camera.Receive_data, uartGroup.camera.len);
// HAL_UART_Receive_DMA(&huart1, uartGroup.L1.Receive_data, uartGroup.L1.len);
__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
FOCinit();
OLED_Init();
cloudpidinit();
vTaskDelete(NULL);
  /* Infinite loop */
  /* USER CODE END InitPrc */
}

/* USER CODE BEGIN Header_MsgPrc */
/**
* @brief ??????????????   ??????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MsgPrc */
void MsgPrc(void *argument)
{
  /* USER CODE BEGIN MsgPrc */

  /* Infinite loop */
  for(;;)
  {
//OLED**************************************
	OLED_ShowString(x1,y1,"Ang:",OLED_6X8);
	OLED_ShowFloatNum(x2,y1,angle1.Actual*57.3f,3,2,OLED_6X8);
	OLED_ShowFloatNum(x4,y1,angle2.Actual*57.3f,3,2,OLED_6X8);
	OLED_ShowString(x1,y2,"Tar:",OLED_6X8);
	OLED_ShowFloatNum(x2,y2,FOC1.angle*57.3f,3,2,OLED_6X8);
  OLED_ShowFloatNum(x4,y2,FOC2.angle*57.3f,3,2,OLED_6X8);
  OLED_ShowString(x1,y3,"Vel:",OLED_6X8);
  OLED_ShowFloatNum(x2,y3,speed1.Actual,3,2,OLED_6X8);
	OLED_ShowFloatNum(x4,y3,0,3,2,OLED_6X8);	  
  OLED_ShowString(x1,y4,"Tar:",OLED_6X8);
	OLED_ShowFloatNum(x2,y4,FOC1.speed,3,2,OLED_6X8);
	OLED_ShowFloatNum(x4,y4,FOC2.speed,3,2,OLED_6X8);	
  OLED_ShowString(x1,y5,"AIM:",OLED_6X8);
  OLED_ShowNum(x2,y5,aim.x,3,OLED_6X8);
	OLED_ShowNum(x3,y5,aim.y,3,OLED_6X8);
  OLED_ShowNum(x5,y5,scan,4,OLED_6X8);
  // OLED_ShowFloatNum(x3,y5,_electricalAngle_2(),3,2,OLED_6X8);
	OLED_Update();
//Key*******************************************
  if(S3)
    {
  // if(!S1)FOC1.angle+=0.01744f;
	// if(!S2)FOC1.angle-=0.01744f;
  // if(!S1)FOC1.speed+=0.1f;
	// if(!S2)FOC1.speed-=0.1f;
	if(!S1)FOC2.angle+=0.001744f;
	if(!S2)FOC2.angle-=0.001744f;
    } 
  else
    {
	// if(!S1)FOC1.angle+=0.1744f;
	// if(!S2)FOC1.angle-=0.1744f;
  // if(!S1)FOC1.speed+=0.5f;
	// if(!S2)FOC1.speed-=0.5f;
  if(!S1)FOC2.angle+=0.01744f;
	if(!S2)FOC2.angle-=0.01744f;
    }
  osDelay(50);
  }

  /* USER CODE END MsgPrc */
}

/* USER CODE BEGIN Header_Foc1Prc */
/**
* @brief FOC???   ??????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Foc1Prc */
void Foc1Prc(void *argument)
{
  /* USER CODE BEGIN Foc1Prc */
  FOC1.angle = 175.0f/57.3f;
  FOC1.speed = 1.0f;
  FOC1.maxangle =  270.0f * PI / 180.0f;
  FOC1.minangle = 80.0f * PI / 180.0f;
  static int foundsta = 0;
  /* Infinite loop */
  for(;;)
  {
    if(FOCOK)
    {
      float target_angle = FOC1.angle + Cloud1.out;
      target_angle = target_angle < FOC1.minangle ? FOC1.minangle : (target_angle > FOC1.maxangle ? FOC1.maxangle : target_angle);
      Set_Angle_1(target_angle);
      // Set_Speed_1(FOC1.speed);

      if(target_lost_flag==1)
      {
        if(foundsta==0)FOC1.angle+=(0.001744f*2);
        else if(foundsta==1)FOC1.angle-=(0.001744f*2); 

        if (target_angle >= FOC1.maxangle-30*PI/180.0f)
        foundsta=1;
        else if (target_angle <= FOC1.minangle+30*PI/180.0f)
        foundsta=0;
      }  
    }
    osDelay(3);
  }
  /* USER CODE END Foc1Prc */
}

/* USER CODE BEGIN Header_Foc2Prc */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Foc2Prc */
void Foc2Prc(void *argument)
{
  /* USER CODE BEGIN Foc2Prc */
  FOC2.angle =80.0f / 57.3f;
  FOC2.minangle = 60.0f * PI / 180.0f;
  FOC2.maxangle =  100.0f * PI / 180.0f;

  /* Infinite loop */
  for(;;)
  {
    if(FOCOK)
    {
      float target_angle = FOC2.angle + Cloud2.out;
      target_angle = target_angle < FOC2.minangle ? FOC2.minangle : (target_angle > FOC2.maxangle ? FOC2.maxangle : target_angle);
      Set_Angle_2(target_angle);
      // setTorque_2(3,_electricalAngle_2());
    }
    osDelay(3);
  }
  /* USER CODE END Foc2Prc */
}

/* USER CODE BEGIN Header_CtrlPrc */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CtrlPrc */
void CtrlPrc(void *argument)
{
  /* USER CODE BEGIN CtrlPrc */
    uint8_t L1Start[5] = {0xA5, 0x5A, 0x04, 0x00, 0xFB};
    HAL_UART_Transmit(&huart1, L1Start, 5, 10);
  /* Infinite loop */
  for(;;)
  {
    //Uart************************************
    MsgSendPrc();
    CameraPrc();
    // L1Prc();
    cloud1();
    cloud2();
    osDelay(5);
  }
  /* USER CODE END CtrlPrc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

