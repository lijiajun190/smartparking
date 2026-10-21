/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "app_events.h"
#include "bsp_oled.h"
#include "bsp_buzzer.h"
#include "bsp_keys.h"
#include "bsp_sensors.h"
#include "bsp_rc522.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_stepper.h"
#include "parking_fsm.h"
#include "bsp_sim800.h"
#include "gsm_test.h"
#include "parking_store.h"
#include "motion_config.h"
#include "health_watch.h"
#include "bsp_watchdog.h"
#include "bsp_clock.h"
#include "bsp_console.h"
#include "parking_service.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* 运动命令类型枚举 */
typedef enum
{
    MOTION_CMD_MOVE = 0,  // 移动指定步数
    MOTION_CMD_HOME,      // 回零
    MOTION_CMD_RECOVER_HOME, // 清除锁存后受控回零
    MOTION_CMD_STOP       // 立即停止
} MotionCommandType;

/* 运动命令结构体 */
typedef struct
{
    MotionCommandType type;  // 命令类型
    MotorID motor;           // 目标电机
    uint32_t steps;          // 步数（MOVE命令使用）
    MotorDirection direction; // 方向（MOVE命令使用）
    uint32_t command_id;
} MotionCommand;

static volatile uint8_t motion_active = 0U;
/* 可在调试器观察；UI丢帧由周期快照刷新兜底，业务丢事件则锁止。 */
volatile uint32_t app_parking_event_drops;
volatile uint32_t app_ui_event_drops;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 调试日志开关
 * 0 = 只显示关键事件
 * 1 = 显示详细调试信息
 */
#define DEBUG_VERBOSE 0

/*
 * 临时事件在 OLED 上的显示时间。
 * 传感器触发后显示 SENS，超过该时间恢复 OK。
 */
#define UI_EVENT_HOLD_MS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 全局停车上下文（供UI任务访问） */
ParkingContext g_parking_context;
static ParkingContext ui_snapshot;

/*应用事件队列*/
osMessageQueueId_t parkingEventQueueHandle;  // ParkingTask 业务队列
osMessageQueueId_t uiEventQueueHandle;       // UiTask 显示队列


osMessageQueueId_t motionQueueHandle;
osThreadId_t motionTaskHandle;

/*蜂鸣器互斥锁，防止inputtask和safetask同时操作蜂鸣器*/
osMutexId_t buzzerMutexHandle;

/*三个基础任务句柄*/
osThreadId_t uiTaskHandle;
osThreadId_t inputTaskHandle;
osThreadId_t safetyTaskHandle;
osThreadId_t rfidTaskHandle;
osThreadId_t parkingTaskHandle;

/* GSM 测试任务句柄（已禁用） */
// osThreadId_t gsmTestTaskHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* 任务属性定义 */
const osThreadAttr_t uiTask_attributes = {
  .name = "UiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t inputTask_attributes = {
  .name = "InputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t safetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t rfidTask_attributes = {
  .name = "RfidTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t motionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,  // 介于 Normal 和 High 之间
};

const osThreadAttr_t parkingTask_attributes = {
  .name = "ParkingTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* GSM 测试任务属性（已禁用） */
/*
const osThreadAttr_t gsmTestTask_attributes = {
  .name = "GsmTestTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
*/

/* 函数原型声明 */

static void StartUiTask(void *argument);
static void StartInputTask(void *argument);
static void StartSafetyTask(void *argument);
static void StartRfidTask(void *argument);
static void StartMotionTask(void *argument);
static void StartParkingTask(void *argument);
// static void StartGsmTestTask(void *argument);  /* GSM测试任务已禁用 */

static void app_post_event(AppEventType type, uint32_t value);
static void app_post_card_event(const uint8_t *uid);
static void app_deliver_event(const AppEvent *event);
static void app_post_motion_event(AppEventType type, const MotionCommand *command);
static uint8_t parking_save_context(const ParkingContext *context);
static uint8_t parking_motion_idle(void);
static void parking_print_diagnostics(void);

static void buzzer_beep_short_safe(void);
static void buzzer_beep_done_safe(void);
static void buzzer_on_safe(void);
static void buzzer_off_safe(void);

static uint8_t parking_request_motion(ParkingMotionType type, MotorID motor,
                                   uint32_t steps,
                                   MotorDirection direction, uint32_t command_id);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   (void)xTask; (void)pcTaskName;
   bsp_runtime_panic();
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  health_init(HAL_GetTick());

  /* 系统启动横幅 */
  printf("\r\n");
  printf("========================================\r\n");
  printf("  STM32F4 Smart Parking System v1.0\r\n");
  printf("========================================\r\n");
  printf("[INIT] System clock: %lu MHz\r\n", SystemCoreClock / 1000000);
  printf("[INIT] FreeRTOS initialization...\r\n");

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /*
   * 创建蜂鸣器互斥锁。
   * 蜂鸣器可能同时被 InputTask 和 SafetyTask 使用，
   * 必须保证同一时刻只有一个任务操作它。
   */
  buzzerMutexHandle = osMutexNew(NULL);

  if (buzzerMutexHandle == NULL) {
      Error_Handler();
  }
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  
  /*
   * 创建停车业务事件队列。
   * 
   * 该队列专属于 ParkingTask，保存所有业务相关事件：
   * - 刷卡事件（含完整 UID）
   * - 按键事件（存车/取车/缴费确认）
   * - 运动完成/故障事件
   * - 急停事件
   * 
   * 队列长度 16：足够缓存短时间内的多个事件。
   */
  parkingEventQueueHandle = osMessageQueueNew(16, sizeof(AppEvent), NULL);

  if (parkingEventQueueHandle == NULL)
  {
    Error_Handler();
  }

  /*
   * 创建 UI 显示事件队列。
   * 
   * 该队列专属于 UiTask，只用于驱动 OLED 显示更新。
   * UiTask 不参与业务决策，只负责显示当前状态快照。
   * 
   * 队列长度 8：UI 刷新不需要保留完整历史，只显示最新状态。
   */
  uiEventQueueHandle = osMessageQueueNew(8, sizeof(AppEvent), NULL);

  if (uiEventQueueHandle == NULL)
  {
    Error_Handler();
  }
  /*
 * 所有电机控制请求统一进入该队列。
 * 只有 MotionTask 可以直接调用步进电机驱动。
 */
motionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);

if (motionQueueHandle == NULL)
{
    Error_Handler();
}
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //OLED显示任务
  uiTaskHandle = osThreadNew(StartUiTask, NULL, &uiTask_attributes);

  //按键扫描任务
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  //安全监控任务
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);

  rfidTaskHandle = osThreadNew(StartRfidTask, NULL, &rfidTask_attributes);

  // 步进电机控制任务
  motionTaskHandle = osThreadNew(StartMotionTask, NULL, &motionTask_attributes);

  // 停车业务任务
  parkingTaskHandle = osThreadNew(StartParkingTask, NULL, &parkingTask_attributes);

  // GSM 测试任务（临时，测试完成后应禁用）
  // gsmTestTaskHandle = osThreadNew(StartGsmTestTask, NULL, &gsmTestTask_attributes);

  if ((defaultTaskHandle == NULL) || (uiTaskHandle == NULL) || (inputTaskHandle == NULL) ||
      (safetyTaskHandle == NULL) || (motionTaskHandle == NULL) ||
      (parkingTaskHandle == NULL) || (rfidTaskHandle == NULL) ||
      !parking_service_init(parking_motion_idle, parking_print_diagnostics))
  {
    Error_Handler();
  }

  printf("[INIT] All tasks created successfully\r\n");
  printf("[INIT] System ready, scheduler starting...\r\n");
  printf("========================================\r\n\r\n");

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
    if (!health_check(HAL_GetTick())) bsp_runtime_panic();
    bsp_watchdog_refresh();
    osDelay(100U);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  向业务队列和 UI 队列同时投递事件
 * @param  type 事件类型
 * @param  value 事件附加值
 * @note   所有生产者统一调用此函数
 */
/* 所有队列发送统一检查。业务队列满时不能假装动作已受理。 */
static void app_deliver_event(const AppEvent *event)
{
    if (parkingEventQueueHandle == NULL ||
        osMessageQueuePut(parkingEventQueueHandle, event, 0U, 0U) != osOK) {
        taskENTER_CRITICAL();
        app_parking_event_drops++;
        taskEXIT_CRITICAL();
        stepper_request_stop(MOTOR_FAULT_COMMAND);
    }
    if (uiEventQueueHandle == NULL ||
        osMessageQueuePut(uiEventQueueHandle, event, 0U, 0U) != osOK) {
        taskENTER_CRITICAL();
        app_ui_event_drops++;
        taskEXIT_CRITICAL();
    }
}

static void app_post_event(AppEventType type, uint32_t value)
{
    AppEvent event = {0};
    event.tick = HAL_GetTick();
    event.type = type;
    event.value = value;
    app_deliver_event(&event);
}

static void app_post_motion_event(AppEventType type, const MotionCommand *command)
{
    AppEvent event = {0};
    event.tick = HAL_GetTick();
    event.type = type;
    event.value = (uint32_t)command->motor;
    event.command_id = command->command_id;
    /* 显式转换，避免两套枚举日后调整顺序导致完成事件无法匹配。 */
    switch (command->type) {
        case MOTION_CMD_MOVE: event.command_type = PARK_MOTION_MOVE; break;
        case MOTION_CMD_HOME: event.command_type = PARK_MOTION_HOME; break;
        case MOTION_CMD_RECOVER_HOME: event.command_type = PARK_MOTION_RECOVER_HOME; break;
        default: event.command_type = UINT32_MAX; break;
    }
    app_deliver_event(&event);
}

static void app_post_card_event(const uint8_t *uid)
{
    AppEvent event = {0};
    if (!uid) return;
    event.tick = HAL_GetTick();
    event.type = APP_EVENT_CARD_DETECTED;
    memcpy(event.uid, uid, 4U);
    app_deliver_event(&event);
}

//@brief加锁后发出短提示音
static void buzzer_beep_short_safe(void)
{
  if (buzzerMutexHandle == NULL)
  {
    return;
  }

  if (osMutexAcquire(buzzerMutexHandle, osWaitForever) == osOK)
  {
    (void)buzzer_beep_short();
    if (sensor_emergency_active()) (void)buzzer_on();
    (void)osMutexRelease(buzzerMutexHandle);
  }
}

//加锁后发出完成提示音
static void buzzer_beep_done_safe(void)
{
  if (buzzerMutexHandle == NULL)
  {
    return;
  }

  if (osMutexAcquire(buzzerMutexHandle, osWaitForever) == osOK)
  {
    (void)buzzer_beep_done();
    if (sensor_emergency_active()) (void)buzzer_on();
    (void)osMutexRelease(buzzerMutexHandle);
  }
}

//加锁后打开蜂鸣器
static void buzzer_on_safe(void)
{
  if (buzzerMutexHandle == NULL)
    {
        return;
    }

    if (osMutexAcquire(buzzerMutexHandle, osWaitForever) == osOK)
    {
        (void)buzzer_on();
        (void)osMutexRelease(buzzerMutexHandle);
    }
}

//加锁后关闭蜂鸣器
static void buzzer_off_safe(void)
{
    if (buzzerMutexHandle == NULL)
    {
        return;
    }

    if (osMutexAcquire(buzzerMutexHandle, osWaitForever) == osOK)
    {
        (void)buzzer_off();
        (void)osMutexRelease(buzzerMutexHandle);
    }
}

//根据当前状态刷新oled
static void ui_render(uint32_t sensor_state, AppEventType last_event, const uint8_t *last_uid)
{
  uint8_t emergency;
  const ParkingContext *ctx;
  ParkingContext snapshot;
  char title[17];
  (void)last_uid;

  emergency = (sensor_state & SENSOR_FLAG(SENSOR_EMERGENCY)) ? 1U : 0U;

  /* 获取停车上下文 */
  taskENTER_CRITICAL();
  snapshot = ui_snapshot;
  taskEXIT_CRITICAL();
  ctx = &snapshot;

  //当前oled最多显示四行，一行16字符
  (void)OLED_Clear();

  /* 第1行：标题 */
  if (ctx->state == PARK_WAIT_PAYMENT) {
      if (ctx->quote_uncertain) snprintf(title, sizeof(title), "Fee: CHECK TIME");
      else snprintf(title, sizeof(title), "Fee:%lu.%02lu",
          (unsigned long)(ctx->quote_cents / 100U), (unsigned long)(ctx->quote_cents % 100U));
  } else snprintf(title, sizeof(title), "Parking SMS:%u", parking_service_sms_status());
  OLED_ShowString(1, 1, title);

  /* 第2行：车位占用状态 L11 L12 L21 L22 */
  OLED_ShowString(2, 1, "L11:");
  OLED_ShowChar(2, 5, (ctx != NULL && ctx->slots[0].occupied) ? 'O' : '-');

  OLED_ShowString(2, 7, "L12:");
  OLED_ShowChar(2, 11, (ctx != NULL && ctx->slots[1].occupied) ? 'O' : '-');

  /* 第3行：车位占用状态 L21 L22 */
  OLED_ShowString(3, 1, "L21:");
  OLED_ShowChar(3, 5, (ctx != NULL && ctx->slots[2].occupied) ? 'O' : '-');

  OLED_ShowString(3, 7, "L22:");
  OLED_ShowChar(3, 11, (ctx != NULL && ctx->slots[3].occupied) ? 'O' : '-');

  /* 第4行：系统状态和事件 */
  if (emergency != 0U)
  {
      OLED_ShowString(4, 1, "ESTOP!");
      return;
  }

  /* 显示停车状态 */
  if (ctx != NULL)
  {
      switch (ctx->state)
      {
          case PARK_IDLE:
              OLED_ShowString(4, 1, "IDLE");
              break;
          case PARK_SELECT_ACTION:
              OLED_ShowString(4, 1, "WAIT");
              break;
          case PARK_MOVING_IN:
              OLED_ShowString(4, 1, "IN");
              OLED_ShowNum(4, 4, ctx->selected_slot, 1);
              break;
          case PARK_MOVING_HOME:
              OLED_ShowString(4, 1, "HOMING");
              break;
          case PARK_WAIT_PAYMENT:
              OLED_ShowString(4, 1, "PAY");
              OLED_ShowNum(4, 5, ctx->selected_slot, 1);
              break;
          case PARK_MOVING_OUT:
              OLED_ShowString(4, 1, "OUT");
              OLED_ShowNum(4, 5, ctx->selected_slot, 1);
              break;
          case PARK_FAULT:
              OLED_ShowString(4, 1, ctx->storage_fault ? "DATA!" :
                                  ctx->reconcile_required ? "CHECK!" : "FAULT!");
              break;
          case PARK_FULL:
              OLED_ShowString(4, 1, "FULL");
              break;
          default:
              OLED_ShowString(4, 1, "?");
              break;
      }
  }

  /* 第4行右侧：显示最后事件 */
  switch (last_event)
  {
      case APP_EVENT_KEY_PRESS:
          OLED_ShowString(4, 10, "KEY");
          break;
      case APP_EVENT_KEY_LONG_PRESS:
          OLED_ShowString(4, 10, "LONG");
          break;
      case APP_EVENT_SENSOR_CHANGED:
          OLED_ShowString(4, 10, "SENS");
          break;
      case APP_EVENT_CARD_DETECTED:
          OLED_ShowString(4, 10, "CARD");
          break;
      default:
          OLED_ShowString(4, 10, "OK");
          break;
  }
}

//OLED显示任务，唯一访问OLED，接收按键和传感器事件，周期性刷新传感器状态
static void StartUiTask(void *argument)
{
  AppEvent event;
  AppEventType last_event = APP_EVENT_NONE;
  uint8_t last_uid[4] = {0};  /* 保存最后读取的卡片UID */

  uint32_t sensor_state;
  uint32_t last_sensor_state = 0U;
  uint32_t last_event_tick = 0U;
  uint32_t last_render_tick = 0U;
  uint8_t display_dirty = 1U;
  ParkingState last_parking_state = PARK_FAULT;

  (void)argument;

  last_sensor_state = sensor_read_all();

  for(;;)
  {
    health_beat(HEALTH_UI, HAL_GetTick());
    /* 等待最多 100ms 事件
     * 如果没有事件，也会继续执行下面的周期刷新 */
    if(osMessageQueueGet(uiEventQueueHandle, &event, NULL, 100U) == osOK)

    {
      last_event = event.type;
      last_event_tick = HAL_GetTick();
      display_dirty = 1U;

      /* 如果是刷卡事件，保存UID */
      if (event.type == APP_EVENT_CARD_DETECTED)
      {
          memcpy(last_uid, event.uid, 4);
      }
    }

    sensor_state = sensor_read_all();
    if (g_parking_context.state != last_parking_state)
    {
        last_parking_state = g_parking_context.state;
        display_dirty = 1U;
    }

    if (sensor_state != last_sensor_state)
    {
      last_sensor_state = sensor_state;
      /*
       * 急停状态优先级高于普通传感器变化。
       */
      if ((sensor_state & SENSOR_FLAG(SENSOR_EMERGENCY)) != 0U)
      {
          last_event = APP_EVENT_EMERGENCY_ON;
      }
      else
      {
          last_event = APP_EVENT_SENSOR_CHANGED;
      }

      last_event_tick = HAL_GetTick();
      display_dirty = 1U;
    }

    /*
     * 急停未激活时，临时事件显示超过 1 秒后恢复为 OK。
     * 急停状态不在这里清除，由当前传感器状态决定。
     */
    if (((sensor_state & SENSOR_FLAG(SENSOR_EMERGENCY)) == 0U) &&
        (last_event != APP_EVENT_NONE) &&
        ((HAL_GetTick() - last_event_tick) >= UI_EVENT_HOLD_MS))
    {
        last_event = APP_EVENT_NONE;
        display_dirty = 1U;
    }

    if (display_dirty != 0U || (uint32_t)(HAL_GetTick() - last_render_tick) >= 1000U)
    {
      ui_render(sensor_state, last_event, last_uid);
      display_dirty = 0U;
      last_render_tick = HAL_GetTick();
    }
  }
}

//按键扫描任务，每10ms扫描一次PA0，并消抖
static void StartInputTask(void *argument)
{
  KeyId key_id;
  KeyEvent key_event;
  AppEventType app_event_type;

  (void)argument;

  for(;;)
  {
    key_event = key_scan_all(&key_id);
    health_beat(HEALTH_INPUT, HAL_GetTick());
    app_event_type = APP_EVENT_NONE;

    switch (key_event)
    {
      case KEY_EVENT_PRESS:
      app_event_type = APP_EVENT_KEY_PRESS;
      buzzer_beep_short_safe();
      break;

      case KEY_EVENT_RELEASE:
      app_event_type = APP_EVENT_KEY_RELEASE;
      break;

      case KEY_EVENT_LONG_PRESS:
      app_event_type = APP_EVENT_KEY_LONG_PRESS;
      buzzer_beep_done_safe();
      break;

      default:
      break;
    }

    if (app_event_type != APP_EVENT_NONE)
    {
      app_post_event(app_event_type, (uint32_t)key_id);
    }

    osDelay(KEY_SCAN_PERIOD_MS);
  }
}


/**
 * @brief  步进电机控制任务
 * @param  argument 未使用
 * @note   任务职责:
 *         1. 这是唯一允许直接操作步进电机的任务
 *         2. 从运动命令队列接收命令（MOVE/HOME/STOP）
 *         3. 调用 BSP 层驱动执行运动
 *         4. 检查运动结果并发布事件
 *         5. 确保其他任务不直接调用 stepper_xxx 函数
 */
static void StartMotionTask(void *argument)
{
    MotionCommand command;
    (void)argument;
    for (;;)
    {
        health_beat(HEALTH_MOTION, HAL_GetTick());
        stepper_service_stop();
        if (osMessageQueueGet(motionQueueHandle, &command, NULL, 10U) != osOK)
            continue;
        motion_active = 1U;
        /* 恢复入队后若再次发生急停，也必须拒绝，不能在这里再次清锁。 */
        if (stepper_get_fault() == MOTOR_FAULT_NONE)
        {
            switch (command.type)
            {
                case MOTION_CMD_MOVE:
                    stepper_move_steps(command.motor, command.steps, command.direction);
                    break;
                case MOTION_CMD_HOME:
                case MOTION_CMD_RECOVER_HOME:
                    stepper_home(command.motor);
                    break;
                case MOTION_CMD_STOP:
                    stepper_stop(command.motor);
                    break;
                default:
                    stepper_request_stop(MOTOR_FAULT_COMMAND);
                    break;
            }
        }
        stepper_service_stop();
        MotorFault fault = stepper_get_fault();
        if (fault == MOTOR_FAULT_NONE &&
            stepper_get_status(command.motor) == MOTOR_IDLE &&
            (command.type == MOTION_CMD_MOVE ||
             ((command.type == MOTION_CMD_HOME || command.type == MOTION_CMD_RECOVER_HOME) &&
              stepper_is_homed(command.motor))))
        {
            app_post_motion_event(APP_EVENT_MOTION_DONE, &command);
        }
        else
        {
            printf("[MOTION] Fault motor=%d, reason=%d\r\n", command.motor, fault);
            app_post_motion_event(fault == MOTOR_FAULT_TIMEOUT ?
                           APP_EVENT_MOTION_TIMEOUT : APP_EVENT_MOTION_FAULT,
                           &command);
        }
        /* 完成事件发布后再允许恢复，避免旧完成事件跨越恢复边界。 */
        motion_active = 0U;
    }
}

//安全监控；轮询急停，避免依赖EXTI2；读取传感器变化位图；急停时打开蜂鸣器；急停结束后关闭蜂鸣器
static void StartSafetyTask(void *argument)
{
    uint8_t emergency_active;
    uint8_t emergency_previous = 0U;
    uint32_t changed_flags;

    (void)argument;

    //保存当前任务的原生Free RTOS句柄
    sensor_set_notify_task((void *)xTaskGetCurrentTaskHandle());

    for (; ;) 
    {
      //即使EXTI2没有触发，也通过轮询保证急停可用
      health_beat(HEALTH_SAFETY, HAL_GetTick());
      emergency_active = sensor_emergency_active();
      if (emergency_active != 0U)
          stepper_request_stop(MOTOR_FAULT_EMERGENCY);

      if((emergency_active != 0U) && (emergency_previous == 0U))
      {
        emergency_previous = 1U;
        buzzer_on_safe();
        app_post_event(APP_EVENT_EMERGENCY_ON, 1U);
      }
      else if((emergency_active == 0U) && (emergency_previous != 0U))
      {
        emergency_previous = 0U;
        buzzer_off_safe();
        app_post_event(APP_EVENT_EMERGENCY_OFF, 0U);
      }
      //读取并清除EXTI变化位图
      changed_flags = sensor_take_changed_flags();

      /*
       * 急停由上面的 emergency_active 逻辑单独处理，
       * 不再把急停变化作为普通传感器事件发送，
       * 避免 SENSOR_CHANGED 覆盖急停事件。
       */
      changed_flags &= ~SENSOR_FLAG(SENSOR_EMERGENCY);

      if(changed_flags != 0U)
      {
        app_post_event(APP_EVENT_SENSOR_CHANGED, changed_flags);
      }

      //使用任务通知实现20ms周期等待；如果EXTI中断发生，任务会被提前唤醒
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));
    }
}    

/**
 * @brief  RFID 读卡任务
 * @param  argument 未使用
 * @note   周期性扫描 RC522，检测到卡片后投递事件
 */
static void StartRfidTask(void *argument)
{
    RC522_Status status;
    uint8_t uid[4];
    uint8_t ready = 0U, errors = 0U;
    uint32_t version_tick = 0U;

    (void)argument;

    /* 延迟 100ms 等待系统稳定后再开始扫描 */
    osDelay(100U);

    printf("[RFID] Task started\r\n");

    for (;;)
    {
        health_beat(HEALTH_RFID, HAL_GetTick());
        if (ready && (uint32_t)(HAL_GetTick() - version_tick) >= 1000U) {
            uint8_t version = rc522_read_version();
            version_tick = HAL_GetTick();
            if (version != 0x91U && version != 0x92U && version != 0x88U && version != 0x82U)
                ready = 0U;
        }
        if (!ready) {
            ready = rc522_init() == HAL_OK;
            if (!ready) { osDelay(1000U); continue; }
            errors = 0U;
            printf("[RFID] ready\r\n");
        }

        /* 调用带去抖的读卡接口 (500ms去抖) */
        status = rc522_read_with_debounce(uid, 500U);
        if (status != RC522_OK && status != RC522_NOTAG && status != RC522_COLLISION) {
            if (++errors >= 5U) ready = 0U;
        } else errors = 0U;

        switch (status)
        {
            case RC522_OK:
                /* 成功读取到卡片 */
                printf("[RFID] Card: %02X:%02X:%02X:%02X\r\n",
                       uid[0], uid[1], uid[2], uid[3]);

                /* 投递卡片事件 */
                app_post_card_event(uid);

                /* 发出短提示音 */
                buzzer_beep_short_safe();
                break;

            case RC522_NOTAG:
                /* 无卡，正常情况 */
                break;

            case RC522_ERR:
                /* 通信错误 */
                printf("[RFID] Read error\r\n");
                break;

            case RC522_COLLISION:
                /* 多卡冲突 */
                printf("[RFID] Collision\r\n");
                break;

            case RC522_TIMEOUT:
                /* 读卡超时 */
                printf("[RFID] Timeout\r\n");
                break;

            default:
                break;
        }

        /* 每 150ms 扫描一次 */
        osDelay(150U);
    }
}

/**
 * @brief  停车业务任务
 * @param  argument 未使用
 * @note   维护停车状态机，处理刷卡、按键和运动事件
 */
static void StartParkingTask(void *argument)
{
    AppEvent event;
    ParkingSnapshot snapshot;
    ParkingStoreResult loaded;
    uint8_t i;
    (void)argument;
    parking_init(&g_parking_context);
    g_parking_context.persist = parking_save_context;
    g_parking_context.clock = bsp_clock_now;
    g_parking_context.notice = parking_service_notice;
    bsp_console_init();
    printf("[CONSOLE] 115200 8N1; HELP for commands; local PAY confirms payment\r\n");
    loaded = parking_store_load(&snapshot);
    health_beat(HEALTH_PARKING, HAL_GetTick());
    if (loaded == PARK_STORE_OK) {
        for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
            SensorId sensor = PARKING_SLOTS[i].occupancy_sensor;
            g_parking_context.slots[i].occupied = (uint8_t)snapshot.slots[i].occupied;
            memcpy(g_parking_context.slots[i].uid, snapshot.slots[i].uid, 4U);
            g_parking_context.slots[i].start_tick = HAL_GetTick();
            g_parking_context.slots[i].start_epoch = snapshot.slots[i].start_epoch;
            g_parking_context.slots[i].time_uncertain = snapshot.slots[i].occupied &&
                (!bsp_clock_now() || !snapshot.slots[i].start_epoch ||
                 bsp_clock_now() < snapshot.slots[i].start_epoch);
            if (sensor < SENSOR_ID_COUNT && sensor_read(sensor) != snapshot.slots[i].occupied)
                g_parking_context.reconcile_required = 1U;
        }
        g_parking_context.pending = (uint8_t)snapshot.pending;
        g_parking_context.selected_slot = (uint8_t)snapshot.selected_slot;
        memcpy(g_parking_context.current_uid, snapshot.uid, 4U);
        if (snapshot.pending) g_parking_context.reconcile_required = 1U;
    } else if (loaded == PARK_STORE_ERROR || !parking_save_context(&g_parking_context)) {
        g_parking_context.storage_fault = 1U;
        g_parking_context.reconcile_required = 1U;
    }
    if (loaded == PARK_STORE_EMPTY) {
        for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
            SensorId sensor = PARKING_SLOTS[i].occupancy_sensor;
            if (sensor < SENSOR_ID_COUNT && sensor_read(sensor))
                g_parking_context.reconcile_required = 1U;
        }
    }
    if (g_parking_context.reconcile_required || !motion_config_valid()) {
        AppEvent fault_event = { .type = APP_EVENT_MOTION_FAULT };
        parking_handle_event(&g_parking_context, &fault_event, parking_request_motion);
    } else {
        parking_begin_home(&g_parking_context, parking_request_motion, 0U);
    }
    printf("[PARKING] startup state=%u check=%u data=%u; STATUS for details\r\n",
        g_parking_context.state, g_parking_context.reconcile_required, g_parking_context.storage_fault);

    for (;;)
    {
        osStatus_t status = osMessageQueueGet(parkingEventQueueHandle, &event, NULL, 20U);
        health_beat(HEALTH_PARKING, HAL_GetTick());
        parking_poll(&g_parking_context, HAL_GetTick());
        /* 锁存独立于事件队列：队列满、短急停、急停已释放都不能漏过故障。 */
        if (stepper_get_fault() != MOTOR_FAULT_NONE)
        {
            AppEvent fault_event = { .type = APP_EVENT_MOTION_FAULT };
            parking_handle_event(&g_parking_context, &fault_event, parking_request_motion);
        }
        if (status == osOK)
            parking_handle_event(&g_parking_context, &event, parking_request_motion);
        parking_service_poll(&g_parking_context, parking_request_motion);
        taskENTER_CRITICAL();
        ui_snapshot = g_parking_context;
        taskEXIT_CRITICAL();
    }
}

static uint8_t parking_motion_idle(void)
{
    return !motion_active && motionQueueHandle != NULL &&
        osMessageQueueGetCount(motionQueueHandle) == 0U &&
        stepper_get_status(MOTOR_A) != MOTOR_RUNNING &&
        stepper_get_status(MOTOR_B) != MOTOR_RUNNING;
}

/* STATUS从ParkingTask调用；栈余量单位为32位word，取历史最小值。 */
static void parking_print_diagnostics(void)
{
    printf("[STACK] default=%lu safety=%lu motion=%lu parking=%lu input=%lu ui=%lu rfid=%lu words\r\n",
        (unsigned long)uxTaskGetStackHighWaterMark(defaultTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(safetyTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(motionTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(parkingTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(inputTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(uiTaskHandle),
        (unsigned long)uxTaskGetStackHighWaterMark(rfidTaskHandle));
    printf("[QUEUE] motion=%lu parking=%lu ui=%lu parking_drop=%lu ui_drop=%lu health_failed=0x%lx\r\n",
        (unsigned long)osMessageQueueGetCount(motionQueueHandle),
        (unsigned long)osMessageQueueGetCount(parkingEventQueueHandle),
        (unsigned long)osMessageQueueGetCount(uiEventQueueHandle),
        (unsigned long)app_parking_event_drops, (unsigned long)app_ui_event_drops,
        (unsigned long)health_failed_mask());
}

/* 仅ParkingTask调用；无活动/排队运动时才允许Flash擦写。
 * 同bank Flash擦除会暂停取指，因此不能与运动并发。 */
static uint8_t parking_save_context(const ParkingContext *context)
{
    ParkingSnapshot snapshot = {0};
    uint8_t i;
    if (motion_active || osMessageQueueGetCount(motionQueueHandle) != 0U ||
        stepper_get_status(MOTOR_A) == MOTOR_RUNNING ||
        stepper_get_status(MOTOR_B) == MOTOR_RUNNING) return 0U;
    if (context->storage_fault) {
        /* 写失败后的维护重试必须重新扫描，不能覆盖不确定的日志页。 */
        ParkingSnapshot previous;
        if (parking_store_load(&previous) == PARK_STORE_ERROR) return 0U;
    }
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        snapshot.slots[i].occupied = context->slots[i].occupied;
        memcpy(snapshot.slots[i].uid, context->slots[i].uid, 4U);
        snapshot.slots[i].start_epoch = context->slots[i].start_epoch;
    }
    snapshot.pending = context->pending;
    snapshot.selected_slot = context->selected_slot;
    memcpy(snapshot.uid, context->current_uid, 4U);
    return parking_store_save(&snapshot);
}

/**
 * @brief  获取停车上下文指针（供UI任务访问）
 * @return 停车上下文的只读指针
 * @note   通过访问StartParkingTask中的static变量
 */
const ParkingContext* parking_get_context(void)
{
    /* 声明为extern，指向StartParkingTask中的static context */
    extern ParkingContext g_parking_context;
    return &g_parking_context;
}

/**
 * @brief  运动请求回调函数（由状态机调用）
 * @param  motor 电机 ID
 * @param  steps 步数
 * @param  direction 方向
 * @note   组装运动命令并投递到运动队列
 */
static uint8_t parking_request_motion(ParkingMotionType type, MotorID motor,
                                      uint32_t steps, MotorDirection direction, uint32_t command_id)
{
    MotionCommand cmd;
    if (motionQueueHandle == NULL || (unsigned)motor >= MOTOR_COUNT || command_id == 0U ||
        (type != PARK_MOTION_MOVE && type != PARK_MOTION_HOME && type != PARK_MOTION_RECOVER_HOME) ||
        (direction != DIR_CW && direction != DIR_CCW))
    {
        stepper_request_stop(MOTOR_FAULT_COMMAND);
        return 0U;
    }
    cmd.type = type == PARK_MOTION_MOVE ? MOTION_CMD_MOVE :
               type == PARK_MOTION_RECOVER_HOME ? MOTION_CMD_RECOVER_HOME : MOTION_CMD_HOME;
    cmd.motor = motor;
    cmd.steps = steps;
    cmd.direction = direction;
    cmd.command_id = command_id;

    if (type == PARK_MOTION_RECOVER_HOME)
    {
        uint8_t accepted = 0U;
        taskENTER_CRITICAL();
        if (!motion_active && !sensor_emergency_active())
        {
            /* 清旧队列、清锁与入队作为一次恢复操作。
             * 入队后任何新急停都保持锁存，MotionTask不得再次清锁。 */
            if (osMessageQueueReset(motionQueueHandle) == osOK &&
                osMessageQueueReset(parkingEventQueueHandle) == osOK && stepper_clear_fault())
            {
                accepted = osMessageQueuePut(motionQueueHandle, &cmd, 0U, 0U) == osOK;
                if (!accepted)
                    stepper_request_stop(MOTOR_FAULT_COMMAND);
            }
        }
        taskEXIT_CRITICAL();
        return accepted;
    }
    if (stepper_get_fault() != MOTOR_FAULT_NONE || sensor_emergency_active())
        return 0U;
    if (osMessageQueuePut(motionQueueHandle, &cmd, 0U, 0U) != osOK)
    {
        stepper_request_stop(MOTOR_FAULT_COMMAND);
        return 0U;
    }
    return 1U;
}

/**
 * @brief  GSM 测试任务（已禁用，测试完成后不再需要）
 * @note   如需重新测试 GSM，取消注释相关代码：
 *         1. 取消注释 gsmTestTaskHandle 声明
 *         2. 取消注释 gsmTestTask_attributes
 *         3. 取消注释 StartGsmTestTask 函数原型
 *         4. 在 MX_FREERTOS_Init 中取消注释任务创建
 */
/*
static void StartGsmTestTask(void *argument)
{
    (void)argument;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM Test Task Started\r\n");
    printf("========================================\r\n");

    printf("[GSM_TEST] Waiting 5 seconds for system startup...\r\n");
    osDelay(5000);

    printf("[GSM_TEST] Starting full test sequence...\r\n\r\n");

    uint8_t result = gsm_test_full();

    if (result == 0)
    {
        printf("\r\n");
        printf("****************************************\r\n");
        printf("*  GSM Test: ALL PASSED                *\r\n");
        printf("*  SIM800L is ready for integration    *\r\n");
        printf("****************************************\r\n");
    }
    else
    {
        printf("\r\n");
        printf("****************************************\r\n");
        printf("*  GSM Test: FAILED (code %d)          *\r\n", result);
        printf("*  Check wiring and power supply       *\r\n");
        printf("****************************************\r\n");
    }

    printf("\r\n[GSM_TEST] Task finished, suspending...\r\n");
    vTaskSuspend(NULL);

    for (;;)
    {
        osDelay(1000);
    }
}
*/

/* USER CODE END Application */

