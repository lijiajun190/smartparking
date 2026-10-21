#ifndef PARKING_FSM_H
#define PARKING_FSM_H

#include <stdint.h>
#include "app_events.h"
#include "bsp_stepper.h"


typedef enum
{
    PARK_IDLE = 0,  //空闲，等待刷卡
    PARK_SELECT_ACTION,   //已识别新卡，等待存车
    PARK_MOVING_HOME,
    PARK_MOVING_IN,   //正在执行存车动作
    PARK_PARKED,   //停好开始计费
    PARK_WAIT_PAYMENT,   //等待缴费确认
    PARK_MOVING_OUT,    //执行取车
    PARK_FULL,       //无空位，仍允许已有用户刷卡取车
    PARK_FAULT       //发生急停，限位，电机故障
} ParkingState;

typedef struct
{
    uint8_t occupied;   //是否有存车
    uint8_t uid[4];     //绑定的卡号
    uint32_t start_tick;    //开始停车的时间
    uint8_t time_uncertain; //重启后旧uptime不可用于跨断电计费
    uint32_t start_epoch;   //有效RTC的Unix秒；0表示无可靠绝对时间
    uint64_t elapsed_ms;    //本次运行内由poll累计，避免49.7天tick回绕丢失时长
} ParkingSlot;

/**
 * @brief 运动阶段枚举
 * 用于多步运动序列的状态跟踪
 */
typedef enum
{
    MOTION_PHASE_IDLE = 0,      // 空闲，无运动
    MOTION_PHASE_HORIZONTAL,    // 第1步：水平移动 (Motor_A)
    MOTION_PHASE_VERTICAL,      // 第2步：垂直移动 (Motor_B)
    MOTION_PHASE_RETURN_V,      // 第3步：垂直回零 (Motor_B)
    MOTION_PHASE_RETURN_H       // 第4步：水平回零 (Motor_A)
} MotionPhase;

typedef struct ParkingContext ParkingContext;
typedef uint8_t (*ParkingPersistCallback)(const ParkingContext *context);
typedef uint32_t (*ParkingClockCallback)(void);
typedef void (*ParkingNoticeCallback)(const ParkingContext *context, uint8_t kind);

struct ParkingContext
{
    ParkingState state;
    uint8_t selected_slot;
    uint8_t current_uid[4];
    ParkingSlot slots[PARKING_SLOT_COUNT];
    MotionPhase motion_phase;   // 当前运动阶段
    uint32_t next_command_id;
    uint32_t expected_command_id;
    uint32_t expected_command_type;
    MotorID expected_motor;
    uint8_t pending;            //0无事务，1存车，2取车
    uint8_t reconcile_required; //中断动作后须由维护接口核对
    uint8_t storage_fault;
    ParkingPersistCallback persist;
    ParkingClockCallback clock;
    ParkingNoticeCallback notice; /* 1存车成功，2取车成功，3故障；回调只入队 */
    uint32_t quote_cents, quote_seconds, selection_tick;
    uint8_t quote_uncertain;
};

/*
 * 运动请求回调。
 * ParkingTask 只提出运动请求，不直接操作 GPIO。
 * freertos.c 中的回调负责入队，MotionTask 独占执行电机输出。
 */
typedef enum {
    PARK_MOTION_MOVE = 0,
    PARK_MOTION_HOME,
    PARK_MOTION_RECOVER_HOME
} ParkingMotionType;

/* 返回0表示命令未入队，状态机必须停止推进。 */
typedef uint8_t (*ParkingMoveCallback)(ParkingMotionType type, MotorID motor,
                                    uint32_t steps,
                                    MotorDirection direction, uint32_t command_id);

/* 维护接口：由ParkingTask调用；仅FAULT、两轴停止且急停释放时接受。
 * 成功也不自动启动，仍需长按执行B/A回零。 */
uint8_t parking_reconcile(ParkingContext *context, const ParkingSlot *slots);

void parking_init(ParkingContext *context);
void parking_begin_home(ParkingContext *context, ParkingMoveCallback callback,
                        uint8_t recover);

void parking_handle_event(ParkingContext *context,
                          const AppEvent *event,
                          ParkingMoveCallback move_callback);

ParkingState parking_get_state(const ParkingContext *context);

/**
 * @brief 获取停车上下文指针（供UI任务访问车位状态）
 * @return 停车上下文的只读指针
 * @note 仅供UI任务读取车位占用状态，不可修改
 */
const ParkingContext* parking_get_context(void);
void parking_poll(ParkingContext *context, uint32_t now);
/* 仅等待缴费且时间不可信时允许维护人员核定费用。 */
uint8_t parking_set_fee(ParkingContext *context, uint32_t cents);

#endif /* PARKING_FSM_H */
