#ifndef __BSP_STEPPER_H
#define __BSP_STEPPER_H

#include "main.h"
#include <stdint.h>
#include "parking_config.h"

/* 电机编号定义 */
typedef enum {
    MOTOR_A = 0,  // 水平移动电机（第一个ULN2003）
    MOTOR_B = 1,   // 垂直移动电机（第二个ULN2003）
    MOTOR_COUNT
} MotorID;

/* 旋转方向定义 */
typedef enum {
    DIR_CW = 0,   // 顺时针
    DIR_CCW = 1   // 逆时针
} MotorDirection;

/* 电机状态定义 */
typedef enum {
    MOTOR_IDLE = 0,    // 空闲
    MOTOR_RUNNING = 1, // 运行中
    MOTOR_STOPPED = 2, // 已停止（不可当作运动完成）
    MOTOR_FAULT = 3
} MotorStatus;

typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_EMERGENCY,
    MOTOR_FAULT_LIMIT,
    MOTOR_FAULT_STOP,
    MOTOR_FAULT_NOT_HOMED,
    MOTOR_FAULT_TRAVEL,
    MOTOR_FAULT_HOME_NOT_FOUND,
    MOTOR_FAULT_HOME_STUCK,
    MOTOR_FAULT_TIMEOUT,
    MOTOR_FAULT_COMMAND
} MotorFault;

/* 两轴以 PARK_HOME_DIRECTION 寻找原点；退让后的点才是软件零点。 */

/* 步进模式定义 */
typedef enum {
    STEP_FULL = 0,  // 全步模式（四相四拍，扭矩大，功耗高）
    STEP_HALF = 1   // 半步模式（四相八拍，精度高，运行平滑）
} StepMode;

/* 电机配置结构体 */
typedef struct {
    GPIO_TypeDef* in1_port;  // IN1引脚端口
    uint16_t in1_pin;         // IN1引脚号
    GPIO_TypeDef* in2_port;  // IN2引脚端口
    uint16_t in2_pin;         // IN2引脚号
    GPIO_TypeDef* in3_port;  // IN3引脚端口
    uint16_t in3_pin;         // IN3引脚号
    GPIO_TypeDef* in4_port;  // IN4引脚端口
    uint16_t in4_pin;         // IN4引脚号
    uint16_t step_delay_us;   // 单步延时（微秒）
    uint32_t max_steps;       // 最大步数限制
    StepMode mode;            // 步进模式
} MotorConfig;

/* 电机运行状态结构体 */
typedef struct {
    MotorStatus status;       // 当前状态
    uint8_t current_phase;    // 当前相位索引（0-3全步 或 0-7半步）
    int32_t step_count;       // 当前位置（步数计数，可为负）
    volatile MotorDirection direction; // 方向同时由限位ISR读取
} MotorState;

/* 初始化函数 */
void stepper_init(void);

/* 电机控制函数 */
void stepper_move_steps(MotorID motor, uint32_t steps, MotorDirection dir);
void stepper_stop(MotorID motor);
void stepper_home(MotorID motor);

/* 可由任务/ISR 锁存停止；GPIO 只由运动任务关闭，循环不再重新励磁。 */
void stepper_request_stop(MotorFault fault);
void stepper_sensor_irq(void);
MotorFault stepper_get_fault(void);
uint8_t stepper_clear_fault(void);
uint8_t stepper_is_homed(MotorID motor);
void stepper_service_stop(void);

/* 工具函数 */
MotorStatus stepper_get_status(MotorID motor);
int32_t stepper_get_position(MotorID motor);
void stepper_set_speed(MotorID motor, uint16_t delay_us);
void stepper_set_mode(MotorID motor, StepMode mode);
void stepper_reset_position(MotorID motor);

#endif /* __BSP_STEPPER_H */
