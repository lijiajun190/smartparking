#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdint.h>
#include "bsp_sensors.h"
#include "bsp_stepper.h"

typedef enum
{
    SAFETY_OK = 0,
    SAFETY_FAULT_EMERGENCY,
    SAFETY_FAULT_LIMIT,
    SAFETY_FAULT_MOTOR
} SafetyStatus;

/*
 * 检查当前系统是否允许电机继续运动。
 * 返回 0 表示允许，非 0 表示必须停止。
 */
SafetyStatus safety_check_motor(MotorID motor,
                                MotorDirection direction);

#endif /* APP_SAFETY_H */