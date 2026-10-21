#include "safety.h"

SafetyStatus safety_check_motor(MotorID motor,
                                MotorDirection direction)
{
    if (sensor_emergency_active() != 0U)
    {
        return SAFETY_FAULT_EMERGENCY;
    }

    /* 与驱动共享原点方向；离开原点时允许脱离已触发的开关。 */
    if (motor == MOTOR_A)
    {
        if ((direction == PARK_HOME_DIRECTION) &&
            (sensor_read(PARK_AXIS_A_LIMIT) != 0U))
        {
            return SAFETY_FAULT_LIMIT;
        }
    }
    else if (motor == MOTOR_B)
    {
        if ((direction == PARK_HOME_DIRECTION) &&
            (sensor_read(PARK_AXIS_B_LIMIT) != 0U))
        {
            return SAFETY_FAULT_LIMIT;
        }
    }
    else
    {
        return SAFETY_FAULT_MOTOR;
    }

    return SAFETY_OK;
}
