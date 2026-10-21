#ifndef APP_EVENTS_H
#define APP_EVENTS_H


#include <stdint.h>


//brief应用层事件类型：所有输入设备和安全模块都通过AppEvent，向后续的ParkingTask传递事件
typedef enum
{
    APP_EVENT_NONE = 0,

    APP_EVENT_KEY_PRESS,
    APP_EVENT_KEY_RELEASE,
    APP_EVENT_KEY_LONG_PRESS,

    /*RIFD事件*/
    APP_EVENT_CARD_DETECTED,

    //传感器事件
    APP_EVENT_SENSOR_CHANGED,

    //急停事件
    APP_EVENT_EMERGENCY_ON,
    APP_EVENT_EMERGENCY_OFF,

    //步进电机事件
    APP_EVENT_MOTION_DONE,
    APP_EVENT_MOTION_FAULT,
    APP_EVENT_MOTION_TIMEOUT
}AppEventType;

//brief应用层同一事件结构
typedef struct
{
    AppEventType type;  //事件类型
    uint32_t tick;  //事件发生时的HAL tick
    uint32_t value;    /*附加数据
                        *按键事件：保存KeyID
                        *传感器事件：保存变化位图
                        *急停事件：0或1
                        */
    uint8_t uid[4];  //保存完整的4字节UID                    
    uint32_t command_id;   /* 运动命令序号，0 表示非运动事件 */
    uint32_t command_type; /* ParkingMotionType */
}AppEvent;

#endif
