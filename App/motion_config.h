#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "parking_config.h"
#include "bsp_stepper.h"
#include "bsp_sensors.h"

typedef struct {
    uint32_t motor_a_steps; /* 水平，退让后的零点为0 */
    uint32_t motor_b_steps; /* 垂直，退让后的零点为0 */
} ParkingPosition;

typedef struct {
    ParkingPosition position;
    SensorId occupancy_sensor; /* COUNT: 无独立占用反馈，禁止复用轴限位 */
    uint8_t priority;          /* 数字越小越先分配 */
} ParkingSlotConfig;

/* 槽位索引：0 L11左上、1 L12右上、2 L21左下、3 L22右下。
 * 以下步数仅为调试初值；真实载车板交接路径、负载与行程需实机标定。 */
static const ParkingSlotConfig PARKING_SLOTS[PARKING_SLOT_COUNT] = {
    {{1000U, 2000U}, SENSOR_IR_1, 0U},
    {{2500U, 2000U}, SENSOR_IR_2, 2U},
    {{1000U,    0U}, SENSOR_ID_COUNT, 1U},
    {{2500U,    0U}, SENSOR_ID_COUNT, 3U}
};

static inline ParkingPosition motion_get_slot_position(uint8_t slot_id)
{
    ParkingPosition invalid = {0U, 0U};
    return slot_id < PARKING_SLOT_COUNT ? PARKING_SLOTS[slot_id].position : invalid;
}

static inline uint8_t motion_config_valid(void)
{
    uint8_t i, j;
    if ((PARK_HOME_DIRECTION != DIR_CW && PARK_HOME_DIRECTION != DIR_CCW) ||
        (PARK_AWAY_DIRECTION != DIR_CW && PARK_AWAY_DIRECTION != DIR_CCW) ||
        PARK_HOME_DIRECTION == PARK_AWAY_DIRECTION ||
        (PARK_AXIS_A_LIMIT != SENSOR_LIMIT_1 && PARK_AXIS_A_LIMIT != SENSOR_LIMIT_2) ||
        (PARK_AXIS_B_LIMIT != SENSOR_LIMIT_1 && PARK_AXIS_B_LIMIT != SENSOR_LIMIT_2) ||
        PARK_AXIS_A_LIMIT == PARK_AXIS_B_LIMIT)
        return 0U;
    if (PARK_AXIS_MAX_STEPS + STEPPER_HOME_BACKOFF_STEPS > STEPPER_HOME_SEEK_STEPS)
        return 0U;
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        const ParkingSlotConfig *s = &PARKING_SLOTS[i];
        if (s->position.motor_a_steps > PARK_AXIS_MAX_STEPS ||
            s->position.motor_b_steps > PARK_AXIS_MAX_STEPS ||
            s->priority >= PARKING_SLOT_COUNT ||
            (s->occupancy_sensor != SENSOR_ID_COUNT &&
             s->occupancy_sensor != SENSOR_IR_1 && s->occupancy_sensor != SENSOR_IR_2))
            return 0U;
        for (j = 0U; j < i; j++)
            if (s->priority == PARKING_SLOTS[j].priority ||
                (s->occupancy_sensor != SENSOR_ID_COUNT &&
                 s->occupancy_sensor == PARKING_SLOTS[j].occupancy_sensor))
                return 0U;
    }
    return 1U;
}
#endif
