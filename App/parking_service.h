#ifndef PARKING_SERVICE_H
#define PARKING_SERVICE_H
#include "parking_fsm.h"
typedef uint8_t (*ParkingIdleCallback)(void);
typedef void (*ParkingDiagnosticsCallback)(void);
uint8_t parking_service_init(ParkingIdleCallback idle, ParkingDiagnosticsCallback diagnostics);
void parking_service_poll(ParkingContext *context, ParkingMoveCallback move);
void parking_service_notice(const ParkingContext *context, uint8_t kind);
uint8_t parking_service_sms_status(void); /* 0未配号码，1待连接，2就绪，3发送失败 */
#endif
