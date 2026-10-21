#ifndef CONSOLE_COMMAND_H
#define CONSOLE_COMMAND_H
#include <stdint.h>
typedef enum { CONSOLE_INVALID, CONSOLE_HELP, CONSOLE_STATUS, CONSOLE_CARD,
    CONSOLE_PAY, CONSOLE_CANCEL, CONSOLE_HOME, CONSOLE_STOP, CONSOLE_TIME,
    CONSOLE_PHONE, CONSOLE_RECON_BEGIN, CONSOLE_RECON_SLOT, CONSOLE_RECON_COMMIT,
    CONSOLE_FEE, CONSOLE_FORMAT, CONSOLE_CONFIRM, CONSOLE_SMS_TEST } ConsoleCommandType;
typedef struct {
    ConsoleCommandType type;
    uint32_t value;
    uint8_t uid[4], occupied;
    char phone[20];
} ConsoleCommand;
/* 严格解析完整行，不执行动作；拒绝溢出、尾随字段和AT控制字符。 */
uint8_t console_parse(const char *line, ConsoleCommand *command);
#endif
