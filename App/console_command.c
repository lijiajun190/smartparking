#include "console_command.h"
#include <string.h>
#include <stddef.h>
static uint8_t number(const char *s, uint32_t *value)
{
    uint32_t n = 0U;
    if (!*s) return 0U;
    while (*s) {
        unsigned digit = (unsigned)(*s++ - '0');
        if (digit > 9U || n > (UINT32_MAX - digit) / 10U) return 0U;
        n = n * 10U + digit;
    }
    *value = n; return 1U;
}
static int hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static uint8_t uid_parse(const char *s, uint8_t *uid)
{
    unsigned i;
    if (strlen(s) != 8U) return 0U;
    for (i = 0U; i < 4U; i++) {
        int a = hex(s[i * 2U]), b = hex(s[i * 2U + 1U]);
        if (a < 0 || b < 0) return 0U;
        uid[i] = (uint8_t)(a * 16 + b);
    }
    return 1U;
}
uint8_t console_parse(const char *line, ConsoleCommand *c)
{
    char buffer[96], *args[4], *p;
    unsigned count = 0U, i;
    if (!line || !c || strlen(line) >= sizeof(buffer)) return 0U;
    memset(c, 0, sizeof(*c)); strcpy(buffer, line); p = buffer;
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        if (count == 4U) return 0U;
        args[count++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    if (!count) return 0U;
    if (count == 1U) {
        static const char *names[] = {"HELP", "STATUS", "PAY", "CANCEL", "HOME", "STOP", "CONFIRM"};
        static const ConsoleCommandType types[] = {CONSOLE_HELP, CONSOLE_STATUS, CONSOLE_PAY,
            CONSOLE_CANCEL, CONSOLE_HOME, CONSOLE_STOP, CONSOLE_CONFIRM};
        for (i = 0U; i < 7U; i++) if (!strcmp(args[0], names[i])) { c->type = types[i]; return 1U; }
    }
    if (count == 2U) {
        if (!strcmp(args[0], "CARD") && uid_parse(args[1], c->uid)) c->type = CONSOLE_CARD;
        else if (!strcmp(args[0], "TIME") && number(args[1], &c->value)) c->type = CONSOLE_TIME;
        else if (!strcmp(args[0], "FEE") && number(args[1], &c->value)) c->type = CONSOLE_FEE;
        else if (!strcmp(args[0], "RECON") && !strcmp(args[1], "BEGIN")) c->type = CONSOLE_RECON_BEGIN;
        else if (!strcmp(args[0], "RECON") && !strcmp(args[1], "COMMIT")) c->type = CONSOLE_RECON_COMMIT;
        else if (!strcmp(args[0], "FORMAT") && !strcmp(args[1], "CONFIRM")) c->type = CONSOLE_FORMAT;
        else if (!strcmp(args[0], "SMS") && !strcmp(args[1], "TEST")) c->type = CONSOLE_SMS_TEST;
        else if (!strcmp(args[0], "PHONE")) {
            const char *phone = args[1]; size_t length = strlen(phone);
            if (!strcmp(phone, "OFF")) { c->type = CONSOLE_PHONE; return 1U; }
            if (*phone == '+') phone++;
            if (length < 5U || length > 18U || !*phone) return 0U;
            while (*phone) { if (*phone < '0' || *phone > '9') return 0U; phone++; }
            strcpy(c->phone, args[1]); c->type = CONSOLE_PHONE;
        }
    }
    if (count == 3U && !strcmp(args[0], "SLOT") && number(args[1], &c->value) && c->value < 4U) {
        if (!strcmp(args[2], "-")) c->type = CONSOLE_RECON_SLOT;
        else if (uid_parse(args[2], c->uid)) { c->type = CONSOLE_RECON_SLOT; c->occupied = 1U; }
    }
    return c->type != CONSOLE_INVALID;
}
