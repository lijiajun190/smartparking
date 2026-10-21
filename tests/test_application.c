#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "billing.h"
#include "console_command.h"
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static unsigned tests;
static void pass(const char *s) { tests++; printf("PASS %s\n", s); }
static void billing_tests(void)
{
    BillingConfig rate = {1800U, 500U};
    unsigned y, m, d, h, min, sec;
    uint32_t epoch;
    CHECK(billing_calculate_cents(0U, &rate) == 0U);
    CHECK(billing_calculate_cents(1U, &rate) == 500U);
    CHECK(billing_calculate_cents(1800U, &rate) == 500U);
    CHECK(billing_calculate_cents(1801U, &rate) == 1000U);
    CHECK(billing_calculate_cents(86400U, &rate) == 24000U);
    rate.unit_seconds = 1U; rate.rate_cents = UINT32_MAX;
    CHECK(billing_calculate_cents(2U, &rate) == UINT32_MAX);
    rate.unit_seconds = 0U;
    CHECK(billing_calculate_cents(1U, &rate) == UINT32_MAX);
    CHECK(billing_calculate_cents(1U, NULL) == UINT32_MAX);
    CHECK(billing_elapsed_seconds(0xFFFFFF00U, 1744U) == 2U);
    pass("billing rounds partial units upward, saturates overflow, rejects invalid rate and handles tick wrap");

    CHECK(billing_epoch(2000, 1, 1, 0, 0, 0) == 946684800U);
    CHECK(billing_epoch(2024, 2, 29, 0, 0, 0) == 1709164800U);
    CHECK(billing_epoch(2099, 12, 31, 23, 59, 59) == 4102444799U);
    CHECK(!billing_epoch(2023, 2, 29, 0, 0, 0));
    CHECK(!billing_epoch(2024, 4, 31, 0, 0, 0));
    CHECK(!billing_epoch(2100, 1, 1, 0, 0, 0));
    CHECK(!billing_epoch(2024, 0, 1, 0, 0, 0));
    CHECK(!billing_epoch(2024, 1, 1, 24, 0, 0));
    CHECK(!billing_calendar(946684799U, &y, &m, &d, &h, &min, &sec));
    CHECK(!billing_calendar(4102444800U, &y, &m, &d, &h, &min, &sec));
    for (epoch = 946684800U; epoch < 4102358399U; epoch += 86400U) {
        CHECK(billing_calendar(epoch, &y, &m, &d, &h, &min, &sec));
        CHECK(billing_epoch(y, m, d, h, min, sec) == epoch);
    }
    CHECK(billing_calendar(4102444799U, &y, &m, &d, &h, &min, &sec));
    CHECK(y == 2099U && m == 12U && d == 31U && h == 23U && min == 59U && sec == 59U);
    pass("known Unix dates, leap days, invalid calendars and every supported calendar day are checked");
}
static void parser_tests(void)
{
    ConsoleCommand c;
    const char *invalid[] = {"", " ", "CARD", "CARD 112233", "CARD 112233GG",
        "CARD 11223344 PAY", "TIME -1", "TIME 4294967296", "TIME 12x", "PAY NOW",
        "SLOT 4 -", "SLOT -1 -", "SLOT 0 aaaaaaaa extra", "RECON", "RECON FINISH",
        "FORMAT", "FORMAT YES", "PHONE +", "PHONE 1234\"", "PHONE 12345\rAT",
        "PHONE +123456789012345678", "PHONE OFF extra", "FEE -1", "STOP\nHOME", "SMS SEND"};
    unsigned i;
    CHECK(console_parse("  CARD aAbB1234\t", &c) && c.type == CONSOLE_CARD);
    CHECK(c.uid[0] == 0xAAU && c.uid[1] == 0xBBU && c.uid[2] == 0x12U && c.uid[3] == 0x34U);
    CHECK(console_parse("TIME 1792454400", &c) && c.value == 1792454400U);
    CHECK(console_parse("FEE 4294967295", &c) && c.value == UINT32_MAX);
    CHECK(console_parse("CONFIRM", &c) && c.type == CONSOLE_CONFIRM);
    CHECK(console_parse("PAY", &c) && c.type == CONSOLE_PAY);
    CHECK(console_parse("STOP", &c) && c.type == CONSOLE_STOP);
    CHECK(console_parse("HOME", &c) && c.type == CONSOLE_HOME);
    CHECK(console_parse("CANCEL", &c) && c.type == CONSOLE_CANCEL);
    CHECK(console_parse("HELP", &c) && c.type == CONSOLE_HELP);
    CHECK(console_parse("STATUS", &c) && c.type == CONSOLE_STATUS);
    CHECK(console_parse("SMS TEST", &c) && c.type == CONSOLE_SMS_TEST);
    CHECK(console_parse("PHONE +8613800138000", &c) && !strcmp(c.phone, "+8613800138000"));
    CHECK(console_parse("PHONE OFF", &c) && !c.phone[0]);
    pass("console parses complete business, diagnostics, clock and phone commands");
    CHECK(console_parse("RECON BEGIN", &c) && c.type == CONSOLE_RECON_BEGIN);
    CHECK(console_parse("SLOT 3 11223344", &c) && c.value == 3U && c.occupied && c.uid[3] == 0x44U);
    CHECK(console_parse("SLOT 0 -", &c) && !c.occupied && !c.uid[0]);
    CHECK(console_parse("RECON COMMIT", &c) && c.type == CONSOLE_RECON_COMMIT);
    CHECK(console_parse("FORMAT CONFIRM", &c) && c.type == CONSOLE_FORMAT);
    for (i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); i++) CHECK(!console_parse(invalid[i], &c));
    CHECK(!console_parse(NULL, &c)); CHECK(!console_parse("STATUS", NULL));
    pass("maintenance commands are explicit; malformed UID, overflow, trailing fields and AT injection are rejected");
}
int main(void)
{
    billing_tests(); parser_tests();
    printf("%u application test groups passed\n", tests);
    return 0;
}
