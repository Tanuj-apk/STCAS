/* Include Files */
#include "i2c.h"
#include "i2c_UD.h"

#define GPS_EPOCH_YEAR 1980
#define GPS_EPOCH_DAY_OFFSET 5U   /* 6 Jan = day 5 from Jan 1 */
#define SECONDS_PER_DAY 86400UL

static const uint8_t days_in_month[12] =
{
    31, /* Jan */
    28, /* Feb */
    31, /* Mar */
    30, /* Apr */
    31, /* May */
    30, /* Jun */
    31, /* Jul */
    31, /* Aug */
    30, /* Sep */
    31, /* Oct */
    30, /* Nov */
    31  /* Dec */
};
static inline uint8_t is_leap_year(uint16_t year)
{
    return ((year % 4U == 0U && year % 100U != 0U) ||
            (year % 400U == 0U));
}


uint32_t calendar_to_seconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    uint32_t days = 0;
    uint16_t base_year;

    /* Years since GPS epoch (1980) */
    for (base_year = GPS_EPOCH_YEAR; base_year < year; base_year++)
    {
        days += is_leap_year(base_year) ? 366U : 365U;
    }

    /* Months of current year */
    uint8_t m;
    for (m = 1; m < month; m++)
    {
        days += days_in_month[m - 1];
        if (m == 2 && is_leap_year(year))
        {
            days += 1U;
        }
    }

    /* Days of current month */
    days += (uint32_t)(day - 1U);

    /* Remove GPS epoch offset (Jan 6 = day 5) */
    if (days < GPS_EPOCH_DAY_OFFSET)
        return 0;   /* before GPS epoch, invalid */

    days -= GPS_EPOCH_DAY_OFFSET;

    return (days * SECONDS_PER_DAY) +
           ((uint32_t)hour * 3600UL) +
           ((uint32_t)min  * 60UL) +
           (uint32_t)sec;
}
void seconds_to_calendar(uint32_t seconds, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t  *sec)
{
    uint32_t days;
    uint16_t base_year;
    uint8_t  m;

    /* Add GPS epoch offset (Jan 6 = +5 days) */
    seconds += (GPS_EPOCH_DAY_OFFSET * SECONDS_PER_DAY);

    *sec  = seconds % 60U;
    seconds /= 60U;
    *min  = seconds % 60U;
    seconds /= 60U;
    *hour = seconds % 24U;
    days  = seconds / 24U;

    /* Year starting from 1980 */
    for (base_year = GPS_EPOCH_YEAR; ; base_year++)
    {
        uint16_t year_days = is_leap_year(base_year) ? 366U : 365U;
        if (days < year_days)
            break;
        days -= year_days;
    }
    *year = base_year;

    /* Month */
    for (m = 1; m <= 12; m++)
    {
        uint8_t dim = days_in_month[m - 1];
        if (m == 2 && is_leap_year(*year))
            dim++;

        if (days < dim)
            break;
        days -= dim;
    }
    *month = m;

    /* Day */
    *day = (uint8_t)(days + 1U);
}

//Not Used
void rtc_set_time_sec(uint32_t unix_seconds)
{
    uint16_t year;
    uint8_t month, day, hour, min, sec;

    seconds_to_calendar(unix_seconds, &year, &month, &day, &hour, &min, &sec);

    rtc_set[0] = sec;
    rtc_set[1] = min;
    rtc_set[2] = hour;
    rtc_set[4] = day;
    rtc_set[5] = month;
    rtc_set[6] = (uint8_t)(year - 2000U); /* DS1307: 00 = 2000 */

    RTC_WriteByte(rtc_set, 0);
    RTC_WriteByte(rtc_set, 1);
    RTC_WriteByte(rtc_set, 2);
    RTC_WriteByte(rtc_set, 4);
    RTC_WriteByte(rtc_set, 5);
    RTC_WriteByte(rtc_set, 6);
}
uint8_t rtc_get_time_sec(uint32_t *unix_seconds)
{
    uint16_t year;
    uint8_t month, day, hour, min, sec;

    RTC_ReadByte(rtc_raw, 0);
    RTC_ReadByte(rtc_raw, 1);
    RTC_ReadByte(rtc_raw, 2);
    RTC_ReadByte(rtc_raw, 4);
    RTC_ReadByte(rtc_raw, 5);
    RTC_ReadByte(rtc_raw, 6);

    sec   = BCD2Binary(rtc_raw[0] & 0x7FU);
    min   = BCD2Binary(rtc_raw[1] & 0x7FU);
    hour  = BCD2Binary(rtc_raw[2] & 0x3FU);
    day   = BCD2Binary(rtc_raw[4] & 0x3FU);
    month = BCD2Binary(rtc_raw[5] & 0x1FU);
    year  = 2000U + BCD2Binary(rtc_raw[6]);

    /* Basic sanity check */
    if (month == 0 || month > 12 || day == 0 || day > 31)
        return 0;

    *unix_seconds = calendar_to_seconds(year, month, day, hour, min, sec);
    return 1;
}
//

//Remove delayms when not needed
/* USER CODE BEGIN*/
/* Approximate millisecond delay using busy-wait */
void delay_ms(uint32_t ms)
{
    volatile uint32 j;
    while (ms--)
    {
        for (j = 0; j < 4000U; j++)
        {
            __asm(" nop");
        }
    }
}

/* Convert binary value to BCD format (DS1307 requirement) */
uint16_t Binary2BCD(uint16_t bin)
{
    uint16_t temp1, temp2;
    temp1 = bin % 10;
    temp1 = temp1 & 0x0F;
    bin = bin / 10;
    temp2 = bin % 10;
    temp2 = 0x0F & temp2;
    temp2 = temp2 << 4;
    temp2 = 0xF0 & temp2;
    temp1 = temp1 | temp2;
    return temp1;
}

/* Convert BCD value read from RTC to binary */
uint16_t BCD2Binary(uint16_t bcd)
{
    uint16_t temp1, temp2;
    temp1 = bcd & 0x0F;
    temp2 = temp1;
    bcd = 0xF0 & bcd;
    temp1 = bcd >> 4;
    temp1 = 0x0F & temp1;
    temp2 = temp1 * 10 + temp2;
    return temp2;
}

/* Write one byte to DS1307:
 * Sequence:
 * START → SLA+W → register address → data → STOP
 */
void RTC_WriteByte(uint8_t* data, uint8 bit)
{
    /* Ensure I2C bus is idle */
    while (i2cIsBusBusy(i2cREG1));

    /* Convert data to BCD */
    uint32_t BCDdata = Binary2BCD(data[bit]);

    /* Set RTC slave address */
    i2cSetSlaveAdd(i2cREG1, SLAVE_ADDR);

    /* Two bytes: register address + data */
    i2cSetCount(i2cREG1, 2);

    /* Load register address */
    i2cREG1->DXR = bit;

    /* Configure I2C as master-transmitter and start transaction */
    i2cSetMode(i2cREG1, I2C_MASTER);
    i2cSetDirection(i2cREG1, I2C_TRANSMITTER);
    i2cSetStart(i2cREG1);
    i2cSetStop(i2cREG1);

    /* Wait until register address has been transmitted */
    while (!(i2cIsTxReady(i2cREG1)));

    /* Transmit data byte */
    i2cREG1->DXR = BCDdata;

    /* Wait for STOP condition */
    while (!(i2cIsStopDetected(i2cREG1)));

    /* Ensure bus is released */
    while (i2cIsBusBusy(i2cREG1));
}

/* Read one byte from DS1307 using repeated START:
 * START → SLA+W → register address
 * RESTART → SLA+R → data → NACK → STOP
 */
void RTC_ReadByte(uint8_t *buf, uint8_t bit)
{
    /* Ensure I2C bus is idle */
    while (i2cIsBusBusy(i2cREG1));

    /* Set RTC slave address */
    i2cSetSlaveAdd(i2cREG1, SLAVE_ADDR);

    /* ---------- Phase 1: write register pointer ---------- */
    i2cSetCount(i2cREG1, 1);
    i2cREG1->DXR = bit;
    i2cSetDirection(i2cREG1, I2C_TRANSMITTER);
    i2cSetStart(i2cREG1);
    i2cSetMode(i2cREG1, I2C_MASTER);

    /* Disable STOP and NACK for repeated START sequence */
    i2cREG1->MDR &= ~(uint32_t)(1U << 11U);
    i2cREG1->MDR &= ~(uint32_t)(1U << 15U);

    /* Wait until address phase is complete */
    while ((i2cREG1->STR & (uint32)I2C_ARDY_INT) == 0U);

    /* ---------- Phase 2: repeated START + read ---------- */
    i2cSetCount(i2cREG1, 1);
    i2cSetMode(i2cREG1, I2C_MASTER);
    i2cSetDirection(i2cREG1, I2C_RECEIVER);
    i2cSetStart(i2cREG1);
    i2cSetStop(i2cREG1);

    /* Enable NACK for single-byte read */
    i2cREG1->MDR |= (uint32)I2C_NACK_MODE;

    /* Wait for received data */
    while (!(i2cIsRxReady(i2cREG1)));

    /* Read received byte */
    buf[bit] = (uint8_t)i2cREG1->DRR;

    /* Ensure STOP has completed */
    while (i2cIsBusBusy(i2cREG1));
}
/* USER CODE END */
