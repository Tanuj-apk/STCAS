#ifndef HEADERS_I2C_UD_H_
#define HEADERS_I2C_UD_H_

#include <stdint.h>
#include "i2c.h"

/* DS1307 I2C slave address */
#define SLAVE_ADDR  0x68

/* Number of RTC registers accessed (0x00–0x06) */
#define RX_LEN      7

/* Decoded RTC values in binary format */
uint8_t seconds, minutes, hours;
uint8_t date, month, year;

uint8_t start_rtc_read;
uint8_t rtc_read_count;
uint8_t start_rtc_write;
uint8_t rtc_write_count;

/* Raw RTC register values read from DS1307 (BCD format) */
uint8_t rtc_raw[RX_LEN];

/* Initial RTC values to be written (binary format)
 * Index mapping:
 * 0 = seconds
 * 1 = minutes
 * 2 = hours
 * 3 = day-of-week (intentionally skipped)
 * 4 = date
 * 5 = month
 * 6 = year
 */
uint8_t rtc_set[RX_LEN];

/* Simple blocking delay (approximate, CPU-frequency dependent) */
void delay_ms(uint32_t ms);

/* Write a single RTC register */
void RTC_WriteByte(uint8_t* data, uint8_t bit);

/* Read a single RTC register */
void RTC_ReadByte(uint8_t *buf, uint8_t bit);

/* Binary <-> BCD conversion helpers */
uint16_t Binary2BCD(uint16_t a);
uint16_t BCD2Binary(uint16_t a);
uint32_t calendar_to_seconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

void seconds_to_calendar(uint32_t seconds, uint16_t *year, uint8_t  *month, uint8_t  *day, uint8_t  *hour, uint8_t  *min, uint8_t  *sec);
void rtc_set_time_sec(uint32_t unix_seconds);
uint8_t rtc_get_time_sec(uint32_t *unix_seconds);

#endif /* HEADERS_I2C_UD_H_ */
