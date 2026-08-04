#ifndef PULSE_GENERATOR_H
#define PULSE_GENERATOR_H

#include <stdint.h>

/* ================= CONFIG ================= */

#define PPR                 30U
#define RESOLUTION          2U
#define COUNTS_PER_REV      (PPR * RESOLUTION)

#define SYSCLK_HZ           107000000.0f
#define CAP_PRESCALER       8.0f
#define CAP_CLK             (SYSCLK_HZ / CAP_PRESCALER)

#define WHEEL_DIA_M         1.010f
#define PI                  3.14159265f

#define RPM_FAULT_THRESHOLD 5.0f

/* ================= VARIABLES ================= */

extern float rpm_ch1;
extern float rpm_ch2;
extern float speed_kmh;
extern float speed_ms;
extern float distance_m;
extern float distance_km;

extern uint8_t direction;
extern uint8_t sensor_fault;

extern uint8_t reverse_distance_flag;
extern float speed_ms_filtered;

/* ================= FUNCTIONS ================= */

void eqep_speed_init(void);
void eqep_speed_update(void);

#endif
