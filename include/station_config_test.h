#ifndef STATION_CONFIG_TEST_H
#define STATION_CONFIG_TEST_H

#include "station_config.h"

/*
 * TEST ONLY.  Set to 0 for production/NVM-loader builds.
 * When enabled, startup constructs and activates the incomplete Nawandgi
 * engineering configuration directly in RAM.
 */
#ifndef STATION_CONFIG_USE_NAWANDGI_TEST_CONFIG
#define STATION_CONFIG_USE_NAWANDGI_TEST_CONFIG 1U
#endif

station_config_result_t station_config_test_activate_nawandgi(
    station_config_validation_report_t *report);

#endif /* STATION_CONFIG_TEST_H */
