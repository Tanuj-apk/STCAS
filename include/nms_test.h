#ifndef NMS_TEST_H
#define NMS_TEST_H

#include <stdint.h>

/* Set to 1U only in a bench-test build.  When enabled, NMS_Test_1sTask()
 * queues one message every NMS_TEST_PERIOD_SEC seconds and cycles through all
 * supported NMS message types. */
#define NMS_TEST_ENABLE       1U
#define NMS_TEST_PERIOD_SEC   5U

extern volatile uint8_t nms_test_last_message_type;
extern volatile uint16_t nms_test_message_sequence;

void NMS_Test_Init(void);
void NMS_Test_1sTask(void);
void NMS_Test_QueueNextMessage(void);
void NMS_Test_QueueMessage(uint8_t message_type);

#endif
