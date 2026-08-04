/*
 * BIUController.h
 *
 *  Created on: 14-May-2026
 *      Author: gamer
 */

#ifndef INCLUDE_BIUCONTROLLER_H_
#define INCLUDE_BIUCONTROLLER_H_

#include <stdint.h>

/* ============================================================
 * CONSTANTS
 * ============================================================ */

#define MAX_TARGETS 16U
#define MAX_TARGET_DISTANCE 50000U

#define WARNING_ENTER_MARGIN 0.0f
#define WARNING_EXIT_MARGIN  10.0f

#define RELEASE_ENTER_SPEED_KMH 15U
#define RELEASE_EXIT_SPEED_KMH  18U

/* ============================================================
 * BIU COMMANDS
 * ============================================================ */

typedef enum
{
    BIU_CMD_RELEASE = 0,
    BIU_CMD_SERVICE,
    BIU_CMD_PENALTY,
    BIU_CMD_EMERGENCY

} BIU_Command_t;

/* ============================================================
 * TARGET TYPES
 * ============================================================ */

typedef enum
{
    TARGET_EOA = 0,
    TARGET_SVL,
    TARGET_PSR,
    TARGET_TSR,
    TARGET_LOOP,
    TARGET_SOS

} TargetType_t;

/* ============================================================
 * TARGET STRUCTURE
 * ============================================================ */

typedef struct
{
    TargetType_t type;
    uint32_t target_distance_m;
    uint32_t target_speed_kmh;
    uint8_t valid;
} Target_t;

/* ============================================================
 * TRAIN CONFIGURATION
 * ============================================================ */

typedef struct
{
    uint32_t train_length_m;
    uint32_t max_train_speed_kmh;
    float rotating_mass_percent;
    float eb_decel_step[4];
    float fsb_decel_step[4];
    uint32_t speed_step_kmh[4];
    float eb_tp;
    float eb_tb;
    float fsb_tp;
    float fsb_tb;
    float traction_cutoff_time;
    float warning_time;
    float driver_reaction_time;
    uint32_t release_speed_kmh;
} TrainConfig_t;

/* ============================================================
 * SUPERVISION CURVES
 * ============================================================ */

typedef struct
{
    float ebi_margin;
    float sbi_margin;
    float warning_margin;
    float permitted_margin;

} SupervisionCurves_t;

/* ============================================================
 * BIU CONTROL
 * ============================================================ */

typedef struct
{
    BIU_Command_t command;
    uint8_t service_level;
    uint8_t emergency_active;
    uint8_t penalty_active;
    uint8_t watchdog_ok;
    uint8_t healthy;
} BIU_Control_t;

/* ============================================================
 * GRADIENT
 * ============================================================ */

#define MAX_GRADIENT_SEGMENTS 32U

typedef struct
{
    uint32_t start_distance_m;
    uint32_t end_distance_m;
    /* per mille
       uphill  = positive
       downhill = negative
    */
    int16_t gradient_permille;

} GradientSegment_t;

extern GradientSegment_t g_gradient_profile[MAX_GRADIENT_SEGMENTS];
extern uint8_t g_gradient_count;

/* ============================================================
 * MRSP ENGINE
 * ============================================================ */

typedef struct
{
    uint32_t start_m;
    uint32_t end_m;
    uint32_t speed_kmh;
    uint8_t valid;

} MRSP_Section_t;

#define MAX_MRSP_SECTIONS 32U

extern MRSP_Section_t g_mrsp[MAX_MRSP_SECTIONS];
extern uint8_t g_mrsp_count;

/* ============================================================
 * GLOBALS
 * ============================================================ */

extern BIU_Control_t g_biu_ctrl;
extern TrainConfig_t g_train_config;
extern SupervisionCurves_t g_curves;
extern Target_t g_targets[MAX_TARGETS];
extern float ref_BIU_Target_distance[MAX_TARGETS];

/* ============================================================
 * API
 * ============================================================ */

void BIU_Init(void);
void Target_Set(uint8_t index, TargetType_t type, uint32_t target_distance_m, uint32_t speed_kmh);
void Targets_ClearAll(void);
void Target_Prune(void);
void BrakeSupervisor(void);
void BIUStateMachine(void);
void Supervision_Update(void);
float Gradient_GetBrakeCompensation(uint32_t position_m);
int16_t Gradient_GetWorstBrakingGradient(uint32_t front_pos_m);
uint32_t MRSP_GetSpeed(uint32_t pos_m);

#endif /* INCLUDE_BIUCONTROLLER_H_ */
