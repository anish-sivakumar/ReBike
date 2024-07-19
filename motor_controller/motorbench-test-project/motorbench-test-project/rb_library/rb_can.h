/* 
 * File:   rb_can.h
 * Author: chygg
 *
 * Created on June 26, 2024, 2:49 PM
 */

#ifndef RB_CAN_DEFS_H
#define	RB_CAN_DEFS_H

#include <stdbool.h>
#include <stdint.h>
#include "rb_logging.h"

#define RB_CAN_CYCLE_COUNT 2048
#define RB_CAN_CYCLE_COUNT_MINUS1 RB_CAN_CYCLE_COUNT - 1


typedef enum tagCAN_ID {
    // TODO: fill with can message IDs that we will need
    // Note: enum cannot be used if we want 32 bit IDs. 
    // If were using extended CAN identifiers, we'll need 
    // to change the IDs to #defines instead
    CAN_ID_BMS_SOC = 0x355,
    CAN_ID_UIC = 0x334,
    CAN_ID_BIKE_STATUS = 0x330,
    CAN_ID_MOTOR_VOLTAGES = 0x331,
    CAN_ID_MOTOR_REAL_CURRENTS = 0x332,
    CAN_ID_MOTOR_CALC_VALUES = 0x333
} CAN_ID;

typedef struct tagCAN_FRAME {
    CAN_ID id;
    uint8_t len;
    // TODO: confirm we are using only 8-bit data
    uint8_t data[8];
} CAN_FRAME;

typedef enum {
    RBCAN_MESSAGE1,
    RBCAN_MESSAGE2,
    RBCAN_MESSAGE3,
    RBCAN_MESSAGE4,
    RBCAN_IDLE
} RB_CAN_FSM;

typedef struct tagRB_CAN_CONTROL {
    RB_CAN_FSM state;
    uint16_t timestamp;
    uint16_t counter;
} RB_CAN_CONTROL;

/**
 * Returns throttle value from CAN message otherwise value 0
 * @param canFrame0
 * @param throttle
 * @return
 */
int8_t RB_CAN_ReadThrottle(CAN_FRAME *canFrame0);

/**
 * Sends CAN message with specified values 
 * @param buffer
 * @param can_id
 * @param timestamp
 * @param speed
 * @param bridgeTemp
 * @param throttleInput
 * @param errorWarning
 */
bool RB_CAN_SendCANMessageV1(uint8_t buffer, CAN_ID can_id, uint16_t timestamp, uint16_t speed, uint16_t bridgeTemp, int8_t throttleInput, uint8_t errorWarning);

/**
 * Sends CAN message with specified values 
 * @param buffer
 * @param can_id
 * @param timestamp
 * @param value1
 * @param value2
 * @param value3
 */
bool RB_CAN_SendCANMessageV2(uint8_t buffer, CAN_ID can_id, uint16_t timestamp, int16_t value1, int16_t value2, int16_t value3);

/**
 * Carries out CAN Service Routine  
 * @param canFrame0
 * @param throttleCmd
 * @param CANControl
 * @param throttleInput
 * @param errorWarning
 * @param avg
 */
void RB_CAN_Service(CAN_FRAME *canFrame0, int8_t *throttleCmd, RB_CAN_CONTROL *CANControl, int8_t throttleInput, uint8_t errorWarning, RB_LOGGING_AVERAGES avg);

/**
 * Sends CAN message to controller 
 * @param buffer
 * @param frame
 */
void RB_CAN_SendTxFrame(uint8_t buffer, CAN_FRAME* frame);

#endif	/* RB_CAN_DEFS_H */

