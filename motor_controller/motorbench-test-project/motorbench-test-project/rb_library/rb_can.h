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

typedef enum tagCAN_ID {
    // TODO: fill with can message IDs that we will need
    // Note: enum cannot be used if we want 32 bit IDs. 
    // If were using extended CAN identifiers, we'll need 
    // to change the IDs to #defines instead
    CAN_ID_BMS_SOC = 0x355,
    CAN_ID_UIC = 0x333

} CAN_ID;

typedef struct tagCAN_FRAME {
    CAN_ID id;
    uint8_t len;
    // TODO: confirm we are using only 8-bit data
    uint8_t data[8];
} CAN_FRAME;

/**
 * Reads throttle value from CAN message and updates throttleCmd_q15 
 * @param canFrame0
 * @param throttle
 */
void RB_CAN_ReadThrottle(CAN_FRAME *canFrame0, uint8_t *throttle);

/**
 * Checks to see if controller is ready to transmit
 * @param buffer
 * @return 
 */
bool RB_CAN_IsTxReady(uint8_t buffer);

/**
 * Loads CAN message with motor speed, power, and board temperature 
 * @param canFrameTx
 * @param speed
 * @param power
 * @param boardTemp
 */
void RB_CAN_LoadMotorParams(CAN_FRAME *canFrameTx, int16_t speed, int16_t power, int16_t boardTemp);

/**
 * Sends CAN message with loaded parameters  
 * @param buffer
 * @param frame
 */
void RB_CAN_SendTxFrame(uint8_t buffer, CAN_FRAME* frame);

#endif	/* RB_CAN_DEFS_H */

