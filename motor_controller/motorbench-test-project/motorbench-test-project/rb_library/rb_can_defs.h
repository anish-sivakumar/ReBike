/* 
 * File:   rb_can_defs.h
 * Author: chygg
 *
 * Created on June 26, 2024, 2:49 PM
 */

#ifndef RB_CAN_DEFS_H
#define	RB_CAN_DEFS_H

typedef enum tagCAN_ID {
    // TODO: fill with can message IDs that we will need
    // Note: enum cannot be used if we want 32 bit IDs. 
    // If were using extended CAN identifiers, we'll need 
    // to change the IDs to #defines instead
    CAN_ID_TEST
} CAN_ID;

typedef struct tagCAN_FRAME{
    CAN_ID id;
    uint8_t len;
    // TODO: confirm we are using only 8-bit data
    uint8_t data[8]; 
} CAN_FRAME;

#endif	/* RB_CAN_DEFS_H */

