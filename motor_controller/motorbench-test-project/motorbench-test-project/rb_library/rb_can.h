/* 
 * File:   rb_can.h
 * Author: Chris
 *
 * Created on June 20, 2024, 12:23 PM
 */

#ifndef RB_CAN_H
#define	RB_CAN_H

#include "rb_can_defs.h"

#include "stdint.h"
#include "stdbool.h"

uint16_t RB_CAN_Init(void);

bool RB_CAN_McpReset(void);

bool RB_CAN_McpSetReg(MCP_REGISTER reg, uint8_t data);

bool RB_CAN_McpSetRegs(MCP_REGISTER firstReg, const uint8_t* data, uint8_t n);

bool RB_CAN_McpModReg(MCP_REGISTER reg, uint8_t mask, uint8_t data);

bool RB_CAN_McpGetReg(MCP_REGISTER reg, const uint8_t* data);

void RB_CAN_Send(void);

void RB_CAN_Receive(void);

#endif	/* RB_CAN_H */