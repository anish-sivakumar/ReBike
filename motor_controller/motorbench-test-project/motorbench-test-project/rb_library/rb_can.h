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

/**
 * Initializes CAN, including MCP2515 CAN controller module.
 * @return the number of errors that occured during initialization. Probably 
 * shouldn't proceed if non zero.
 */
uint16_t RB_CAN_Init(void);

/**
 * Sets a register on the MCP2515
 * @param reg the register to set
 * @param data the data to fill the register with
 * @return true if write was successfull, false otherwise
 */
bool RB_CAN_McpSetReg(MCP_REGISTER reg, uint8_t data);

/**
 * Sets multiple consecutive registers on the MCP2515
 * @param firstReg the first register to set
 * @param data an array of data to fill the registers with
 * @param n the number of registers to write to
 * @return true if write was successfull, false otherwise
 */
bool RB_CAN_McpSetRegs(MCP_REGISTER firstReg, const uint8_t* data, uint8_t n);

/**
 * Modifies certain bits of a register on the MCP2515
 * @param reg the register to modify
 * @param mask specifies what bits to modify
 * @param data the data to modify the register with
 * @return 
 */
bool RB_CAN_McpModReg(MCP_REGISTER reg, uint8_t mask, uint8_t data);

/**
 * requests data from a register on the MCP2515
 * @param reg the register to get
 * @param data a buffer to fill with the retrieved data
 * @return true if read was successfull, false otherwise
 */
bool RB_CAN_McpGetReg(MCP_REGISTER reg, const uint8_t* data);

#endif	/* RB_CAN_H */