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
 * @return the number of errors that occurred during initialization. Probably 
 * shouldn't proceed if non zero.
 */
uint16_t RB_CAN_Init(void);

/**
 * Sends a reset command to the MCP2515
 * @return true on successful reset, false otherwise.
 */
bool RB_CAN_McpReset();

/**
 * Sets the mode of the MCP2515. After reset, the mode needs to be set to
 * MCP_CAN_MODE_NORMAL in order to send or receive messages. 
 * @param mode the mode to set 
 * @return true on successful mode set, false otherwise
 */
bool RB_CAN_McpSetMode(MCP_CAN_MODE mode);

/**
 * Sets a register on the MCP2515
 * @param reg the register to set
 * @param data the data to fill the register with
 * @return true if write was successful, false otherwise
 */
bool RB_CAN_McpSetReg(MCP_REGISTER reg, uint8_t data);

/**
 * Sets multiple consecutive registers on the MCP2515
 * @param firstReg the first register to set
 * @param data an array of data to fill the registers with
 * @param n the number of registers to write to
 * @return true if write was successful, false otherwise
 */
bool RB_CAN_McpSetRegs(MCP_REGISTER firstReg, uint8_t* data, uint8_t n);

/**
 * Modifies certain bits of a register on the MCP2515
 * @param reg the register to modify
 * @param mask specifies what bits to modify
 * @param data the data to modify the register with
 * @return 
 */
bool RB_CAN_McpModReg(MCP_REGISTER reg, uint8_t mask, uint8_t data);

/**
 * Requests data from a register on the MCP2515
 * @param reg the register to get
 * @param data a buffer to fill with the retrieved data
 * @return true if read was successful, false otherwise
 */
bool RB_CAN_McpGetReg(MCP_REGISTER reg, uint8_t* data);


/**
 * Requests data from multiple registers on the MCP2515, starting at firstReg
 * @param firstReg the address of the first register to get 
 * @param data a buffer to fill with the retrieved data
 * @return true if read was successful, false otherwise
 */
bool RB_CAN_McpGetRegs(MCP_REGISTER firstReg, const uint8_t* data, uint8_t n);

/**
 * Reads the "read status" register of the MCP2515. By applying the flags in the
 * MCP_STAT enum to the returned status, we can tell which buffers have received 
 * messages
 * @param status the returned status
 * @return true if the status was successfully read, false otherwise
 */
bool RB_CAN_McpReadStat(uint8_t* status);


/**
 * Reads a CAN message from the specified rx buffer 
 * @param rxBufId the ID of the receiving buffer, only 0 and 1 are valid
 * @param frame the received CAN message frame
 * @return true if a valid message was received, false otherwise
 */
bool RB_CAN_McpReadRx(uint16_t rxBufId, CAN_FRAME* frame);

/**
 * Loads a CAN message into the specified tx buffer
 * @param txBufId the ID of the transmitting buffer, only 0, 1 and 2 are valid
 * @param frame the frame to transmit 
 * @return true if the message was successfully loaded, false otherwise
 */
bool RB_CAN_McpLoadTx(uint16_t txBufId, const CAN_FRAME* frame);

/**
 * Sends a request to the MCP2515 to send the messages in the tx buffers.
 * @return 
 */
bool RB_CAN_McpSend();


#endif	/* RB_CAN_H */