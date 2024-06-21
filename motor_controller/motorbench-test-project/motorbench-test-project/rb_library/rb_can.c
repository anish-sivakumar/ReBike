#include "rb_can.h"

#include "system/pins.h"
#include "spi_host/spi1.h"

#include "string.h" 


// Static function defines
// These can only be used inside this file
static bool StartTransaction();
static void EndTransaction();

uint16_t RB_CAN_Init(void) {
    // Check for errors during initialization
    uint16_t errors = 0;
    SPI1_CS_SetHigh();
    errors += !StartTransaction();
    SPI1_ByteWrite(MCP_INSTR_RESET);
    EndTransaction();

    // TODO: MCP2515 arduino lib has a delay here, likely need something similar
    // Datasheet says it takes 2us to reset
    uint8_t zeros[14];
    memset(zeros, 0, sizeof (zeros));
    errors += !RB_CAN_McpSetRegs(MCP_REG_TXB0CTRL, zeros, 14);
    errors += !RB_CAN_McpSetRegs(MCP_REG_TXB1CTRL, zeros, 14);
    errors += !RB_CAN_McpSetRegs(MCP_REG_TXB2CTRL, zeros, 14);

    errors += !RB_CAN_McpSetReg(MCP_REG_RXB0CTRL, 0);
    errors += !RB_CAN_McpSetReg(MCP_REG_RXB1CTRL, 0);

    // TODO: set interrupt settings here

    // Note from arduino lib:
    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    errors += !RB_CAN_McpModReg(
            MCP_REG_RXB0CTRL,
            MCP_MASK_RXBnCTRL_RXM | MCP_RXB0CTRL_BUKT | MCP_MASK_RXB0CTRL_FILHIT,
            MCP_RXBnCTRL_RXM_STDEXT | MCP_RXB0CTRL_BUKT | MCP_RXB0CTRL_FILHIT
            );
    errors += !RB_CAN_McpModReg(
            MCP_REG_RXB1CTRL,
            MCP_MASK_RXBnCTRL_RXM | MCP_MASK_RXB1CTRL_FILHIT,
            MCP_RXBnCTRL_RXM_STDEXT | MCP_RXB1CTRL_FILHIT
            );

    // TODO: set message filter settings here

    return errors;
}

bool RB_CAN_McpSetReg(MCP_REGISTER reg, uint8_t data) {
    bool success = false;
    if (SPI1_IsTxReady() && StartTransaction()) {
        SPI1_ByteWrite(MCP_INSTR_WRITE);
        SPI1_ByteWrite(reg);
        SPI1_ByteWrite(data);
        EndTransaction();
        success = true;
    } 
    return success;
}

bool RB_CAN_McpSetRegs(MCP_REGISTER firstReg, const uint8_t* data, uint8_t n) {
    bool success = false;
    if (SPI1_IsTxReady() && StartTransaction()) {
        SPI1_ByteWrite(MCP_INSTR_WRITE);
        SPI1_ByteWrite(firstReg);
        for (int i = 0; i < n; i++) {
            SPI1_ByteWrite(data[i]);
        }
        EndTransaction();
        success = true;
    }
    return success;
}

bool RB_CAN_McpModReg(MCP_REGISTER reg, uint8_t mask, uint8_t data) {
    bool success = false;
    if (SPI1_IsTxReady() && StartTransaction()) {
        SPI1_ByteWrite(MCP_INSTR_BITMOD);
        SPI1_ByteWrite(reg);
        SPI1_ByteWrite(mask);
        SPI1_ByteWrite(data);
        EndTransaction();
        success = true;
    }
    return success;
}

bool RB_CAN_McpGetReg(MCP_REGISTER reg, uint8_t* data) {
    bool success = false;
    if (SPI1_IsTxReady() && StartTransaction()) {
        SPI1_ByteWrite(MCP_INSTR_READ);
        SPI1_ByteWrite(reg);
        *data = SPI1_ByteRead();
        EndTransaction();
        success = true;
    } 
    return success;
}

static bool StartTransaction() {
    bool success;
    success = SPI1_Open(0);
    if (success) {
        SPI1_CS_SetLow();
    }
    return success;
}

static void EndTransaction() {
    SPI1_Close();
    SPI1_CS_SetHigh();
}
