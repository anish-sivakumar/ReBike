#include "rb_mcp.h"

#include "system/pins.h"
#include "spi_host/spi1.h"
#include "timer/delay.h"

#include "string.h" 

static uint8_t spiBuf[10];

// Static function defines
// These can only be used inside this file
static bool StartTransaction();
static void EndTransaction();

uint16_t RB_MCP_Init(void) {
    // Check for errors during initialization
    uint16_t errors = 0;
        
    errors += !RB_MCP_Reset();

    uint8_t zeros[14];
    memset(zeros, 0, sizeof (zeros));
    errors += !RB_MCP_SetRegs(MCP_REG_TXB0CTRL, zeros, 14);
    errors += !RB_MCP_SetRegs(MCP_REG_TXB1CTRL, zeros, 14);
    errors += !RB_MCP_SetRegs(MCP_REG_TXB2CTRL, zeros, 14);

    errors += !RB_MCP_SetReg(MCP_REG_RXB0CTRL, 0);
    errors += !RB_MCP_SetReg(MCP_REG_RXB1CTRL, 0);

    // TODO: set interrupt settings here

    // Note from arduino lib:
    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    errors += !RB_MCP_ModReg(
            MCP_REG_RXB0CTRL,
            MCP_MASK_RXBnCTRL_RXM | MCP_RXB0CTRL_BUKT | MCP_MASK_RXB0CTRL_FILHIT,
            MCP_RXBnCTRL_RXM_STDEXT | MCP_RXB0CTRL_BUKT | MCP_RXB0CTRL_FILHIT
            );
    errors += !RB_MCP_ModReg(
            MCP_REG_RXB1CTRL,
            MCP_MASK_RXBnCTRL_RXM | MCP_MASK_RXB1CTRL_FILHIT,
            MCP_RXBnCTRL_RXM_STDEXT | MCP_RXB1CTRL_FILHIT
            );

    // TODO: set message filter settings here

    // set CAN bitrate to 500kbps
    errors += !RB_MCP_SetReg(MCP_REG_CNF1, MCP_20MHz_500kBPS_CFG1);
    errors += !RB_MCP_SetReg(MCP_REG_CNF2, MCP_20MHz_500kBPS_CFG2);
    errors += !RB_MCP_SetReg(MCP_REG_CNF3, MCP_20MHz_500kBPS_CFG3);

    // set normal mode to begin CAN send/receive capabilities;
    errors += !RB_MCP_SetMode(MCP_CAN_MODE_NORMAL);

    return errors;
}

bool RB_MCP_Reset() {
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        spiBuf[0] = MCP_INSTR_RESET;
        SPI1_BufferWrite(spiBuf,1);
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_SetMode(MCP_CAN_MODE mode) {
    // set the mode
    RB_MCP_ModReg(MCP_REG_CANCTRL, MCP_MASK_CANCTRL_REQOP, mode);
    // double check that the mode was set correctly
    uint8_t newmode;
    RB_MCP_GetReg(MCP_REG_CANSTAT, &newmode);
    newmode &= MCP_MASK_CANCTRL_REQOP;
    bool success = mode == newmode;
    return success;
}

bool RB_MCP_SetReg(MCP_REGISTER reg, uint8_t data) {
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        spiBuf[0] = MCP_INSTR_WRITE;
        spiBuf[1] = reg;
        spiBuf[2] = data;
        SPI1_BufferWrite(spiBuf,3);
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_SetRegs(MCP_REGISTER firstReg, uint8_t* data, uint8_t n) {
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        // first use the spi buffer to write the instruction type and reg address
        spiBuf[0] = MCP_INSTR_WRITE;
        spiBuf[1] = firstReg;
        SPI1_BufferWrite(spiBuf,2);
        // second use the data buffer to write the remaining data
        SPI1_BufferWrite(data, n);
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_ModReg(MCP_REGISTER reg, uint8_t mask, uint8_t data) {
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        spiBuf[0] = MCP_INSTR_BITMOD;
        spiBuf[1] = reg;
        spiBuf[2] = mask;
        spiBuf[3] = data;
        SPI1_BufferWrite(spiBuf, 4);
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_GetReg(MCP_REGISTER reg, uint8_t* data) {
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        
        spiBuf[0] = MCP_INSTR_READ;
        spiBuf[1] = reg;
        spiBuf[2] = 0;
        SPI1_BufferExchange(spiBuf, 3);
        *data = spiBuf[2];
        success = true;
    }
    EndTransaction();

    return success;
}

bool RB_MCP_ReadStat(uint8_t* status){
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        spiBuf[0] = MCP_INSTR_READ_STATUS;
        spiBuf[1] = 0;
        SPI1_BufferExchange(spiBuf, 2);
        *status = spiBuf[1];
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_ReadRx(uint16_t rxBufId, CAN_FRAME* frame);

// Static Helper Functions

static bool StartTransaction() {
    bool success;
    success = SPI1_Open(0);
    if (success) {
        SPI1_CS_SetLow();
    }
    return success;
}

static void EndTransaction() {
    while(SPI1STATLbits.SPIBUSY); // wait until all messages are finished sending
    SPI1_Close();
    SPI1_CS_SetHigh();
}
