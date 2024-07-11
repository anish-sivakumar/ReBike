#include "rb_mcp.h"

#include "system/pins.h"
#include "spi_host/spi1.h"
#include "timer/delay.h"

#include "string.h" 

static uint8_t spiBuf[20];

extern uint8_t canTestArr[20];


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
    errors += !RB_MCP_SetReg(MCP_REG_CANINTE, MCP_INTF_RX0IF | MCP_INTF_RX1IF | MCP_INTF_ERRIF | MCP_INTF_MERRF);

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

    // TODO: finish configuring message filter settings here

    errors += !RB_MCP_SetFilter(0, CAN_ID_BMS_SOC);
    errors += !RB_MCP_SetMask(0, MCP_RX_MASK_STD);

    errors += !RB_MCP_SetFilter(1, CAN_ID_UIC);
    errors += !RB_MCP_SetMask(1, MCP_RX_MASK_STD);

    // set CAN bitrate to 500kbps
    errors += !RB_MCP_SetReg(MCP_REG_CNF1, MCP_20MHz_500kBPS_CFG1);
    errors += !RB_MCP_SetReg(MCP_REG_CNF2, MCP_20MHz_500kBPS_CFG2);
    errors += !RB_MCP_SetReg(MCP_REG_CNF3, MCP_20MHz_500kBPS_CFG3);

//    errors += !RB_MCP_GetReg(MCP_REG_RXF0SIDH, &canTestArr[0]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXF0SIDL, &canTestArr[1]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXF0EID8, &canTestArr[2]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXF0EID0, &canTestArr[3]);
//
//    errors += !RB_MCP_GetReg(MCP_REG_RXM0SIDH, &canTestArr[4]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXM0SIDL, &canTestArr[5]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXM0EID8, &canTestArr[6]);
//    errors += !RB_MCP_GetReg(MCP_REG_RXM0EID0, &canTestArr[7]);



    // set normal mode to begin CAN send/receive capabilities;
    errors += !RB_MCP_SetMode(MCP_CAN_MODE_NORMAL);

//    errors += !RB_MCP_GetReg(MCP_REG_CANINTE, &canTestArr[19]);

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

uint8_t newmode;
bool RB_MCP_SetMode(MCP_CAN_MODE mode) {
    // set the mode
    RB_MCP_ModReg(MCP_REG_CANCTRL, MCP_MASK_CANCTRL_REQOP, mode);
    // double check that the mode was set correctly
    DELAY_microseconds(100);
    RB_MCP_GetReg(MCP_REG_CANSTAT, &newmode);
    bool success = mode == (newmode & MCP_MASK_CANCTRL_REQOP);
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
        SPI1_ByteWrite(MCP_INSTR_WRITE);
        SPI1_ByteWrite(firstReg);
        for (int i = 0; i < n; i++){
            SPI1_ByteWrite(data[i]);
        }
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

bool RB_MCP_ReadStat(uint8_t* readStatus){
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        spiBuf[0] = MCP_INSTR_READ_STATUS;
        spiBuf[1] = 0;
        SPI1_BufferExchange(spiBuf, 2);
        *readStatus = spiBuf[1];
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_SetFilter(uint16_t filterId, CAN_ID canId)
{
    uint8_t firstReg;
    uint8_t buffer[4];
    switch (filterId)
    {
        case 0:
            firstReg = MCP_REG_RXF0SIDH;
            break;
        case 1:
            firstReg = MCP_REG_RXF1SIDH;
            break;
        case 2:
            firstReg = MCP_REG_RXF2SIDH;
            break;
        case 3:
            firstReg = MCP_REG_RXF3SIDH;
            break;
        case 4:
            firstReg = MCP_REG_RXF4SIDH;
            break;
        case 5:
            firstReg = MCP_REG_RXF5SIDH;
            break;
        default:
            return false;
    }
    buffer[0] = (uint8_t) (canId >> 3);
    buffer[1] = (uint8_t) ((canId & 0x07 ) << 5);
    buffer[2] = 0;
    buffer[3] = 0;
    return RB_MCP_SetRegs(firstReg, buffer, 4);
}

bool RB_MCP_SetMask(uint16_t rxBufId, uint32_t mask){
    uint8_t firstReg;
    switch (rxBufId){
        case 0:
            firstReg = MCP_REG_RXM0SIDH;
            break;
        case 1:
            firstReg = MCP_REG_RXM1SIDH;
            break;
        default:
            return false;
    }   
    
    spiBuf[0] = (uint8_t)((mask >> 24)& 0xFF);
    spiBuf[1] = (uint8_t)((mask >> 16)& 0xFF);
    spiBuf[2] = (uint8_t)((mask >> 8)& 0xFF);
    spiBuf[3] = (uint8_t)(mask & 0xFF);
    
    return RB_MCP_SetRegs(firstReg, spiBuf, 4);
}


bool RB_MCP_RxStat(uint8_t* rxStatus){
    spiBuf[0] = MCP_INSTR_RX_STATUS;
    spiBuf[1] = 0;
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        SPI1_BufferExchange(spiBuf, 2);
        *rxStatus = spiBuf[1];
        success = true;
    }
    EndTransaction();
    return success;
}

bool RB_MCP_ReadRx(uint16_t rxBufId, CAN_FRAME* frame, bool dataOnly) {
    
    switch (rxBufId){
        case 0:
            spiBuf[0] = MCP_INSTR_READ_RX0;
            break;
        case 1: 
            spiBuf[0] = MCP_INSTR_READ_RX1;
            break;
        default:
            return false;
    }
    bool success = false;
    if (StartTransaction() && SPI1_IsTxReady()) {
        SPI1_BufferExchange(spiBuf, 14);
        frame->id = ((uint16_t)spiBuf[1] << 3) | ((uint16_t)(spiBuf[2] & 0b11100000) >> 5);
        frame->len = spiBuf[5] & 0b00001111;
        memcpy(frame->data, &spiBuf[6], 8);
        success = true;
    }
    EndTransaction();
    return success;
}

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
