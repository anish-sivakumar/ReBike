/* 
 * File:   rb_mcp_defs.h
 * Author: chygg
 *
 * Created on June 20, 2024, 10:10 AM
 */

#ifndef RB_MCP_DEFS_H
#define	RB_MCP_DEFS_H

#include <stdint.h>
#include <stdbool.h>

// Register masks
#define MCP_MASK_TXB_EXIDE          0x08
#define MCP_MASK_DLC                0x0F
#define MCP_MASK_RTR                0x40
#define MCP_MASK_RXBnCTRL_RXM       0x60
#define MCP_MASK_RXB0CTRL_FILHIT    0x03
#define MCP_MASK_RXB1CTRL_FILHIT    0x07
#define MCP_MASK_CANCTRL_REQOP      0xE0
#define MCP_MASK_STAT_RXIF          MCP_STAT_RX0IF | MCP_STAT_RX1IF
#define MCP_MASK_EFLG_ERROR         MCP_EFLG_RX1OVR | MCP_EFLG_RX0OVR | MCP_EFLG_TXBO | MCP_EFLG_TXEP | MCP_EFLG_RXEP

// Miscelaneous Register settings
#define MCP_RXBnCTRL_RXM_STD        0x20
#define MCP_RXBnCTRL_RXM_EXT        0x40
#define MCP_RXBnCTRL_RXM_NOFILT     0x60
#define MCP_RXBnCTRL_RXM_STDEXT     0x00
#define MCP_RXBnCTRL_RTR            0x08
#define MCP_RXB0CTRL_BUKT           0x04
#define MCP_RXB0CTRL_FILHIT         0x00
#define MCP_RXB1CTRL_FILHIT         0x01

// Settings for 500kbps CAN bus
// For some god-forbidden reason, every resource out there claims these settings are for 1000kbps CAN.
// However, these are the only settings which seem to work for our 500kbps bus, so we're using them.
#define MCP_20MHz_500kBPS_CFG1      0x00
#define MCP_20MHz_500kBPS_CFG2      0xD9
#define MCP_20MHz_500kBPS_CFG3      0x82

// receive buffer masks for CAN ID types
#define MCP_RX_MASK_NONE    0x00000000
#define MCP_RX_MASK_STD     0xFFE00000
#define MCP_RX_MASK_EXT     0xFFE3FFFF

typedef enum tagMCP_INSTRUCTION {
    MCP_INSTR_WRITE = 0x02,
    MCP_INSTR_READ = 0x03,
    MCP_INSTR_BITMOD = 0x05,
    MCP_INSTR_LOAD_TX0 = 0x40,
    MCP_INSTR_LOAD_TX1 = 0x42,
    MCP_INSTR_LOAD_TX2 = 0x44,
    MCP_INSTR_RTS_TX0 = 0x81,
    MCP_INSTR_RTS_TX1 = 0x82,
    MCP_INSTR_RTS_TX2 = 0x84,
    MCP_INSTR_RTS_ALL = 0x87,
    MCP_INSTR_READ_RX0 = 0x90,
    MCP_INSTR_READ_RX1 = 0x94,
    MCP_INSTR_READ_STATUS = 0xA0,
    MCP_INSTR_RX_STATUS = 0xB0,
    MCP_INSTR_RESET = 0xC0
} MCP_INSTRUCTION;

typedef enum tagMCP_REGISTER {
    MCP_REG_RXF0SIDH = 0x00,
    MCP_REG_RXF0SIDL = 0x01,
    MCP_REG_RXF0EID8 = 0x02,
    MCP_REG_RXF0EID0 = 0x03,
    MCP_REG_RXF1SIDH = 0x04,
    MCP_REG_RXF1SIDL = 0x05,
    MCP_REG_RXF1EID8 = 0x06,
    MCP_REG_RXF1EID0 = 0x07,
    MCP_REG_RXF2SIDH = 0x08,
    MCP_REG_RXF2SIDL = 0x09,
    MCP_REG_RXF2EID8 = 0x0A,
    MCP_REG_RXF2EID0 = 0x0B,
    MCP_REG_CANSTAT = 0x0E,
    MCP_REG_CANCTRL = 0x0F,
    MCP_REG_RXF3SIDH = 0x10,
    MCP_REG_RXF3SIDL = 0x11,
    MCP_REG_RXF3EID8 = 0x12,
    MCP_REG_RXF3EID0 = 0x13,
    MCP_REG_RXF4SIDH = 0x14,
    MCP_REG_RXF4SIDL = 0x15,
    MCP_REG_RXF4EID8 = 0x16,
    MCP_REG_RXF4EID0 = 0x17,
    MCP_REG_RXF5SIDH = 0x18,
    MCP_REG_RXF5SIDL = 0x19,
    MCP_REG_RXF5EID8 = 0x1A,
    MCP_REG_RXF5EID0 = 0x1B,
    MCP_REG_TEC = 0x1C,
    MCP_REG_REC = 0x1D,
    MCP_REG_RXM0SIDH = 0x20,
    MCP_REG_RXM0SIDL = 0x21,
    MCP_REG_RXM0EID8 = 0x22,
    MCP_REG_RXM0EID0 = 0x23,
    MCP_REG_RXM1SIDH = 0x24,
    MCP_REG_RXM1SIDL = 0x25,
    MCP_REG_RXM1EID8 = 0x26,
    MCP_REG_RXM1EID0 = 0x27,
    MCP_REG_CNF3 = 0x28,
    MCP_REG_CNF2 = 0x29,
    MCP_REG_CNF1 = 0x2A,
    MCP_REG_CANINTE = 0x2B,
    MCP_REG_CANINTF = 0x2C,
    MCP_REG_EFLG = 0x2D,
    MCP_REG_TXB0CTRL = 0x30,
    MCP_REG_TXB0SIDH = 0x31,
    MCP_REG_TXB0SIDL = 0x32,
    MCP_REG_TXB0EID8 = 0x33,
    MCP_REG_TXB0EID0 = 0x34,
    MCP_REG_TXB0DLC = 0x35,
    MCP_REG_TXB0DATA = 0x36,
    MCP_REG_TXB1CTRL = 0x40,
    MCP_REG_TXB1SIDH = 0x41,
    MCP_REG_TXB1SIDL = 0x42,
    MCP_REG_TXB1EID8 = 0x43,
    MCP_REG_TXB1EID0 = 0x44,
    MCP_REG_TXB1DLC = 0x45,
    MCP_REG_TXB1DATA = 0x46,
    MCP_REG_TXB2CTRL = 0x50,
    MCP_REG_TXB2SIDH = 0x51,
    MCP_REG_TXB2SIDL = 0x52,
    MCP_REG_TXB2EID8 = 0x53,
    MCP_REG_TXB2EID0 = 0x54,
    MCP_REG_TXB2DLC = 0x55,
    MCP_REG_TXB2DATA = 0x56,
    MCP_REG_RXB0CTRL = 0x60,
    MCP_REG_RXB0SIDH = 0x61,
    MCP_REG_RXB0SIDL = 0x62,
    MCP_REG_RXB0EID8 = 0x63,
    MCP_REG_RXB0EID0 = 0x64,
    MCP_REG_RXB0DLC = 0x65,
    MCP_REG_RXB0DATA = 0x66,
    MCP_REG_RXB1CTRL = 0x70,
    MCP_REG_RXB1SIDH = 0x71,
    MCP_REG_RXB1SIDL = 0x72,
    MCP_REG_RXB1EID8 = 0x73,
    MCP_REG_RXB1EID0 = 0x74,
    MCP_REG_RXB1DLC = 0x75,
    MCP_REG_RXB1DATA = 0x76
} MCP_REGISTER;

typedef enum tagMCP_STAT {
    MCP_STAT_RX0IF = (1 << 0),
    MCP_STAT_RX1IF = (1 << 1),
    MCP_STAT_TX0REQ = (1 << 2),
    MCP_STAT_TX0IF = (1 << 3),
    MCP_STAT_TX1REQ = (1 << 4),
    MCP_STAT_TX1IF = (1 << 5),
    MCP_STAT_TX2REQ = (1 << 6),
    MCP_STAT_TX2IF = (1 << 7),
            
            
} MCP_STAT;

typedef enum tagMCP_ERROR_FLAG {
    MCP_EFLG_RX1OVR = (1 << 7),
    MCP_EFLG_RX0OVR = (1 << 6),
    MCP_EFLG_TXBO = (1 << 5),
    MCP_EFLG_TXEP = (1 << 4),
    MCP_EFLG_RXEP = (1 << 3),
    MCP_EFLG_TXWAR = (1 << 2),
    MCP_EFLG_RXWAR = (1 << 1),
    MCP_EFLG_EWARN = (1 << 0)
} MCP_ERROR_FLAG;

typedef enum tagMCP_INTERRUPT_FLAG {
    MCP_INTF_RX0IF = 0x01,
    MCP_INTF_RX1IF = 0x02,
    MCP_INTF_TX0IF = 0x04,
    MCP_INTF_TX1IF = 0x08,
    MCP_INTF_TX2IF = 0x10,
    MCP_INTF_ERRIF = 0x20,
    MCP_INTF_WAKIF = 0x40,
    MCP_INTF_MERRF = 0x80
} MCP_INTERRUPT_FLAG;

typedef enum tagMCP_CAN_MODE {
    MCP_CAN_MODE_NORMAL     = 0x00,
    MCP_CAN_MODE_SLEEP      = 0x20,
    MCP_CAN_MODE_LOOPBACK   = 0x40,
    MCP_CAN_MODE_LISTENONLY = 0x60,
    MCP_CAN_MODE_CONFIG     = 0x80,
    MCP_CAN_MODE_POWERUP    = 0xE0
} MCP_CAN_MODE;


#endif	/* RB_CAN_DEFS_H */

