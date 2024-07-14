#include "rb_can.h"
#include "rb_hall.h"
#include "rb_mcp_defs.h"

void RB_CAN_ReadThrottle(CAN_FRAME *canFrame0, uint8_t* throttle) {
    uint8_t mcpReadStat;
    
    RB_MCP_ReadStat(&mcpReadStat);
    if (mcpReadStat & MCP_STAT_RX0IF) {
        RB_MCP_ReadRx(0, &canFrame0, false);
        *throttle = canFrame0.data[0];  
    }
}

bool RB_CAN_IsTxReady(uint8_t buffer) {
    return RB_MCP_IsTxReady(buffer);
}

void RB_CAN_SendTxFrame(uint8_t buffer, CAN_FRAME* frame) {
    RB_MCP_LoadTx(buffer, frame, false);
    RB_MCP_SendOne(buffer);
}

void RB_CAN_LoadMotorParams(CAN_FRAME *canFrameTx, int16_t speed, int16_t power, int16_t temperature) {
    
    // Place the speed, power, and temperature into the CAN frame
    canFrameTx->data[0] = (uint8_t)(speed & 0xFF);
    canFrameTx->data[1] = (uint8_t)((speed >> 8) & 0xFF);
    canFrameTx->data[2] = (uint8_t)(power & 0xFF);
    canFrameTx->data[3] = (uint8_t)((power >> 8) & 0xFF);
    canFrameTx->data[4] = (uint8_t)(temperature & 0xFF);
    canFrameTx->data[5] = (uint8_t)((temperature >> 8) & 0xFF);

    // Set remaining data bytes to zero (or other values as needed)
    canFrameTx->data[6] = 0;
    canFrameTx->data[7] = 0;

    // Send the CAN frame
    RB_CAN_SendTxFrame(0, &canFrameTx);
}

































/*readCounter++;
  writeCounter++;

    if (readCounter >= 2000){
        RB_MCP_ReadStat(&mcpReadStat);
        if (mcpReadStat & MCP_STAT_RX0IF){
            RB_MCP_ReadRx(0, &canFrame0, false);
        }
        else if (mcpReadStat & MCP_STAT_RX1IF)
        {
            RB_MCP_ReadRx(1, &canFrame1, false);
        }
        readCounter = 0;
    }
    else if (writeCounter >= 20000){
        tx_ready = RB_MCP_IsTxReady(0);
        if (tx_ready){
            RB_MCP_LoadTx(0,&canFrameTx,false);
            RB_MCP_SendOne(0);
        }
    writeCounter = 0;
    }*/