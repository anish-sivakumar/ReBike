#include "rb_can.h"
#include "rb_hall.h"
#include "rb_mcp_defs.h"
#include "rb_mcp.h"

int8_t RB_CAN_ReadThrottle(CAN_FRAME *canFrame0) {
    int8_t throttle;
   
    RB_MCP_ReadRx(0, canFrame0, false);
    throttle = (int8_t)(canFrame0->data[0]);
    return throttle;
}

void RB_CAN_SendTxFrame(uint8_t buffer, CAN_FRAME* frame) {
    RB_MCP_LoadTx(buffer, frame, false);
    RB_MCP_SendOne(buffer);
}


bool RB_CAN_SendCANMessageV1(uint8_t buffer, CAN_ID can_id, uint16_t timestamp, uint16_t speed, uint16_t bridgeTemp, int8_t throttleInput, uint8_t errorWarning) {
    if (RB_MCP_IsTxReady(buffer)) {
        CAN_FRAME canFrameTx;
        canFrameTx.id = can_id;
        canFrameTx.len = 8;
        canFrameTx.data[0] = (uint8_t)(timestamp >> 8);
        canFrameTx.data[1] = (uint8_t)(timestamp & 0xFF);
        canFrameTx.data[2] = (uint8_t)(speed >> 8);
        canFrameTx.data[3] = (uint8_t)(speed & 0xFF);
        canFrameTx.data[4] = (uint8_t)(bridgeTemp >> 8);
        canFrameTx.data[5] = (uint8_t)(bridgeTemp & 0xFF);
        canFrameTx.data[6] = (uint8_t)(throttleInput);
        canFrameTx.data[7] = (uint8_t)(errorWarning);
        
        RB_CAN_SendTxFrame(buffer, &canFrameTx);
        return true;
    }
    return false;
}

bool RB_CAN_SendCANMessageV2(uint8_t buffer, CAN_ID can_id, uint16_t timestamp, int16_t value1, int16_t value2, int16_t value3) {
    if (RB_MCP_IsTxReady(buffer)) {
        CAN_FRAME canFrameTx;
        canFrameTx.id = can_id;
        canFrameTx.len = 8;
        canFrameTx.data[0] = (uint8_t)(timestamp >> 8);
        canFrameTx.data[1] = (uint8_t)(timestamp & 0xFF);
        canFrameTx.data[2] = (uint8_t)(value1 >> 8);
        canFrameTx.data[3] = (uint8_t)(value1 & 0xFF);
        canFrameTx.data[4] = (uint8_t)(value2 >> 8);
        canFrameTx.data[5] = (uint8_t)(value2 & 0xFF);
        canFrameTx.data[6] = (uint8_t)(value3 >> 8);
        canFrameTx.data[7] = (uint8_t)(value3 & 0xFF);
        
        RB_CAN_SendTxFrame(buffer, &canFrameTx);
        return true;
    }
    return false;
}

void RB_CAN_Service(CAN_FRAME *canFrame0, int8_t *throttleCmd, RB_CAN_CONTROL *CANControl, int8_t throttleInput, uint8_t errorWarning, RB_LOGGING_AVERAGES avg){
    bool messageSent = false;
    
    switch(CANControl->state){
        case RBCAN_MESSAGE1:
            messageSent = RB_CAN_SendCANMessageV1(0, CAN_ID_BIKE_STATUS, CANControl->timestamp, avg.speed, avg.bridgeTemp, throttleInput, errorWarning);                
            if (messageSent){
                CANControl->state = RBCAN_MESSAGE2; 
            }
            break;
            
        case RBCAN_MESSAGE2:
            messageSent = RB_CAN_SendCANMessageV2(1, CAN_ID_MOTOR_VOLTAGES, CANControl->timestamp, avg.vDC, avg.vA, avg.vB);
            if (messageSent){
                CANControl->state = RBCAN_MESSAGE3; 
            }
            break;
            
        case RBCAN_MESSAGE3:
            messageSent = RB_CAN_SendCANMessageV2(2, CAN_ID_MOTOR_REAL_CURRENTS, CANControl->timestamp, avg.iDC, avg.iA, avg.iB);
            if (messageSent){
                CANControl->state = RBCAN_MESSAGE4; 
            }
            break;
            
        case RBCAN_MESSAGE4:
            messageSent = RB_CAN_SendCANMessageV2(0, CAN_ID_MOTOR_CALC_VALUES, CANControl->timestamp, avg.iqRef, avg.iqFdb, avg.power);    
            if (messageSent){
                CANControl->state = RBCAN_IDLE; 
            }
            break;
            
        case RBCAN_IDLE:  
            
            if (CANControl->counter == RB_CAN_CYCLE_COUNT_MINUS1){
                CANControl->timestamp++;
                CANControl->state = RBCAN_MESSAGE1; 
            }    
            break;
    
    }

    if(!messageSent){
        uint8_t mcpReadStat;
        RB_MCP_ReadStat(&mcpReadStat);
        if (mcpReadStat & MCP_STAT_RX0IF) {
        *throttleCmd = RB_CAN_ReadThrottle(canFrame0); // lower priority than writing a message
        }
    }
}
   
  
