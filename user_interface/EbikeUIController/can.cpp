#include "can.h"

// CAN message flags 
static bool CAN_MB0_BikeStatus_Flag = false;
static bool CAN_MB1_MotorVoltages_Flag = false;
static bool CAN_MB2_RealCurrents_Flag = false;
static bool CAN_MB3_CalcValues_Flag = false;
static bool CAN_MB4_BmsSoc_Flag = false;

// function to initialize the CAN communication
void canInit(FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16>& can) {
    Serial.begin(115200); // Set baud rate to 115200
    can.begin(); // CAN initialization
    can.setBaudRate(500000); // Set CAN baud rate to 500kbit
    can.setMBFilter(REJECT_ALL);
    can.setMBFilter(MB0, CAN_ID_BIKE_STATUS); // Set mailbox CAN ID filters
    can.setMBFilter(MB1, CAN_ID_MOTOR_VOLTAGES);
    can.setMBFilter(MB2, CAN_ID_MOTOR_REAL_CURRENTS);
    can.setMBFilter(MB3, CAN_ID_MOTOR_CALC_VALUES);
    can.setMBFilter(MB4, CAN_ID_BMS_SOC);
    can.enableMBInterrupt(); // Enable all mailboxes to interrupts
    can.onReceive(MB0, CANUpdate_BikeStatus); // Set mailbox 0 to receive system parameters
    can.onReceive(MB1, CANUpdate_MotorVoltages); // Set mailbox 1 to receive system parameters
    can.onReceive(MB2, CANUpdate_RealCurrents); // Set mailbox 2 to receive system parameters
    can.onReceive(MB3, CANUpdate_CalcValues); // Set mailbox 3 to receive system parameters
    can.onReceive(MB4, CANUpdate_BmsSoc); // Set mailbox 4 to receive system parameters
    can.setMB(MB5, TX); // Set mailbox 5 as transmit
}

//
void CANSendThrottleMsg(FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16>& can, int throttle) {
  CAN_message_t msg;
  msg.id = CAN_ID_UI; // ID: 0x334
  msg.len = 8;
  msg.buf[0] = throttle & 0xFF; // Throttle value in byte 0
  can.write(MB5, msg); // Send message to mailbox 6 (transmit mailbox)
}

// function used as callback when bike status message enters mailbox
void CANUpdate_BikeStatus(const CAN_message_t &msg){
    // parses CAN message values into variables  
    CAN_BikeStatus_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_BikeStatus_Struct.speed = (msg.buf[2] << 8) | msg.buf[3];
    CAN_BikeStatus_Struct.temp = (msg.buf[4] << 8) | msg.buf[5];
    CAN_BikeStatus_Struct.throttle = msg.buf[6];
    CAN_BikeStatus_Struct.error = msg.buf[7];
    CAN_MB0_BikeStatus_Flag = true; // updated values have been read
}

// function used as callback when motor voltage message enters mailbox
void CANUpdate_MotorVoltages(const CAN_message_t &msg){
    // parses CAN message values into variables 
    CAN_MotorVoltages_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_MotorVoltages_Struct.vDC = (msg.buf[2] << 8) | msg.buf[3];
    CAN_MotorVoltages_Struct.vA = (msg.buf[4] << 8) | msg.buf[5];
    CAN_MotorVoltages_Struct.vB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB1_MotorVoltages_Flag = true; // updated values have been read
}

// function used as callback when real current message enters mailbox
void CANUpdate_RealCurrents(const CAN_message_t &msg){
    // parses CAN message values into variables 
    CAN_RealCurrents_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_RealCurrents_Struct.iDC = (msg.buf[2] << 8) | msg.buf[3];
    CAN_RealCurrents_Struct.iA = (msg.buf[4] << 8) | msg.buf[5];
    CAN_RealCurrents_Struct.iB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB2_RealCurrents_Flag = true; // updated values have been read
}

// function used as callback when calculated values message enters mailbox
void CANUpdate_CalcValues(const CAN_message_t &msg){
    // parses CAN message values into variables 
    CAN_CalcValues_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_CalcValues_Struct.iqRef = (msg.buf[2] << 8) | msg.buf[3];
    CAN_CalcValues_Struct.iqFdb = (msg.buf[4] << 8) | msg.buf[5];
    CAN_CalcValues_Struct.power = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB3_CalcValues_Flag = true; // updated values have been read
}

// function used as callback when BMS message enters mailbox
void CANUpdate_BmsSoc(const CAN_message_t &msg){
    // parses CAN message values into variables 
    CAN_BmsSoc_Struct.soc = (msg.buf[0] << 8) | msg.buf[1];
    CAN_BmsSoc_Struct.soh = (msg.buf[2] << 8) | msg.buf[3];
    CAN_MB4_BmsSoc_Flag = true; // updated values have been read
}

// gets the received values from the CAN message and stores values in main
void getCAN_BikeStatus_Values(uint16_t &timestamp, uint16_t &Speed, uint16_t &temp, uint8_t &throttleInput, uint8_t &ErrorWarning){
    timestamp = CAN_BikeStatus_Struct.timestamp; 
    Speed = CAN_BikeStatus_Struct.speed;
    temp = CAN_BikeStatus_Struct.temp_fet;
    throttleInput = CAN_BikeStatus_Struct.throttleInput;
    ErrorWarning = CAN_BikeStatus_Struct.errorWarning; 
    CAN_MB0_BikeStatus_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_MotorVoltages_Values(uint16_t &timestamp, uint16_t &VDC, uint16_t &VA, uint16_t &VB){
    timestamp = CAN_BikeStatus_Struct.timestamp;
    VDC = CAN_MotorVoltages_Struct.vDC;
    VA = CAN_MotorVoltages_Struct.vA;
    VB = CAN_MotorVoltages_Struct.vB;
    CAN_MB1_MotorVoltages_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_RealCurrents_Values(uint16_t &timestamp, uint16_t &IDC, uint16_t &IA, uint16_t &IB){
    timestamp = CAN_BikeStatus_Struct.timestamp;
    IDC = CAN_RealCurrents_Struct.iDC;
    IA = CAN_RealCurrents_Struct.iA;
    IB = CAN_RealCurrents_Struct.iB;
    CAN_MB2_RealCurrents_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_CalcValues_Values(uint16_t &timestamp, uint16_t &Ref_iq, uint16_t &Fdb_iq, uint16_t &Power){
    timestamp = CAN_BikeStatus_Struct.timestamp;
    Ref_iq = CAN_CalcValues_Struct.iqRef;
    Fdb_iq = CAN_CalcValues_Struct.iqFdb;
    Power = CAN_CalcValues_Struct.power;
    CAN_MB3_CalcValues_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_BmsSoc_Values(uint16_t &SOC, uint16_t &SOH){
    SOC = CAN_BmsSoc_Struct.soc;
    SOH = CAN_BmsSoc_Struct.soh;
    CAN_MB4_BmsSoc_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// returns state of bike status CAN msg
bool CANPendingBikeStatusMsg(){
    return CAN_MB0_BikeStatus_Flag;
}

// returns state of motor voltages CAN msg
bool CANPendingMotorVoltagesMsg(){
    return CAN_MB1_MotorVoltages_Flag;
}

// returns state of real currents CAN msg
bool CANPendingRealCurrentsMsg(){
    return CAN_MB2_RealCurrents_Flag;
}

// returns state of calculated values CAN msg
bool CANPendingCalcValuesMsg(){
    return CAN_MB3_CalcValues_Flag;
}

// returns state of BMS state of charge CAN msg
bool CANPendingBmsSocMsg(){
    return CAN_MB4_BmsSoc_Flag;
}