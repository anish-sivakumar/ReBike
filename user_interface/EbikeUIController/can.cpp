#include "can.h"

// global can object
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

// global objects to hold can data;
CAN_BikeStatus_Struct     bikeStatus;
CAN_MotorVoltages_Struct  motorVoltages;
CAN_RealCurrents_Struct   realCurrents;
CAN_CalcValues_Struct     calcValues;
CAN_BmsSoc_Struct         bmsSoc;
CAN_BmsVI_Struct          bmsVI;



// CAN message flags 
static bool CAN_MB0_BikeStatus_Flag = false;
static bool CAN_MB1_MotorVoltages_Flag = false;
static bool CAN_MB2_RealCurrents_Flag = false;
static bool CAN_MB3_CalcValues_Flag = false;
static bool CAN_MB4_BmsSoc_Flag = false;
static bool CAN_MB6_BmsVI_Flag = false;


// function to initialize the CAN communication
void canInit() {
    can.begin(); // CAN initialization
    can.setBaudRate(500000); // Set CAN baud rate to 500kbit
    can.setMaxMB(16);
    can.setMB(MB4, RX, STD); // Set mailbox 4 as standard ID receive
    can.setMB(MB6, RX, STD); // Set mailbox 6 as standard ID receive
    can.setMB(MB5, TX); // Set mailbox 5 as transmit
    can.setMBFilter(REJECT_ALL);
    can.setMBFilter(MB0, CAN_ID_BIKE_STATUS); // Set mailbox CAN ID filters
    can.setMBFilter(MB1, CAN_ID_MOTOR_VOLTAGES);
    can.setMBFilter(MB2, CAN_ID_MOTOR_REAL_CURRENTS);
    can.setMBFilter(MB3, CAN_ID_MOTOR_CALC_VALUES);
    can.setMBFilter(MB4, CAN_ID_BMS_SOC);
    can.setMBFilter(MB6, CAN_ID_BMS_VI);
    can.onReceive(MB0, CANUpdateBikeStatus); // Set mailbox 0 to receive system parameters
    can.onReceive(MB1, CANUpdateMotorVoltages); // Set mailbox 1 to receive system parameters
    can.onReceive(MB2, CANUpdateRealCurrents); // Set mailbox 2 to receive system parameters
    can.onReceive(MB3, CANUpdateCalcValues); // Set mailbox 3 to receive system parameters
    can.onReceive(MB4, CANUpdateBmsSoc); // Set mailbox 4 to receive system parameters
    can.onReceive(MB6, CANUpdateBmsVI);  // Set mailbox 6 to receive battery voltage and current
    can.enableMBInterrupts(); // Enable all mailboxes to interrupts

    can.mailboxStatus();
}

//
void CANSendThrottleMsg(int throttle) {
  CAN_message_t msg;
  msg.id = CAN_ID_UI; // ID: 0x334
  msg.len = 8;
  msg.buf[0] = throttle & 0xFF; // Throttle value in byte 0
  can.write(msg); // Send message to mailbox 6 (transmit mailbox)
}

// function used as callback when bike status message enters mailbox
void CANUpdateBikeStatus(const CAN_message_t &msg){
  if (msg.id == CAN_ID_BIKE_STATUS){
    // parses CAN message values into variables  
    bikeStatus.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    bikeStatus.speed = (msg.buf[2] << 8) | msg.buf[3];
    bikeStatus.tempFet = ((uint16_t) msg.buf[4] << 8) | (uint16_t)msg.buf[5];
    bikeStatus.throttleInput = msg.buf[6];
    bikeStatus.errorWarning = msg.buf[7];
    CAN_MB0_BikeStatus_Flag = true; // updated values have been read
  }
}

// function used as callback when motor voltage message enters mailbox
void CANUpdateMotorVoltages(const CAN_message_t &msg){
  if (msg.id == CAN_ID_MOTOR_VOLTAGES){
    // parses CAN message values into variables 
    motorVoltages.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    motorVoltages.vDC = (msg.buf[2] << 8) | msg.buf[3];
    motorVoltages.vA = (msg.buf[4] << 8) | msg.buf[5];
    motorVoltages.vB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB1_MotorVoltages_Flag = true; // updated values have been read
  }
}

// function used as callback when real current message enters mailbox
void CANUpdateRealCurrents(const CAN_message_t &msg){
  if (msg.id == CAN_ID_MOTOR_REAL_CURRENTS){
    // parses CAN message values into variables 
    realCurrents.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    realCurrents.iDC = (msg.buf[2] << 8) | msg.buf[3];
    realCurrents.iA = (msg.buf[4] << 8) | msg.buf[5];
    realCurrents.iB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB2_RealCurrents_Flag = true; // updated values have been read
  }
}

// function used as callback when calculated values message enters mailbox
void CANUpdateCalcValues(const CAN_message_t &msg){
  if (msg.id == CAN_ID_MOTOR_CALC_VALUES){
    // parses CAN message values into variables 
    calcValues.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    calcValues.iqRef = (msg.buf[2] << 8) | msg.buf[3];
    calcValues.iqFdb = (msg.buf[4] << 8) | msg.buf[5];
    calcValues.power = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB3_CalcValues_Flag = true; // updated values have been read
  }
}

// function used as callback when BMS message enters mailbox
void CANUpdateBmsSoc(const CAN_message_t &msg){
    // parses CAN message values into variables 
    bmsSoc.soc = msg.buf[0];
    bmsSoc.soh = msg.buf[2];
    CAN_MB4_BmsSoc_Flag = true; // updated values have been read
}

// function used as callback when BMS message enters mailbox
void CANUpdateBmsVI(const CAN_message_t &msg){
    // parses CAN message values into variables 
    bmsVI.voltage = (uint16_t) msg.buf[0] | (uint16_t)(msg.buf[1] << 8);
    bmsVI.current = (uint16_t) msg.buf[2] | (uint16_t)(msg.buf[3] << 8);
    CAN_MB6_BmsVI_Flag = true; // updated values have been read
}

// gets the received values from the CAN message and stores values in main
void CANGetBikeStatus(uint16_t &timestamp, uint16_t &speed, uint16_t &temp, int8_t &throttleInput, uint8_t &ErrorWarning){
    timestamp = bikeStatus.timestamp; 
    speed = bikeStatus.speed;
    temp = bikeStatus.tempFet;
    throttleInput = bikeStatus.throttleInput;
    ErrorWarning = bikeStatus.errorWarning; 
    CAN_MB0_BikeStatus_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void CANGetMotorVoltages(uint16_t &timestamp, int16_t &VDC, int16_t &VA, int16_t &VB){
    timestamp = motorVoltages.timestamp;
    VDC = motorVoltages.vDC;
    VA = motorVoltages.vA;
    VB = motorVoltages.vB;
    CAN_MB1_MotorVoltages_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void CANGetRealCurrents(uint16_t &timestamp, int16_t &IDC, int16_t &IA, int16_t &IB){
    timestamp = realCurrents.timestamp;
    IDC = realCurrents.iDC;
    IA = realCurrents.iA;
    IB = realCurrents.iB;
    CAN_MB2_RealCurrents_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void CANGetCalcValues(uint16_t &timestamp, int16_t &Ref_iq, int16_t &Fdb_iq, int16_t &power){
    timestamp = calcValues.timestamp;
    Ref_iq = calcValues.iqRef;
    Fdb_iq = calcValues.iqFdb;
    power = calcValues.power;
    CAN_MB3_CalcValues_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void CANGetBmsSoc(uint16_t &SOC, uint16_t &SOH){
    SOC = bmsSoc.soc;
    SOH = bmsSoc.soh;
    CAN_MB4_BmsSoc_Flag = false; // updated values have been set in main, set flag back to false
    return;
}

// gets the received values from the CAN message and stores values in main
void CANGetBmsVI(int16_t &VDC, int16_t &IDC){
    VDC = bmsVI.voltage;
    IDC = bmsVI.current;
    CAN_MB6_BmsVI_Flag = false; // updated values have been set in main, set flag back to false
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

// returns state of bike status CAN msg
bool CANPendingBmsVIMsg(){
    return CAN_MB6_BmsVI_Flag;
}