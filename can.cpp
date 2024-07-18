#include "can.h"

// CAN message flags 
bool CAN_MB0_BikeStatus_Flag = false;
bool CAN_MB1_MotorVoltages_Flag = false;
bool CAN_MB2_RealCurrents_Flag = false;
bool CAN_MB3_CalcValues_Flag = false;
bool CAN_MB4_BmsSoc_Flag = false;

// Function to initialize the CAN communication
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
    can.onReceive(MB0, updateCAN_BikeStatus_Struct); // Set mailbox 0 to receive system parameters
    can.onReceive(MB1, updateCAN_MotorVoltages_Struct); // Set mailbox 1 to receive system parameters
    can.onReceive(MB2, updateCAN_MotorRealCurrents_Struct); // Set mailbox 2 to receive system parameters
    can.onReceive(MB3, updateCAN_CalcValues_Struct); // Set mailbox 3 to receive system parameters
    can.onReceive(MB4, updateCAN_BmsSoc_Struct); // Set mailbox 4 to receive system parameters
    can.setMB(MB5, TX); // Set mailbox 5 as transmit
}

//function used as callback when bike status message enters mailbox
void updateCAN_BikeStatus_Struct(const CAN_message_t &msg) {
    // parses CAN message values into variables  
    CAN_BikeStatus_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_BikeStatus_Struct.speed = (msg.buf[2] << 8) | msg.buf[3];
    CAN_BikeStatus_Struct.temp = (msg.buf[4] << 8) | msg.buf[5];
    CAN_BikeStatus_Struct.throttle = msg.buf[6];
    CAN_BikeStatus_Struct.error = msg.buf[7];
    CAN_MB0_BikeStatus_Flag = true; // updated values have been read
}

//function used as callback when motor voltage message enters mailbox
void updateCAN_MotorVoltages_Struct(const CAN_message_t &msg) {
    // parses CAN message values into variables 
    CAN_MotorVoltages_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_MotorVoltages_Struct.vDC = (msg.buf[2] << 8) | msg.buf[3];
    CAN_MotorVoltages_Struct.vA = (msg.buf[4] << 8) | msg.buf[5];
    CAN_MotorVoltages_Struct.vB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB1_MotorVoltages_Flag = true; // updated values have been read
}

//function used as callback when real current message enters mailbox
void updateCAN_RealCurrents_Struct(const CAN_message_t &msg) {
    // parses CAN message values into variables 
    CAN_RealCurrents_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_RealCurrents_Struct.iDC = (msg.buf[2] << 8) | msg.buf[3];
    CAN_RealCurrents_Struct.iA = (msg.buf[4] << 8) | msg.buf[5];
    CAN_RealCurrents_Struct.iB = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB2_RealCurrents_Flag = true; // updated values have been read
}

//function used as callback when calculated values message enters mailbox
void updateCAN_CalcValues_Struct(const CAN_message_t &msg) {
    // parses CAN message values into variables 
    CAN_CalcValues_Struct.timestamp = (msg.buf[0] << 8) | msg.buf[1];
    CAN_CalcValues_Struct.iqRef = (msg.buf[2] << 8) | msg.buf[3];
    CAN_CalcValues_Struct.iqFdb = (msg.buf[4] << 8) | msg.buf[5];
    CAN_CalcValues_Struct.power = (msg.buf[6] << 8) | msg.buf[7];
    CAN_MB3_CalcValues_Flag = true; // updated values have been read
}

//function used as callback when BMS message enters mailbox
void updateCAN_BmsSoc_Struct(const CAN_message_t &msg) {
    // parses CAN message values into variables 
    CAN_BmsSoc_Struct.soc = (msg.buf[0] << 8) | msg.buf[1];
    CAN_BmsSoc_Struct.soh = (msg.buf[2] << 8) | msg.buf[3];
    CAN_MB4_BmsSoc_Flag = true; // updated values have been read
}

// gets the received values from the CAN message and stores values in main
void getCAN_BikeStatus_Values(uint16_t &timestamp, uint16_t &Speed, uint16_t &temp, uint8_t &throttleInput, uint8_t &ErrorWarning) {
    timestamp = CAN_BikeStatus_Struct.timestamp; 
    Speed = CAN_BikeStatus_Struct.speed;
    temp = CAN_BikeStatus_Struct.temp_fet;
    throttleInput = CAN_BikeStatus_Struct.throttleInput;
    ErrorWarning = CAN_BikeStatus_Struct.errorWarning; 
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_MotorVoltages_Values(uint16_t &timestamp, uint16_t &VDC, uint16_t &VA, uint16_t &VB) {
    timestamp = CAN_BikeStatus_Struct.timestamp;
    VDC = CAN_MotorVoltages_Struct.vDC;
    VA = CAN_MotorVoltages_Struct.vA;
    VB = CAN_MotorVoltages_Struct.vB;
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_RealCurrents_Values(uint16_t &timestamp, uint16_t &IDC, uint16_t &IA, uint16_t &IB) {
    timestamp = CAN_BikeStatus_Struct.timestamp;
    IDC = CAN_RealCurrents_Struct.iDC;
    IA = CAN_RealCurrents_Struct.iA;
    IB = CAN_RealCurrents_Struct.iB;
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_CalcValues_Values(uint16_t &timestamp, uint16_t &Ref_iq, uint16_t &Fdb_iq, uint16_t &Power) {
    timestamp = CAN_BikeStatus_Struct.timestamp;
    Ref_iq = CAN_CalcValues_Struct.iqRef;
    Fdb_iq = CAN_CalcValues_Struct.iqFdb;
    Power = CAN_CalcValues_Struct.power;
    return;
}

// gets the received values from the CAN message and stores values in main
void getCAN_BmsSoc_Values(uint16_t &SOC, uint16_t &SOH) {
    SOC = CAN_BmsSoc_Struct.soc;
    SOH = CAN_BmsSoc_Struct.soh;
    return;
}