typedef enum tagCAN_ID {
    CAN_ID_BIKE_STATUS = 0x330,
    CAN_ID_MOTOR_VOLTAGES = 0x331,
    CAN_ID_MOTOR_REAL_CURRENTS = 0x332,
    CAN_ID_MOTOR_CALC_VALUES = 0x333,
    CAN_ID_UI = 0x334,
    CAN_ID_BMS_SOC = 0x355
} CAN_ID;

typedef struct tagCAN_BikeStatus_Struct{
    uint16_t timestamp;
    uint16_t speed; //speed of motor
    uint8_t throttleInput; // receive throttleInput from motor controller for verification 
    uint8_t errorWarning; // motor controller error/warning 
    uint16_t temp_fet; // mosfet bridge temperature
}CAN_BikeStatus_Struct;

typedef struct tagCAN_MotorVoltages_Struct{
    uint16_t timestamp;
    uint16_t vDC; // DC voltage
    uint16_t vA; // phase A voltage
    uint16_t vB; // phase B voltage
}CAN_MotorVoltages_Struct;

typedef struct tagCAN_RealCurrents_Struct{
    uint16_t timestamp;
    uint16_t iDC; // DC current
    uint16_t iA; // phase A current
    uint16_t iB; // phase B current
}CAN_RealCurrents_Struct;

typedef struct tagCAN_CalcValues_Struct{
    uint16_t timestamp;
    uint16_t iqRef; // q-axis current reference
    uint16_t iqFdb; // q-axis current 
    uint16_t power; // calculated power
}CAN_CalcValues_Struct;

typedef struct tagCAN_BmsSoc_Struct{
    uint16_t soc; // BMS state of charge
    uint16_t soh; // BMS state of health
}CAN_BmsSoc_Struct;

void canInit(FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16>& can);

void CANSendThrottleMsg(FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16>& can, int throttle);

void updateCAN_BikeStatus_Struct(const CAN_message_t &msg);

void updateCAN_MotorVoltages_Struct(const CAN_message_t &msg);

void updateCAN_RealCurrents_Struct(const CAN_message_t &msg);

void updateCAN_CalcValues_Struct(const CAN_message_t &msg);

void updateCAN_BmsSoc_Struct(const CAN_message_t &msg);

void getCAN_BikeStatus_Struct(uint16_t &timestamp, uint16_t &Speed, uint16_t &temp, uint8_t &throttleInput, uint8_t &ErrorWarning);

void getCAN_MotorVoltages_Struct(uint16_t &timestamp, uint16_t &VDC, uint16_t &VA, uint16_t &VB);

void getCAN_RealCurrents_Struct(uint16_t &timestamp, uint16_t &IDC, uint16_t &IA, uint16_t &IB);

void getCAN_CalcValues_Struct(uint16_t &timestamp, uint16_t &Ref_iq, uint16_t &Fdb_iq, uint16_t &Power);

void getCAN_BmsSoc_Values(uint16_t &SOC, uint16_t &SOH);

bool CANPendingBikeStatusMsg();

bool CANPendingMotorVoltagesMsg();

bool CANPendingRealCurrentsMsg();

bool CANPendingCalcValuesMsg();

bool CANPendingBmsSocMsg();






