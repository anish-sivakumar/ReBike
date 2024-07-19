#include <stdint.h>
#include <FlexCAN_T4.h>

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
    int16_t speed; //speed of motor
    int8_t throttleInput; // receive throttleInput from motor controller for verification 
    uint8_t errorWarning; // motor controller error/warning 
    uint16_t tempFet; // mosfet bridge temperature
}CAN_BikeStatus_Struct;

typedef struct tagCAN_MotorVoltages_Struct{
    uint16_t timestamp;
    int16_t vDC; // DC voltage
    int16_t vA; // phase A voltage
    int16_t vB; // phase B voltage
}CAN_MotorVoltages_Struct;

typedef struct tagCAN_RealCurrents_Struct{
    uint16_t timestamp;
    int16_t iDC; // DC current
    int16_t iA; // phase A current
    int16_t iB; // phase B current
}CAN_RealCurrents_Struct;

typedef struct tagCAN_CalcValues_Struct{
    uint16_t timestamp;
    int16_t iqRef; // q-axis current reference
    int16_t iqFdb; // q-axis current 
    int16_t power; // calculated power
}CAN_CalcValues_Struct;

typedef struct tagCAN_BmsSoc_Struct{
    uint16_t soc; // BMS state of charge
    uint16_t soh; // BMS state of health
}CAN_BmsSoc_Struct;

void canInit();

void CANSendThrottleMsg(int throttle);

void CANUpdateBikeStatus(const CAN_message_t &msg);

void CANUpdateMotorVoltages(const CAN_message_t &msg);

void CANUpdateRealCurrents(const CAN_message_t &msg);

void CANUpdateCalcValues(const CAN_message_t &msg);

void CANUpdateBmsSoc(const CAN_message_t &msg);

void getCAN_BikeStatus_Values(uint16_t &timestamp, uint16_t &speed, uint16_t &temp, int8_t &throttleInput, uint8_t &ErrorWarning);

void getCAN_MotorVoltages_Values(uint16_t &timestamp, int16_t &VDC, int16_t &VA, int16_t &VB);

void getCAN_RealCurrents_Values(uint16_t &timestamp, int16_t &IDC, int16_t &IA, int16_t &IB);

void getCAN_CalcValues_Values(uint16_t &timestamp, int16_t &Ref_iq, int16_t &Fdb_iq, int16_t &Power);

void getCAN_BmsSoc_Values(uint16_t &SOC, uint16_t &SOH);

bool CANPendingBikeStatusMsg();

bool CANPendingMotorVoltagesMsg();

bool CANPendingRealCurrentsMsg();

bool CANPendingCalcValuesMsg();

bool CANPendingBmsSocMsg();






