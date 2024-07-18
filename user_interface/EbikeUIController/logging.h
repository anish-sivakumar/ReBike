
#define CAN_RX_MSG_COUNT 4  // number of different can messages received from the motor controller
#define LOGGING_CS_PIN 9    // chip select pin for sd card module
#define LOGGING_HEADER "timestamp,vDC,iDC,iA,iB,vA,vB,speed,regen_method,throttle,iqRef,iqFdb,temp_fet,power,error"

typedef struct tagLoggingData {
  int16_t vDC;    // DC voltage
  int16_t iDC;    // DC current
  int16_t iA;     // phase A current
  int16_t iB;     // phase B current
  int16_t vA;     // phase A voltage
  int16_t vB;     // phase B voltage
  int16_t speed;  // motor speed [RPM]
  int16_t regen_method;
  int8_t throttle;
  int16_t iqRef;     // q-axis current reference
  int16_t iqFdb;     // q-axis current something???
  uint16_t temp_fet;  // mosfet bridge temperature
  int16_t power;     // calculated power
  int8_t error;      // error code from motor controller
} LoggingData;

// TODO: decide if this is needed
typedef struct tagLoggingTimestamps {
  uint16_t expected;
  uint16_t received_list[CAN_RX_MSG_COUNT];
} LoggingTimestamps;

bool loggingInit(int logging_cs_pin);
bool loggingStepReady(const uint16_t timestamp_list[CAN_RX_MSG_COUNT], uint16_t timestamp_expected);
uint16_t loggingNextExpectedTimestamp(const uint16_t timestamp_list[CAN_RX_MSG_COUNT]);
void loggingStepWrite(uint16_t timestamp, const LoggingData &logging_data);