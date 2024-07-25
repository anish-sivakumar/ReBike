#include <SD.h>
#include <string>

#include "logging.h"

static String log_name;

bool loggingInit(int sd_cs_pin){
  
  bool success = false;
  if (SD.begin(sd_cs_pin)){
    // make a log name that doesnt already exist
    int log_number = 0;
    log_name = "log" + String(log_number) + ".csv";
    while (SD.exists(log_name.c_str())){
      log_number++;
      log_name = "log" + String(log_number) + ".csv";
    }
    File log_file = SD.open(log_name.c_str(), FILE_WRITE);
    log_file.println(LOGGING_HEADER);
    log_file.close();
    success = true;
  }
  return success;
}


bool loggingStepReady(const uint16_t timestamp_list[4], uint16_t timestamp_expected){
  // Assume not ready to log.
  bool ready = false; 

  // If all the received timestamps are the expected, then we received all the messages and are ready to log.
  if (
    timestamp_list[0] == timestamp_expected && 
    timestamp_list[1] == timestamp_expected &&
    timestamp_list[2] == timestamp_expected &&
    timestamp_list[3] == timestamp_expected){
      ready = true;
    }
  // If any one of the timestamps is greater than the expected, then we missed a message.
  // There are probably better ways to handle this, but for simplicity lets log anyways to avoid having missing timestamps in the log files.
  else if (
    timestamp_list[0] > timestamp_expected ||
    timestamp_list[1] > timestamp_expected ||
    timestamp_list[2] > timestamp_expected ||
    timestamp_list[3] > timestamp_expected){
      ready = true;
    }

  return ready;
}

uint16_t loggingNextExpectedTimestamp(const uint16_t timestamp_list[4]){
  // If all timestamps are the same, then we logged a full message and the next timestamp is incremented
  if (
    timestamp_list[0] == timestamp_list[1] && 
    timestamp_list[0] == timestamp_list[2] &&
    timestamp_list[0] == timestamp_list[3]){
      return timestamp_list[0] + 1;
    }
  // If all timestamps are not the same, then we missed one of the previous messages.
  // In this case, the next timestamp has already been received so the largest timestamp in the list is the next expected
  else{
    uint16_t max = timestamp_list[0];
    for (int i = 1; i < CAN_RX_MSG_COUNT; i++)
    {
      if (timestamp_list[i] > max){
        max = timestamp_list[i];
      }
    }
    return max;
  }
}

void loggingStepWrite(uint16_t timestamp, int throttle, int speed, int power, int temp, int batterySOC, int regenMethod, const LoggingData &logging_data){
  size_t start = millis();
  File log_file = SD.open(log_name.c_str(), FILE_WRITE);
  log_file.print(timestamp);
  log_file.write(',');
  log_file.print(logging_data.vDC);
  log_file.write(',');
  log_file.print(logging_data.iDC);
  log_file.write(',');
  log_file.print(logging_data.iA);
  log_file.write(',');
  log_file.print(logging_data.iB);
  log_file.write(',');
  log_file.print(logging_data.vA);
  log_file.write(',');
  log_file.print(logging_data.vB);
  log_file.write(',');
  log_file.print(speed);
  log_file.write(',');
  log_file.print(regenMethod);
  log_file.write(',');
  log_file.print(throttle);
  log_file.write(',');
  log_file.print(logging_data.iqRef);
  log_file.write(',');
  log_file.print(logging_data.iqFdb);
  log_file.write(',');
  log_file.print(temp);
  log_file.write(',');
  log_file.print(power);
  log_file.write(',');
  log_file.print(logging_data.error);
  log_file.write(',');
  log_file.print(batterySOC);
  log_file.write(',');
  log_file.print(logging_data.vBMS);
  log_file.write(',');
  log_file.print(logging_data.iBMS);
  log_file.write('\n');

  log_file.close();
  SPI.endTransaction();

  Serial.print("time to log:");
  Serial.println(millis() - start);
}

