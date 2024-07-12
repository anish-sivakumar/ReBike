#include "globals.h"
#include "functions.h"
#include "bitmaps.h"

#include <FlexCAN_T4.h>           // Include for CAN communication


void displayInit() {

  // Initialize the OLED display
  u8g2.begin();

  // Display the logo during setup
  u8g2.firstPage();
  do {
    // Draw logo during system startup
    u8g2.drawBitmap(0, 0, 128/8, 64, epd_bitmap_REBIKE_Logo);
  } while (u8g2.nextPage());

  // Wait for 4 seconds to show the logo
  delay(4000);

}

void canInit() {

  Serial.begin(115200); // Set baudrate to 500kbit
  can0.begin(); // CAN initialization
  can0.setBaudRate(500000); // Set CAN baud rate to 500kbit

  can0.setMBFilter(REJECT_ALL);
  can0.setMBFilter(MB0, Soc_ID); // Set mailbox CAN ID filters
  can0.setMBFilter(MB1, Motor_ID);
  can0.enableMBInterrupt(MB0); // Enable mailbox interrupts
  can0.enableMBInterrupt(MB1); 
  can0.onReceive(MB0, updateBatterySOC); // Set mailbox 0 to receive battery state of charge, assign updateBatterySOC function
  can0.onReceive(MB1, updateMotor); // Set mailbox 1 to receive Motor parameters, assing updateMotor
  can0.setMB(MB2, TX); // Set mailbox 2 as transmit
  
  msg.id = 0x333; // set CAN ID 0x333 to throttle/regen info 
  msg.len = 8; // data length set to 8 bytes

}

void toggleActiveRegenMethod(void) {
  msg.buf[0] = throttle & 0xFF; // throttle value in byte 0 
  msg.buf[1] = activeRegen & 0xFF; // regen mode in byte 1	
  can0.write(MB2, msg); // send message to mailbox 2 (transmit mailbox)
  throttle_flag = 0; regen_flag = 0; // reset flags
  activeRegen = (activeRegen == 1) ? 2 : 1; // Toggle between 1 and 2 
  //delay(500); do we need a delay?  
}

void updateMotorParams(const CAN_message_t &msg) {
  speed = (msg.buf[1] << 8) | msg.buf[0]; // speed allocated to bytes 0-1 
  power = (msg.buf[3] << 8) | msg.buf[2]; // power allocated to bytes 2-3
  temp  = msg.buf[4]; // temp allocated to byte 4
}

void updateBatterySOC(const CAN_message_t &msg) {
  batterySOC = (msg.buf[1] << 8) | msg.buf[0]; // data is in the first two bytes
}

void adjustSpeedRequest(int adjustment) {

  // Code to send request to motor controller FW to adjust the current speed goes here

  if (adjustment == 1) {
    // Send increase speed request to motor controller

    if (throttle == 100) {
      return;
    } else {
      throttle += 5;
    }

  } else if (adjustment == -1) {
    // Send decrease speed request to motor controller
    
    if (throttle == 0) {
      return;
    } else {
      throttle -= 5;
    }
  
  }

}


void activateRegenRecovery() {

  // Code to send request via CAN to motor controller FW to activate regerative braking and recover energy for battery

}

// Function to update the display with the latest values
void updateDisplay(void) {

  itoa(speed, speed_string, 10);
  itoa(throttle, throttle_string, 10);
  itoa(power, power_string, 10);
  itoa(temp, temp_string, 10);
  itoa(batterySOC, battery_string, 10);

  speed_string_length = strlen(speed_string);
  throttle_string_length = strlen(throttle_string);
  power_string_length = strlen(power_string);
  temp_string_length = strlen(temp_string);
  battery_string_length = strlen(battery_string);

  u8g2.firstPage();
  do {
    
    for (int i = 0; i < speed_string_length; i++) {
      u8g2.drawBitmap( (56 - speed_string_length * 28 ) + 30*i, 2, 24/8, 36, epd_bitmap_speedDigitsArray[ speed_string[i] - 48 ]);
    }

    for (int i = 0; i < throttle_string_length; i++) {
      u8g2.drawBitmap( (20 - throttle_string_length * 10) + 10*i , 41, 8/8, 12, epd_bitmap_throttleDigitsArray[ throttle_string[i] - 48 ]);
    }

    for (int i = 0; i < power_string_length; i++) {
      u8g2.drawBitmap( (102 - power_string_length * 4 ) + 6*i, 9, 8/8, 6, epd_bitmap_motorDigitsArray[ power_string[i] - 48 ]);
    }

    for (int i = 0; i < temp_string_length; i++) {
      u8g2.drawBitmap( (102 - temp_string_length * 4 ) + 6*i, 17, 8/8, 6, epd_bitmap_motorDigitsArray[ temp_string[i] - 48 ]);
    }

    for (int i = 0; i < battery_string_length; i++) {
      u8g2.drawBitmap( (102 - battery_string_length * 4 ) + 6*i, 58, 8/8, 6, epd_bitmap_motorDigitsArray[ battery_string[i] - 48 ]);
    }

    u8g2.drawBitmap(80, 25, 48/8, 7, epd_bitmap_BATTERY_Label);
    u8g2.drawBitmap(78, 0, 8/8, 64, epd_bitmap_Vert_Line);
    u8g2.drawBitmap(0, 55, 88/8, 1, epd_bitmap_Lower_Horiz_Line);
    u8g2.drawBitmap(80, 24, 48/8, 1, epd_bitmap_Upper_Horiz_Line);
    u8g2.drawBitmap(59, 7, 24/8, 25, epd_bitmap_KM_HR_Sym);
    u8g2.drawBitmap(22, 41, 16/8, 12, epd_bitmap_Throttle_Percentage_Sym);
    u8g2.drawBitmap(35, 44, 48/8, 6, epd_bitmap_THROTTLE_Label);
    u8g2.drawBitmap(-2, 58, 72/8, 6, epd_bitmap_ACTIVE_REGEN_Label);
    u8g2.drawBitmap(92, 0, 32/8, 8, epd_bitmap_MOTOR_Label);
    u8g2.drawBitmap(110, 8, 16/8, 7, epd_bitmap_Watts_Sym);
    u8g2.drawBitmap(110, 17, 16/8, 6, epd_bitmap_Celsius_Sym);
    u8g2.drawBitmap(108, 57, 16/8, 6, epd_bitmap_Battery_Percentage_Sym);
    
    if (batterySOC > 80) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_100_);
    } else if (batterySOC > 60 && batterySOC <= 80) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_80_);
    } else if (batterySOC > 40 && batterySOC <= 60) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_60_);
    } else if (batterySOC > 20 && batterySOC <= 40) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_40_);
    } else if (batterySOC > 5 && batterySOC <= 20) {        
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_20_);
    } else {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_0_);
    }

    if (activeRegen == 1) {
      u8g2.drawBitmap(73, 58, 8/8, 6, epd_bitmap_motor_digit_1);
    } else {
      u8g2.drawBitmap(73, 58, 8/8, 6, epd_bitmap_motor_digit_2);
    }

  } while ( u8g2.nextPage() );

}