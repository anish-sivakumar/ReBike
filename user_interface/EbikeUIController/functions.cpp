#include "globals.h"
#include "functions.h"
#include "bitmaps.h"


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


void updateSpeed(void) {

  // Code to read the current speed signal from the motor controller and update the speed variable goes here

  // This is connected to a potentiometer for testing purposes only.
  speed = map(analogRead(14), 0, 1023, 0, 99);
  // throttle = map(analogRead(14), 0, 1023, 0, 99);

}


void updatePower(void) {  

  // Code to read the current motor power output signal from the motor controller and update the power variable goes here
  
  // This is connected to a potentiometer for testing purposes only.
  power = map(analogRead(14), 0, 1023, 0, 999);

}


void updateTemp(void) {

  // Code to read the current motor temperature signal from the motor controller and update the temperature variable goes here
  
  // This is connected to a potentiometer for testing purposes only.
  temp = map(analogRead(14), 0, 1023, 0, 99);

}


void updateBatteryRange(void) {

  // Code to read the current battery range signal from the BMS and update the battery range variable goes here

  // This is connected to a potentiometer for testing purposes only.
  batteryRange = map(analogRead(14), 0, 1023, 0, 99);

}


void activateRegenRecovery() {

  // Code to send request via CAN to motor controller FW to activate regerative braking and recover energy for battery

}


void toggleRegenMethod() {
 
  activeRegen = (activeRegen == 1) ? 2 : 1; // Toggle between 1 and 2 

  // Code to send request via CAN to motor controller FW to toggle the active regenerative braking method goes here

}


void updateDisplay(void) {

  itoa(speed, speed_string, 10);
  itoa(throttle, throttle_string, 10);
  itoa(power, power_string, 10);
  itoa(temp, temp_string, 10);
  itoa(batteryRange, battery_string, 10);

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
    
    if (batteryRange > 80) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_100_);
    } else if (batteryRange > 60 && batteryRange <= 80) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_80_);
    } else if (batteryRange > 40 && batteryRange <= 60) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_60_);
    } else if (batteryRange > 20 && batteryRange <= 40) {
      u8g2.drawBitmap(93, 36, 32/8, 20, epd_bitmap_Battery_Level_Sym_40_);
    } else if (batteryRange > 5 && batteryRange <= 20) {        
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
