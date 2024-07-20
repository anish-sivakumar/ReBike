#include "functions.h"
#include "bitmaps.h"
#include <SPI.h>
////////////////////
// SCREEN GLOBALS // 
////////////////////
// Create an instance for a 128x64 display with software SPI

//static U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clk=*/ OLED_CLK, /* data=*/ OLED_MOSI, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);
static U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);
// String buffers for displaying values
char speed_string[10];            // String to display speed
int speed_string_length;          // Length of the speed string

char throttle_string[10];         // String to display throttle value
int throttle_string_length;       // Length of the throttle string

char power_string[10];            // String to display power value
int power_string_length;          // Length of the power string

char temp_string[10];             // String to display temperature
int temp_string_length;           // Length of the temperature string

char battery_string[10];          // String to display battery range
int battery_string_length;        // Length of the battery string

void displayInit() {

  u8g2.begin(); // Initialize the OLED display

  // Display the logo during setup
  u8g2.firstPage();
  do {
    u8g2.drawBitmap(0, 0, 128/8, 64, epd_bitmap_REBIKE_Logo); // Draw logo during system startup
  } while (u8g2.nextPage());

  delay(4000); // Wait for 4 seconds to show the logo

}

void pinModesInit() {
  pinMode(E_BRAKE, INPUT_PULLUP);
  pinMode(REGEN_METHOD_TOGGLE, INPUT_PULLUP);
  pinMode(THROTTLE_SPEED_INPUT, INPUT); // Analog input pin does not need pull-up
}

bool handleThrottleInput(UserInputRequest inputRequest, int8_t &throttle, bool &activatedRegen, RegenMethod regenMethod){
  bool throttle_changed = false;

  switch (inputRequest) {
    case INCREASE:
      // Increase throttle if not at maximum
      if (throttle == 100) {
        return false;
      } else {
        throttle += 5; // Increase throttle by 20
        throttle_changed = 1; // Set throttle flag
      }
      break; 

    case DECREASE:
      // Decrease throttle if not at minimum
      if (throttle == 0) {
        return false;
      } else {
        throttle -= 5; // Decrease throttle by 20
        throttle_changed = 1; // Set throttle flag
      }
      break; 

    case REGEN:
      switch (regenMethod) {
        case DIGITAL:
          throttle = -100;
          break; 
        case ANALOG:
          // No action required as the currentBrakeState has already been set in the timerISR
          break; 
      }
      
      activatedRegen = true;
      throttle_changed = 1;
      break; 
  }

  
  // // Check if any flag is set to send CAN message
  // if (throttle_changed == 1) {
  //   sendCANMSG(); // Send CAN message if any flag is set
  //   throttle_changed = 0;
  // } Commented out until CAN functionality is integrated to avoid compilation errors

  return throttle_changed;
}

// Function to update the display with the latest values
void updateDisplay(int throttle, int speed, int power, int temp, int batterySOC, int regenMethod) {

  // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // 8 MHz, most significant bit first, SPI mode 0
  // digitalWrite(OLED_CS, LOW); // Select OLED display

  bool negativeThrottle;
  // Handle negative throttle values
  if (throttle < 0) {
    itoa(throttle * -1, throttle_string, 10);
    negativeThrottle = true;  
  } else {
    itoa(throttle, throttle_string, 10);
    negativeThrottle = false;
  }

  int speedKph = (int)((float)speed * 0.135);
  // Convert integer values to string representations
  itoa(speedKph, speed_string, 10);
  itoa(power, power_string, 10);
  itoa(temp, temp_string, 10);
  itoa(batterySOC, battery_string, 10);

  // Calculate string lengths for positioning
  speed_string_length = strlen(speed_string);
  throttle_string_length = strlen(throttle_string);
  power_string_length = strlen(power_string);
  temp_string_length = strlen(temp_string);
  battery_string_length = strlen(battery_string);
  
  // Start the display update process
  u8g2.firstPage();
  do {

    // Draw speed value on the display
    for (int i = 0; i < speed_string_length; i++) {
      u8g2.drawBitmap((56 - speed_string_length * 28) + 30 * i, 2, 24 / 8, 36, epd_bitmap_speedDigitsArray[speed_string[i] - 48]);
    }

    if (negativeThrottle) {
      u8g2.drawBitmap(2, 46, 8/8, 2, epd_bitmap_negative_sym); // Draw negative sign
    }

    // Draw throttle value on the display
    for (int i = 0; i < throttle_string_length; i++) {
      u8g2.drawBitmap((38 - throttle_string_length * 10) + 10 * i, 41, 8 / 8, 12, epd_bitmap_throttleDigitsArray[throttle_string[i] - 48]);
    }

    // Draw power value on the display
    for (int i = 0; i < power_string_length; i++) {
      u8g2.drawBitmap((102 - power_string_length * 4) + 6 * i, 9, 8 / 8, 6, epd_bitmap_motorDigitsArray[power_string[i] - 48]);
    }

    // Draw temperature value on the display
    for (int i = 0; i < temp_string_length; i++) {
      u8g2.drawBitmap((102 - temp_string_length * 4) + 6 * i, 17, 8 / 8, 6, epd_bitmap_motorDigitsArray[temp_string[i] - 48]);
    }

    // Draw battery SOC value on the display
    for (int i = 0; i < battery_string_length; i++) {
      u8g2.drawBitmap((102 - battery_string_length * 4) + 6 * i, 58, 8 / 8, 6, epd_bitmap_motorDigitsArray[battery_string[i] - 48]);
    }

    // Draw static labels and symbols on the display
    u8g2.drawBitmap(80, 25, 48 / 8, 7, epd_bitmap_BATTERY_Label);
    u8g2.drawBitmap(78, 0, 8 / 8, 64, epd_bitmap_Vert_Line);
    u8g2.drawBitmap(0, 55, 88 / 8, 1, epd_bitmap_Lower_Horiz_Line);
    u8g2.drawBitmap(80, 24, 48 / 8, 1, epd_bitmap_Upper_Horiz_Line);
    u8g2.drawBitmap(59, 7, 24 / 8, 25, epd_bitmap_KM_HR_Sym);
    u8g2.drawBitmap(39, 41, 16 / 8, 12, epd_bitmap_Throttle_Percentage_Sym);
    u8g2.drawBitmap(52, 44, 32 / 8, 6, epd_bitmap_THRTL_label);
    u8g2.drawBitmap(-2, 58, 72 / 8, 6, epd_bitmap_ACTIVE_REGEN_Label);
    u8g2.drawBitmap(92, 0, 32 / 8, 8, epd_bitmap_MOTOR_Label);
    u8g2.drawBitmap(110, 8, 16 / 8, 7, epd_bitmap_Watts_Sym);
    u8g2.drawBitmap(110, 17, 16 / 8, 6, epd_bitmap_Celsius_Sym);
    u8g2.drawBitmap(108, 57, 16 / 8, 6, epd_bitmap_Battery_Percentage_Sym);

    // Draw battery level indicator based on SOC
    if (batterySOC > 80) {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_100_);
    } else if (batterySOC > 60 && batterySOC <= 80) {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_80_);
    } else if (batterySOC > 40 && batterySOC <= 60) {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_60_);
    } else if (batterySOC > 20 && batterySOC <= 40) {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_40_);
    } else if (batterySOC > 5 && batterySOC <= 20) {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_20_);
    } else {
      u8g2.drawBitmap(93, 36, 32 / 8, 20, epd_bitmap_Battery_Level_Sym_0_);
    }

    // Draw active regen status
    if (regenMethod == 1) {
      u8g2.drawBitmap(73, 58, 8 / 8, 6, epd_bitmap_motor_digit_1);
    } else {
      u8g2.drawBitmap(73, 58, 8 / 8, 6, epd_bitmap_motor_digit_2);
    }

  } while (u8g2.nextPage());
  

}
