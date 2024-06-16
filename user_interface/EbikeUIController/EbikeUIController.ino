#include <U8g2lib.h>
#include <SPI.h>                  // Include the SPI library for communication with the CAN module
#include <Wire.h>                 // Include the Wire library for I2C communication
#include <Toggle.h>               // Include the Toggle library for handling button states

// Define OLED pin connections
#define OLED_MOSI   11
#define OLED_CLK    13
#define OLED_DC     5
#define OLED_CS     10
#define OLED_RESET  23

// Create an instance for a 128x64 display with software SPI
U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clk=*/ OLED_CLK, /* data=*/ OLED_MOSI, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);



// Global variables to store different parameters
int speed = 0;                    // Speed of the e-bike
int batteryRange = 0;             // Battery range of the e-bike
int power = 0;                    // Power consumption of the e-bike
int temp = 0;                     // Temperature of the e-bike system
int regenMethod = 1;              // Regenerative braking method (1 or 2)

// Create Toggle instances for buttons to increase throttle, decrease throttle, and toggle regenerative braking
Toggle increaseThrottle(10);       // Button to increase throttle
Toggle decreaseThrottle(11);       // Button to decrease throttle
Toggle toggleRegen(12);            // Button to toggle regenerative braking method

void setup() {

  // Set the built-in LED as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize the SSD1306 OLED display with the specified parameters
  // SSD1306_SWITCHCAPVCC generates display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {

    // If display initialization fails, print an error message and enter an infinite loop
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever

  }

  // Clear the display buffer
  display.clearDisplay();

  // Initialize the button toggles
  increaseThrottle.begin(10);
  decreaseThrottle.begin(11);
  toggleRegen.begin(12);

}

void loop() {

  // Poll the state of each button
  increaseThrottle.poll();
  decreaseThrottle.poll();
  toggleRegen.poll();

  // Update various parameters (speed, battery range, power, temperature)
  updateSpeed();
  updateBatteryRange();
  updatePower();
  updateTemp();

  // Check if the increase throttle button has been pressed
  if (increaseThrottle.onPress()) {
    speed = speed + 1;
    // updateDisplay();
    // Adjust the speed request by increasing it
    // adjustSpeedRequest(+1);
    Serial.println("Request to increase motor speed sent to Motor Controller via CAN");
    // Serial.write("Request to increase motor speed sent to Motor Controller via CAN");

    // Optionally, turn on the built-in LED to indicate the button press
    // digitalWrite(LED_BUILTIN, HIGH);

  }

  // Check if the decrease throttle button has been pressed
  if (decreaseThrottle.onPress()) {
    speed = speed - 1;
    // updateDisplay();
    // Adjust the speed request by decreasing it
    // adjustSpeedRequest(-1);
    Serial.println("Request to decrease motor speed sent to Motor Controller via CAN");
    // Serial.write("Request to decrease motor speed sent to Motor Controller via CAN");

    // Turn on the built-in LED to indicate the button press
    // digitalWrite(LED_BUILTIN, HIGH);

  }

   // Set the text size to 2 (normal 1:1 pixel scale)
    display.setTextSize(2);

    // Set the text color to white
    display.setTextColor(SSD1306_WHITE);

    // Set the cursor to the top-left corner
    display.setCursor(4, 0);

  // Check if the toggle regenerative braking button has been pressed
  if (toggleRegen.onPress()) {

    // Change the active regenerative braking method
    // changeActiveRegenRequest();
    Serial.println("Request to change the active regenerative braking method has been sent to Motor Controller via CAN");
    // Serial.write("Request to change the active regenerative braking method has been sent to Motor Controller via CAN");

    // Update the display to show the new regenerative braking method
    updateRegen();

    // Turn on the built-in LED to indicate the button press
    // digitalWrite(LED_BUILTIN, HIGH);

  }

  // Update the display to show the latest values
  updateDisplay();
  
}

// Function to update the speed parameter
void updateSpeed(void) {
  // Code to update the speed variable goes here
}

// Function to update the battery range parameter
void updateBatteryRange(void) {
  // Code to update the battery range variable goes here
}

// Function to update the power parameter
void updatePower(void) {
  // Code to update the power variable goes here
}

// Function to update the temperature parameter
void updateTemp(void) {
  // Code to update the temperature variable goes here
}

// Function to update the active regenerative braking method
void updateRegen(void) {
  // Code to update the active regenerative braking method variable goes here
}

// Function to update the display with the latest values
void updateDisplay(void) {

  // Clear the display buffer
  display.clearDisplay();

  // Display the current speed value
  display.println(speed);

  // Send the buffer to the display to update it
  display.display();

}
