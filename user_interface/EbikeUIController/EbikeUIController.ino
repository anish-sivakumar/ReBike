#include <SPI.h>                  // Include the SPI library for communication with the CAN module
#include <Wire.h>                 // Include the Wire library for I2C communication
#include <Adafruit_GFX.h>         // Include the Adafruit GFX library for graphics functions
#include <Adafruit_SSD1306.h>     // Include the Adafruit SSD1306 library for OLED display functions
#include <Toggle.h>               // Include the Toggle library for handling button states

#define SCREEN_WIDTH 128          // Define the width of the OLED display in pixels
#define SCREEN_HEIGHT 32          // Define the height of the OLED display in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// The address of the display can be 0x3D for 128x64 displays, or 0x3C for 128x32 displays
#define OLED_RESET     -1         // Reset pin (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C       // I2C address for the OLED display

// Create an instance of the Adafruit_SSD1306 class with the specified width, height, and I2C parameters
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global variables to store different parameters
int speed = 0;                    // Speed of the e-bike
int batteryRange = 0;             // Battery range of the e-bike
int power = 0;                    // Power consumption of the e-bike
int temp = 0;                     // Temperature of the e-bike system
int regenMethod = 1;              // Regenerative braking method (1 or 2)

// Create Toggle instances for buttons to increase throttle, decrease throttle, and toggle regenerative braking
Toggle increaseThrottle(0);       // Button to increase throttle
Toggle decreaseThrottle(1);       // Button to decrease throttle
Toggle toggleRegen(2);            // Button to toggle regenerative braking method

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
  increaseThrottle.begin(0);
  decreaseThrottle.begin(1);
  toggleRegen.begin(2);

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

    // Adjust the speed request by increasing it
    adjustSpeedRequest(+1);

    // Optionally, turn on the built-in LED to indicate the button press
    // digitalWrite(LED_BUILTIN, HIGH);

  }

  // Check if the decrease throttle button has been pressed
  if (decreaseThrottle.onPress()) {

    // Adjust the speed request by decreasing it
    adjustSpeedRequest(-1);

    // Turn on the built-in LED to indicate the button press
    // digitalWrite(LED_BUILTIN, HIGH);

  }

  // Check if the toggle regenerative braking button has been pressed
  if (toggleRegen.onPress()) {

    // Change the active regenerative braking method
    changeActiveRegenRequest();

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

// Function to update the display with the latest values
void updateDisplay(void) {
  // Clear the display buffer
  display.clearDisplay();

  // Set the text size to 2 (normal 1:1 pixel scale)
  display.setTextSize(2);

  // Set the text color to white
  display.setTextColor(SSD1306_WHITE);

  // Set the cursor to the top-left corner
  display.setCursor(4, 0);

  // Display the current speed value
  display.println(speed);

  // Send the buffer to the display to update it
  display.display();
}
