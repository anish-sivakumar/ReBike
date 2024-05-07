#include <Arduino.h>

#define LED 13

// put function declarations here:
int myFunction(int, int);
void blink(int val);
int led_state;

void setup()
{
	// put your setup code here, to run once:
	pinMode(LED, OUTPUT);
	led_state = 0;

}

void loop()
{
	// put your main code here, to run repeatedly:
	blink(led_state);
	led_state = !led_state;
	delay(1000);
}

// put function definitions here:
int myFunction(int x, int y)
{
	return x + y;
}

void blink(int val)
{
	digitalWrite(LED,val);
}