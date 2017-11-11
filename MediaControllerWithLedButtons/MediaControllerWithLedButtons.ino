#include <Encoder.h>
#include "HID-Project.h"
#include <Bounce2.h>
#include <SoftPWM.h>

/**
 * Arduino based Media key controller V1.0
 * By George Timmermans
 * http://www.georgetimmermans.com/usb-media-controller.html
 */

/*** Libraries ***
 * Encoder Library - http://www.pjrc.com/teensy/td_libs_Encoder.html
 * HID Project -     https://github.com/NicoHood/HID/wiki/Consumer-API
 * Bounce Library -  https://github.com/thomasfredericks/Bounce-Arduino-Wiring
 * SoftPWM Library - https://code.google.com/archive/p/rogue-code/wikis/SoftPWMLibraryDocumentation.wiki
 */

/*** Pro Micro Pinout ***
 *                    ___
 *                ___|   |___
 *               |    ---    |
 *               |* 1    RAW*|
 *               |* 0    GND*|
 *               |* GND  RST*|
 *               |* GND  VCC*|
 *  encB -       |* 2     21*| - playPausePin
 *  encA -       |* 3     20*| - led1Pin
 *  encMutePin-  |* 4     19*| - stopPin
 *  encBlueLED ~ |* 5     18*| - led2Pin
 *  encGreenLED ~|* 6     15*| - nextPin
 *               |* 7     14*| - led3Pin
 *  vcc_source - |* 8     16*| - previousPin
 *  encRedLED ~  |* 9     10*| - led4Pin
 *                -----------
 */

// constants won't change. They're used here to set pin numbers:
const uint8_t playPausePin = 21;     // the number of the pushbutton pin
const uint8_t led1Pin =  20;      // the number of the LED pin

const uint8_t stopPin = 19;     // the number of the pushbutton pin
const uint8_t led2Pin =  18;      // the number of the LED pin

const uint8_t nextPin = 15;     // the number of the pushbutton pin
const uint8_t led3Pin =  10;      // the number of the LED pin

const uint8_t previousPin = 16;     // the number of the pushbutton pin
const uint8_t led4Pin =  14;      // the number of the LED pin

const uint8_t vcc_source = 8;
const uint8_t redLedPin =  9;      // the number of the LED pin
const uint8_t greenLedPin =  6;      // the number of the LED pin
const uint8_t blueLedPin =  5;      // the number of the LED pin

const uint8_t encA = 3;
const uint8_t encB = 2;
const uint8_t mutePin = 4;   // the number of the pushbutton pin

uint8_t brightness = 120;    // how bright the LED is, start at half brightness
long oldPosition  = -999;
bool muteState = false;      // Remeber what state was the button last time

// Change these two numbers to the pins connected to your encoder.
// -Best Performance: both pins have interrupt capability
// -Good Performance: only the first pin has interrupt capability
// -Low Performance:  neither pin has interrupt capability
Encoder myEnc(encB, encA);

// Instantiate a Bounce object
Bounce muteButton = Bounce(); 
Bounce playPauseButton = Bounce(); 
Bounce stopButton = Bounce(); 
Bounce nextButton = Bounce(); 
Bounce previousButton = Bounce(); 
// Debounce time in milliseconds
const unsigned long intervalMs = 5;

//----------------------------------------------------------------
void setup() {
  Consumer.begin();
  SoftPWMBegin();
  Serial.begin(9600);

  // Setup the Bounce instance :
  pinMode(mutePin, INPUT_PULLUP);
  muteButton.attach(mutePin);
  muteButton.interval(intervalMs); 
  
  pinMode(playPausePin, INPUT_PULLUP);
  playPauseButton.attach(playPausePin);
  playPauseButton.interval(intervalMs); 
  
  pinMode(stopPin, INPUT_PULLUP);
  stopButton.attach(stopPin);
  stopButton.interval(intervalMs);
  
  pinMode(nextPin, INPUT_PULLUP);
  nextButton.attach(nextPin);
  nextButton.interval(intervalMs);
  
  pinMode(previousPin, INPUT_PULLUP);
  previousButton.attach(previousPin);
  previousButton.interval(intervalMs);
  
  // initialize the rotary encoder RGB LED pins as an output:
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, LOW);

  // create a 3.3V source for the breakout board
  pinMode(vcc_source, OUTPUT);
  digitalWrite(vcc_source, HIGH);

  // Sets the PWM value to 255 for the buttons built-in LEDs.
  SoftPWMSet(led1Pin, 255);
  SoftPWMSet(led2Pin, 255);
  SoftPWMSet(led3Pin, 255);
  SoftPWMSet(led4Pin, 255);
  
  // Sets the default fade time for the button LEDs to
  // 1500 ms fade-up and 500 ms to fade-down.
  SoftPWMSetFadeTime(led1Pin, 1500, 500);
  SoftPWMSetFadeTime(led2Pin, 1500, 500);
  SoftPWMSetFadeTime(led3Pin, 1500, 500);
  SoftPWMSetFadeTime(led4Pin, 1500, 500);
}
//----------------------------------------------------------------
void loop() {
  checkVolumeControl();
  checkMuteButton();
  checkPlayPauseButton();
  checkStopButton();
  checkNextButton();
  checkPreviousButton();
}
//----------------------------------------------------------------
void checkPreviousButton() {
  previousButton.update();
  const byte buttonState = previousButton.read();
  if (buttonState == LOW && previousButton.fallingEdge()) {
    Consumer.write(MEDIA_PREVIOUS);
    buttonAnimation(led3Pin);
  }
}
//----------------------------------------------------------------
void checkNextButton() {
  nextButton.update();
  const byte buttonState = nextButton.read();
  if (buttonState == LOW && nextButton.fallingEdge()) {
    Consumer.write(MEDIA_NEXT);
    buttonAnimation(led4Pin);
  }
}
//----------------------------------------------------------------
void checkPlayPauseButton() {
  playPauseButton.update();
  const byte buttonState = playPauseButton.read();
  if (buttonState == LOW && playPauseButton.fallingEdge()) {
    Consumer.write(MEDIA_PLAY_PAUSE);
    buttonAnimation(led1Pin);
  }
}
//----------------------------------------------------------------
void checkStopButton() {
  stopButton.update();
  const byte buttonState = stopButton.read();
  if (buttonState == LOW && stopButton.fallingEdge()) {
    Consumer.write(MEDIA_STOP);
    buttonAnimation(led2Pin);
  }
}
//----------------------------------------------------------------
void checkMuteButton() {
  muteButton.update();
  const byte buttonState = muteButton.read();
  if (buttonState == LOW && muteButton.fallingEdge()) {
    Consumer.write(MEDIA_VOLUME_MUTE);
    if (!muteState) {
      muteState = true;
      analogWrite(redLedPin, 0);
      analogWrite(greenLedPin, 0);
      analogWrite(blueLedPin, 0);
    } else {
      muteState = false;
      analogWrite(redLedPin, 255 - brightness);
      analogWrite(greenLedPin, brightness);
      analogWrite(blueLedPin, 255);
    }
  } 
}
//----------------------------------------------------------------
void buttonAnimation(const uint8_t pin) {
  // Turn off LED
  SoftPWMSet(pin, 0);
  // Wait for the fade-down, and some extra delay.
  delay(750);
  // Turn on LED
  SoftPWMSet(pin, 255);
}
//----------------------------------------------------------------
void checkVolumeControl() {
  // how many points to fade the LED by
  const uint8_t fadeAmount = 5;    
  const long newPosition = myEnc.read();
  analogWrite(blueLedPin, 255);
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition) {
      Consumer.write(MEDIA_VOLUME_UP);
      // increase the brightness, dont go over 255
      if(brightness + fadeAmount <= 255) {
        brightness += fadeAmount;    
      } else {
        brightness = 255;
      }
    } else {
      Consumer.write(MEDIA_VOLUME_DOWN);
      // lower the brightness, dont go below 0
      if(brightness - fadeAmount >= 0) {
        brightness -= fadeAmount;
      } else {
        brightness = 0;
      }
    }
    oldPosition = newPosition;
    muteState = false;
    analogWrite(redLedPin, 255 - brightness);
    analogWrite(greenLedPin, brightness);
    Serial.print("redLedPin: "); Serial.println(255 - brightness);
    Serial.print("greenLedPin: "); Serial.println(brightness);
    Serial.println();
  }
}
//----------------------------------------------------------------
