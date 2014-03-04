/*-----------------------------------------------------------------------------

Arduino Analog/Digital Inputs

Monitor digital inputs, then emit a serial character depending on the pin
status. The character will be uppercase for pin connected (N.O. button push)
and lowercase for pin disconnected (N.O. button released). It will begin with
"A" for the first I/O pin, "B" for the next, and so on. Currently, with pins
0 and 1 used for serial TX/RF, this leaves pins 2-12 available (10), with pin
13 reserved for blinking the onboard LED as a heartbeat "we are alive"
indicator.

This software was written to allow an Arduino act as a cheap input/trigger
interface to software such as VenueMagic. As I do not own a copy of this
software, I could only test it under the 15 day trial. There may be other
issues...

2012-10-09 0.0 allenh - Initial version.
2012-10-11 0.1 allenh - Updated debounce to work with timing rollover, via:
                        http://www.arduino.cc/playground/Code/TimingRollover
                        Fixed bug where last DI pin was not being used.
2012-10-15 0.2 allenh - Adding support for analog inputs, too.
2012-10-16 0.3 allenh - Adding '!' command to recalibrate analog inputs.

TODO:
1. Add DELAY value, to limit how fast/often an input can be triggered.
-----------------------------------------------------------------------------*/
#define VERSION "0.3"

//#include <EEPROM.h>
#include <avr/wdt.h>

/* TX/RX pins are needed to USB/serial. */
#define DI_PIN_START  2
#define DI_PIN_END    12
#define DI_PIN_COUNT  (DI_PIN_END-DI_PIN_START+1)
#define DI_DEBOUNCE_MS 50 // 100ms (1/10th second)

#define AI_PIN_START  0
#define AI_PIN_END    5 
#define AI_PIN_COUNT  (AI_PIN_END-AI_PIN_START+1)
#define AI_DEBOUNCE_MS 10 // 100ms (1/10th second)

#define LED_PIN 13
#define LEDBLINK_MS 1000

#define ANALOG_CALIBRATION_MS 5000 // 5000ms (5 seconds)

#define SHOW_OFF false // Show "off"?
//#define SHOW_OFF true // Show "off"?

/*---------------------------------------------------------------------------*/
/*
 * Some sanity checks to make sure the #defines are reasonable.
 */
#if (DI_PIN_END >= LED_PIN)
#error PIN CONFLICT: PIN END goes past LED pin.
#endif

#if (DI_PIN_START < 2)
#error PIN CONFLICT: PIN START covers 0-TX and 1-RX pins.
#endif

#if (DI_PIN_START > DI_PIN_END)
#error PIN CONFLICT: PIN START and END should be a range.
#endif
/*---------------------------------------------------------------------------*/

/* For I/O pin status and debounce. */
unsigned int  digitalStatus[DI_PIN_COUNT];      // Last set PIN mode.
unsigned long digitalDebounceTime[DI_PIN_COUNT];    // Debounce time.
unsigned int  digitalDebounceRate = DI_DEBOUNCE_MS;   // Debounce rate.
unsigned long digitalCounter[DI_PIN_COUNT];

unsigned int  analogStatus[AI_PIN_COUNT];
unsigned int  analogMin[AI_PIN_COUNT];         // Calibration min level.
unsigned int  analogMax[AI_PIN_COUNT];         // Calibration max level.
unsigned long analogDebounce[AI_PIN_COUNT];    // Debounce time.
unsigned int  analogDebounceRate = AI_DEBOUNCE_MS;// Debounce rate.
unsigned long analogCounter[DI_PIN_COUNT];

/* For the blinking LED (heartbeat). */
unsigned int  ledStatus = LOW;             // Last set LED mode.
unsigned long ledBlinkTime = 0;            // LED blink time.
unsigned int  ledBlinkRate = LEDBLINK_MS;  // LED blink rate.

unsigned int pinsOn = 0;

unsigned char showOff = SHOW_OFF;          // Show "off" mode?

/*---------------------------------------------------------------------------*/

void setup()
{
  // Just in case it was left on...
  wdt_disable();

  // Initialize the serial port.
  Serial.begin(9600);

  // Docs say this isn't necessary for Uno.
  while(!Serial) {
    ;
  }

  showHeader();
  
  // Initialize watchdog timer for 2 seconds.
//  wdt_enable(WDTO_4S);
  
  // Initialize the pins and digitalPin array.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Set pin to be digital input using pullup resistor.
    pinMode(thisPin+DI_PIN_START, INPUT_PULLUP);
    // Set the current initial pin status.
    digitalStatus[thisPin] = HIGH; //digitalRead(thisPin+DI_PIN_START);
    // Clear debounce time.
    digitalDebounceTime[thisPin] = 0;

    digitalCounter[thisPin] = 0;
  }

  // Set pin to be digital input using pullup resistor.
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  // Initialize the analog pins and calibrate.
  for (int thisPin=0; thisPin < AI_PIN_COUNT; thisPin++)
  {
    // Set the initial pin status.
    analogStatus[thisPin] = HIGH; // Active low, so default to off to match digital.
    // Clear debounce time.
    analogDebounce[thisPin] = 0;

    analogCounter[thisPin] = 0;
    
    // These are now set inside the calibration function.
    //analogMin[thisPin] = 1023;
    //analogMax[thisPin] = 0;
  }

  Serial.print("Calibrating analog inputs for ");
  Serial.print(ANALOG_CALIBRATION_MS/1000);
  Serial.print(" seconds...");
  calibrateAnalogInputs();
  Serial.println("Ready.");
  showAnalogCalibrationValues();
  
  // Set pin 13 to output, since it has an LED we can use.
  pinMode(LED_PIN, OUTPUT);
}

/*---------------------------------------------------------------------------*/

void loop()
{
  // Tell the watchdog timer we are still alive.
  wdt_reset();

  // Check for serial data.
  if (Serial.available() > 0) {
    // If data ready, read a byte.
    int incomingByte = Serial.read();
    // Parse the byte we read.
    switch(incomingByte)
    {
      case '?':
        showStatus();
        break;
      case '!':
        Serial.print("Recalibrating analog inputs for ");
        Serial.print(ANALOG_CALIBRATION_MS/1000);
        Serial.println(" seconds...");
        calibrateAnalogInputs();
        Serial.println("Ready.");
        break;
      case '@':
        toggleShowOff();
        break;
      default:
        break;
    }
  }

  // LED blinking heartbeat. Yes, we are alive.
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    if (ledStatus==LOW)  // If LED is LOW...
    {
      ledStatus = HIGH;  // ...make it HIGH.
    } else {
      ledStatus = LOW;   // ...else, make it LOW.
    }
    // Set LED pin status.
    if (pinsOn==0) digitalWrite(LED_PIN, ledStatus);
    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+ledBlinkRate;
  }
    
  /*-------------------------------------------------------------------------*/

  // Loop through each Analog Input pin.
  for (int thisPin=0; thisPin < AI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    unsigned int status = analogRead(thisPin+AI_PIN_START);
    //status = map(status, analogMin[thisPin], analogMax[thisPin], 0, 255);

    // Determine pin status by checking analog value to calibration value.
    if (status>analogMax[thisPin])
    {
      status = LOW;
    }
    else if (status<=analogMax[thisPin])
    {
      status = HIGH;
    }
    else
    {
      // Limbo. No change.
    }

    // In pin status has changed from our last toggle...
    if (status != analogStatus[thisPin])
    {
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (analogDebounce[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        analogDebounce[thisPin] = millis()+analogDebounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (analogDebounce[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        if ( (long)(millis()-analogDebounce[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status==LOW)
            {
              // Emit UPPERCASE "On" character.
              Serial.println(char(65+DI_PIN_COUNT+thisPin));
              analogCounter[thisPin]++;
              pinsOn++;
              digitalWrite(LED_PIN, HIGH);
            } else {
             
              // Emit lowercase "Off" character.
              if (showOff==true) Serial.println(char(97+DI_PIN_COUNT+thisPin));
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            analogStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            analogDebounce[thisPin] = 0;
        } // End of if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        
      } // End of if (digitalDebounceTime[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      analogDebounce[thisPin] = 0;
    }
  } // End of (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )

  /*-------------------------------------------------------------------------*/

  // Loop through each Digital Input pin.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    int status = digitalRead(thisPin+DI_PIN_START);

    // In pin status has changed from our last toggle...
    if (status != digitalStatus[thisPin])
    {
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (digitalDebounceTime[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        digitalDebounceTime[thisPin] = millis()+digitalDebounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (digitalDebounceTime[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status==LOW)
            {
              // Emit UPPERCASE "On" character.
              Serial.println(char(65+thisPin));
              digitalCounter[thisPin]++;
              pinsOn++;
              digitalWrite(LED_PIN, HIGH);
            } else {
              // Emit lowercase "Off" character.
              if (showOff==true) Serial.println(char(97+thisPin));
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            digitalStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            digitalDebounceTime[thisPin] = 0;
        } // End of if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        
      } // End of if (digitalDebounceTime[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      digitalDebounceTime[thisPin] = 0;
    }
  } // End of (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
}

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

void showHeader()
{
  // Emit some startup stuff to the serial port.
  Serial.println("");
  Serial.print("ArduinoAIDI ");
  Serial.print(VERSION);
  Serial.println(" by Allen C. Huffman (alsplace@pobox.com)");
  Serial.print(DI_PIN_COUNT);
  Serial.print(" DI Pins (");
  Serial.print(DI_PIN_START);
  Serial.print("-");
  Serial.print(DI_PIN_END);
  Serial.print("), ");
  Serial.print(digitalDebounceRate);
  Serial.print("ms Debounce. ");

  Serial.print(AI_PIN_COUNT);
  Serial.print(" AI Pins (");
  Serial.print(AI_PIN_START);
  Serial.print("-");
  Serial.print(AI_PIN_END);
  Serial.print("), ");
  Serial.print(analogDebounceRate);
  Serial.println("ms Debounce. ");
  //Serial.println("(Nathaniel is a jerk.)");
}

/*---------------------------------------------------------------------------*/

void showStatus()
{
  showDigitalInputStatus();
 
  showAnalogInputStatus();
 
  showAnalogCalibrationValues(); 
}

/*---------------------------------------------------------------------------*/

void showDigitalInputStatus()
{
  Serial.print("Digital Inputs: ");
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    Serial.print("D");
    Serial.print(thisPin+DI_PIN_START);
    Serial.print(":");
    Serial.print(digitalRead(thisPin+DI_PIN_START));
    Serial.print(" ");
    //Serial.print(" (");
    //Serial.print(digitalCounter[thisPin]);
    //Serial.print(") ");
  }
  Serial.println("");
}

/*---------------------------------------------------------------------------*/
/* ANALOG INPUT STUFF */

void calibrateAnalogInputs()
{
  unsigned long calibrateUntil = millis()+ANALOG_CALIBRATION_MS;

  // First, reset calibration values.
  for (int thisPin=0; thisPin < AI_PIN_COUNT; thisPin++ )
  {
    analogMin[thisPin] = 1023;
    analogMax[thisPin] = 0;
  }

  // Calibrate the analog pins.
  while (millis() < calibrateUntil)
  {
    unsigned int analogValue = 0;
    for (int thisPin=0; thisPin < AI_PIN_COUNT; thisPin++)
    {
      analogValue = analogRead(thisPin);
   
     // record the maximum sensor value
      if (analogValue > analogMax[thisPin]) {
        analogMax[thisPin] = analogValue;
      }
   
     // record the minimum sensor value
      if (analogValue < analogMin[thisPin]) {
        analogMin[thisPin] = analogValue;
      }
    } 
  }
}

/*---------------------------------------------------------------------------*/

void showAnalogCalibrationValues()
{
  Serial.print("Calibrated AIs: ");
  for (int thisPin=0; thisPin<AI_PIN_COUNT; thisPin++)
  {
    Serial.print("A");
    Serial.print(thisPin+AI_PIN_START);
    Serial.print("(");
    Serial.print(analogMin[thisPin]);
    Serial.print("-");
    Serial.print(analogMax[thisPin]);
    Serial.print(") ");
  }
  Serial.println("");
}

/*---------------------------------------------------------------------------*/

void showAnalogInputStatus()
{
  Serial.print("Analog Inputs : ");
  for (int thisPin=0; thisPin < AI_PIN_COUNT; thisPin++ )
  {
    Serial.print("A");
    Serial.print(thisPin+AI_PIN_START);
    Serial.print(":");
    Serial.print(analogRead(thisPin+AI_PIN_START));
    Serial.print(" ");
    //Serial.print(" (");
    //Serial.print(analogCounter[thisPin]);
    //Serial.print(") ");
  }
  Serial.println("");
}

void toggleShowOff()
{
  if (showOff==true)
  {
    showOff=false;
    Serial.println("OFF notification disabled.");
  } else {
    showOff=true;
    Serial.println("OFF notification enabled.");
  }
}

/*---------------------------------------------------------------------------*/
// End of file.

