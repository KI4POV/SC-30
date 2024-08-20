#include <si5351.h> // https://github.com/NT7S/Si5351
#include <SimpleTimer.h> //https://www.arduino.cc/reference/en/libraries/simpletimer/
#include <Morse.h> //https://github.com/etherkit/MorseArduino

 

Morse morse(LED_BUILTIN, 10);


const uint32_t bandStart30 = 10100000;       // start of 30m
const uint32_t bandEnd30   = 10150000;       // end of 30m
const uint32_t bandInit    = 10105000;       // where to initially set the frequency
volatile long oldfreq = 0;
volatile long currentfreq = 0;


// This is the amount of offset between freq (TX freqency) and RXfreq (RX frequency)
// It will change if you implement RIT. If you do not implement RIT, set it to whatever sidetone you like
// to listen to.
volatile uint32_t foffset = 650;   

volatile uint32_t freq = bandInit;     // this is a variable (changes) - set it to the beginning of the band
volatile uint32_t RXfreq = freq + foffset;
volatile uint32_t radix = 50;         // how much to change the frequency by
int morseoutfreq = freq / 1000;         // this is also a variable; used for morse frequency readout routine

// Rotary encoder pins
static const int rotBPin = 2;
static const int rotAPin = 3;


//Setup button for front panel
static const int pushPin = 4;   //Triggers Morse Frequency Readout
int oldPbState = 0;

//Setup Key
static const int key = 7;
int keyState;
int oldkeyState = 0;

static const int mutepin = 6;

//Debounce Time Variables
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;
unsigned long previousMillis = 0;
const long BreakInDelay = 1;

// Rotary encoder variables, used by interrupt routines
volatile int rotState = 0;
volatile int rotAval = 1;
volatile int rotBval = 1;

// Instantiate the Objects
Si5351 si5351;
SimpleTimer timer;

void morserefresh()
{
  morse.update();
}

void setup()
{
  
  // Set up frequency, radix, RIT, and key
  pinMode(rotAPin, INPUT_PULLUP);
  pinMode(rotBPin, INPUT_PULLUP);
  pinMode(pushPin, INPUT_PULLUP);
  pinMode(key, INPUT_PULLUP);
  pinMode(mutepin, OUTPUT);


  // Set up interrupt pins
  attachInterrupt(digitalPinToInterrupt(rotAPin), ISRrotAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotBPin), ISRrotBChange, CHANGE);


  //Set Timer to refresh morse output
  timer.setInterval(1, morserefresh);

  // Initialize the DDS
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(31830, SI5351_PLL_INPUT_XO);      // Set to specific Si5351 calibration number
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_freq((RXfreq * 100ULL), SI5351_CLK0);

}


void loop()
{

  timer.run();
 
  int reading = digitalRead(key);
   // If the switch changed, due to noise or pressing:
  if (reading != oldkeyState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != keyState) {
      keyState = reading;

      oldkeyState = reading;
    }
  }

  if (keyState == LOW && oldkeyState == 0) { // If Key Down
    digitalWrite(mutepin, LOW);              // Mute Receiver Output
    si5351.output_enable(SI5351_CLK0, 0);    // Turn off LO                                                     
    si5351.output_enable(SI5351_CLK2, 1);    // Turn on TX oscillator
    si5351.set_freq((freq * 100ULL), SI5351_CLK2); 
    tone(9,abs(foffset));     // added Sidetone
    oldkeyState = 1;
  }
  if ( digitalRead(LED_BUILTIN) == HIGH ) {
    tone(9,abs(foffset));
  }
   if (keyState == HIGH && oldkeyState == 1) {
    si5351.output_enable(SI5351_CLK2, 0); 
    noTone (9);                      // added disable Sidetone              
    si5351.output_enable(SI5351_CLK0, 1);             // and enable RX Clock
    unsigned long currentMillis = millis();
  //  if (currentMillis - previousMillis >= BreakInDelay) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    digitalWrite(mutepin, HIGH);
   // }
    oldkeyState = 0;
  }
  if (keyState == HIGH && morse.busy == true && digitalRead(LED_BUILTIN) == LOW) {
    noTone (9);
  }
  if (keyState == HIGH && morse.busy == false) {
    noTone (9);
  }

  currentfreq = getfreq();                  // Interrupt safe method to get the current frequency

  if (currentfreq != oldfreq)
  {
    SendFrequency();
    oldfreq = currentfreq;
  }
  
  if (digitalRead(pushPin) == LOW)
  {
    morsefreq();
  }

}

void morsefreq()
{
  morseoutfreq = ((freq / 1000) - 10100);
  String outputstr;
  outputstr = String(morseoutfreq);
  const char* opchar = outputstr.c_str();
  morse.send (opchar);
}


long getfreq()
{
  long temp_freq;
  cli();
  temp_freq = freq;
  sei();
  return temp_freq;
}



// Interrupt routines
void ISRrotAChange()
{
  if (digitalRead(rotAPin))
  {
    rotAval = 1;
    UpdateRot();
  }
  else
  {
    rotAval = 0;
    UpdateRot();
  }
}


void ISRrotBChange()
{
  if (digitalRead(rotBPin))
  {
    rotBval = 1;
    UpdateRot();
  }
  else
  {
    rotBval = 0;
    UpdateRot();
  }
}

void UpdateRot()
{
  switch (rotState)
  {

    case 0:                                         // Idle state, look for direction
      if (!rotBval)
        rotState = 1;                               // CW 1
      if (!rotAval)
        rotState = 11;                              // CCW 1
      break;

    case 1:                                         // CW, wait for A low while B is low
      if (!rotBval)
      {
        if (!rotAval)
        {
            freq = freq + radix;
            if (freq > bandEnd30)
              freq = bandEnd30;
          rotState = 2;                             // CW 2
        }
      }
      else if (rotAval)
        rotState = 0;                               // It was just a glitch on B, go back to start
      break;

    case 2:                                         // CW, wait for B high
      if (rotBval)
        rotState = 3;                               // CW 3
      break;

    case 3:                                         // CW, wait for A high
      if (rotAval)
        rotState = 0;                               // back to idle (detent) state
      break;

    case 11:                                        // CCW, wait for B low while A is low
     if (!rotAval)
      {
        if (!rotBval)
        {
            freq = freq - radix;
            if (freq < bandStart30)
              freq = bandStart30;
            
          rotState = 12;                            // CCW 2
        }
      }
      else if (rotBval)
        rotState = 0;                               // It was just a glitch on A, go back to start
      break;

    case 12:                                        // CCW, wait for A high
      if (rotAval)
        rotState = 13;                              // CCW 3
      break;

    case 13:                                        // CCW, wait for B high
      if (rotBval)
        rotState = 0;                               // back to idle (detent) state
      break;
  }
}
        

void SendFrequency()
{
  RXfreq = freq + foffset;
  si5351.set_freq((RXfreq * 100ULL), SI5351_CLK0);
} 
