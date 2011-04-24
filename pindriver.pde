#include <TimerOne.h>
#include <lpd6803.h>

#define MotorForward true
#define MotorBackward false

// Choose which 2 pins you will use for output.
// Can be any valid output pins.
const int ledDataPin = 10;       // 'yellow' wire
const int ledClockPin = 11;      // 'green' wire
const int mikePin = 0;

// clock pin must be PWM
const int clockPin = 9;      // PIN 11     SH_CP     Shift register clock pin
const int latchPin = 12;     // PIN 12     ST_CP     Storage register clock pin (latch pin)
const int dataPin = 6;      // PIN 14     DS     Serial data input
//int outputEnable = 7;  // PIN 13     OE     Output enable, active low

const int kMotorSteps = 200;     // change this depending on the number of steps per revolution of your motor
const int kMotorSpeed = 150;                    
const int kDistance = 900;

const int motorDirection[4] = {-1, -1, -1, 1};
const int motorPins[4][4] =
  {
    {4,5,6,7},
    {0,1,2,3},
    {4,5,6,7},
    {0,1,2,3}
  };
const int motorAnimation[4][6] =
  {
    {0,500,0, 500, 250, 0},
    {500, 250, 500, 200, 500, 0},
    {125, 0, 250, 0, 500, 0},
    {0, 125, 0, 250, 10, 0}
  };

int motorPosition[4] = {0,0,0,0};
long motorLastStepMilli[4];
unsigned long motorStepDelay[4];
int motorLastStep[4] = {0,0,0,0};

uint16_t colors[6] = {Color(63,0,0), Color(63,30,0), Color(63, 30, 30), Color(30, 63, 30), Color(0, 63, 30), Color(0,0,63)};
// Set the first variable to the NUMBER of pixels. 20 = 20 pixels in a row
LPD6803 strip = LPD6803(36, ledDataPin, ledClockPin);
 
static void setSpeed(int motorIndex, long whatSpeed, int motorStepsPerRev)
{
  Serial.print("set speed ");Serial.print(whatSpeed);
  motorStepDelay[motorIndex] = 60L * 1000L / motorStepsPerRev / whatSpeed;
  Serial.print(") to ");Serial.println(motorStepDelay[motorIndex]);
}
 
static void moveTowardsDestination(int motorIndex, int destination)
{
  //Serial.print("::move [");Serial.print(motorIndex);Serial.print("] towards ");Serial.print(destination);Serial.print(" from ");Serial.println(motorPosition[motorIndex]);
 
  if (motorPosition[motorIndex] > destination)
  {       
    //Serial.println(":::move backwards 1");
    backward(motorIndex, 1);
  }
  else if (motorPosition[motorIndex] < destination)
  {
    //Serial.println(":::move forwards 1");
    forward(motorIndex, 1);
  } 
  else
  {
    ; //do nothing
  }
}
 
  static void forward(int motorIndex, int steps)
  {
    //Serial.print("::forward ");Serial.print(steps);Serial.print(", dir(");Serial.print(motorDirection[motorIndex]);
    //Serial.print(") increasing position from ");Serial.print(motorPosition[motorIndex]);Serial.print(" to ");Serial.println(motorPosition[motorIndex]+steps);
    motorPosition[motorIndex]++;
    step(motorIndex, steps * motorDirection[motorIndex]);
    //Serial.print("::forward now at position ");Serial.println(motorPosition[motorIndex]);
  }
 
  static void backward(int motorIndex, int steps)
  {
    //Serial.print("<-");Serial.print(steps);Serial.print(", dir ");Serial.print(motorDirection[motorIndex]);
    //Serial.print(") decreasing position from ");Serial.print(motorPosition[motorIndex]);Serial.print(" to ");Serial.println(motorPosition[motorIndex]-steps);
    motorPosition[motorIndex]--;
    step(motorIndex, -steps * motorDirection[motorIndex]);
    //Serial.print("::backward now at position ");Serial.println(motorPosition[motorIndex]);
  }
 
 
  static void step(int motorIndex, int steps)
  { 
    //Serial.print("::step ");Serial.println(steps);
    // decrement the number of steps, moving one step each time:
    while(true)
    {
      //Serial.print("steps left ");Serial.println(steps - step_number);
   
      // move only if the appropriate delay has passed:     
      if ((millis() - motorLastStepMilli[motorIndex]) >= motorStepDelay[motorIndex])
      {       
        //Serial.print("time to make step ");Serial.println(step_number + 1);
        // get the timeStamp of when you stepped:
        motorLastStepMilli[motorIndex] = millis();
   
        // increment or decrement the step number, depending on direction:
        if (steps > 0) // I'm going forward
        {
          //motorLastStep[motorIndex]++;
          stepMotor(motorIndex, ++motorLastStep[motorIndex]);
          if (motorLastStep[motorIndex] >= steps) {
            break;
          }
        }
        else
        {
          //motorLastStep[motorIndex]--;
          stepMotor(motorIndex, --motorLastStep[motorIndex]);
          if (motorLastStep[motorIndex] <= steps) {
            break;
          }
        }
      }
    }
  }
   /*
   * Moves the motor forward or backwards.
   */
   // todo: make work with arrays of motors
  static void stepMotor(int motorIndex, int stepNumber)
  {
    static byte firstBits;
    static byte secondBits;
    int stepIndex = stepNumber % 4;
    //Serial.print("stepmotor "); Serial.print(stepNumber);Serial.print(" to ");Serial.println(stepIndex);
    // todo: make motor effect the second set of motors
   
    byte bitsToSend = 0;
    digitalWrite(latchPin, LOW); // turn off the motors
 
    switch (stepIndex) {
      case 0:    // 1010
        bitWrite(bitsToSend, motorPins[motorIndex][0], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][1], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][2], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][3], LOW);
      break;
      case 1:
      case -3:    // 0110
        bitWrite(bitsToSend, motorPins[motorIndex][0], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][1], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][2], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][3], LOW);
        break;
      case 2:
      case -2:    //0101
        bitWrite(bitsToSend, motorPins[motorIndex][0], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][1], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][2], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][3], HIGH);
      break;
      case 3:
      case -1:    //1001
        bitWrite(bitsToSend, motorPins[motorIndex][0], HIGH);
        bitWrite(bitsToSend, motorPins[motorIndex][1], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][2], LOW);
        bitWrite(bitsToSend, motorPins[motorIndex][3], HIGH);
      break;
    }

    if (motorIndex == 0 || motorIndex == 1)
    {
      firstBits = bitsToSend;
    }
    else
    {
      secondBits = bitsToSend;
    }
    shiftOut(dataPin, clockPin, MSBFIRST, firstBits);
    shiftOut(dataPin, clockPin, MSBFIRST, secondBits);
     
    // turn on the output so the LEDs can light up:
    digitalWrite(latchPin, HIGH);
  }
 
void setup() {
  strip.setCPUmax(50);  // start with 50% CPU usage. up this if the strand flickers or is slow
 
  for (int i = 0;i<4;i++)
  {
    motorLastStepMilli[i] = millis();
  }
 
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT); 
  pinMode(clockPin, OUTPUT);
  pinMode(mikePin, INPUT);
  pinMode(3, OUTPUT);
 
  Serial.begin(9600);
  Serial.println("reset");
 
  colors[0] = Color(63, 0, 0);
  colors[1] = Color(0, 63, 0);
  colors[2] = Color(0, 0, 63);
  colors[3] = Color(63, 31, 0);
  colors[4] = Color(63, 0, 31);
  colors[5] = Color(31, 63, 0);
 
 
  strip.begin();    // Start up the LED counter
  strip.show();    // Update the strip, to start they are all 'off'

  setSpeed(0, kMotorSpeed, kMotorSteps);
  setSpeed(1, kMotorSpeed, kMotorSteps);
  setSpeed(2, kMotorSpeed, kMotorSteps);
  setSpeed(3, kMotorSpeed, kMotorSteps);
}

void animate()
{
  Serial.println("about to animate");

  int a;
  long mils = millis();
  byte pinCounter = 0;
  byte colorCounter = 0;
  for(a = 0;a < 1;a++)
  {
    int i;
    for (i = 0;i<6;i++)
    {
      int j;
      for (j = 0;j<10000;j++)
      {
        if (millis() > mils + 50)
        {
          if (pinCounter > 35)
            pinCounter = 0 ;
          strip.setPixelColor(pinCounter++, Wheel(colorCounter++));
          strip.show();
          mils = millis();
        }
        moveTowardsDestination(0, motorAnimation[0][i]);
        moveTowardsDestination(1, motorAnimation[1][i]);       
        moveTowardsDestination(2, motorAnimation[2][i]);
        moveTowardsDestination(3, motorAnimation[3][i]);       
      }
      //delay(200);
    } 
    //delay(500);
  }
  Serial.println("done animating");

}

void loop() {
/*
  for (int i = 0; i<100;i++)
    moveTowardsDestination(0, 100);
  delay(2000);
  for (int i = 0; i<100;i++)
    moveTowardsDestination(1, 100);
  delay(2000);
  for (int i = 0; i<100;i++)
    moveTowardsDestination(2, 100);
  delay(2000);
  for (int i = 0; i<100;i++)
    moveTowardsDestination(3, 100);
  delay(2000);

  for (int i = 0; i<5000;i++)
  {
    moveTowardsDestination(0, 0);
    moveTowardsDestination(1, 0);       
    moveTowardsDestination(2, 0);
    moveTowardsDestination(3, 0);      
  }
  delay(2000);
  */
 
  static int peak;
  static int average;
  static int duty;
 
  int value = analogRead(mikePin);
  Serial.print("volume");Serial.println(value);
  if (value > peak)
  {
     peak = value;
  }
 
  average += (value - average) / 100;
 
  if (value > (average * 1.3))
  {
    Serial.print("peaking ");Serial.print(value);Serial.print(" average ");Serial.println(average);
    duty = value-average;   
  }
 
  if (duty > 0) 
  {
    duty--;
    analogWrite(3, duty/2);
  } else
  {
    analogWrite(3,0);
  }
 
  
  colorWipe(Color(63, 0, 0), 50);
  //animate();
  rainbowCycle(50);
  colorWipe(Color(0, 63, 0), 50);
  //animate();
  colorWipe(Color(0, 0, 63), 50);
  //animate();

  rainbow(50);
  //animate();

  //animate();

}

void registerWrite(int whichPin, int whichState) {
// the bits you want to send
  byte bitsToSend = 0;
  // turn off the output so the pins don't light up
  // while you're shifting bits:
  digitalWrite(latchPin, LOW);

  // turn on the next highest bit in bitsToSend:
  bitWrite(bitsToSend, whichPin, whichState);

  // shift the bits out:
  shiftOut(dataPin, clockPin, MSBFIRST, bitsToSend);

    // turn on the output so the LEDs can light up:
  digitalWrite(latchPin, HIGH);

}


void rainbow(uint8_t wait) {
  int i, j;
  
  for (j=0; j < 96 * 3; j++) {     // 3 cycles of all 96 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i + j) % 96));
    } 
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed
// along the chain
void rainbowCycle(uint8_t wait) {
  int i, j;
 
  for (j=0; j < 96 * 5; j++) {     // 5 cycles of all 96 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 96 / strip.numPixels()) + j) % 96) );
    } 
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint16_t c, uint8_t wait) {
  int i;
 
  for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}
/* Helper functions */

// Create a 15 bit color value from R,G,B
unsigned int Color(byte r, byte g, byte b)
{
  //Take the lowest 5 bits of each value and append them end to end
  return( ((unsigned int)g & 0x1F )<<10 | ((unsigned int)b & 0x1F)<<5 | (unsigned int)r & 0x1F);
}

//Input a value 0 to 127 to get a color value.
//The colours are a transition r - g -b - back to r
unsigned int Wheel(byte WheelPos)
{
  byte r,g,b;
  switch(WheelPos >> 5)
  {
    case 0:
      r=31- WheelPos % 32;   //Red down
      g=WheelPos % 32;      // Green up
      b=0;                  //blue off
      break;
    case 1:
      g=31- WheelPos % 32;  //green down
      b=WheelPos % 32;      //blue up
      r=0;                  //red off
      break;
    case 2:
      b=31- WheelPos % 32;  //blue down
      r=WheelPos % 32;      //red up
      g=0;                  //green off
      break;
  }
  return(Color(r,g,b));
}
