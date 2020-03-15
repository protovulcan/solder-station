#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

double Setpoint = 100, Input, Output;
PID IronPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  Serial.begin(115200);
  setupRotaryEncoder();
  setupLCD();
  setupPID();
}


static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

void setupRotaryEncoder()
{
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
}

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


void setupLCD()
{
  lcd.init();
  lcd.backlight();
}

#define PID_DRIVE_PIN 13

int WindowSize = 5000;
unsigned long windowStartTime;

void setupPID()
{
  windowStartTime = millis();
  IronPID.SetOutputLimits(0, WindowSize);
  IronPID.SetMode(AUTOMATIC);
  pinMode(PID_DRIVER_PIN, OUTPUT);
}



/*
  Two measurements:
    * 51 = 37 deg C
    * 115 = 204 deg C
  
  y = ax+b
  a=173/64
  b=311
*/

#define MEASUREMENTS 400
int acnt = 0;
int mSum = 0;
double old;
double measureTemp()
{
  if (acnt < MEASUREMENTS)
  {
    mSum += analogRead(A7);
    acnt++;
    return old;
  }
  else
  {
    double nin = (173.0 * mSum / (64.0 * MEASUREMENTS)) - 107.0;
    acnt = 0;
    mSum = 0;
    return old = nin;
  }
}

char buffer=[' ',' ','0','.','0',b11011111,'C',0];

char *sprintTemp(double temp)
{
  if (temp>=100.0) {
    buffer[0]='0'+((int)(temp/100))%10;
  } else {
    buffer[0]=' ';
  }
  if (temp>=10.0) {
    buffer[1]='0'+((int)(temp/10))%10;
  } else {
    buffer[1]=' ';
  }
  if (temp>=10.0) {
    buffer[2]='0'+((int)temp)%10;
  } else {
    buffer[2]=' ';
  }
  buffer[4]='0'+((int)(temp*10))%10;
  return buffer;
}

void redrawLcd()
{
  lcd.setCursor(0, 0);
  lcd.print(sprintTemp(Input));
  
  lcd.setCursor(8, 0);
  lcd.print(sprintTemp(Setpoint));
}

void loop()
{
  double temp = measureTemp();
  if (temp != Input)
  {
    Input = temp;
    redrawLcd();
  }
  IronPID.Compute();

  if (encoderPos!=0) {
    Setpoint += encoderPos;
    encoderPos=0;
  }

  unsigned long now = millis();
  if (now - windowStartTime > W13indowSize)
  {
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime)
  {
    digitalWrite(PID_DRIVER_PIN, HIGH);
  }
  else
  {
    digitalWrite(PID_DRIVER_PIN, LOW);
  }
}
