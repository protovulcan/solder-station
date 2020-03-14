#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

double Setpoint=100, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();

  Serial.begin(115200);
  windowStartTime = millis();
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  pinMode(13,OUTPUT);
}

void redrawLcd()
{
  int temp = Input;
  lcd.setCursor(0,0);
  lcd.print(temp);
}
/*
int refresh=0;

// 51=37
// 115=204
37 - a*51 = b
204-311 = 311 +  b
a=173/64
b=311

x = 173*y/64-107

*/

#define MEASUREMENTS 100
int acnt = 0;
int mSum = 0;
double measureTemp() {
  if (acnt<MEASUREMENTS) {
    mSum+=analogRead(A7);
    acnt++;
    return Input;
  } else {
    double nin = (173.0*mSum/(64*MEASUREMENTS))-107;
    acnt=0;
    mSum=0;
    return nin;
  }
}

char * printTemp(double temp)
{
  
}

void loop()
{
   double temp = measureTemp();
   if (temp!=Input) {
      Input = temp;
      redrawLcd();
   }
   myPID.Compute();
    
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) 
  {
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime) 
  {
    digitalWrite(13, HIGH);
  }
  else 
  {
    digitalWrite(13, LOW);
  }
}

/*
void setup()
{
  Serial.begin(9600);
  Serial.println("****MAX6675 thermocouple library by E. Kremer****");
  Serial.println();
}

void loop()
{
  float celsius = tcouple.readTempC();
  float fahrenheit = tcouple.readTempF();
  Serial.print("T in С = ");
  Serial.print(celsius);
  Serial.print(". T in Fahrenheit = ");
  Serial.println(fahrenheit);
  delay(500);
}

*/
