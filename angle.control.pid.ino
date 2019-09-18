#include <PID_v1.h>

// H-Bridge Pins
#define enM 11
#define in3 10
#define in4 9

// Rotary Encoder Pins
#define pinA 3 // Connected to CLK
#define pinB 4 // Connected to DT

// Rotary Encoder Variables
int encoderPosCount = 0;
int LastAVal;
int aVal;
boolean bCW;

// PID Variables
double SetPoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1; // PID Parameters
// PID Instance
PID Angle(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);



/****************************************/

void setup() {
  // Serial Monitor
  Serial.begin (9600);

  // Rotary Encoder Initialization
  pinMode(pinA,INPUT);
  pinMode(pinB,INPUT);
  // Read Pin A
  LastAVal = digitalRead(pinA);

  // Motor Initialization
  pinMode(enM, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // PID Initialization
  SetPoint = 36 / 18;
  Angle.SetMode(AUTOMATIC);
  Angle.SetOutputLimits(100, 255);
}

void loop() {
  // Reading Input From Encoder
  aVal = digitalRead(pinA);
  if (aVal != LastAVal)
  {
    if (digitalRead(pinB) != aVal)
    { // Means pin A Changed first Clockwise.
      encoderPosCount ++;
    }
    else // Otherwise B changed first and we're moving CCW
    {
      encoderPosCount--;
    }
  }
  LastAVal = aVal;
  
  // Modifying Input
  if (encoderPosCount <= 0)
  {
    Input = encoderPosCount * -1;
  }
  else
  {
    Input = encoderPosCount;
  }
  
  // Calculate Direction
  if (encoderPosCount > SetPoint)
  {
    digitalWrite(in3, );
    digitalWrite(in4, );
  }
  else
  {
    digitalWrite(in3, );
    digitalWrite(in4, );
  }

  // PID Calculation
  Angle.Compute();
  // Wirte Output to Motor
  analogWrite(enM, Output);
}
