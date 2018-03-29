#include <Wire.h>
//#include <Servo.h>              // Used for plunging the 
#include <Adafruit_ADS1015.h>   // Used as dual channel differential ADCs
#include <DRV8825.h>            // Stepper motor driver
//#define MotorRelay 5



// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 3600 // we set this high for smoothing
// Target RPM for cruise speed
#define RPM 1
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 1000
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 16


#define DIRB 13
#define DIRD 12
#define DIRC 11
#define DIRA 10
#define DIR  10
#define STEP  9
#define SLEEP 8
#define RESET 7
#define MODE2 6
#define MODE1 5
#define MODE0 4
#define ENABLE 3

#define DIRADIR LOW
#define DIRBDIR LOW
#define DIRCDIR HIGH
#define DIRDDIR LOW

//DRV8825 drivetrain(MOTOR_STEPS, DIR, STEP, ENABLE, MODE0, MODE1, MODE2);
DRV8825 drivetrain(MOTOR_STEPS, DIR, STEP);

//Servo myservo;

#define NUMTHERM  1
#define VCC       4.92
#define ADCMUL    0.186
// ADCs communicate over i2c (SCL and SDA)
Adafruit_ADS1115 ads1115_1;
#if (NUMTHERM > 2)
Adafruit_ADS1115 ads1115_2(0x49);
#endif
#if (NUMTHERM > 4)
Adafruit_ADS1115 ads1115_3(0x4A);
#endif
#if (NUMTHERM > 6)
Adafruit_ADS1115 ads1115_4(0x4B);
#endif

int ResistanceVals[] = {9990, 10010, 9980, 10050, 9990, 10010, 10000, 10020};
float c1Arr[] = {  0.00184847419,  1.835135071E-03};
float c2Arr[] = {  1.172333013E-04, 1.198489835E-04};
float c3Arr[] = {  5.161206729E-07, 5.003643336E-07};

float room[] = {0, 0, 0, 0, 0, 0, 0, 0};
float multiplier = 0.186;

int speakerpin = A7;

//int pos = 0;

//const int motorPin1  = 11;  // Pin 14 of L293
//const int motorPin2  = 12;  // Pin 10 of L293
//const int motorPin3  = 13; // Pin  7 of L293


void setup(void)
{
  Serial.begin(115200);
  //pinMode(MotorRelay, OUTPUT);
  //myservo.attach(2);
  /*
    Serial.println(F("Hello Safety Team!"));
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
  */
  pinMode(SLEEP, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(DIRC, OUTPUT);
  pinMode(DIRD, OUTPUT);
  
  digitalWrite(SLEEP,HIGH);
  digitalWrite(RESET,HIGH);
  digitalWrite(DIRA,LOW);
  digitalWrite(DIRB,LOW);
  digitalWrite(DIRC,HIGH); // HI
  digitalWrite(DIRD,LOW);
  
  ads1115_1.begin();
  //ads1115_1.setSPS(ADS1115_DR_860SPS);
#if (NUMTHERM > 2)
  ads1115_2.begin();
  //ads1115_2.setSPS(ADS1115_DR_860SPS);
#endif
#if (NUMTHERM > 4)
  ads1115_3.begin();
  //ads1115_3.setSPS(ADS1115_DR_860SPS);
#endif
#if (NUMTHERM > 6)
  ads1115_4.begin();
  //ads1115_4.setSPS(ADS1115_DR_860SPS);
#endif
  /*
  //start filling
  Serial.println("Filling Started");
  digitalWrite(MotorRelay, LOW);
  Serial.println("Plunge Started");
  plunge();
  Serial.println("Plunge Ended");
  //begin acceleration
  Serial.println("Begin Acceleration");
  for (int x = 100; x < 255; x++) {
  Run(1, x, 40);
  }

  Serial.println("End Acceleration");
  //stop filling
  */
  while(millis()<3000){
    TakeTemp();
    Serial.println();
  }
  
  Serial.println("Filling Ended");
  //digitalWrite(MotorRelay, HIGH);
  
  TakeTemp();
  
  //drivetrain.setSpeedProfile(drivetrain.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  //drivetrain.begin(RPM, MICROSTEPS);
  //drivetrain.enable();
  //drivetrain.startRotate(360);
}

void loop(void) {
  unsigned wait_time = drivetrain.nextAction();
  float resistance[NUMTHERM];
  GetResistance(resistance,NUMTHERM,ResistanceVals,ADCMUL);
  float temps[NUMTHERM];
  GetTemps(temps, NUMTHERM, resistance);
  if (ShouldRun(temps, 1000)) {
    //Run(1, 255, 5);
  } else {
    /*
    for (int x = 100; x < 110; x = x + 2) {
      Run(1, -x, 40);
      Serial.println(x);
    }
    Run(1, 0, 10000);
    */
    Serial.println( "Motor disabled");
    drivetrain.disable();
    while (true) {
      ;
    }
  }
}

void GetTemps(float *TempVals, int n, float *Res) {

  for (int y = 0; y < n; y++) {
    TempVals[y] = Res[y];
    TempVals[y] = log(TempVals[y]);
    TempVals[y] = (1.0 / (c1Arr[y] + c2Arr[y] * TempVals[y] + c3Arr[y] * TempVals[y] * TempVals[y] * TempVals[y]));
    TempVals[y] = TempVals[y] - 273.15;
    TempVals[y] = (TempVals[y] * 9.0) / 5.0 + 32.0;
  }
}

void GetResistance(float *RT, int n, int R1[], int adcmul) {

  long VT[NUMTHERM];
#ifdef CHANNELFLIP
  VT[0] = ads1115_1.readADC_Differential_0_1()   * -1;
#if (NUMTHERM > 1)
  VT[1] = ads1115_1.readADC_Differential_2_3()   * -1;
#endif
#if (NUMTHERM > 2)
  VT[2] = ads1115_2.readADC_Differential_2_3() * -1;
#endif
#if (NUMTHERM > 3)
  VT[3] = ads1115_2.readADC_Differential_0_1() * -1;
#endif
#if (NUMTHERM > 4)
  VT[4] = ads1115_3.readADC_Differential_2_3() * -1;
#endif
#if (NUMTHERM > 5)
  VT[5] = ads1115_3.readADC_Differential_0_1() * -1;
#endif
#if (NUMTHERM > 6)
  VT[6] = ads1115_4.readADC_Differential_0_1() * -1;
#endif
#if (NUMTHERM > 7)
  VT[7] = ads1115_4.readADC_Differential_2_3() * -1;
#endif

#else   // no CHANNELFLIP

  VT[0] = ads1115_1.readADC_Differential_0_1();
#if (NUMTHERM > 1)
  VT[1] = ads1115_1.readADC_Differential_2_3();
#endif
#if (NUMTHERM > 2)
  VT[2] = ads1115_2.readADC_Differential_2_3();
#endif
#if (NUMTHERM > 3)
  VT[3] = ads1115_2.readADC_Differential_0_1();
#endif
#if (NUMTHERM > 4)
  VT[4] = ads1115_3.readADC_Differential_2_3();
#endif
#if (NUMTHERM > 5)
  VT[5] = ads1115_3.readADC_Differential_0_1();
#endif
#if (NUMTHERM > 6)
  VT[6] = ads1115_4.readADC_Differential_0_1();
#endif
#if (NUMTHERM > 7)
  VT[7] = ads1115_4.readADC_Differential_2_3();
#endif

#endif // CHANNELFLIP

  // Voltage Divider
  // Vo = Vcc * R2 / (R1 + R2)
  // R2 = - (Vo * R1) / (Vo - Vcc)

  for (int x = 0; x < n; x++) {
    float VTf = ((float)VT[x] * ADCMUL) / 1000; // 0.186 multiplier to mV, then divide by 1000mV/V
    RT[x] = ( - (VTf * R1[x]) / ( VTf - VCC )); // Where VT is the Voltage across the Thermistor
    //RT[x] = (R1[x] * ( (VCC/2) / ( ( (float)VT[x] * ADCMUL ) / 1000  ))); // to resistance
    //RT[x] = ((float)VT[x] * ADCMUL) / 1000 ;  // return voltage rather than resistance
  }
}

void printArr(long arr[], int n, long t) {
  Serial.print(t); Serial.print(" ");
  for (int x = 0; x < n; x++) {
    Serial.print(arr[x]); Serial.print(" ");
  }
  Serial.println(" ");
}

void printArr(float arr[], int n, long t) {
  Serial.print(t); Serial.print(" ");
  for (int x = 0; x < n; x++) {
    Serial.print(arr[x], 4); Serial.print(" ");
  }
  Serial.println(" ");
}

/*
void Run(int motor, int velocity, int t) {
  if (motor == 1) {
    if (velocity > 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(motorPin3, velocity);
      delay(long(t));
    } else if (velocity < 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(motorPin3, -1 * velocity);
      delay(long(t));
    }
    else {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, HIGH);
      analogWrite(motorPin3, velocity);
      delay(long(t * 1000));
    }
  } else {
    if (velocity > 0) {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, velocity);
      analogWrite(motorPin3, velocity);
      delay(long(t * 1000));
    } else if (velocity < 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(motorPin3, velocity);
      delay(long(t * 1000));
    } else  {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, HIGH);
      analogWrite(motorPin3, velocity);
      while (true) {
        delay(long(1 * 1000));
      }
    }
  }
}
*/

boolean ShouldRun(float arr[], int n) { //, float limit) {
  int count = 0;
  if (millis() < 3000) {
    Serial.println("Time : " + String(millis()) + " : Waiting");
    printArr(arr, n, millis());
    Serial.print(" : Waiting");
    Serial.println();
    return true;
  } else if (room[0] == 0) {
    for (int i = 0; i < n; i++) {
      room[i] = arr[i];
    }
  } else {
    for (int x = 0; x < n; x++) {
      if (arr[x] > 5.5 + room[x]) {
        count++;
        if (count > 2) {
          return false;
        }
      }
    }
    float off[NUMTHERM];
    for (int c = 0; c < n; c++) {
      off[c] = arr[c] - room[c];
    }
    printArr(off, n, millis());
    Serial.println();
  }
  return true;
}

/* // Servo functions plunge(), Forward(), Backward()
  void plunge() {
  Serial.println("Start Plunge");
  long t = millis();
  Serial.println(t);
  for (int x = 70; x < 120; x = x + 10) {
    Forward(75, x, x + 10);
    Backward(75, x + 10, x + 5);
    Forward(75, x + 5, x + 10);
  }
  Backward(50, 125, 65);
  Serial.println(millis() - t);
  }

  void Forward(int s, int st, int en) {
  for (pos = st; pos <= en; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    TakeTemp();
    Serial.println();
    //myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(s - 22);                     // waits 15ms for the servo to reach the position
  }
  }

  void Backward(int s, int st, int en) {
  for (pos = st; pos >= en; pos -= 1) { // goes from 180 degrees to 0 degrees
    TakeTemp();
    Serial.println();
    //myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(s - 22);                     // waits 15ms for the servo to reach the position
  }
  }
*/
/*
void setColor(int red, int green, int blue, int ledNum)
{
  if (ledNum == 1) {
#ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
#endif
    analogWrite(redPin1, red);
    analogWrite(greenPin1, green);
    analogWrite(bluePin1, blue);
  }
  else {
#ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
#endif
    analogWrite(redPin2, red);
    analogWrite(greenPin2, green);
    analogWrite(bluePin2, blue);
  }
}
*/
void TakeTemp() {
  long t = millis();
  Serial.println(t);
  float resistance[NUMTHERM];
  GetResistance(resistance, NUMTHERM, ResistanceVals, multiplier);
  float temps[NUMTHERM];
  GetTemps(temps, NUMTHERM, resistance);
  printArr(temps, NUMTHERM, millis());
  // Serial.println(millis()-t);
}
