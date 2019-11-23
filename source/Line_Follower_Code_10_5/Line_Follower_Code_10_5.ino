/******************************************************************************/
//          UCSD ECE 5 Lab 4 Code: Line Following Robot with PID
/******************************************************************************/

//  1)  Declare Variables     - declares many variables as global variables so
//                              each variable can be accessed from every fn
//  2)  Setup (Main)          - runs once at beginning when you press button on
//                              arduino/motor drive/when you open srl monitor
//  3)  Loop  (Main)          - loops forever calling on a series of functions
//  4)  Calibration           - makes white = 0 and black = 100 (a few seconds
//                              to prep, a few seconds on white, a few seconds
//                              to move to black, a few seconds of black)
//  5)  Read Potentiometers   - reads each potentiometer
//  6)  Run Motors            - runs motors
//  7)  Read Photoresistors   - reads each photoresistor
//  8)  Calculate Error       - calculate error from photoresistor readings
//  9)  PID Turn              - takes the error and implements PID control
//  10) Print                 - used for debugging but should commented out when
//                              not debugging because it slows down program

// ****************************************************************************/
// 1) Declare Variables


// Variables and Libaries for Motor (This part is from Motor_Driver_Code 10_3)
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//  Motors can be switched here (1) <--> (2) (if you see motors responding to
//  error in the opposite way they should be)
//  Adjust the same way if you did in 10_3
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);

// this is the nominal speed for the motors when not using potentiometer
int M1Sp = 20, M2Sp = 20;
int M1SpeedtoMotor, M2SpeedtoMotor;

// Variables for Potentiometer (This part is from Potentiometer_Code 10_1)
const int S_pin = A0; //proportional control
const int P_pin = A1; //proportional control
const int I_pin = A2; //integral control
const int D_pin = A3; //derivative control
int       SpRead = 0; int Sp; //Speed Increase
int       kPRead = 0; //proportional gain
int       kIRead = 0; //integral gain
int       kDRead = 0; //derivative gain

// Variables for Light Sensors (This part is from Calibration_Code 10_4)
int LDR_Pin[7] = {A8, A9, A10, A11, A12, A13, A14}; // store photoresistor pins
int LDR[7]; // store photoresistor readings

// Calibration Variables (This part is from Calibration_Code 10_4)
int   led_Pin = 13; // This is a led set up to indicate what part of the
                    // calibration you are on.
float Mn[7];  // You could also connect a larger/more visible LED to Pin 13 or
              // other digital pin
float Mx[7];
float LDRf[7] = {0., 0., 0., 0., 0., 0., 0.};

// Error Calculation Variables (This part is from Calibration_Code 10_4)
int   MxRead;
int   MxIndex;
float AveRead;
int   CriteriaForMax;
float WeightedAve;
int   ii;
int   im0, im1, im2;

// For Motor/Control
int   Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;



// ****************************************************************************/
// 2) Setup
// refer to the lab manual
void setup()
{
  Serial.begin(9600);       // For serial communication set up
  AFMS.begin();             // For motor setup
  pinMode(led_Pin, OUTPUT); // Note that all analog pins used are INPUTs by
                            // default so don't need pinMode
  Calibrate();  // 4
  ReadPotentiometers(); // 5
  RunMotors(); // 6
}

// ****************************************************************************/
// 3) Loop
void loop()
{
  ReadPotentiometers(); // 5
  ReadPhotoResistors(); // 7
  CalcError(); // 8
  PID_Turn(); // 9
  RunMotors();
}

// ****************************************************************************/
// 4) Calibration
void Calibrate()
{
  // wait to make sure in position
  for (int calii = 0; calii < 4; calii++)
  {
    digitalWrite(led_Pin, HIGH);   // turn the LED on
    delay(100);                    // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);    // turn the LED off
    delay(900);                    // wait for 0.9 seconds
  }

  // Calibration
  // White Calibration
  int numMeas = 10;  // number of samples to read for calibration
  for(int calii = 0; calii < numMeas; calii++)
  {
    digitalWrite(led_Pin, HIGH);   // turn the LED on
    delay(100);                    // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);    // turn the LED off
    delay(100);                    // wait for 0.1 seconds

    for (int ci = 0; ci < 7; ci++)
    {
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]);
      delay(2);
    }
  }

  for(int cm = 0; cm < 7; cm++)
  {
    Mn[cm] = round(LDRf[cm] / (float)numMeas); // take average
    LDRf[cm] = 0.;
  }

  // Time to move from White to Black Surface
  for(int calii = 0; calii < 10; calii++)
  {
    digitalWrite(led_Pin, HIGH);
    delay(100);
    digitalWrite(led_Pin, LOW);
    delay(900);
  }

  // Black Calibration
  for (int calii = 0; calii < numMeas; calii++)
  {
    digitalWrite(led_Pin, HIGH);
    delay(100);
    digitalWrite(led_Pin, LOW);
    delay(100);

    for(int ci = 0; ci < 7; ci++)
    {
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]);
      delay(2);
    }
  }
  for(int cm = 0; cm < 7; cm++)
  {
    Mx[cm] = round(LDRf[cm] / (float)numMeas); // take average
    LDRf[cm] = 0.;
  }

}

// ****************************************************************************/
// 5) Read & Map Potentiometers
void ReadPotentiometers()
{
  SpRead = map(analogRead(S_pin), 0, 1023, 0, 50);
  Sp = SpRead;
  kPRead = map(analogRead(P_pin), 0, 1023, 0, 10);
  kIRead = map(analogRead(I_pin), 0, 1023, 0, 5);
  kDRead = map(analogRead(D_pin), 0, 1023, 0, 10);
}

// ****************************************************************************/
// 6) Run Motors
// motors start by using nominal speed + speed addition from potentiometer
void RunMotors()
{
  // Remember you can insert your custom control in here
  // to control motor behaviour from error, etc...
  M1SpeedtoMotor = min(M1Sp + Sp + M1P, 255); // limits speed to 255
  M2SpeedtoMotor = min(M2Sp + Sp + M2P, 255); // remember M1Sp & M2Sp is defined
                                              // @ beginning of code(default 60)
  Motor1->setSpeed(abs(M1SpeedtoMotor));
  Motor2->setSpeed(abs(M2SpeedtoMotor));

  if (M1SpeedtoMotor > 0)
  {
    Motor1->run(FORWARD);
  }
  else
  {
    Motor1->run(BACKWARD);
  }

  if (M2SpeedtoMotor > 0)
  {
    Motor2->run(FORWARD);
  }
  else
  {
    Motor2->run(BACKWARD);
  }

}

// ****************************************************************************/
// 7) Read Photoresistors
// function to read photo resistors, map from 0 to 100, and find darkest
// photoresitor (MxIndex)
void ReadPhotoResistors()
{
  for (int Li = 0; Li < 7; Li++)
  {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    delay(2);
  }
}

// ****************************************************************************/
// 8) Calculate Error
// Calculate error from photoresistor readings
void CalcError()
{
  MxRead = -99;
  AveRead = 0.0;
  // Step 1)
  //     Iterate through photoresistor mapped values, find darkest/max (MxRead),
  //     it's index (im1) and weighted index (MxIndex)
  //     Weighted Index: from left to right: 3 - 2 - 1 - 0 (center) - 1 - 2 - 3
  for (int ii = 0; ii < 7; ii++)
  {
    if (MxRead < LDR[ii])
    {
      MxRead = LDR[ii];
      MxIndex = -1 * (ii - 3);
      im1 = (float)ii;
    }
    AveRead = AveRead + (float)LDR[ii] / 7.;
  }

  // Step 2)  Calculate error from weighted average, based off the readings
  // around the maximum value
  CriteriaForMax = 2; // max should be at least twice as big as the other values
  if (MxRead > CriteriaForMax * AveRead)  // only when the max is big enough
  {
    if (im1 != 0 && im1 != 6)  // max not on either ends
    {
      im0 = im1 - 1;  // index for left
      im2 = im1 + 1;  // index for right
      if (LDR[im0] + LDR[im1] + LDR[im2] == 0)  // if the denominator calculates
                                                // to 0, jump out and do not
                                                // update error
      {
        return;
      }
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1] * im1 + LDR[im2] * im2))
                    / ((float)(LDR[im0] + LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 0)  // max on left end
    {
      im2 = im1 + 1;
      if (LDR[im1] + LDR[im2] == 0)  // if the denominator calculates to 0, jump
                                     // out and do not update error
        return;
      WeightedAve = ((float)(LDR[im1] * im1 + LDR[im2] * im2)) /
                      ((float)(LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 6)  // max on right end
    {
      im0 = im1 - 1;
      if (LDR[im0] + LDR[im1] == 0)  // if the denominator calculates to 0, jump
                                     // out and do not update error
        return;
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1] * im1)) /
                      ((float)(LDR[im0] + LDR[im1]));
      error = -1 * (WeightedAve - 3);
    }
  }
}


// ****************************************************************************/
// 9) PID Turn
// function to make a turn ( a basic P controller)
void PID_Turn()
{
  // *Read values are between 0 and 100, scale to become PID Constants
  kP = (float)kPRead / 1.;    // each of these scaling factors can change
                              // depending on how influential you want them 2 be
  kI = (float)kIRead / 1000.;  // the potentiometers will also scale them
  kD = (float)kDRead / 100.;
  // error holds values from -3 to 3

  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!

  sumerror = sumerror + error;
  if (sumerror > 5) {
    sumerror = 5; // prevents integrator wind-up
  }
  else if (sumerror < -5) {
    sumerror = -5;
  }

  lasterror = error;

  // Set the motor speed
  M1P = Turn;
  M2P = -Turn;
}


// ****************************************************************************/
// 10) Print
// function to print values of interest
void Print()
{
  Serial.print(SpRead); Serial.print(" ");  // Initial Speed addition from
                                            // potentiometer
  Serial.print(kP); Serial.print(" ");      // PID values from potentiometers
                                            // after scaling
  Serial.print(kI); Serial.print(" ");
  Serial.print(kD); Serial.print("    ");

  Serial.print(LDR[0]); Serial.print(" ");  // Each photo resistor value shown
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print("    ");

  Serial.print(MxRead); Serial.print(" ");    // the maximum value from the
                                              // photoresistors is shown again
  Serial.print(MxIndex); Serial.print("    "); // this is the index of that
                                              // maximum (0 through 6)
                                              // (aka which element in LDR)
  Serial.print(error); Serial.print("    ");  // this will show the calculated
                                              // error (-3 through 3)

  Serial.print(M1SpeedtoMotor); Serial.print(" "); // This prints the arduino
                                                   // output to each motor so
                                                  // you can see what the values
                                                  // (0-255)
  Serial.println(M2SpeedtoMotor);                  //  that are sent to the
                                                  // motors would be without
                                                  // actually needing to
                                                  // power/run the motors
  // slow down the output for easier reading if wanted
  // ensure delay is commented when actually running your robot or this will
  // slow down sampling too much
  // delay(100);

}
