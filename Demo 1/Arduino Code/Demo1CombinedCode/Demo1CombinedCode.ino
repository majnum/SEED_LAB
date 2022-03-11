//******************************************************************************************
//Combined Arduino Code for Demo 1
//******************************************************************************************

#include <Encoder.h>
#include <wire.h>

//******************************************************************************************
//          GLOBAL VARIABLES
//******************************************************************************************

//Define Motor Pins
#define D2Left =        4  //Used for digital write to left motor
#define D2Right =       5  //Used for digital write to right motor
#define motDirLeft =    7
#define motDirRight =   8   
#define motorVolLeft =  9  //PWM for left motor
#define motorVolRight = 10 //PWM for right motor

//Define Encoder Pins
#define RightPinA = 2   //Uses Interrupt pin
#define RightPinB = 13
#define LeftPinA = 3    //Uses Interuppt pin
#define LeftPinB = 6


//time variables
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;
float samplingTime = 0;

//angular velocity variables
double thetaDotRight
double thetaDotLeft = 0;
double velocityRight = 0;
double velocityLeft = 0;
double velocityError = 0;

//angle position variables
double newDegreeLeft = 0;
double oldDegreLeft = 0;
double newdegreeRight = 0;
double oldDegreeRight = 0;

//wheel constants
float r = 0.23958333; // radius of wheel as a fraction of one foot
float b = 1.156; // distance between wheels as a fraction of one foot


//encoder viables
int counter = 0;
int currentState;
int lastStateRight;
int lastStateLeft;

Encoder rightEnc(RightPinA, RightPinB);
Encoder leftEnc(LeftPinA, LeftPinB);

// Controller parameters

double Kp = 0;
double Ki = 0;



//******************************************************************************************

//******************************************************************************************
void setup() {
  // put your setup code here, to run once:
  serial.begin(115200);
  lastStateRight = digitalRead(RightPinA);
  lastStateLeft = digitalRead(LeftPinA);
  

  //assign Pins I/O Logic
  pinMode(RightPinA, INPUT_PULLUP);
  pinMode(LeftPinA, INPUT_PULLUP);
  pinMode(RightPinB, INPUT_PULLUP);
  pinMode(LeftPinB, INPUT_PULLUP);
  pinMode(EnableMotor, OUTPUT);
  pinMode(D2Left, OUTPUT);
  pinMode(D2Right, OUTPUT);
  pinMode(motVolLeft, OUTPUT);
  pinMode(motVoldRight, OUTPUT);
  pinMode(motDirRight, OUTPUT);
  pinMode(motDirLeft, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //run the motors
  digitalWrite(D2Left, HIGH);
  digitalWrite(D2Right, HIGH);
  
  
  static double analogLeft = 0;
  static double analogRight = 0;

  //Controller
  currentTime = millis();
  elapsedTimed = currentTime - previousTime

  newDegreeLeft = (double)(leftEnc.read() );
  newDegreeRight = (double)(rightEnc.read() );

  


  //calculate angular velocity of wheels
  velocityRight = (1000 * (newDgreeRight - oldDegreeRight))/SamplingTime;
  velocityLeft = (1000 * (newDgreeLeft - oldDegreeLeft))/SamplingTime;

  velocityError = vlocityRight + velocityLeft;
 

}
