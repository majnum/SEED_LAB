
#include <Wire.h>

//Communication Vars
#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int len = 0;
int in_data[32] = {};
int desired_angle = -1;

//Define Some Useful Constants 
#define r 0.05 
#define b 0.1

#define rot 3200 
#define CLK 2 
#define DT 3  
#define CLK2 4

#define DT2 5 
#define pi 3.14159
#define TriStatePin 4
#define Motor1DirControlPin 7
#define Motor1VoltControlPin 9  //PWM_OUTPUT PIN
#define FaultPin 12
#define PWM_BOUND 255 



volatile long int ang_right = 0; 
volatile long int ang_left = 0;

volatile int time_l = 0;
volatile int time_r = 0; 

volatile double theta_r;
volatile double theta_l;


//****************************************************************************************


//Control Variables
volatile long int oldPosition  = 0;

volatile double Kp =  3; //Proportional Controllor
volatile double given = 0; //Desired Angle
volatile int directionsign = 0;
volatile double Ki = 0.1; //Integral Controller
volatile double Kd = 0; //Deriv. Controller -- Set to Zero to Disable. System is physically overdamped.  
volatile double integral = 0; //Used in controller 
volatile double deriv = 0; //Not Used with Kd = 0
volatile double analog; //Used to write to PWM wave controlling 
volatile double currentPosition = 0;
volatile double delta = 0; //Error Signal

float currentTime = 0; 
float samplingTime = 0;
float storedTime = 0; 
float rad = 0; //Current Angle
boolean newCount = false;
String currentDir = "";
int counter = 0;
int currentStateCLK;
int lastStateCLK;

//****************************************************************************************



void setup() {

  //Enable serial communication
  pinMode(13, OUTPUT);
  Serial.begin(115200); //start serial for output
  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callabcks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  //Serial.println("Ready!");

// initialize digital pin 

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(CLK2, INPUT_PULLUP);
  pinMode(DT2, INPUT_PULLUP);

  //Encoder position
  lastStateCLK = digitalRead(CLK);
  

 // Create two ISRs
  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), updateEncoder, CHANGE);


  //Motor Setup
  pinMode(TriStatePin, OUTPUT); 
  analogWrite(TriStatePin, 255); 

  pinMode(Motor1VoltControlPin, OUTPUT);
  pinMode(Motor1DirControlPin, OUTPUT);
}

//****************************************************************************************

void receiveData(int byteCount){
  int i = 0;
  if(byteCount > 1){
    while(Wire.available()){
      in_data[i] = Wire.read();
      //Serial.print(in_data[i]);
      //Serial.print(' ');
      i++;
    }
    //Serial.print('\n');
    i--;
    len = i;
  }
  else{
    read_offset = Wire.read();
  }
  
  given = in_data[1] * (PI/2);

  Serial.println(given); 

}


void sendData(){
  static byte data[32] = {};
  for(int i = 0; i < 32; i++){
    data[i] = 0;
  }
  Serial.println(counter);
  String start_val = String(counter);
  for(int i = 0; i < start_val.length(); i++){
    data[i] = (int)start_val[i];
  }
  //delay(200);
  Wire.write(data, 32);
}

//***************************************************************

// the loop function runs over and over again forever
void loop() {

        
      //Run the controller – turns wheel to specified position 
      PIDController(); 

}


//***********************************************************************************************************

//Controller implementation 
void PIDController() { 
  
 //Takes current time 
  currentTime = millis(); 
  samplingTime = currentTime - storedTime;


  //Derivative Controller
  deriv = (rad - currentPosition) * 100; 

   
 //Gets current position 
  currentPosition = rad;
  delta = given - currentPosition; 
  
  
  integral = integral + delta * (0.01); 
  analog = ((Kp * delta + Ki * integral + deriv * Kd)*34); 
  
//  if (abs(analog) > PWM_BOUND) 
//  { 
//    analog = constrain(analog, -1, 1) * PWM_BOUND; 
//    delta = constrain (delta, -1, 1) * min(abs(delta), PWM_BOUND / Kp); 
//    integral = (analog - Kp * delta) / Ki; 
//  } 
  
  

 

  int controlSignal = abs(analog); 

  if (controlSignal > 255){
    controlSignal = 255;
    //integral = (controlSignal - Kp * delta) / Ki; 
      }
  
  if  (abs(delta) < 0.005) 
  { 
    controlSignal = 0; 
    analogWrite(Motor1VoltControlPin, controlSignal); 
  } 
  else if (delta > 0) 
  { 
    digitalWrite (Motor1DirControlPin, HIGH); 
    analogWrite(Motor1VoltControlPin, controlSignal); 
  } else if (delta < 0) 
  { 
    digitalWrite (Motor1DirControlPin, LOW); 
    analogWrite(Motor1VoltControlPin, controlSignal); 
  } 
 
  //Serial.println(integral); 
  //Serial.print("\t"); 
  Serial.println(analog); 
  integral = integral; 
  storedTime = currentTime; 

  while (millis() < currentTime + 10); //timer for consistency of next control input calculation - 10ms
} 






//***************************************************************************************************************************
//            Rotate code


void updateEncoder(){
  
  currentStateCLK = digitalRead(CLK);

  
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter --;
      currentDir ="CW";
    }

    rad = (counter*2*PI)/800;
     if(rad>=6.2831){
      rad=rad-6.2831;
      counter = 0;
    }else if(rad<=-6.2831){
      rad = rad+6.2831;
      counter = 0;

    
    }
    
//    Serial.print("Direction: ");
//    Serial.print(currentDir);
//    Serial.print(" | Counter: ");
//    Serial.println(counter);
    
    newCount = true;
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}
