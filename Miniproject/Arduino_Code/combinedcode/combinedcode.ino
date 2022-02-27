
#include <Wire.h>


#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int len = 0;
int in_data[32] = {};
int desired_angle = -1;

// A program that reads the direction of two Encoders and calculates movement. 
// By Joey Thurman 2/23/2022

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

volatile double Kp =  10.06;

volatile double given = PI/2;

volatile int directionsign = 0;

volatile double Ki = 2.3;

volatile double Kd = .1;

//volatile unsigned long currentTime = 0;

volatile double integral = 0;

volatile double deriv = 0;

volatile double analog;

//volatile int closeEnough = 10;

volatile double currentPosition = 0;

volatile double delta = 0;

//****************************************************************************************


//***Used in new control code***
float currentTime = 0; 
float samplingTime = 0;
float storedTime = 0; 
//float motorPosition = 0;
//float motorTheta = 0;
float rad = 0;
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
  

 // Create an ISR attached to pin 2 on a rising edge. 
  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), updateEncoder, CHANGE);


  //MotorSetup
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
  
  given = in_data[1] * (pi / 2);

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

  //  motorPosition = theta_r;
  //  motorTheta = theta_r; //2 * PI * (double)motorPosition / (double)rot;
      
      //Print the movement When their is new encoder data.


      //rotate_r();
      //rotate_l();
      
//      theta_r = (double) ang_right*(2*pi) / (double) rot; 
//      theta_l = (double) ang_left*(2*pi) / (double) rot; 
//      Serial.print(theta_r);
//      //Serial.print(" \n");

       
      //Run the controller – turns wheel to specified position 
      PIDController(); 

}


//***********************************************************************************************************
// Note from ELi - I tried to redo the Controller code after talking with Darren Mcsweeny,
// he gave me a run down of what the code should look like, the old code is commented out below

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
  
  

  if(analog < 0){
    digitalWrite (Motor1DirControlPin, true);
  } else {
    digitalWrite (Motor1DirControlPin, false);

  }

  int controlSignal = abs(analog); 

  if (controlSignal > 255){
    controlSignal = 255;
    //integral = (controlSignal - Kp * delta) / Ki; 
    analogWrite(Motor1VoltControlPin, controlSignal);
  }
  
  if  (abs(delta) < 0.008) 
  { 
    controlSignal = 0; 
    analogWrite(Motor1VoltControlPin, controlSignal); 
  } 
  else if (delta > 0) 
  { 
    digitalWrite (Motor1DirControlPin, HIGH); 
    //analogWrite(Motor1VoltControlPin, controlSignal); 
  } else if (delta < 0) 
  { 
    digitalWrite (Motor1DirControlPin, LOW); 
    //analogWrite(Motor1VoltControlPin, controlSignal); 
  } 
 
  Serial.println(integral); 
  //Serial.print("\t"); 
  //Serial.println(currentPosition); 
  integral = integral; 
  storedTime = currentTime; 

  while (millis() < currentTime + 10); //timer for consistency of next control input calculation - 10ms
} 



//Control Code *****************************************************************************
//    currentTime = millis();
//    static double newPosition = 0;
//    static double deriv = 0; 
//
//    newPosition = theta_r;
//    delta = given - newPosition;
//
//    Serial.print("\t");
//    Serial.println(delta);
//
//     //integral calculation
////     if (abs(newPosition - given) < ) {
//      integral = integral + (double)(delta) * 0.01;
//      
////    }
////    if (abs(newPosition - given) > intThreshholdCounts) {
// //   integral = 0; //zero out the integral when we're close enough to desired position
////    
//   
//    
//   //deriv = Kd*((double)(newPosition - oldPosition)*100);
//
//       
//   
//    //in_data[0] = newPosition / 20;
//    
//    analog = (delta*Kp + integral* Ki); //movement speed of motor
//    if (analog < 0) {
//      digitalWrite(Motor1DirControlPin,HIGH);
//    }else{
//      digitalWrite(Motor1DirControlPin,LOW);
//    }
//    analog = abs(analog);
//    if (analog > 255) {
//      analog = 255;
//    } //limiter for out of bounds for output of controller
//    if (analog < 0) {
//      analog = 0;
//    }
//
//    if(abs(delta) < .01 ){
//      analog = 0;
//      integral = 0; 
//      analogWrite(Motor1VoltControlPin, analog);
//    }else if (delta > 0){
//      digitalWrite(Motor1DirControlPin,HIGH);
//      analogWrite(Motor1VoltControlPin, analog);
//
//    } else if (delta < 0){
//      digitalWrite(Motor1DirControlPin,LOW);
//      analogWrite(Motor1VoltControlPin, analog);
//
//    }
//    
//    
//    analogWrite(Motor1VoltControlPin, analog);
//   
//    
////    
////    if (abs(newPosition - given) < intThreshholdCounts) {
////      integral += (double)abs(newPosition - given) * 0.05;
////    }
//    //Serial.println((double)newPosition * (PI / 1600.0));
//    
//    while (millis() < currentTime + 10); //timer for consistency of next control input calculation - 10ms
//}


//***************************************************************************************************************************
//            Rotate code

////When a rising edge on pin 2 is detected check the direction of the Encoder based on previous inputs. 
//void  rotate_r(){
//
//  //Read the data from the encoder
//  byte r_update = digitalRead(DT);
//  float oldangright = ang_right;
//
//  //Check which way its going
//  if(r_update != digitalRead(CLK)){
//
//    
//    ang_right += 1; 
//
//    if(ang_right > rot){
//      ang_right = 0;
//    }
//    
//    
//  } else{
//   
//    ang_right -= 1; 
//
//    if(ang_right < 0){
//      ang_right = rot -1;
//    }
//    
//  }
//
//  
//
//}
//      
//void  rotate_l(){
//
//  //Read the data from the encoder
//  byte l_update = digitalRead(DT2);
//  float ang_old = ang_left;
//  //Check which way its going
//  if(l_update != digitalRead(CLK2)){
//    
//    ang_left += 1; 
//
//    if(ang_left > rot){
//      ang_left = 0;
//    }
//    
//  } else{
//
//    ang_left -= 1; 
//
//    if(ang_left < 0){
//      ang_left = rot;
//    }
//    
//  }

//*******************************************************************
//               Get current position   
 


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
