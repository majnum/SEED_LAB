
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
#define rot 300
#define CLK 2
#define DT 4
#define CLK2 3
#define DT2 5 
#define pi 3.14159
#define TriStatePin 4
#define Motor1DirControlPin 7
#define Motor1VoltControlPin 9
#define FaultPin 12



volatile int ang_right = 0; 
volatile int ang_left = 0;

volatile int time_l = 0;
volatile int time_r = 0; 

volatile float theta_r;
volatile float theta_l;


//Control Variables
long int oldPosition  = 0;
double Kp = .50;
int given = 75;
int directionsign = 0;
double Ki = 0.019;
double Kd = 0.0334;
unsigned long currentTime = 0;
double integral = 0;
double analog;
int closeEnough = 10;
long int newPosition;
int delta =0;

void setup() {
  //Enable serial communication
    pinMode(13, OUTPUT);
  Serial.begin(115200); //start serial for output
  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callabcks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
// initialize digital pin 

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(CLK2, INPUT_PULLUP);
  pinMode(DT2, INPUT_PULLUP);

 // Create an ISR attached to pin 2 on a rising edge. 
  attachInterrupt(digitalPinToInterrupt(CLK), rotate_r, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK2), rotate_l, RISING);


  //MotorSetup
  pinMode(TriStatePin, OUTPUT); 
  analogWrite(TriStatePin, 255); 

  pinMode(Motor1VoltControlPin, OUTPUT);
  pinMode(Motor1DirControlPin, OUTPUT);

  
  
}

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
  
  given = in_data[1] * 75;
  Serial.println(given);
  
}

void sendData(){
  static byte data[32] = {};
  for(int i = 0; i < 32; i++){
    data[i] = 0;
  }
  Serial.println(ang_right);
  String start_val = String(ang_right);
  for(int i = 0; i < start_val.length(); i++){
    data[i] = (int)start_val[i];
  }
  //delay(200);
  Wire.write(data, 32);
  
}

/*
void sendData(){
  byte data[32] = {};
  //if(read_offset == in_data[0]){
    String val = String(ang_right);
    //Serial.print(val);
    for(int i = 0; i < val.length(); i++){
      data[i] = val[i];
      if(val[i] != ' '){
        //Serial.print(val[i] + " ");
      }
  //}
  //Serial.print("\n");
  Wire.write(data, 32);
}
}
*/

// the loop function runs over and over again forever
void loop() {
      
      //Print the movement When their is new encoder data.
      
      theta_r = (float) ang_right*rot / (2*pi); 
      theta_l = (float) ang_left*rot / (2*pi); 
      //Serial.print(theta_r);
      //Serial.print(" \n");


//Control Code *****************************************************************************
    currentTime = millis();
    long int newPosition = ang_right;
    if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    }

     //integral calculation
//     if (abs(newPosition - given) < intThreshholdCounts) {
      integral += (double)(newPosition-oldPosition) * 0.05;
//    }
//    else if (abs(newPosition - given) > intThreshholdCounts) {
//    integral = 0; //zero out the integral when we're close enough to desired position
//    }
   if (abs(newPosition - given) < closeEnough) {
    integral = 0;
    }
    
 

       
    if (newPosition - given < 0 || newPosition - given > 0) {
    
    newPosition = ang_right;
    in_data[0] = newPosition / 20;
    delta = given - newPosition;
    analog = delta*(Kp + Kd * (double)(newPosition - oldPosition)*20 + integral * Ki); //movement speed of motor
    if (analog < 0) {
      directionsign = 255;
    }else{
      directionsign = 0;
    }
    analog = abs(analog);
    if (analog > 255) {
      analog = 255;
    } //limiter for out of bounds for output of controller
    if (analog < 0) {
      analog = 0;
    }
    
    analogWrite(Motor1VoltControlPin, analog);
    analogWrite(Motor1DirControlPin,directionsign);
    
//    
//    if (abs(newPosition - given) < intThreshholdCounts) {
//      integral += (double)abs(newPosition - given) * 0.05;
//    }
    //Serial.println((double)newPosition * (PI / 1600.0));
    }

    while (millis() < currentTime + 50); //timer for consistency of next control input calculation - 50ms
}

//When a rising edge on pin 2 is detected check the direction of the Encoder based on previous inputs. 
void  rotate_r(){

  //Read the data from the encoder
  byte r_update = digitalRead(DT);
  float oldangright = ang_right;

  //Check which way its going
  if(r_update != digitalRead(CLK)){

    
    ang_right += 1; 

    if(ang_right > rot-1){
      ang_right = 0;
    }
    
    
  } else{
   
    ang_right -= 1; 

    if(ang_right < 0){
      ang_right = rot - 1;
    }
    
  }

  

}
      
void  rotate_l(){

  //Read the data from the encoder
  byte l_update = digitalRead(DT2);
  float ang_old = ang_left;
  //Check which way its going
  if(l_update != digitalRead(CLK2)){
    
    ang_left += 1; 

    if(ang_left > rot-1){
      ang_left = 0;
    }
    
  } else{

    ang_left -= 1; 

    if(ang_left < 0){
      ang_left = rot-1;
    }
    
  }

   
 
}
