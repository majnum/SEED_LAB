
#include <Wire.h>

//Communication Vars
#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int len = 0;
int in_data[32] = {};
int desired_angle = -1;

//Define Some Useful Constants 
#define r 5.5 
#define b 0.1
#define rot 800 

#define CLK_R 2 
#define DT_R 6  
#define CLK_L 3
#define DT_L 8 

#define pi 3.14159
#define TriStatePin 4
#define Motor1DirControlPin 7
#define Motor1VoltControlPin 9  //PWM_OUTPUT PIN
#define FaultPin 12
#define PWM_BOUND 255 



volatile int time_l = 0;
volatile int time_r = 0; 

volatile double theta_dot_R;
volatile double theta_dot_L;

double x_loc = 0;
double y_loc = 0; 

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
float rad_R = 0; //Current Angle
float rad_L = 0; //Current Angle


//Encoder Vars
boolean newCount_R = false;
String currentDir_R = "";
int counter_R = 0;
int currentStateCLK_R;
int lastStateCLK_R;

boolean newCount_L = false;
String currentDir_L = "";
int counter_L = 0;
int currentStateCLK_L;
int lastStateCLK_L;


double rho_dot = 0; 
double phi_dot = 0; 


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

  pinMode(CLK_R, INPUT_PULLUP);
  pinMode(DT_R, INPUT_PULLUP);
  pinMode(CLK_L, INPUT_PULLUP);
  pinMode(DT_L, INPUT_PULLUP);

  //Encoder position
  lastStateCLK_R = digitalRead(CLK_R);
  lastStateCLK_L = digitalRead(CLK_L);
  

 // Create two ISRs
  attachInterrupt(digitalPinToInterrupt(CLK_R), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK_L), updateEncoder_L, CHANGE);


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
  Serial.println(counter_R);
  String start_val = String(counter_R);
  for(int i = 0; i < start_val.length(); i++){
    data[i] = (int)start_val[i];
  }
  //delay(200);
  Wire.write(data, 32);
}

//***************************************************************

// the loop function runs over and over again forever
void loop() {

      //Localization Code


      rho_dot = r * (theta_dot_L + theta_dot_R) / (double) 2;

      phi_dot = r * (theta_dot_L - theta_dot_R) / (double) b; 


      x_loc = x_loc + rho_dot*cos(phi_dot); 
      y_loc = y_loc + rho_dot*sin(phi_dot);
   




        
      //Run the controller – turns wheel to specified position 
      //PIDController_R(); 

}


//***********************************************************************************************************

//Controller implementation 
void PIDController_R() { 
  
 //Takes current time 
  currentTime = millis(); 
  samplingTime = currentTime - storedTime;


  //Derivative Controller
  deriv = (rad_R - currentPosition) * 100; 

   
 //Gets current position 
  currentPosition = rad_R;
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


void updateEncoder_R(){
  
  currentStateCLK_R = digitalRead(CLK_R);
  int currentStateDT = digitalRead(DT_R);
  
  int newTime = millis();
  static int oldTime = 0; 
  

  
  if (currentStateCLK_R != lastStateCLK_R  && currentStateCLK_R == 1){

    
    if (currentStateDT != currentStateCLK_R) {
      counter_R --;
      currentDir_R ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter_R ++;
      currentDir_R ="CW";
    }

    double oldRad = rad_R;

    rad_R = (counter_R*2*PI)/800;
    double deltaT = ((newTime-oldTime)*0.001);
    theta_dot_R = (rad_R - oldRad) / (double) deltaT; 
    
    
    
     if(rad_R>=6.2831){
      rad_R=rad_R-6.2831;
      counter_R = 0;
    }else if(rad_R<=-6.2831){
      rad_R = rad_R+6.2831;
      counter_R = 0;

    
    }
    
    Serial.print("Direction: ");
    Serial.print(currentDir_R);
    Serial.print(" | counter_R: ");
    Serial.println(counter_R);
    Serial.println(theta_dot_R);
    
    newCount_R = true;
  }

  // Remember last CLK state
  lastStateCLK_R = currentStateCLK_R;

  oldTime = newTime;
}

void updateEncoder_L(){
  
  currentStateCLK_L = digitalRead(CLK_L);
  int currentStateDT = digitalRead(DT_L);

  int newTime = millis();
  static int oldTime = 0; 
  

  
  if (currentStateCLK_L != lastStateCLK_L  && currentStateCLK_L == 1){

    
    if (currentStateDT != currentStateCLK_L) {
      counter_L --;
      currentDir_L ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter_L ++;
      currentDir_L ="CW";
    }

  
    double oldRad = rad_L;

    rad_L = (counter_L*2*PI)/800;
    double deltaT = ((newTime-oldTime)*0.001);
    theta_dot_L = (rad_L - oldRad) / (double) deltaT; 

    
     if(rad_L>=6.2831){
      rad_L=rad_L-6.2831;
      counter_L = 0;
    }else if(rad_L<=-6.2831){
      rad_L = rad_L+6.2831;
      counter_L = 0;

    
    }
    
   Serial.print("Direction: ");
    Serial.print(currentDir_L);
    Serial.print(" | counter_L: ");
    Serial.println(counter_L);
    Serial.println(theta_dot_L);
    
    newCount_L = true;
  }


  oldTime = newTime;
  
  // Remember last CLK state
  lastStateCLK_L = currentStateCLK_L;
  static int lastStateDT = currentStateDT; 

  
}
