#include <Wire.h>


//I2C communication constant
#define SLAVE_ADDRESS 0x04

//Define Motor Pins
#define TriStatePin    4
#define MotorDirLeft     7
#define MotorDirRight    8   
#define MotorVoltLeft   9  //PWM for left motor
#define MotorVoltRight  10 //PWM for right motor

//Define Encoder Pins
#define CLK_R  2   //Uses Interrupt pin
#define DT_R   6
#define CLK_L  3    //Uses Interuppt pin
#define DT_L   11

void receiveData(int byteCount);
void sendData();

//I2C variables
float out = 0;
short int STATE = 0;//Finite State Machine
float Phi_PI_READ = 0;
int dist = 36;
float ang = 0; 

//wheel constants
float r = 0.24479; // radius of wheel as a fraction of one foot
float b = 1.23958; // distance between wheels as a fraction of one foot
#define rot 800


//encoder variables
int currentStateCLK_R;
int currentStateCLK_L;
int counter_L_old = 0;
int counter_L_new = 0;
int counter_L = 0;
int counter_R_old = 0;
int counter_R_new = 0;
int counter_R = 0;
int deltaCounter_R;
int deltaCounter_L;
int lastStateCLK_R;
int lastStateCLK_L;
String currentDirRight;
String currentDirLeft;

//Keep track of time for both motors with these variables
int tOldRight; // time old
int tOldLeft;
int tNewRight; // time new
int tNewLeft;
int deltaTRight = 0; // time new - time old
int deltaTLeft = 0;

// Controller parameters
double Kp = 10.5;
double Ki = 6.5;

double Kp_rho = 8.5; 
double Ki_rho = 5.5;

//Angle Desired
double phi_des = 0; 

//Distance Desired 
double rho_s = 0;

//Distance Vars
double rho_dot_des = 0; 
double rho = 0;
bool CLOSE = false;

//time variables
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;
float samplingTime = 0;

//angular velocity variables
double theta_dot_L = 0;
double theta_dot_R = 0;
double velocityRight = 0;
double velocityLeft = 0;
double velocityError = 0;

//angle position variables
double rad_L = 0;
double rad_R = 0;
double phi_curr = 0; 

//X and Y Position
long int x = 0;
long int y = 0; 


void setup() {
  // Set up I2C pin assignment and start serial 
  pinMode(13, OUTPUT);
  Serial.begin(9600);

  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
}

void loop() {
  Phi_PI_READ = phi_curr;
  switch(STATE){
    case 0:
        //Don't Change
      break;
    case 1:
     //Find Tape
      phi_des  = 1.8*PI;
      
      rho_s = rho;
      
      if(phi_curr > 5){
        //Send Pi Flag it is time to Transisition
        //Send 10.69 to pi
        Phi_PI_READ = 100;    
        //phi_des = (double) ang / 12.0;    
      }
       break;
       
      case 2:

      //For Moving to the Line of Tape
      //Distance and Angle Set by the Pi
      
      if(((abs(rho - (double) dist/12.0 ) < 1) && dist > 1) && CLOSE == false){// TODO Change based on where camera loses sight.        
        rho_s = rho + 1;
        phi_des = phi_curr;
        CLOSE = true;
        Serial.println("No");       
      } 
      
      if(CLOSE == false){
        phi_des = phi_curr + ang*0.01745;
        rho_s = rho + (double) dist / 12.0; 
        Serial.println("Yay"); 
      }

      if(rho - rho_s < 0.1){       
        Phi_PI_READ = 10.69;       
      }
        
        break;
        
      case 3:
      //Reorient to line up to travel along tape.  
      rho_s = rho;
      phi_des = (double) ang;  

      if(phi_des - phi_curr < 0.1){
        //Send Pi Flag it is time to Transisition
        //Send 10.69 to pi
        Phi_PI_READ = 100;
        //phi_des = (double) ang / 12.0;     
      }  
        break;
     

      case 5: //STOP MOVING!
        rho_s = rho;    
        phi_des = r* ((rad_R) - rad_L) / b;
        break;
      
      default: //Data Error
        break;
    }
  PID_CONTROL(); //Start the PID Control Loop
}




//I2C code
void receiveData(int byteCount){
  
}

void sendData(){
  Wire.write((byte*) &out, sizeof(float));
}

//******************************************************************************************************************************
//Nested PID COntroller

void PID_CONTROL(){
    // Outer Loop Time
    int outerLoopTime = micros();
    
    
    //Phi to phidot control 

    static double phi_er;
    static double phi_integral = 0;

    //double phi_curr = r* abs((rad_L) - rad_R) / b; 

    phi_curr = r* ((rad_R) - rad_L) / b; 
    phi_er = phi_des - phi_curr;

    if(phi_er < 3){ 
      phi_integral += phi_er;
    }

    
    //Serial.print("phi_curr = ");
    //Serial.println(phi_curr);

    
    double phi_dot_des = phi_er * Kp + phi_integral * Ki * 0.001;

    //Find how far we are from desired location and decide if to keep moving.

    //New Outer Loop Control of Rho

    static double rho_er;
    static double rho_integral = 0;

    rho = r * (abs(rad_R + rad_L) / 2.0);
    
    rho_er  = rho_s - rho;

    if(rho_er < 2){
      rho_integral += rho_er;
    }

    //Serial.print("rho_curr = ");
    //Serial.println(rho);
    
    rho_dot_des = rho_er * Kp_rho  + Ki_rho * rho_integral*0.001; 
       
    //Serial.print("rho_dot = ");
    //Serial.println(rho);

    //Inner Loop w/Time
    //Has Code to implement rho dot and phi dot. 
    for(int j = 0; j < 4; j++){
    
    int innerLoopTime = micros();

    //Rho dot control 
    static double rho_dot_er;
    double rho_dot_curr = r * (theta_dot_R + theta_dot_L) *0.5;
    
    rho_dot_er = rho_dot_des - rho_dot_curr; 

    double rho_V = Kp_rho * rho_dot_er;
    //Serial.print("rho_V = ");
    //Serial.println(rho_V);
   
    //Phi dot control 
    static double phi_dot_er;
    phi_dot_er = (r * ((theta_dot_R) - theta_dot_L) / (double) b);
    phi_dot_er = phi_dot_des - phi_dot_er;

    double phi_dot_V = Kp * phi_dot_er;    
    //Serial.print("phi_dot_V = ");
    //Serial.println(phi_dot_V);   

    //MOTOR_R write : Va + deltaVa / 2
    double V1 = (rho_V + phi_dot_V)* 0.5;
    //Serial.print("V1 = ");
    //Serial.println(V1);
    if(V1>0){
      digitalWrite(MotorDirRight, HIGH);
    }else{
      digitalWrite(MotorDirRight, LOW);
    }

    //Convert to PWM friendly value.
    V1 = abs(V1);
    
    if(V1 > 255){
      V1 = 255;
    }

    if(STATE == 1){
      if(V1 > 52){
        V1 = 52;
      }
    }

    analogWrite(MotorVoltRight,V1); 
    
    //MOTOR_Lwrite : Va + deltaVa / 2
    double V2 = (rho_V - phi_dot_V) *0.5;
    //Serial.print("V2 = ");
    //Serial.println(V2);
    if(V2>0){
      digitalWrite(MotorDirLeft, LOW);
    }else{
      digitalWrite(MotorDirLeft, HIGH);
    }

    //Convert to PWM friendly value.
    V2 = abs(V2);
    
    if(V2 > 255){
      V2 = 255;
    }

    if(STATE == 1){
      if(V2 > 52){
        V2 = 52;
      }
    }

    analogWrite(MotorVoltLeft,V2); 

    //Keep Track of Position   
    x = x + rho_dot_curr*0.0001*cos(phi_curr);
    y = y + rho_dot_curr*0.0001*sin(phi_curr);

    while(innerLoopTime + micros() < 100);  //Timing For Inner Control Loop -- 100 Microseconds each
    //End Inner Loop

    }
    while(outerLoopTime + micros() <  500); //Timing for outer Loop control -- 500 microseconds each.
    //End Outer Loop 
}

//***************************************************************************************************************************
//            Rotate code


void updateEncoder_R(){
  
  currentStateCLK_R = digitalRead(CLK_R);
  int currentStateDT = digitalRead(DT_R);
  
  int newTime = micros();
  static int oldTime = 0; 
   
  if (currentStateCLK_R != lastStateCLK_R  && currentStateCLK_R == 1){
  
    if (currentStateDT != currentStateCLK_R) {
      counter_R ++;
    } else {
      // Encoder is rotating CW so increment
      counter_R --;
    }

    double oldRad = rad_R;

    rad_R = (counter_R*2*PI)/(double)800;
    double deltaT = ((newTime-oldTime)*0.000001);
    theta_dot_R = (rad_R - oldRad) / (double) deltaT; 
    
  }

  // Remember last CLK state
  lastStateCLK_R = currentStateCLK_R;

  oldTime = newTime;
}


void updateEncoder_L(){
  
  currentStateCLK_L = digitalRead(CLK_L);
  int currentStateDT = digitalRead(DT_L);

  int newTime = micros();
  static int oldTime = 0; 
    
  if (currentStateCLK_L != lastStateCLK_L  && currentStateCLK_L == 1){
   
    if (currentStateDT != currentStateCLK_L) {
      counter_L --;
    } else {
      // Encoder is rotating CW so increment
      counter_L ++;
    }
 
    double oldRad = rad_L;

    rad_L = (counter_L*2*PI)/800;
    double deltaT = ((newTime-oldTime)*0.000001);
    theta_dot_L = (rad_L - oldRad) / (double) deltaT; 
  }


  oldTime = newTime;
  
  // Remember last CLK state
  lastStateCLK_L = currentStateCLK_L;
  static int lastStateDT = currentStateDT; 
}
