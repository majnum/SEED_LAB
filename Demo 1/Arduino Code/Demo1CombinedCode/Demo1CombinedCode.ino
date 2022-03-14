//******************************************************************************************
//Combined Arduino Code for Demo 1
//******************************************************************************************


//******************************************************************************************
//          GLOBAL VARIABLES
//******************************************************************************************

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
double Kp = 10;
double Ki = 3;

double Kp_rho = 7.5; 

double phi_des = PI/4; 
double rho_dot_des = 0; 
double rho = 0;
//double rho_s = 5 - 0.175*5;
double rho_s = 5;

//******************************************************************************************

//******************************************************************************************
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //assign Pins I/O Logic
  pinMode(CLK_R, INPUT_PULLUP);
  pinMode(DT_R, INPUT_PULLUP);
  pinMode(CLK_L, INPUT_PULLUP);
  pinMode(DT_L, INPUT_PULLUP);

  lastStateCLK_R = digitalRead(CLK_R);
  lastStateCLK_L = digitalRead(CLK_L); 
  
  //One interrupt per motor
  attachInterrupt(digitalPinToInterrupt(CLK_R), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK_L), updateEncoder_L, CHANGE);

  //Motor Setup
  pinMode(TriStatePin, OUTPUT); 
  digitalWrite(TriStatePin, HIGH); 

  //Set up outputs
  pinMode(MotorVoltLeft, OUTPUT);
  pinMode(MotorVoltRight, OUTPUT);
  pinMode(MotorDirLeft, OUTPUT);
  pinMode(MotorDirRight, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //static double analogLeft = 0;
  //static double analogRight = 0;

 
  //Calculate Rho and Phi dot
  
  
  //Controller
  PID_CONTROL(); 



  //calculate angular velocity of wheels
  
 

}


//******************************************************************************************************************************
//Nested PID COntroller

void PID_CONTROL(){
    // Outer Loop Time
    int outerLoopTime = micros();
    static bool goFoward = false;
    
    //Phi to phidot control 

    static double phi_er;
    static double phi_integral = 0;
    double phi_curr = r* (abs(rad_L) - rad_R) / b; 
    phi_er = phi_des - phi_curr;
    phi_integral += phi_er;
    //Serial.print("phi_curr = ");
    //Serial.println(phi_curr);

    
    double phi_dot_des = phi_er * Kp + phi_integral * Ki * 0.001;


    //Find how far we are from desired location.

    if((abs(rho_s - rho) > 0.2) && (abs(phi_er < PI/64))){ //While not at desired position and wait two seconds for robot to orient correctly 
     if(!goFoward){
      phi_des = phi_curr;
      phi_integral = 0;
      goFoward = true;
     }
     
     if(rho_s - rho > 0){
      rho_dot_des = 20;
     } else{
      rho_dot_des = -20;
     }
    } else if((abs(rho_s - rho) > 0.1) && abs(phi_er < PI/4)){
      rho_dot_des = 0; 
    }
    //Serial.print("rho_dot = ");
    //Serial.println(rho_dot_des);


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
    phi_dot_er = (r * (abs(theta_dot_R) - theta_dot_L) / (double) b);
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
    
    V1 = abs(V1);
    
    if(V1 > 255){
      V1 = 255;
    }


    analogWrite(MotorVoltRight,V1*0.8); 
    


    //MOTOR_Lwrite : Va + deltaVa / 2
    double V2 = (rho_V - phi_dot_V) *0.5;
    //Serial.print("V2 = ");
    //Serial.println(V2);
    if(V2>0){
      digitalWrite(MotorDirLeft, LOW);
    }else{
      digitalWrite(MotorDirLeft, HIGH);
    }
    
    V2 = abs(V2);
    
    if(V2 > 255){
      V2 = 255;
    }


    analogWrite(MotorVoltLeft,V2); 


    //Keep Track of Position
    rho = rho + rho_dot_curr*0.0001;
     



    while(innerLoopTime + micros() < 100); 
    //End Inner Loop

    }





    

    while(outerLoopTime + micros() <  500);
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
    
    
    
//     if(rad_R>=6.2831){
//      rad_R=rad_R-6.2831;
//      counter_R = 0;
//    }else if(rad_R<=-6.2831){
//      rad_R = rad_R+6.2831;
//      counter_R = 0;
//
//    
//    }
    
  
    //Serial.print(" | counter_R: ");
    //Serial.println(counter_R);
    //Serial.println(theta_dot_R);
    
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

//    
//     if(rad_L>=6.2831){
//      rad_L=rad_L-6.2831;
//      counter_L = 0;
//    }else if(rad_L<=-6.2831){
//      rad_L = rad_L+6.2831;
//      counter_L = 0;
//
//    
//    }
//    
 
    //Serial.print(" | counter_L: ");
    //Serial.println(counter_L);
    //Serial.println(theta_dot_L);
    
  }


  oldTime = newTime;
  
  // Remember last CLK state
  lastStateCLK_L = currentStateCLK_L;
  static int lastStateDT = currentStateDT; 

  
}
