//******************************************************************************************
//Combined Arduino Code for Demo 1
//******************************************************************************************


//******************************************************************************************
//          GLOBAL VARIABLES
//******************************************************************************************

//Define Motor Pins
#define TriStatePin 4
#define motDirLeft =    7
#define motDirRight =   8   
#define motorVolLeft =  9  //PWM for left motor
#define motorVolRight = 10 //PWM for right motor

//Define Encoder Pins
#define CLK_R  2   //Uses Interrupt pin
#define DT_R  6
#define CLK_L  3    //Uses Interuppt pin
#define DT_L  7



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
float r = 0.23958333; // radius of wheel as a fraction of one foot
float b = 1.156; // distance between wheels as a fraction of one foot
#define rot 800


//encoder variables

int currentStateCLK_R;
int currentStateCLK_L;
int counter_L = 0;
int counter_R = 0;
int lastStateCLK_R;
int lastStateCLK_L;



// Controller parameters

double Kp = 0;
double Ki = 0;



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
  
  
  
  attachInterrupt(digitalPinToInterrupt(CLK_R), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK_L), updateEncoder_L, CHANGE);

  pinMode(TriStatePin, OUTPUT); 
  analogWrite(TriStatePin, 255); 

}

void loop() {
  // put your main code here, to run repeatedly:

  
  
  
  static double analogLeft = 0;
  static double analogRight = 0;

  //Controller
  currentTime = millis();




  //calculate angular velocity of wheels
  
 

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
    } else {
      // Encoder is rotating CW so increment
      counter_R ++;
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
    
  
    Serial.print(" | counter_R: ");
    Serial.println(counter_R);
    Serial.println(theta_dot_R);
    
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
    } else {
      // Encoder is rotating CW so increment
      counter_L ++;
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
    
 
    Serial.print(" | counter_L: ");
    Serial.println(counter_L);
    Serial.println(theta_dot_L);
    
  }


  oldTime = newTime;
  
  // Remember last CLK state
  lastStateCLK_L = currentStateCLK_L;
  static int lastStateDT = currentStateDT; 

  
}
