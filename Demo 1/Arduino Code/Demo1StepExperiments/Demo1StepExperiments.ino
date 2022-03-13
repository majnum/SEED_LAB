//Code that hopefully read both encoders and give us our transfer functions 
//Two interrupts used, one on each motor/encoder
//Motor voltages are the inputs and vectorV and DeltaV will be the outputs


#define clkRight 2
#define clkLeft  3
#define dtRight  6
#define dtLeft   11       

#define TriStatePin 4
#define motorDirLeft     7
#define motorDirRight    8   
#define motorVolLeft   9  //PWM for left motor
#define motorVolRight  10 //PWM for right motor

//Right and Left Encoder Variables
int counterRight = 0;
int newCounterRight = 0;
int oldCounterRight = 0;
int deltaCounterRight;
int counterLeft = 0;
int newCounterLeft = 0;
int oldCounterLeft = 0;
int deltaCounterLeft = 0;
int currentStateRight;
int currentStateLeft;
int lastStateRight;
int lastStateLeft;
String currentDirRight;
String currentDirLeft;


float thetaDotRight; // angular velocity
float thetaDotLeft;

//Keep track of time for both motors with these variables
int tOldRight; // time old
int tOldLeft;
int tNewRight; // time new
int tNewLeft;
int deltaTRight = 0; // time new - time old
int deltaTLeft = 0;

//Wheel variables
float wheelRadius = 0.23958333;           
float distanceBetweenWheels = 1.156;
float radRight;
float radLeft;

//Important Math stuff
int vectorV = 1;
int deltaV = 0;
double Va1 = (vectorV + deltaV)*0.5;
double Va2 = (vectorV - deltaV)*0.5;

//Overall time 
float masterTime;
double newTime;
double oldTime;

int moveFlag = 0;
bool flagMoving = false;



//Variables to track
float dist = 0;
float phi;

int gatekeeperOld = 0; // only one conditional statement is accesed per ISR loop
int gatekeeperNew = 1;

bool flag = false; // flag for printing in loop

//******************************************************************************************
//        Setup Function and Loop Functions
//******************************************************************************************

void setup() {
pinMode(clkRight, INPUT); // sets up inputs and baud rate and interrupts
pinMode(dtRight, INPUT);
pinMode(clkLeft, INPUT); // sets up inputs and baud rate and interrupts
pinMode(dtLeft, INPUT);
Serial.begin(250000);

lastStateRight = digitalRead(clkRight);
lastStateLeft = digitalRead(clkLeft);

//One interrupt per motor
attachInterrupt(digitalPinToInterrupt(clkRight), updateEncoder, CHANGE);
attachInterrupt(digitalPinToInterrupt(clkLeft), updateEncoder2, CHANGE);

//Motor Setup
pinMode(TriStatePin, OUTPUT); 
analogWrite(TriStatePin, 255); 

  pinMode(motorVolLeft, OUTPUT);
  pinMode(motorVolRight, OUTPUT);
  pinMode(motorDirLeft, OUTPUT);
  pinMode(motorDirRight, OUTPUT);

}



void loop() {

  //Calculate Rho and Phi dot
  float rhoDot = wheelRadius*(thetaDotRight + thetaDotLeft)*0.5;
  float phiDot = wheelRadius*(thetaDotRight + thetaDotLeft)*0.86505190311;


  digitalWrite(motorDirLeft,LOW);
  digitalWrite(motorDirRight,HIGH);
  
  analogWrite(motorVolLeft, 255);      //sets the motors speed
  analogWrite(motorVolRight, 255);      //sets the motors speed

  
  if(flag == true){ // only prints when the ISR is immediately triggered before
    newTime = micros();
    masterTime = (newTime - oldTime);
    dist = (counterLeft + counterRight)*1.57/(2*800);
    phi = ((-36.0*(counterRight))/(157.0));
    Serial.print("angular velocity (right/Left): "); // prints angular velocity
    Serial.println(thetaDotRight); // prints AV
    Serial.println(thetaDotLeft); // prints AV
    Serial.print("Rho Dot is: "); 
    Serial.println(rhoDot); // prints rhodot
    Serial.print("Phi Dot is: "); 
    Serial.println(phiDot); // prints rhodot
    Serial.print("Va1: "); 
    Serial.println(Va1); // prints Va1
    Serial.print("Va2: "); 
    Serial.println(Va2); // prints Va2
  
  flag = false; // resets flag
  oldTime = micros();
  }
   if(moveFlag<=1000){
  rotate(0);
  }else if(moveFlag>=1000){
    if(flagMoving == false){
      flagMoving = true;
      counterLeft = 0;
      counterRight = 0;
    }
    moveDistance(1);
  }
  
  
  }// END OF LOOP

void rotate(float desAngle){
  if(phi<desAngle+1 && phi>desAngle-1){
    thetaDotRight = 0;
    thetaDotLeft = 0;
    moveFlag++;
  }else if(phi<desAngle){
    thetaDotRight = -65;
    thetaDotLeft = 65;
  }else if(phi>desAngle){
    thetaDotRight = 65;
    thetaDotLeft = -65;
  }
    analogWrite(motorVolRight,0);
    analogWrite(motorVolLeft,0);
    //md.setM2Speed(speedM2);
}

void moveDistance(float desDist){
  if(dist<desDist){
      thetaDotRight=100;
      thetaDotLeft=100;
    digitalWrite(motorVolRight,0);
    digitalWrite(motorVolLeft,0);
  }else{
    digitalWrite(motorVolRight,0);
    digital   Write(motorVolLeft,0);
  }
}

//******************************************************************************************
//     Encoder Functions (Right Motor = updateEncoder) and (Left Motor = updateEncoder2)
//******************************************************************************************

void updateEncoder(){ // ISR for updating encoder
  currentStateRight = digitalRead(clkRight);


  if(currentStateRight != lastStateRight && currentStateRight == 1){ // conditional to read the encoder
    if(gatekeeperOld == 0){ 
      tNewRight = micros();
      gatekeeperNew = 0; // resets gates 
      gatekeeperOld = 1; // resets gates
      deltaTRight = tNewRight - tOldRight; // difference between time
      deltaCounterRight = newCounterRight - oldCounterRight;
      thetaDotRight = (deltaCounterRight*2*3.14)/(800*deltaTRight); // calculates angular velocity
    }
    oldCounterRight = counterRight;
    if(digitalRead(dtRight) != currentStateRight){ // increments encoder counts (either increasing or decreasing based on direction
      counterRight --;
      currentDirRight = "CCW";
    }
    else{
      counterRight ++;
      currentDirRight = "CW";
    }
    radRight = (counterLeft*2*3.14)/800;
    newCounterRight = counterRight;
    tOldRight = micros();
  }
  lastStateRight = currentStateRight;

  if(gatekeeperNew == 0){ // second conditional to capture time and set the flag to print angular velocity
    tOldRight = micros();
    gatekeeperOld = 0;
    gatekeeperNew = 1;
    flag = true;
  }

  
} // END OF RIGHT ENCODER

void updateEncoder2(){ // ISR for updating encoder
  currentStateLeft = digitalRead(clkLeft);


  if(currentStateLeft != lastStateLeft && currentStateLeft == 1){ // conditional to read the encoder
    if(gatekeeperOld == 0){ 
      tNewLeft = micros();
      gatekeeperNew = 0; // resets gates 
      gatekeeperOld = 1; // resets gates
      deltaTLeft = tNewLeft - tOldLeft; // difference between time
      deltaCounterLeft = newCounterLeft - oldCounterLeft;
      thetaDotLeft = (counterLeft*2*3.14)/(800*deltaTLeft); // calculates angular velocity
    }

    if(digitalRead(dtLeft) != currentStateLeft){ // increments encoder counts (either increasing or decreasing based on direction
      counterLeft --;
      currentDirLeft = "CCW";
    }
    else{
      counterLeft ++;
      currentDirLeft = "CW";
    }
    radLeft = (counterLeft*2*3.14)/800;
    newCounterLeft = counterLeft;
    tOldLeft = micros();
  }
  
  lastStateLeft = currentStateLeft;

  if(gatekeeperNew == 0){ // second conditional to capture time and set the flag to print angular velocity
    tOldLeft = micros();
    gatekeeperOld = 0;
    gatekeeperNew = 1;
    flag = true;
  }
  
  
} // END OF LEFT ENCODER
