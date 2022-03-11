//Code that hopefully read both encoders and give us our transfer functions 
//Two interrupts used, one on each motor/encoder
//Motor voltages are the inputs and vectorV and DeltaV will be the outputs


#define clkRight 2
#define clkLeft  3
#define dtRight  4
#define dtLeft   5

//Right and Left Encoder Variables
int counterRight = 0;
int counterLeft = 0;
int currentStateRight;
int currentStateLeft;
int lastStateRight;
int lastStateLeft;
String currentDirRight;
String currentDirLeft;
float thetaDotRight; // angular velocity
float thetaDotLeft;

//Wheel variables
float wheelRadius = 0.23958333;           
float distanceBetweenWheels = 1.156;

//Important Math stuff
int vectorV = 0;
int deltaV = 0;
double Va1 = (vectorV + deltaV)*0.5;
double Va2 = (vectorV - deltaV)*0.5;


//Keep track of time for both motors with these variables
int tOldRight; // time old
int tOldLeft;
int tNewRight; // time new
int tNewLeft;
int deltaTRight = 0; // time new - time old
int deltaTLeft = 0;

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
attachInterrupt(clkRight, updateEncoder, CHANGE);
attachInterrupt(clkLeft, updateEncoder2, CHANGE);
}



void loop() {

  //Calculate Rho and Phi dot
  float rhoDot = wheelRadius*(thetaDotRight + thetaDotLeft)*0.5;
  float phiDot = wheelRadius*(thetaDotRight + thetaDotLeft)*0.86505190311;
  
  if(flag == true){ // only prints when the ISR is immediately triggered before
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
  }

  
  
  }// END OF LOOP

//******************************************************************************************
//     Encoder Functions (Right Motor = updateEncoder) and (Left Motor = updateEncoder2)
//******************************************************************************************

void updateEncoder(){ // ISR for updating encoder
  currentStateRight = digitalRead(clkRight);


  if(currentStateRight != lastStateRight && currentStateRight == 1){ // conditional to read the encoder
    if(gatekeeperOld == 0){ 
      tNewRight = millis();
      gatekeeperNew = 0; // resets gates 
      gatekeeperOld = 1; // resets gates
      deltaTRight = tNewRight - tOldRight; // difference between time
      thetaDotRight = (counter*2*3.14)/(800*deltaT); // calculates angular velocity
    }

    if(digitalRead(dtRight) != currentStateRight){ // increments encoder counts (either increasing or decreasing based on direction
      counter --;
      currentDirRight = "CCW";
    }
    else{
      counter ++;
      currentDirRight = "CW";
    }
  }
  lastStateRight = currentStateRight;

  if(gatekeeperNew == 0){ // second conditional to capture time and set the flag to print angular velocity
    tOldRight = millis();
    gatekeeperOld = 0;
    gatekeeperNew = 1;
    flag = true;
  }

  
} // END OF RIGHT ENCODER

void updateEncoder2(){ // ISR for updating encoder
  currentStateLeft = digitalRead(clkLeft);


  if(currentStateLeft != lastStateLeft && currentStateLeft == 1){ // conditional to read the encoder
    if(gatekeeperOld == 0){ 
      tNewLeft = millis();
      gatekeeperNew = 0; // resets gates 
      gatekeeperOld = 1; // resets gates
      deltaTLeft = tNewLeft - tOldLeft; // difference between time
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
  }
  lastStateLeft = currentStateLeft;

  if(gatekeeperNew == 0){ // second conditional to capture time and set the flag to print angular velocity
    tOld = millis();
    gatekeeperOld = 0;
    gatekeeperNew = 1;
    flag = true;
  }
  
  
} // END OF LEFT ENCODER
