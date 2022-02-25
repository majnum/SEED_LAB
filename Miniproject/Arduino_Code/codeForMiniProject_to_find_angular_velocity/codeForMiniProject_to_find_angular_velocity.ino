#define clk 2
#define dt 3

int counter = 0;
int currentState;
int lastState;
String currentDir;
float thetaDot; // angular velocity

int tOld; // time old
int tNew; // time new
int deltaT = 0; // time new - time old

int gatekeeperOld = 0; // only one conditional statement is accesed per ISR loop
int gatekeeperNew = 1;

bool flag = false; // flag for printing in loop


void setup() {
pinMode(clk, INPUT); // sets up inputs and baud rate and interrupts
pinMode(dt, INPUT);
Serial.begin(250000);
lastState = digitalRead(clk);
attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
}



void loop() {
  if(flag == true){ // only prints when the ISR is immediately triggered before
  Serial.print("angular velocity is: "); // prints angular velocity
  Serial.println(thetaDot); // prints AV
  flag = false; // resets flag
  }
  }

void updateEncoder(){ // ISR for updating encoder
  currentState = digitalRead(clk);


  if(currentState != lastState && currentState == 1){ // conditional to read the encoder
    if(gatekeeperOld == 0){ 
      tNew = millis();
      gatekeeperNew = 0; // resets gates 
      gatekeeperOld = 1; // resets gates
      deltaT = tNew - tOld; // difference between time
      thetaDot = (counter*2*3.14)/(800*deltaT); // calculates angular velocity
    }

    if(digitalRead(dt) != currentState){ // increments encoder counts (either increasing or decreasing based on direction
      counter --;
      currentDir = "CCW";
    }
    else{
      counter ++;
      currentDir = "CW";
    }
  }
  lastState = currentState;

  if(gatekeeperNew == 0){ // second conditional to capture time and set the flag to print angular velocity
    tOld = millis();
    gatekeeperOld = 0;
    gatekeeperNew = 1;
    flag = true;
  }
  
  
}
