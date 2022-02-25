// Rotary Encoder Inputs
#define CLK 2
#define DT 3

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
float rad = 0;
boolean newCount = false;

void setup() {
  
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);

  Serial.begin(9600);

 
  lastStateCLK = digitalRead(CLK);
  
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
}

void loop() {
  if (newCount){
    Serial.print("rad: ");
      Serial.println(rad);
      newCount=false;
  }
}

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

    rad = (counter*2*3.1415)/640;
    
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    
    newCount = true;
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}
