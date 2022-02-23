
#define TriStatePin 4

#define Motor1DirControlPin 7
#define Motor2DirControlPin 8

#define Motor1VoltControlPin 9
#define Motor2VoltControlPin 10

#define FaultPin 12


void setup() {

  pinMode(TriStatePin, OUTPUT); 
  analogWrite(TriStatePin, HIGH); 

  pinMode(Motor1VoltControlPin, OUTPUT);
  pinMode(Motor1DirControlPin, OUTPUT);

  pinMode(Motor2VoltControlPin, OUTPUT);
  pinMode(Motor2DirControlPin, OUTPUT);

  pinMode(FaultPin, INPUT);
  
}

void loop() {

  static int motorVolt;
  motorVolt = (motorVolt + 32) % 256;
  int direct = 0;

  analogWrite(Motor1DirControlPin, direct); 
  
  analogWrite(Motor1VoltControlPin, motorVolt);

  

}
