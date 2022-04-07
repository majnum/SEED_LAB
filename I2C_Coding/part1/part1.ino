#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200); //start serial for output
  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callabcks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

void receiveData(int byteCount){
  
  Serial.print("data recieved: ");
  while(Wire.available()){
    number = Wire.read() + 5;
    Serial.print(number);
    Serial.print(' ');  
  }
  Serial.println(' ');
}

void sendData(){
  Wire.write(number);
}
