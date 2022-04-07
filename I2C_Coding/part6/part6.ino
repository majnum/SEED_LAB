
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int sensorPin = A0;


void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callabcks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
}

void loop(){
  delay(100);
}

void receiveData(int byteCount){
  
}

void sendData(){
  static byte data[32] = {};
  for(int i = 0; i < 32; i++){
    data[i] = 0;
  }
  /*
  String start_val = String(analogRead(sensorPin));
  for(int i = 0; i < start_val.length(); i++){
    data[i] = (int)start_val[i];
  }
  */
  //delay(200);
  Wire.write(data, 32);
  
}
