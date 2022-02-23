#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int len = 0;
int in_data[32] = {};

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
  int i = 0;
  if(byteCount > 1){
    while(Wire.available()){
      in_data[i] = Wire.read();
      Serial.print(in_data[i]);
      Serial.print(' ');
      i++;
    }
    Serial.print(' ');
    i--;
    len = i;
  }
  else{
    read_offset = Wire.read();
    //Serial.print(read_offset);
  }
  
  /*
  Serial.print("data recieved: ");
  while(Wire.available()){
    number = Wire.read();
    Serial.print(number);
    Serial.print(' ');  
  }
  Serial.println(' ');
  */
  
}

void sendData(){
  byte data[32] = {};
  if(read_offset == in_data[0]){
    int j = 30;
    for(int i = 1; i < 32; i++){
      int val = in_data[i];
      if(val != 0){
        data[j] = in_data[i];
      }
       j--;
    }
  }
  Wire.write(data, 32);
}
