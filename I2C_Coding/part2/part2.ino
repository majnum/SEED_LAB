#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int data[32] = {};

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
      data[i] = Wire.read();
      Serial.print(data[i]);
      Serial.print(' ');
      i++;
    }
    Serial.print(' ');
    i--;
  }
  else{
    read_offset = Wire.read();
    Serial.print(read_offset);
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
  if(read_offset == data[0]){
    switch(data[0]){
      case 0: 
        Wire.write(data[1] + 5);
        break;
      case 1:
        Wire.write(data[1] + 10);
        break; 
      default:
        Wire.write(data[1]);
        break;     
    }
  }
  else{
    Wire.write(0);
  }  
}
