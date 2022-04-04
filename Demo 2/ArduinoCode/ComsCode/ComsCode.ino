#include <Wire.h>



#define SLAVE_ADDRESS 0x04
int read_offset = 0;
int state = 0;
int len = 0;
int in_data[32] = {};

int dist = 0;
float ang = 0; 


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
    
    //Break array into parts
    int j = 0;
    state = in_data[j];
    String now = ""; 

    //Get the distance
    for (j = 1; j < 4; j++){
      now = now + char(in_data[j]);
    }
    dist = now.toInt();


    now = "";
    for (j = 4; j < 21; j++){
        now = now + char(in_data[j]);
    }
    ang = now.toFloat();
 
  }
  else{
    read_offset = Wire.read();
  }
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
