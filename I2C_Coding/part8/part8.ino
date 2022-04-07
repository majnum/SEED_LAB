String data;
bool DataRead;

void setup() {
  Serial.begin(115200);

}

void loop() {
  
  if (DataRead) {
    int val = data.toInt();
    val = val + 5;
    Serial.print("Converted the value you sent to: ");
    Serial.println(val);
    DataRead = false;
  }

}

void serialEvent(){
  if(Serial.available() > 0){
    //data = Serial.read();
    data = Serial.readStringUntil('\n');
    DataRead = true;
  }
  Serial.flush();
}
