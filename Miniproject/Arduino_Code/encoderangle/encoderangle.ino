
// A program that reads the direction of two Encoders and calculates movement. 
// By Joey Thurman 2/23/2022

#define r 0.05 
#define b 0.1
#define rot 3200
#define CLK 2
#define DT 4
#define CLK2 3
#define DT2 5 
#define pi 3.14159

volatile float ang_right = 0; 
volatile float ang_left = 0;

volatile int time_l = 0;
volatile int time_r = 0; 

volatile float theta_r;
volatile float theta_l;

volatile float x = 0;
volatile float y = 0; 
volatile float phi = 0; 

volatile byte data = 0;


void setup() {
  //Enable serial communication
  Serial.begin(9600);
// initialize digital pin 

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(CLK2, INPUT_PULLUP);
  pinMode(DT2, INPUT_PULLUP);

 // Create an ISR attached to pin 2 on a rising edge. 
  attachInterrupt(digitalPinToInterrupt(CLK), rotate_r, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK2), rotate_l, RISING);

  
  
}

// the loop function runs over and over again forever
void loop() {
      
      //Print the movement When their is new encoder data.
      
      theta_r = (float) ang_right*rot / (2*pi); 
      theta_l = (float) ang_left*rot / (2*pi); 

       
      
      
      
}

//When a rising edge on pin 2 is detected check the direction of the Encoder based on previous inputs. 
void  rotate_r(){

  //Read the data from the encoder
  byte r_update = digitalRead(DT);
  float oldangright = ang_right;

  //Check which way its going
  if(r_update != digitalRead(CLK)){

    
    ang_right += 1; 

    if(ang_right > rot-1){
      ang_right = 0;
    }
    
    
  } else{
   
    ang_right -= 1; 

    if(ang_right < 0){
      ang_right = rot - 1;
    }
    
  }

  

}
      
void  rotate_l(){

  //Read the data from the encoder
  byte l_update = digitalRead(DT2);
  float ang_old = ang_left;
  //Check which way its going
  if(l_update != digitalRead(CLK2)){
    
    ang_left += 1; 

    if(ang_left > rot-1){
      ang_left = 0;
    }
    
  } else{

    ang_left -= 1; 

    if(ang_left < 0){
      ang_left = rot-1;
    }
    
  }

   
 
}
