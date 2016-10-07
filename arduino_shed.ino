#include <LowPower.h>

unsigned long cycle_count = 0;
unsigned long last_motion_detected = 0;
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 5;
int motion = 11;
int light = 12;
int status_led = 13;

int ptt_pin = 8;
int tx_pin = 9;

boolean lights_on = false;

void fade(boolean up){
  int i = 0;
  while (i<=255) {
    if(up){
      analogWrite(led, i);
    }else{
      analogWrite(led, 255-i);
    }
    i = i + 1;
    delay(5);
  }
}

// Put the Arduino to sleep.
void sleep()
{
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
  cycle_count = cycle_count + 1;
}

// the setup routine runs once when you press reset:
void setup() {  

  Serial.begin(9600); 
    
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(motion, INPUT);
  pinMode(light, INPUT);
  pinMode(status_led, OUTPUT);
  digitalWrite(status_led, LOW);
  digitalWrite(motion, LOW);
  digitalWrite(light, HIGH); 
  analogWrite(led, 0);
}

// the loop routine runs over and over again forever:
void loop() {
  int motion_detected = digitalRead(motion);
  int darkness = digitalRead(light);
  
  if(motion_detected==1){
    last_motion_detected = cycle_count;
    
    for(int i=0;i<5;i++){
      digitalWrite(status_led, HIGH);
      delay(30);
      digitalWrite(status_led, LOW);
      delay(30);
    }
    
    //sendMessage(true);
    Serial.println("motion detected"); 
  }
  
  if(darkness==1){
    digitalWrite(status_led, HIGH);
    Serial.println("darkness detected");
  }
  else{
    digitalWrite(status_led, LOW);
    Serial.println("darkness NOT detected");
  }
  
  delay(100); // wait for UART to finish
  
  if ((darkness==1) && (lights_on==false) && (motion_detected==1)){
    fade(true);
    lights_on = true; 
  }
  if (((darkness==0) || (motion_detected==0)) && (lights_on==true) && ((cycle_count-last_motion_detected)>30)){
    fade(false);
    lights_on = false;
  }
  
  if((motion_detected==0)&&((cycle_count%30)==0)){
    //sendMessage(false);  
  }
  
  sleep();
}
