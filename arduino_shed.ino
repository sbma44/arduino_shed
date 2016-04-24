#include <avr/sleep.h>
#include <avr/power.h>

volatile bool watchdogActivated = false;
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

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  cycle_count = cycle_count + 1;
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
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
 
   // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP2);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
 
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
