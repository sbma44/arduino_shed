#include <SimpleDHT.h>

#define pinDHT11 6
SimpleDHT11 dht11;

#include <Adafruit_ESP8266.h>
#include <SoftwareSerial.h>

#define ARD_RX_ESP_TX   2
#define ARD_TX_ESP_RX   3
#define ESP_RST         4
SoftwareSerial softser(ARD_RX_ESP_TX, ARD_TX_ESP_RX); // Arduino RX = ESP TX, Arduino TX = ESP RX

Adafruit_ESP8266 wifi(&softser, &Serial, ESP_RST);

#define ESP_SSID "Q Continuum" // Your network name here
#define ESP_PASS "section31" // Your network password here

#define HOST     "192.168.0.50"     // Host to contact
#define PAGE     "/" // Web page to request
#define PORT     8080

#include <avr/sleep.h>
#include <avr/power.h>

volatile bool watchdogActivated = false;
unsigned long cycle_count = 0;
unsigned long last_motion_detected = 0;
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
#define OVERHEAD_LIGHT_PIN 5
#define MOTION_DETECT_PIN 11
#define LIGHT_DETECT_PIN 12
#define STATUS_LED 13

boolean lights_on = false;

void fade(boolean up){
  int i = 0;
  while (i<=255) {
    if(up){
      analogWrite(OVERHEAD_LIGHT_PIN, i);
    }else{
      analogWrite(OVERHEAD_LIGHT_PIN, 255-i);
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
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}

// the setup routine runs once when you press reset:
void setup() {  

  Serial.begin(9600); 

  softser.begin(9600); // Soft serial connection to ESP8266
    
  // initialize the digital pin as an output.
  pinMode(OVERHEAD_LIGHT_PIN, OUTPUT);
  pinMode(MOTION_DETECT_PIN, INPUT);
  pinMode(LIGHT_DETECT_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(MOTION_DETECT_PIN, LOW);
  digitalWrite(LIGHT_DETECT_PIN, HIGH); 
  analogWrite(OVERHEAD_LIGHT_PIN, 0);
 
   // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  /* 
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP2);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  */
  // Enable interrupts again.
  interrupts();
 
  wifi.setBootMarker(F("Version:0.9.2.4]\r\n\r\nready"));
}

// the loop routine runs over and over again forever:
void loop() {
  int motion_detected = digitalRead(MOTION_DETECT_PIN);
  int darkness = digitalRead(LIGHT_DETECT_PIN);
  
  if(motion_detected==1){
    last_motion_detected = cycle_count;
    
    for(int i=0;i<5;i++){
      digitalWrite(STATUS_LED, HIGH);
      delay(30);
      digitalWrite(STATUS_LED, LOW);
      delay(30);
    }
    
    Serial.println("motion detected"); 
  }
  
  if(darkness==1){
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("darkness detected");
  }
  else{
    digitalWrite(STATUS_LED, LOW);
    Serial.println("darkness NOT detected");
  }
  
  delay(100); // wait for UART to finish
  
  if ((darkness==1) && (lights_on==false) && (motion_detected==1)){
    fade(true);
    sendActivity();
    lights_on = true; 
  }
  if (((darkness==0) || (motion_detected==0)) && (lights_on==true) && ((cycle_count-last_motion_detected)>300)){
    fade(false);
    lights_on = false;
  }
  
  if((motion_detected==0)&&((cycle_count%300)==0)){
    sendTemp();
  }

  cycle_count = cycle_count + 1;
  //sleep();
}

void sendTemp() {
  byte temperature = 0;
  byte humidity = 0;
  if (dht11.read(pinDHT11, &temperature, &humidity, NULL)) {
    Serial.print("Read DHT11 failed.");
    return;
  }
  Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" %");

  String s = "temp=";
  s.concat(String(temperature));
  s.concat("&humidity=");
  s.concat(String(humidity));
  sendWifiMessage(s);
}

void sendActivity() {
  String s = "activity detected";
  sendWifiMessage(s);
}

void sendWifiMessage(String msg) {
//  return;
  char buffer[50];

  // Test if module is ready
  Serial.print(F("Hard reset..."));
  if(!wifi.hardReset()) {
    Serial.println(F("no response from module."));
    return;
  }
  Serial.println(F("OK."));

  Serial.print(F("Soft reset..."));
  if(!wifi.softReset()) {
    Serial.println(F("no response from module."));
    return;
  }
  Serial.println(F("OK."));

  Serial.print(F("Checking firmware version..."));
  wifi.println(F("AT+GMR"));
  if(wifi.readLine(buffer, sizeof(buffer))) {
    Serial.println(buffer);
    wifi.find(); // Discard the 'OK' that follows
  } else {
    Serial.println(F("error"));
  }
  
  Serial.print(F("Connecting to WiFi..."));
  if(wifi.connectToAP(F(ESP_SSID), F(ESP_PASS))) {

    // IP addr check isn't part of library yet, but
    // we can manually request and place in a string.
    Serial.print(F("OK\nChecking IP addr..."));
    wifi.println(F("AT+CIFSR"));
    if(wifi.readLine(buffer, sizeof(buffer))) {
      Serial.println(buffer);
      wifi.find(); // Discard the 'OK' that follows

      Serial.print(F("Connecting to host..."));
      if(wifi.connectTCP(F(HOST), PORT)) {
        Serial.print(F("OK\nRequesting page..."));

        String s = PAGE;
        s.concat("?");
        s.concat(msg);
        char c[s.length()+2];
        s.toCharArray(c, s.length()+1);
        if(wifi.requestURL(c)) {
          Serial.println("OK\nSearching for string...");
          // Search for a phrase in the open stream.
          // Must be a flash-resident string (F()).
          if(wifi.find(F("hello"), true)) {
            Serial.println(F("found!"));
          } else {
            Serial.println(F("not found."));
          }
        } else { // URL request failed
          Serial.println(F("error"));
        }
        wifi.closeTCP();
      } else { // TCP connect failed
        Serial.println(F("D'oh!"));
      }
    } else { // IP addr check failed
      Serial.println(F("error"));
    }
    wifi.closeAP();
  } else { // WiFi connection failed
    Serial.println(F("FAIL"));
  }

  wifi.println(F("AT+GSLP=600000"));
}

