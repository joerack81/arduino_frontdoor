#include <Homotica.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SimpleTimer.h>
#include <SPI.h> 
#include <OneWire.h>
#include <Adafruit_BMP085_U.h>
#include <Wire.h>

#define I2C_SCL D0
#define SCL_PORT PORTC      
#define I2C_SDA D7      
#define SDA_PORT PORTC

//FRONT DOOR=2 GATE=D1   GLASS DOOR = D5

#define WDOOR D2
#define GDOOR D5
#define GATE D1
const int buttonPin = D6; // The number of the Pushbutton pin


//BUTTON declarations
SimpleTimer timer;

int buttonState = 0;  // Variable for reading the pushbutton status
long previousMillis = 0;        // will store last time LED was updated
long interval = 5000;           // interval at which to blink (milliseconds)
long currentValue;
float Dtemp;

int buttonstatedoor = 0;




//MODIFY THESE PARAMETERS
const char* ssid = "JOENET";                             //your wifi ssid
const char* password = "";                     //your wifi password

IPAddress ip(10,0,0,190);                             //your arduino IP
IPAddress gateway(10,0,0,1);                         //your network gateway
IPAddress subnet(255, 255, 255, 0);                        //your network subnet mask

unsigned int mUdpPort = 5858;
static String code = "qawsedrf";
Homotica homotica;

unsigned long lastMillis = 0;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10180);
bool bmpSensorDetected = true;

void setup() {
    Serial.begin(9600);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  //D4= DOOR D5=GATE

  
  pinMode(buttonPin, INPUT);
  pinMode(WDOOR, OUTPUT);
  pinMode(GATE, OUTPUT);
  pinMode(WDOOR, OUTPUT);

 // digitalWrite(WDOOR, HIGH);


  homotica.addUsedPinOutput(WDOOR);
  homotica.addUsedPinOutput(GATE);
  homotica.addUsedPinOutput(GDOOR);
 
  homotica.attachInputFunction(MCustomInputFunction, 'M');
  homotica.attachInputFunction(NCustomInputFunction, 'N');


 //homotica.setActiveLow();  //<-- only if you need it
 homotica.set(mUdpPort, code);

//homotica.turnOn(WDOOR);
//homotica.turnOn(GDOOR);
homotica.turnOn(GATE);  

  //BMP180
  if (!bmp.begin())

  {
    Serial.println("No BMP sensor detected");
    bmpSensorDetected = false;
  }
    timer.setInterval(1200L, emergencybtn);
    timer.setInterval(2000L, gateoff);

}

void loop() {
  homotica.refresh();
  timer.run();
}


int16_t MCustomInputFunction() 
{
      //Send the value to homotica for temperature display (INDOORS SENSOR)
      float temperature;
      bmp.getTemperature( & temperature);
      // Send the value to homotica in degrees centigrade.
      int16_t returnValue = ((temperature) - 1);
      return returnValue;
      
}

int16_t NCustomInputFunction() 
 {
  
  //read sensors or do stuff
    sensors_event_t event;
    bmp.getEvent( & event);
  
    // Send the value to homotica in hectopascals.
    int16_t returnValue = (event.pressure);
    return returnValue;
    
  } 

void emergencybtn()
  {   
   //REDBUTTON EMERGENCY
   previousMillis = millis();
 
   // read the state of the pushbutton value:
   buttonState = digitalRead(buttonPin);

   //The button is pushedz
    while (buttonState == HIGH) 
           

  {

       Serial.println(buttonState);
     
     currentValue = millis() - previousMillis;
    
     if (currentValue > interval)  //If the button has been pressed for over 7 secs, open it
     {          
       // save the last time relays has been turned on
       previousMillis = millis();   
       
       digitalWrite(WDOOR, HIGH);      //opendoor
       delay(4000);     //give time to get in
      digitalWrite(WDOOR, LOW);    //close it
     
     }
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  yield();
    }
  
  }

void gateoff()
{
  // read the state of the pushbutton value:
   buttonstatedoor = digitalRead(GATE);
   //The button is pushed
    if (buttonstatedoor == LOW) 
     {
       Serial.println("Close DA GATE");
       delay (500);
       digitalWrite(GATE, HIGH); 
     }
     
   
}
