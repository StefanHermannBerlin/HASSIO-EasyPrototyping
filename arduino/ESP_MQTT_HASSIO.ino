#include <ESP8266WiFi.h>                                 // general library for ESP8266 module
#include <WiFiClient.h>                                  // wifi library with TLS
#include <PubSubClient.h>
#include <DHT.h>                                         // library for DHT22 humidity and temparature sensor
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif


/* Configuration */
const char* myThingID = "/eingang";               // unique device ID (can be a UUID)
//const char* myThingID = "/garageBackdoor";          // unique device ID (can be a UUID)
//const char* myThingID = "/garageFrontdoor";         // unique device ID (can be a UUID)

const char* ssid = "Falcon";                          // your SSID
const char* password = "warpspeed";                   // your password
const char* mqtt_server = "192.168.178.28";           // IP of your MQTT broker
const uint16_t mqtt_port = 1883;                      // Port of your MQTT broker, usually 1883

int delayTime = 20000;                                // time until updates are published
boolean dthSensor = true;                             // if DHT is attached
int analogThreshold = 50;                             // the value change needed to trigger a signal send, good against flickering of the values

/* Stuff for runtime */
WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN D7                                     // the pin, the DHT22 is connected to
#define DHTTYPE DHT22                                 // the type of the humidity and temparature sensor (DHT22)
DHT dht(DHTPIN, DHTTYPE);                             // initialising humidity and temparature sensor

long lastMsg = 0;
char myMessage[100];
char myTopic[100];
int value = 0;

int digitalIn1Pin = D1;
int digitalIn2Pin = D2;
int directLed1Pin = D4;                               // showing digitalIn1Pin status directly
int directLed2Pin = D0;                               // showing digitalIn2Pin status directly
int rgbPin = D3;                                      // ws2812
int analogInPin = A0;                 
int analogOut1Pin = D5;  // relais?
int analogOut2Pin = D6;  // relais?
// alert on D8?

// last states
int digitalIn1ls = 0;
int digitalIn2ls = 0;
int lastAnalog = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, rgbPin, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(digitalIn1Pin, INPUT_PULLUP);
  pinMode(digitalIn2Pin, INPUT_PULLUP);
  pinMode(directLed1Pin, OUTPUT);
  pinMode(directLed2Pin, OUTPUT);
  pinMode(rgbPin, OUTPUT);
  pinMode(analogOut1Pin, OUTPUT);
  pinMode(analogOut2Pin, OUTPUT);
  delay(500);
  strip.begin();
  strip.setPixelColor(0, strip.Color(0, 0, 0, 0));
  strip.show();
}

void loop() {
  if (!client.connected()) {                                          // reconnect on lost connection
    Serial.print(millis()); Serial.println("Connection lost, reconnecting ...");
    reconnect();
  }
  
  client.loop();                                                      // nessesary for the callback (receiving messages)
  
  long now = millis();

  if (now - delayTime > lastMsg) {                                    // after timeout
    digitalWrite(BUILTIN_LED, LOW);                                   // show that data will get transmitted
    delay(10);
    digitalWrite(BUILTIN_LED, HIGH);
    lastMsg = now;                                                    // timer reset
    
                                                                  // send status of buttons
    if (digitalRead(digitalIn1Pin) == HIGH) {                         // if button 1 not pressed
      publishMessage("digitalIn1", "OFF");                            // send button 1 status
      digitalWrite(directLed1Pin, LOW);                              // switch LED 1
    } else {
      publishMessage("digitalIn1", "ON");                             // send button 1 status
      digitalWrite(directLed1Pin, HIGH);                               // switch LED 1
    }

    if (digitalRead(digitalIn2Pin) == HIGH) {                         // if button 2 not pressed
      publishMessage("digitalIn2", "OFF");                            // send button 2 status
      digitalWrite(directLed2Pin, LOW);                              // switch LED 2
    } else {
      publishMessage("digitalIn2", "ON");                             // send button 2 status
      digitalWrite(directLed2Pin, HIGH);                               // switch LED 2
    }
                                                                    // send status of analog and runtime
    publishMessage("analogIn", analogRead(analogInPin));              // publish analog In pin
    publishMessage("runtime", millis());                              // publish runtime

                      
    if (dthSensor == true) {                                      // if dht sensor connected, read and publish data
      float theTemperature = dht.readTemperature();                   // read and store temparature
      float theHumidity = dht.readHumidity();                         // read and store humidity

      if (isnan(theHumidity) || isnan(theTemperature)) {              // if data is invalid
        Serial.println("Failed to read from DHT sensor!");            // serial output
      } else {
        publishMessage("temperature", theTemperature);                // publish temperature
        publishMessage("humidity", theHumidity);                      // publish humidity
      }
    }
  }

                                                                  // publish message on state change buttons
  int digitalIn1 = digitalRead(digitalIn1Pin);                        
  if (digitalIn1 != digitalIn1ls) {
    digitalIn1ls = digitalIn1;
    if (digitalRead(digitalIn1Pin) == HIGH) {
      publishMessage("digitalIn1", "OFF");
      digitalWrite(directLed1Pin, LOW);
    } else {
      publishMessage("digitalIn1", "ON");
      digitalWrite(directLed1Pin, HIGH);
    }
  }

  // publish message on state change
  int digitalIn2 = digitalRead(digitalIn2Pin);
  if (digitalIn2 != digitalIn2ls) {
    digitalIn2ls = digitalIn2;
    if (digitalRead(digitalIn2Pin) == HIGH) {
      publishMessage("digitalIn2", "OFF");
      digitalWrite(directLed2Pin, LOW);
    } else {
      publishMessage("digitalIn2", "ON");
      digitalWrite(directLed2Pin, HIGH);
    }
  }

                                                                  // publish message on state change analog input
  int analogIn = analogRead(analogInPin);
  if ((analogIn > lastAnalog - analogThreshold) && (analogIn < lastAnalog + analogThreshold)) {
    // no change
  } else {
    publishMessage("analogIn", analogIn);
    lastAnalog = analogIn;
  }
  delay(10);                                                        // time to chill (don't remove!!)
}
