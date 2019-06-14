/*
  Thanks much to Bruh Automation, large parts are based on your work.
   
*/



#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h> 


/************ Configuration for network and security ******************/

#define wifi_ssid "HAPrototype"                       // type your WIFI name in here
#define wifi_password "prototype"                     // your WIFI password
#define mqtt_server "192.168.1.211"                   // the IP of your mqtt broker, e.g. from your Raspbarry Pi running Hassio and the MQTT broker
#define mqtt_user "stefan"                            // your mqtt user name, in the easiest case your Hassio username
#define mqtt_password "super53cret"                   // your mqtt user password, in the easiest case your Hassio password
#define OTApassword "abcdefghijklmnopqrst"            // defined by you! and used to upload new firmware from the Arduino-Software (FOTA: Firmware Over The Air)

/************ Device configuration ******************/

#define SENSORNAME "myGSA1"                           // unique device ID for the gsa

boolean digital1opener = true;                        // defines, if digital 1 is opener or closer
boolean digital2opener = true;                        // defines, if digital 2 is opener or closer

boolean digitalIndicatorLED1 = true;                  // if true show indicator light for digital 1
boolean digitalIndicatorLED2 = true;                  // if true show indicator light for digital 1

boolean relay1 = false;                               // if true, relay can be used, if false analog output can be used (if false, don't connect a relais, it will be damaged by the pwm output!!)
boolean relay2 = false;                               // if true, relay can be used, if false analog output can be used (if false, don't connect a relais, it will be damaged by the pwm output!!)

boolean servo = true;                                // if true, servo can be used, if false analog output can be used (if false, don't connect a servo, it may get damaged by the pwm output!!)

boolean showCommunicationFlash = false;               // if true led flashes on data transmission

float diffAnalog = 20;                                // On how much difference should the analog sensore react on?



/************* specific mqtt and fota configs - nothing to do from here on  **************************/

#define mqtt_port 1883                                // default 1883 – you don't have to change it
int OTAport = 8266;                                   // default 8266 ;-) – you don't have to change it

/************* MQTT TOPICS (getting assembled later using SENSORNAME)  **************************/

char light_state_topic[80];
char light_set_topic[80];

char aout1_state_topic[80];
char aout1_set_topic[80];

char aout2_state_topic[80];
char aout2_set_topic[80];

const char* on_cmd = "ON";
const char* off_cmd = "OFF";

/**************************** PIN DEFINITIONS ********************************************/

const int RGBPIN = D3;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, RGBPIN, NEO_GRB + NEO_KHZ800);

#define PIRPIN    D8  // lieber als output nutzen
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define ANALOGPIN    A0

#define AOUT1     D5
#define AOUT2     D6

#define DIN1      D1
#define DIN2      D2
#define DIN1LED   D4                               // showing DIN1 status directly
#define DIN2LED   D0                               // showing DIN2 status directly

/**************************** SENSOR DEFINITIONS *******************************************/
float analogValue;
int analog;
float calcAnalog;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

Servo myServo;

/******************************** GLOBALS for fade/flash *******************************/

long lastMsg = 0;
int delayTime = 20000;                                // time until updates are published

byte red = 255;
byte green = 255;
byte blue = 255;
byte brightness = 255;

byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = false;
bool aout1StateOn = false;
bool aout2StateOn = false;

// last states
int digitalIn1State = 0;
int digitalIn2State = 0;
int digitalIn1lastState = 0;
int digitalIn2lastState = 0;


bool startFade = false;
unsigned long lastLoop = 0;
int transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int redVal, grnVal, bluVal;

bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashBrightness = brightness;

byte analogOut1Value = 0;
byte analogOut2Value = 0;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

/********************************** Generate Topic Names etc from myThingID *****************************************/
void generateThingID() {
  //snprintf(SENSORNAME, 200, "%s", myThingID);
  //SENSORNAME
  Serial.print("Sensorname: "); Serial.println(SENSORNAME);
  //snprintf(light_state_topic, 200, "%s", myThingID);
  //light_state_topic = SENSORNAME;
  //light_set_topic = String(light_set_topic)+"/set");
  //snprintf(light_set_topic, 200, "%s/%s", myThingID, "set");
  //  light_state_topic = SENSORNAME;
  strcat(light_state_topic, SENSORNAME);
  strcat(light_set_topic, SENSORNAME);
  strcat(light_set_topic, "/set");
  Serial.print("light_state_topic: "); Serial.println(light_state_topic);
  Serial.print("light_set_topic: "); Serial.println(light_set_topic);

  strcat(aout1_state_topic, SENSORNAME);
  strcat(aout1_state_topic, "/aout1");

  strcat(aout1_set_topic, SENSORNAME);
  strcat(aout1_set_topic, "/aout1/set");

  strcat(aout2_state_topic, SENSORNAME);
  strcat(aout2_state_topic, "/aout2");

  strcat(aout2_set_topic, SENSORNAME);
  strcat(aout2_set_topic, "/aout2/set");


  /*  snprintf(aout1_state_topic, 200, "%s/aout1", SENSORNAME);
    snprintf(aout1_set_topic, 200, "%s/aout1/%s", SENSORNAME, "set");

    snprintf(aout2_state_topic, 200, "%s/aout2", SENSORNAME);
    snprintf(aout2_set_topic, 200, "%s/aout2/%s", SENSORNAME, "set");*/
}


/********************************** START SETUP *****************************************/
void setup() {

  Serial.begin(115200);

  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(ANALOGPIN, INPUT);
  pinMode(RGBPIN, INPUT);

  pinMode(DIN1, INPUT_PULLUP);
  pinMode(DIN2, INPUT_PULLUP);
  pinMode(DIN1LED, OUTPUT);
  pinMode(DIN2LED, OUTPUT);

  if (relay1 == true) pinMode(AOUT1, OUTPUT);
  if (relay2 == true) pinMode(AOUT2, OUTPUT);

  if (servo == true) {
    
    myServo.attach(AOUT2);
  }

  strip.begin();

  generateThingID();

  delay(10);

  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(SENSORNAME);
  ArduinoOTA.setPassword((const char *)OTApassword);

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Starting Node named " + String(SENSORNAME));


  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);


  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());
  reconnect();
}




/********************************** START SETUP WIFI*****************************************/
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}



/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strcmp(topic, light_set_topic) == 0) {
    char message[length + 1];
    for (int i = 0; i < length; i++) {
      message[i] = (char)payload[i];
    }
    message[length] = '\0';
    Serial.println(message);

    if (!processJson(message)) {
      return;
    }

    if (stateOn) {
      // Update lights
      realRed = map(red, 0, 255, 0, brightness);
      realGreen = map(green, 0, 255, 0, brightness);
      realBlue = map(blue, 0, 255, 0, brightness);
    }
    else {
      realRed = 0;
      realGreen = 0;
      realBlue = 0;
    }

    startFade = true;
    inFade = false; // Kill the current fade

    sendState();

  }

  if (strcmp(topic, aout1_set_topic) == 0) {
    char message[length + 1];
    for (int i = 0; i < length; i++) {
      message[i] = (char)payload[i];
    }
    message[length] = '\0';
    Serial.println(message);


    if (!processJsonAout1(message)) {
      return;
    }

    sendStateAnalog1();
  }

  if (strcmp(topic, aout2_set_topic) == 0) {
    char message[length + 1];
    for (int i = 0; i < length; i++) {
      message[i] = (char)payload[i];
    }
    message[length] = '\0';
    Serial.println(message);


    if (!processJsonAout2(message)) {
      return;
    }

    sendStateAnalog2();
  }
}



/********************************** START PROCESS JSON*****************************************/
bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.containsKey("flash")) {
    flashLength = (int)root["flash"] * 1000;

    if (root.containsKey("brightness")) {
      flashBrightness = root["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (root.containsKey("color")) {
      flashRed = root["color"]["r"];
      flashGreen = root["color"]["g"];
      flashBlue = root["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else { // Not flashing
    flash = false;

    if (root.containsKey("color")) {
      red = root["color"]["r"];
      green = root["color"]["g"];
      blue = root["color"]["b"];
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else {
      transitionTime = 0;
    }
  }

  return true;
}

bool processJsonAout1(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      aout1StateOn = true;
      if (root.containsKey("brightness")) {
        if (relay1 == false) analogWrite(AOUT1, root["brightness"]);
        else digitalWrite(AOUT1, HIGH);
      } else {
        if (relay1 == false) analogWrite(AOUT1, 255);
        else digitalWrite(AOUT1, HIGH);
      }
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      aout1StateOn = false;
      if (relay1 == false) analogWrite(AOUT1, 0);
      else digitalWrite(AOUT1, LOW);
    }
  }

  return true;
}

bool processJsonAout2(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      aout2StateOn = true;
      if (root.containsKey("brightness")) {
        if (relay2 == false) {
          if (servo==false) analogWrite(AOUT2, root["brightness"]);
          else myServo.write(map(root["brightness"],0,255,0,170));
        }
        else digitalWrite(AOUT2, HIGH);
      } else {
        if (relay2 == false) {
          if (servo==false) analogWrite(AOUT2, 255);
          else myServo.write(170);
        }
        else digitalWrite(AOUT2, HIGH);
      }
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      aout2StateOn = false;
      if (relay2 == false) {
        if (servo==false) analogWrite(AOUT2, 0);
        else myServo.write(0);
      }
      else digitalWrite(AOUT2, LOW);
    }
  }

  return true;
}

/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  JsonObject& color = root.createNestedObject("color");
  color["r"] = red;
  color["g"] = green;
  color["b"] = blue;


  root["brightness"] = brightness;
  root["humidity"] = (String)humValue;
  root["motion"] = (String)motionStatus;
  root["analog"] = (String)analog;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)calculateHeatIndex(humValue, tempValue);

  root["digital1"] = (String)digitalIn1State;
  root["digital2"] = (String)digitalIn2State;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(light_state_topic, buffer, true);
}

void sendStateAnalog1() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (aout1StateOn) ? on_cmd : off_cmd;

  root["brightness"] = brightness;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(aout1_state_topic, buffer, true);
}

void sendStateAnalog2() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (aout2StateOn) ? on_cmd : off_cmd;

  root["brightness"] = brightness;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(aout2_state_topic, buffer, true);
}
/*
   Calculate Heat Index value AKA "Real Feel"
   NOAA heat index calculations taken from
   http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
*/
float calculateHeatIndex(float humidity, float temp) {
  float heatIndex = 0;
  if (temp >= 80) {
    heatIndex = -42.379 + 2.04901523 * temp + 10.14333127 * humidity;
    heatIndex = heatIndex - .22475541 * temp * humidity - .00683783 * temp * temp;
    heatIndex = heatIndex - .05481717 * humidity * humidity + .00122874 * temp * temp * humidity;
    heatIndex = heatIndex + .00085282 * temp * humidity * humidity - .00000199 * temp * temp * humidity * humidity;
  } else {
    heatIndex = 0.5 * (temp + 61.0 + ((temp - 68.0) * 1.2) + (humidity * 0.094));
  }

  if (humidity < 13 && 80 <= temp <= 112) {
    float adjustment = ((13 - humidity) / 4) * sqrt((17 - abs(temp - 95.)) / 17);
    heatIndex = heatIndex - adjustment;
  }

  return heatIndex;
}


/********************************** START SET COLOR *****************************************/
void setColor(int inR, int inG, int inB) {

  /*analogWrite(redPin, inR);
    analogWrite(greenPin, inG);
    analogWrite(bluePin, inB);*/
  strip.setPixelColor(0, strip.Color(inG, inR, inB));
  strip.show();

  Serial.println("Setting LEDs:");
  Serial.print("r: ");
  Serial.print(inR);
  Serial.print(", g: ");
  Serial.print(inG);
  Serial.print(", b: ");
  Serial.println(inB);
}



/********************************** START RECONNECT*****************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SENSORNAME, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(light_set_topic);
      client.subscribe(aout1_set_topic);
      client.subscribe(aout2_set_topic);
      setColor(0, 0, 0);
      sendState();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for (int i = 0; i < 50; i++) {
        digitalWrite(BUILTIN_LED, LOW);
        delay(1);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(200);
      }
    }
  }
}



/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/********************************** Check Buttons ***************************************/

void getDigitalInStatus() {                                         // gets the status of the digital ins
  digitalIn1lastState = digitalRead(DIN1);
  digitalIn2lastState = digitalRead(DIN2);

  if (digitalRead(DIN1) == HIGH) {
    if (digital1opener == true) digitalIn1State = LOW;
    else digitalIn1State = HIGH;
  } else {
    if (digital1opener == true) digitalIn1State = HIGH;
    else digitalIn1State = LOW;
  }

  if (digitalRead(DIN2) == HIGH) {
    if (digital2opener == true) digitalIn2State = LOW;
    else digitalIn2State = HIGH;
  } else {
    if (digital2opener == true) digitalIn2State = HIGH;
    else digitalIn2State = LOW;
  }
  // Serial.print(digitalIn1State);Serial.print("\t");Serial.println(digitalIn2State);
}

void sendDigitalInStatus() {
  if (digitalIn1State == 1) {
    if (digitalIndicatorLED1 == true) digitalWrite(DIN1LED, LOW);
    //publishMessage("digitalIn1", "ON");
  } else {
    if (digitalIndicatorLED1 == true) digitalWrite(DIN1LED, HIGH);
    //publishMessage("digitalIn1", "OFF");
  }

  if (digitalIn2State == 1) {
    if (digitalIndicatorLED2 == true) digitalWrite(DIN2LED, LOW);
    //publishMessage("digitalIn2", "ON");
  } else {
    if (digitalIndicatorLED2 == true) digitalWrite(DIN2LED, HIGH);
    //publishMessage("digitalIn2", "OFF");
  }
  sendState();
}



/********************************** START MAIN LOOP***************************************/
void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    // reconnect();
    software_Reset();
  }
  client.loop();

  long now = millis();

  if (now - delayTime > lastMsg) {                                    // after timeout
    digitalWrite(BUILTIN_LED, LOW);                                   // show that data will get transmitted
    delay(1);
    digitalWrite(BUILTIN_LED, HIGH);
    lastMsg = now;                                                    // timer reset

    getDigitalInStatus();
    sendDigitalInStatus();
  }

  // publish message on state change buttons
  if ((digitalRead(DIN1) != digitalIn1lastState) || (digitalRead(DIN2) != digitalIn2lastState)) {
    getDigitalInStatus();
    sendDigitalInStatus();
  }

  if (!inFade) {

    float newTempValue = dht.readTemperature(); //to use celsius remove the true text inside the parentheses
    float newHumValue = dht.readHumidity();

    //PIR CODE
    pirValue = digitalRead(PIRPIN); //read state of the
    //Serial.print("PIR = ");Serial.println(pirValue);
    if (pirValue == LOW && pirStatus != 1) {
      motionStatus = "standby";
      sendState();
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2) {
      motionStatus = "motion detected";
      sendState();
      pirStatus = 2;
    }

    delay(100);

    if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
      tempValue = newTempValue;
      sendState();
    }

    if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
      humValue = newHumValue;
      sendState();
    }


    int newAnalog = analogRead(ANALOGPIN);

    if (checkBoundSensor(newAnalog, analog, diffAnalog)) {
      analog = newAnalog;
      sendState();
    }

  }

  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue);
      }
      else {
        setColor(0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue);
    }
  }

  if (startFade) {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        setColor(redVal, grnVal, bluVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.println(loopCount);
        loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }
}




/**************************** START TRANSITION FADER *****************************************/
// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS

  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:

    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -

  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.

  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).

  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;          //   divide by 1020
  }

  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }

  return val;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  ESP.reset();
}
