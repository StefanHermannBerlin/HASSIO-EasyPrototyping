void setup_wifi() {
  // connect to wifi
  delay(10);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Attempt to connect
    if (client.connect(myThingID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      snprintf (myTopic, 200, "%s/%s/#", myThingID, "incoming");
      client.subscribe(myTopic);
      //client.subscribe("#");
    } else {
      // Wait 5 seconds before retrying
      for (int i = 0; i < 50; i++) {
        digitalWrite(BUILTIN_LED, LOW);
        delay(100);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(100);
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strstr(topic, "analogOut1Status") != NULL) {
    // mosquitto_pub -V mqttv311 -h 192.168.178.28 -t '/testsensor2/incoming/analogOut1Status/' -m '0'
    String myPayload = "";
    for (int i = 0; i < length; i++) {
      myPayload += (char)payload[i];
    }
    //Serial.print("Analog Out 1 Value = "); Serial.println(atoi(myPayload.c_str()));
    analogWrite(analogOut1Pin, atoi(myPayload.c_str()));
  }

  if (strstr(topic, "analogOut2Status") != NULL) {
    String myPayload = "";
    for (int i = 0; i < length; i++) {
      myPayload += (char)payload[i];
    }
    //Serial.print("Analog Out 2 Value = "); Serial.println(atoi(myPayload.c_str()));
    analogWrite(analogOut2Pin, atoi(myPayload.c_str()));
  }

  if (strstr(topic, "rgb/set") != NULL) {
    // mosquitto_pub -V mqttv311 -h 192.168.178.28 -t '/testsensor2/incoming/rgb/set' -m '{"state": "ON", "color": {"r": 255, "g": 67, "b": 111}, "brightness": 100}'
    char myPayload[length];
    for (int i = 0; i < length; i++) {
      myPayload[i] = (char)payload[i];
    }
    parseRGB(myPayload);
  }

/*  not done yet!
    if (strstr(topic, "alert") != NULL) {
    String myPayload = "";
    for (int i = 0; i < length; i++) {
      myPayload += (char)payload[i];
    }
    //analogWrite(servoPin, atoi(myPayload.c_str()));
  }*/

  /*Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();*/
}

void parseRGB(char theJson[]) {
  // method takes json and applies it to NeoPixel RGB LED (position 0)
  StaticJsonBuffer<200> jsonBuffer;
  //Example String -> 
  //char json[] = "{\"state\": \"ON\", \"color\": {\"r\": 255, \"g\": 67, \"b\": 111}, \"brightness\": 100}";
  JsonObject& root = jsonBuffer.parseObject(theJson);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }

  String state = root["state"];

  if (state == "ON") {
    int brightness = root["brightness"];
    int red = root["color"]["r"];
    int green = root["color"]["g"];
    int blue = root["color"]["b"];
    strip.setPixelColor(0, strip.Color(green, red, blue, brightness));
    //Serial.println("RGB ON");
  } else {
    strip.setPixelColor(0, strip.Color(0, 0, 0, 0));
    //Serial.println("RGB OFF");
  }
  strip.show();
}

void publishMessage(const char* theTopic, long theMessage) {
  snprintf (myTopic, 100, "%s/%s/", myThingID, theTopic);
  snprintf (myMessage, 100, "%d", theMessage);
  client.publish(myTopic, myMessage);
  delay(10);
}

void publishMessage(const char* theTopic, String theMessage) {
  snprintf (myTopic, 100, "%s/%s/", myThingID, theTopic);
  theMessage.toCharArray(myMessage, 100);
  client.publish(myTopic, myMessage);
  delay(10);
}
