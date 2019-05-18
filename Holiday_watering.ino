#include "secrets.h"
#include <DHTesp.h>
#define MQTT_VERSION 3
#include  <WiFiMQTTManager.h>
#include <neotimer.h>

//  Button that will put device into Access Point mode to allow for re-entering WiFi and MQTT settings
#define RESET_BUTTON  12
#define SOIL_EN_PIN   5
#define PUMP_PIN      4
#define DHT_PIN       16
#define RELAY_PIN     14

// warmUp timer, it takes about 300ms to powerup the
// Soil moist capacitive sensor
Neotimer moistWarmUpTimer = Neotimer(300); // 300ms sensor warmup time
Neotimer dbTimer = Neotimer(100); // 100ms de-bounce

DHTesp dht;

WiFiMQTTManager wmm(RESET_BUTTON, AP_PASSWORD);  // AP_PASSWORD is defined in the secrets.h file

unsigned long pumpStartTime;
volatile unsigned long pumpONtime;
boolean pumpON;
boolean relayON;

// heartbeat stuf
long heartBeatArray[] = {50, 100, 15, 1200};
unsigned int hbeatIndex = 1;
long hbeatPrevMillis = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Watering system initialized"));
  // set debug to true to get verbose logging
  // wm.wm.setDebugOutput(true);
  // most likely need to format FS but only on first use
  // wmm.formatFS = true;

  // optional - define the function that will subscribe to topics if needed
  wmm.subscribeTo = subscribeTo;

  // required - allow WiFiMQTTManager to do it's setup
  wmm.setup(__SKETCH_NAME__);

  // optional - define a callback to handle incoming messages from MQTT
  //wmm.client->setCallback(subscriptionCallback);

  // Initialize pump
  pumpONtime = 0;
  pumpON = false;
  relayON = false;
  pinMode(PUMP_PIN,OUTPUT);
  digitalWrite(PUMP_PIN,LOW);
  pinMode(RELAY_PIN,INPUT_PULLUP);
  dbTimer.start();

  // Turn LED off
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  // Initalize Soil moist sensor On/Off pwr via transistor
  pinMode(SOIL_EN_PIN,OUTPUT);
  digitalWrite(SOIL_EN_PIN,LOW);

  // Initialize DHT22
  dht.setup(DHT_PIN, DHTesp::DHT22); // Connect DHT sensor to GPIO 17

}

void loop() {
  // required - allow WiFiMQTTManager to check for new MQTT messages,
  // check for reset button push, and reconnect to MQTT if necessary
  wmm.loop();

  heartBeat(1.5);

  if (digitalRead(RELAY_PIN) == LOW) {
    if (!relayON) {
      relayON = true;
      publish2mqtt("relay","ON");
    }
  } else {
    if (relayON) {
      relayON = false;
      publish2mqtt("relay","OFF");
    }
  }
  // read sensors every second (change it to minute)
  unsigned long now = millis();
  if (now - wmm.lastMsg > 60000) {
    stopHeartBeat();
    wmm.lastMsg = now;

    // Power-up the sensor
    digitalWrite(SOIL_EN_PIN,HIGH);
    // start WarmUp for soil mosit sensor
    moistWarmUpTimer.start();

    // read DHT22
    Serial.println(F("Publish DHT readings"));
    char c_tmp[10];
    publish2mqtt("status",dht.getStatusString());
    snprintf(c_tmp,sizeof(c_tmp),"%.2f",dht.getHumidity());
    publish2mqtt("humidity",c_tmp);
    snprintf(c_tmp,sizeof(c_tmp),"%.2f",dht.getTemperature());
    publish2mqtt("temperature",c_tmp);
  }

  // Check capacitive soil moist sensor
  if(moistWarmUpTimer.done()){
      // Stop the warm-up timer
      moistWarmUpTimer.stop();

      // read soil moist sensor
      unsigned int moist = analogRead(A0);
      delay(10); // double take readings
      moist += analogRead(A0);

      // power down to prevent corosion
      digitalWrite(SOIL_EN_PIN,LOW);
      moist /= 2;

      Serial.println(F("Publish soil moist reading:"));
      char c_tmp[10];
      snprintf(c_tmp,sizeof(c_tmp),"%d",moist);
	  publish2mqtt("moist",c_tmp);
  }

  // turn ON/OFF the water pump
  if (pumpON && (now - pumpStartTime > pumpONtime)) {
    pumpON = false;
    pumpONtime = 0;
    Serial.println(F("turn the pump OFF"));
    digitalWrite(PUMP_PIN,LOW);
  }
  if (!pumpON && (pumpONtime > 1000)) {
    pumpON = true;
    pumpStartTime = now;
    Serial.println(F("turn the pump ON"));
    digitalWrite(PUMP_PIN,HIGH);
  }
}

// subscription callback routine is registered by WiFiMQTTManager
void subscriptionCallback(char* topic, byte* message, unsigned int length) {
  Serial.print(F("Message arrived on topic: "));
  Serial.println(topic);
  Serial.print(F(" Payload: "));
  char c_tmp[10];
  strncpy(c_tmp,(char *)message,length);
  c_tmp[length] = '\0';
  Serial.println(c_tmp);

  // no need to check topic as it is the only one :-)
  pumpONtime = atoi(c_tmp) * 1000;
} // subscriptionCallback()

// optional function to subscribe to MQTT topics
void subscribeTo() {
  Serial.println(F("subscribing to pump switch"));
  char topic[30];
  snprintf(topic, sizeof(topic), "%s%s%s", "switch/", wmm.friendlyName, "/pump");
  wmm.client->subscribe(topic);
}


void publish2mqtt(const char* topic, const char* payload){
      char c_tmp[100];
      strcpy(c_tmp,"sensor/");
      strcat(c_tmp,wmm.friendlyName);
      strcat(c_tmp,"/");
      strcat(c_tmp,topic);
      Serial.print(c_tmp);
      Serial.print(" ");
      Serial.println(payload);
      wmm.client->publish(c_tmp, payload);
}

void heartBeat(float tempo){
    if ((millis() - hbeatPrevMillis) > (long)(heartBeatArray[hbeatIndex] * tempo)) {
        hbeatIndex++;
        if (hbeatIndex > 3) hbeatIndex = 0;

        if ((hbeatIndex % 2) == 0){     // modulo 2 operator will be true on even counts
            digitalWrite(LED_BUILTIN, LOW);
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
        }
        //  Serial.println(hbeatIndex);
        hbeatPrevMillis = millis();
    }
}

void stopHeartBeat(){
    digitalWrite(LED_BUILTIN, HIGH);
}
