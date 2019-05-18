#include "secrets.h"
#include <DHTesp.h>
#define MQTT_VERSION 3
#include  <WiFiMQTTManager.h>
#include <neotimer.h>

// Button that will put device into Access Point mode to allow for re-entering WiFi and MQTT settings
#define RESET_BUTTON  0
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
long heartBeatArray[] = {
    50, 100, 15, 1200};
unsigned int hbeatIndex = 1;    // this initialization is not important
long hbeatPrevMillis;

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
  wmm.client->setCallback(subscriptionCallback);

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
      publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/relay","ON");
    }
  } else {
    if (relayON) {
      relayON = false;
      publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/relay","OFF");
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
    String  dhtstatus = dht.getStatusString();
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    Serial.println("DHT22 [status/humidity/temperature]: "+dhtstatus+"/"+String(humidity,2)+"/"+String(temperature,2));
    publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/dhtstatus",String(dhtstatus));
    publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/humidity",String(humidity,2));
    publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/temperature",String(temperature,2));
  }

  // Check capacitive soil moist sensor
  if(moistWarmUpTimer.done()){
      // read soil moist sensor
      unsigned int moist = analogRead(A0);
      delay(10); // double take readings
      moist += analogRead(A0);
      // power down to prevent corosion
      digitalWrite(SOIL_EN_PIN,LOW);
      moist /= 2;

      Serial.println("Soil moist: "+String(moist,DEC));
	    publish2mqtt("sensor/"+ String(wmm.friendly_name)+"/moist",String(moist,DEC));
      moistWarmUpTimer.stop();
  }

  // turn ON/OFF the water pump
  if (pumpON && (now - pumpStartTime > pumpONtime)) {
    pumpON = false;
    pumpONtime = 0;
    Serial.println("turn the pump OFF at "+String(now,DEC));
    digitalWrite(PUMP_PIN,LOW);
  }
  if (!pumpON && (pumpONtime > 1000)) {
    pumpON = true;
    pumpStartTime = now;
    Serial.println("turn the pump ON at "+String(pumpStartTime,DEC));
    digitalWrite(PUMP_PIN,HIGH);
  }

}

// optional function to subscribe to MQTT topics
void subscribeTo() {
  Serial.println(F("subscribing to some topics..."));
  // subscribe to some topic(s)
  char topic[30];
  snprintf(topic, sizeof(topic), "%s%s%s", "switch/", wmm.friendly_name, "/pump");
  wmm.client->subscribe(topic);
//  delete [] topic;
}

// optional function to process MQTT subscribed to topics coming in
void subscriptionCallback(char* topic, byte* message, unsigned int length) {
  Serial.print(F("Message arrived on topic: "));
  Serial.print(topic);
  Serial.print(F(" Payload: "));
  String msg;
//  msg.reserve(length+1);

  for (int i = 0; i < length; i++) {
//    Serial.print((char)message[i]);
    msg += (char)message[i];
  }
  Serial.println(msg);
  pumpONtime = msg.toInt() * 1000;
  Serial.println("Setting pumpONtime to: "+String(pumpONtime,DEC));
} // subscriptionCallback()

void publish2mqtt(String topic, String payload){
      wmm.client->publish(topic.c_str(), payload.c_str(), false);
      Serial.println("publish --> "+topic+" "+payload);
}

void heartBeat(float tempo){
    if ((millis() - hbeatPrevMillis) > (long)(heartBeatArray[hbeatIndex] * tempo)){
        hbeatIndex++;
        if (hbeatIndex > 3) hbeatIndex = 0;

        if ((hbeatIndex % 2) == 0){     // modulo 2 operator will be true on even counts
            digitalWrite(LED_BUILTIN, LOW);
        }
        else{
            digitalWrite(LED_BUILTIN, HIGH);
        }
        //  Serial.println(hbeatIndex);
        hbeatPrevMillis = millis();
    }
}

void stopHeartBeat(){
    digitalWrite(LED_BUILTIN, HIGH);
}
