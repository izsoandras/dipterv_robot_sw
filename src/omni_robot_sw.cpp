//
// ESP32 Chip model = ESP32-D0WDQ6 Rev 1
// This chip has 2 cores
// Chip ID: 5083536

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "pinout.h"
#include "mymqtt.h"
#include "battery.h"
#include "esp_log.h"
#include "motor.h"


const char *ssid = "OmniBot";
const char *password = "myomni04";


// MQTT structures.
//WiFiClient wifiClient;
//OmniMQTTclient mqttClient("192.168.4.2", 1883, wifiClient);

//Battery batt;

motor_config_t mot_conf = {
    .dirA_pin = pinout::mot3_dirA,
    .dirB_pin = pinout::mot3_dirB,
    .PWM_pin = pinout::mot3_PWM,
  };

Motor mot1(mot_conf, LEDC_CHANNEL_0, 100, 8);

//void updateValuesSlow(void* params);

//void communicationTask(void* params);

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");
  
  /*
  pinMode(pinout::mot2_dirA, OUTPUT);
  pinMode(pinout::mot2_dirB, OUTPUT);
  
  digitalWrite(pinout::mot2_dirA, HIGH);
  digitalWrite(pinout::mot2_dirB, LOW);

  ledcSetup(0, 100, 8);
  ledcAttachPin(pinout::mot2_PWM, 0);
  ledcWrite(0, 200);*/

  //batt.init();
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);
  
  //WiFi.softAP(ssid, password, 1, false, 1);
  //mqttClient.init();
  //mqttClient.setCallback();

  mot1.init();


  
/*
  xTaskCreatePinnedToCore( communicationTask,
                           "Communication",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

  xTaskCreatePinnedToCore( updateValuesSlow,
                           "Slow update",
                           1000,
                           NULL,
                           2,
                           NULL,
                           1);*/
  
  Serial.println("Rotating with 70");
  mot1.rotateCCW(70);
}

void loop() {
  // put your main code here, to run repeatedly:
  //mqttClient.loop();
 // vTaskDelay(100 / portTICK_PERIOD_MS);
}
/*
void updateValuesSlow(void* params){
  while(true){
    batt.updateVoltage();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}*/
/*
void communicationTask(void* params){
  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    }
//    Serial.println(batt.getVoltage());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}*/