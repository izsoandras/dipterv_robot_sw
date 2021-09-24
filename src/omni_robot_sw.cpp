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
#include "encoder.h"
#include "driver/pcnt.h"


const char *ssid = "OmniBot";
const char *password = "myomni04";


// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient);

Battery batt;

motor_config_t mot_conf = {
    .dirA_pin = pinout::mot2_dirA,
    .dirB_pin = pinout::mot2_dirB,
    .PWM_pin = pinout::mot2_PWM,
  };

Motor mot2ident(mot_conf, LEDC_CHANNEL_0, 100, 8);

Encoder ident_enc(pinout::mot3_encB, PCNT_UNIT_0);

void updateValuesSlow(void* params);

void communicationTask(void* params);

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");
  

  batt.init();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  WiFi.softAP(ssid, password, 1, false, 1);
  mqttClient.init();

  mot2ident.init();
  ident_enc.init();
  ident_enc.resume();

  ledcSetup(LEDC_CHANNEL_1, 200, 8);
  ledcAttachPin(pinout::mot1_encA, LEDC_CHANNEL_1);
  ledcWrite(LEDC_CHANNEL_1, 125);

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
                           1);
  
  Serial.println("Rotating with 70");
  mot2ident.rotateCCW(255);
}

void loop() {
  // put your main code here, to run repeatedly:
   ESP_LOGW("ENC","Pulse cound: %d", ident_enc.getCountReset());
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void updateValuesSlow(void* params){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);

  while(true){
    batt.updateVoltage();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void communicationTask(void* params){
  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    } else{
      mqttClient.reconnect();
    }
    mqttClient.loop();
//    Serial.println(batt.getVoltage());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}