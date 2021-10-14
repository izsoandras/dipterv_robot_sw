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
#include "wheel_control.h"
#include "position_control.h"
#include "esp_timer.h"
#include "fusion.h"

const char *ssid = "OmniBot";
const char *password = "omnibot4";

// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient);

Battery batt;

motor_config_t mot1_conf = {
    .dirA_pin = pinout::mot1_dirA,
    .dirB_pin = pinout::mot1_dirB,
    .PWM_pin = pinout::mot1_PWM,
  };

Motor mot1(mot1_conf, LEDC_CHANNEL_0, 5000, 10);

motor_config_t mot2_conf = {
    .dirA_pin = pinout::mot2_dirA,
    .dirB_pin = pinout::mot2_dirB,
    .PWM_pin = pinout::mot2_PWM,
  };

Motor mot2(mot2_conf, LEDC_CHANNEL_1, 5000, 10);

motor_config_t mot3_conf = {
    .dirA_pin = pinout::mot3_dirA,
    .dirB_pin = pinout::mot3_dirB,
    .PWM_pin = pinout::mot3_PWM,
  };

Motor mot3(mot3_conf, LEDC_CHANNEL_2, 5000, 10);

Encoder enc1(pinout::mot1_encB, PCNT_UNIT_0);
Encoder enc2(pinout::mot2_encB, PCNT_UNIT_1);
Encoder enc3(pinout::mot3_encB, PCNT_UNIT_2);

void updateValuesSlow(void* params);

void communicationTask(void* params);

void sendSpeed(void* params);

void param_handler(const char topic[], byte* payload, unsigned int length);
void camera_handler(const char topic[], byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");

  batt.init();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  mot1.init();
  mot2.init();
  mot3.init();
  motors[0] = &mot1;
  motors[1] = &mot2;
  motors[2] = &mot3;

  enc1.init();
  enc2.init();
  enc3.init();
  enc1.resume();
  enc2.resume();
  enc3.resume();
  encoders[0] = &enc1;
  encoders[1] = &enc2;
  encoders[2] = &enc3;
  
  WiFi.softAP(ssid, password, 1, false, 1);
  

  mqttClient.add_calback(param_handler);
  mqttClient.init();
  ESP_LOGI("mqttc","MQTT initialized");
  while(!mqttClient.isConnected()){
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI("mqttc","MQTT connected");
  while(!mqttClient.subscribe("param")){
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI("mqttc","MQTT subscribed");

  xTaskCreatePinnedToCore( communicationTask,
                           "Communication",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

  xTaskCreatePinnedToCore( sendSpeed,
                           "SendSpeed",
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
  
  xTaskCreatePinnedToCore( wheel_control,
                           "wheel control",
                           2000,
                           NULL,
                           2,
                           NULL,
                           1);

  xTaskCreatePinnedToCore(position_control_task,
                          "post_control",
                          2000,
                          NULL,
                          2,
                          NULL,
                          1); 

  ESP_LOGI("Tasks launched");

  ESP_LOGI("Setup finished");
}

void loop() {
  // put your main code here, to run repeatedly:
   //ESP_LOGW("ENC","Pulse cound: %d", ident_enc.getCountReset());
  mqttClient.loop();
  vTaskDelay(pdMS_TO_TICKS(100));
}

void updateValuesSlow(void* params){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  while(true){
    batt.updateVoltage();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sendSpeed(void* params){
  while(true){
    if(mqttClient.isConnected()){
      for(int i = 0; i < 3; i++){
        mqttClient.publishMotSpeed(i, current_speeds[i], speed_setpoints[i]);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}


void communicationTask(void* params){
  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    }
//    Serial.println(batt.getVoltage());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void param_handler(const char topic[], byte* payload, unsigned int length){
  ESP_LOGI(topic,"%s",topic);
  if(strcmp(topic,"param") == 0){
    // Wheel setpoint received
    switch(payload[1]){
      case 0xA4:
        //memcpy(&speed_setpoints, payload + 2, 12);
        ESP_LOGW("wheel", "%f,/t%f,/t%f", speed_setpoints[0], speed_setpoints[1], speed_setpoints[2]);
        break;
      case 0xA9:
        memcpy(&control_vec, payload + 2, 12);
        break;
      case 0xB0:
        memcpy(&ref_vec, payload+2, 12);
        break;
      default:
        ESP_LOGW("MQTT", "Unknown type on params");
        break;
    }
  }
}

void camera_handler(const char topic[], byte* payload, unsigned int length){
  if(strcmp(topic, "cps") == 0){

    memcpy(cam_vec, &payload[3], 3*sizeof(float));
  }
}