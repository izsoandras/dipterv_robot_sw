#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "pinout.h"
#include "mymqtt.h"
#include "battery.h"
#include "esp_log.h"
#include "Wire.h"
#include "VL53L0X.h"


const char *ssid = "OmniBot";
const char *password = "myomni04";


// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient);

Battery batt;

VL53L0X dist_sensor = VL53L0X();
uint16_t distance = 0;


void updateBatt(void* params);

void updateDistance(void* params);
void communicationTask(void* params);
void sendDistance(void* params);

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");
  

  batt.init();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  WiFi.softAP(ssid, password, 1, false, 1);
  mqttClient.init();
    
  ESP_LOGI("setup","MQTT set up");

  Wire.begin();
  dist_sensor.setTimeout(500);
  if(!dist_sensor.init()){
      ESP_LOGE("setup","Sensor set up failed");
  }
  else{
    dist_sensor.setMeasurementTimingBudget(200000);
    ESP_LOGI("setup","Sensor set up");
  }

//   dist_sensor.setMeasurementTimingBudget(200000);

  xTaskCreatePinnedToCore( communicationTask,
                           "Communication",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

  xTaskCreatePinnedToCore( updateBatt,
                           "Slow update",
                           1000,
                           NULL,
                           2,
                           NULL,
                           1);

  xTaskCreatePinnedToCore( updateDistance,
                           "Distance",
                           2000,
                           NULL,
                           5,
                           NULL,
                           1);

xTaskCreatePinnedToCore( sendDistance,
                           "Distance send",
                           2000,
                           NULL,
                           2,
                           NULL,
                           0);
  
  ESP_LOGI("setup","Setup finished");
}

void loop() {
  // put your main code here, to run repeatedly:
//   vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // distance = dist_sensor.readRangeSingleMillimeters();
    //     ESP_LOGW("DIST","Distance: %d", distance);
    
    
    mqttClient.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void updateBatt(void* params){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(5000);

  while(true){
    batt.updateVoltage();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateDistance(void* params){
   TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);

  while(true){
    distance = dist_sensor.readRangeSingleMillimeters();
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    vTaskDelay(pdMS_TO_TICKS(500));
  } 
}

void sendDistance(void* params){
    while(true){
        mqttClient.publish_u16("tel", 0xA3, distance);
        ESP_LOGW("DIST","Distance: %d", distance);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void communicationTask(void* params){
  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    } else{
      mqttClient.reconnect();
    }
//    Serial.println(batt.getVoltage());
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}