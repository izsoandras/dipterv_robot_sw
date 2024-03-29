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
#include "PubSubClient.h"
#include "esp_task.h"
#include "WebServer.h"
#include "SPIFFS.h"

const char *ssid = "OmniBot";
const char *password = "omnibot4";

WebServer server(80);


// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient, 5);

Battery batt;

motor_config_t mot_conf = {
    .dirA_pin = pinout::mot3_dirA,
    .dirB_pin = pinout::mot3_dirB,
    .PWM_pin = pinout::mot3_PWM,
  };

Motor mot2ident(mot_conf, LEDC_CHANNEL_0, 5000, 10);
float motor_speed = 0;

Encoder ident_enc(pinout::mot3_encB, PCNT_UNIT_0);

uint8_t ledpin = 12;
bool hasSpace = true;

void updateSpeed(void* params);
void sendSpeed(void* params);
void updateBattery(void* params);
void sendBattery(void* params);

void ident_mqtt(const char topic[], byte* payload, unsigned int length);


int ident_status = 0;
int ident_prev_status = 0;
float ident_duty = 0;
float ident_duty_current = 0;
bool recording = true;
File meas_file;

void handleRoot(){
  if(!recording){
    File file = SPIFFS.open("/meas.csv", FILE_READ);
    server.streamFile(file, "csv");
    file.close();
  }else{
    server.send(200, "text/plain", "Still recording");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Identification start");
  
  SPIFFS.begin(true);
  SPIFFS.format();

  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, HIGH);

  File file = SPIFFS.open("/meas.csv", FILE_WRITE);
  file.println("Duty, Speed");
  file.close();

  meas_file = SPIFFS.open("/meas.csv", FILE_APPEND);


  batt.init();
  mot2ident.init();
  ident_enc.init();
  ident_enc.resume();
  
  WiFi.softAP(ssid, password, 1, false, 1);
  ESP_LOGI("wifi", "WiFi AP started");
  vTaskDelay(pdMS_TO_TICKS(100));
  
  mqttClient.add_calback(ident_mqtt);
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

  server.on("/", handleRoot);
  server.begin();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI("server", "Webserver started");

  mot2ident.rotateCCW(10);
  vTaskDelay(pdMS_TO_TICKS(100));
  mot2ident.stop();
  vTaskDelay(pdMS_TO_TICKS(100));
  mot2ident.rotateCW(10);
  vTaskDelay(pdMS_TO_TICKS(100));
  mot2ident.stop();
  vTaskDelay(pdMS_TO_TICKS(100));

    xTaskCreatePinnedToCore( sendBattery,
                           "Send Batt",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

    xTaskCreatePinnedToCore( sendSpeed,
                           "Send Sped",
                           5000,
                           NULL,
                           1,
                           NULL,
                           0);

xTaskCreatePinnedToCore( updateSpeed,
                           "Speed meas",
                           2000,
                           NULL,
                           3,
                           NULL,
                           1); 

  xTaskCreatePinnedToCore( updateBattery,
                           "Batt meas",
                           1000,
                           NULL,
                           2,
                           NULL,
                           1);
}

void loop() {
     if(ident_status==3){
      meas_file.close();
      recording = false;
    }

    mqttClient.loop();
    server.handleClient();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void updateBattery(void* params){
    TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(5000);

  while(true){
    batt.updateVoltage();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateSpeed(void* params){
    TickType_t xLastWakeTime;
    uint8_t ident_period = 10; //[ms]
    const TickType_t xFrequency = pdMS_TO_TICKS(ident_period);

    while(true){
        // ESP_LOGW("SPDM","speed updated: %d", motor_speed);
        // motor_speed = ident_enc.getCountReset();
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        motor_speed = ident_enc.ticks2rad(ident_enc.getCountReset())/(ident_period/1000.0);
        
        if(recording){
          char row[16];

          if(hasSpace){
            sprintf(row, "%f,%f", ident_duty_current, motor_speed);
            meas_file.println(row);
          }

        }
        
        //ESP_LOGW("ENC","Pulse cound: %f", motor_speed);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void sendSpeed(void* params){  
    while(true){
        //ESP_LOGW("SPD","Pulse cound: %f", motor_speed);
        uint8_t size= 2*sizeof(float);
        uint8_t payload[size];
        memcpy(payload, &ident_duty_current, sizeof(float));
        float f = motor_speed;
        memcpy(payload+sizeof(float), &f, sizeof(float));

        mqttClient.publishData("tel", 0xA1, payload, size);
        int usedBytes = SPIFFS.usedBytes();
        if(usedBytes > 450000){
          hasSpace = false;
          digitalWrite(ledpin, LOW);
        }

        if(ident_status != 3){
          //ESP_LOGI("used","used space: %d", usedBytes);
          ESP_LOGI("status","status: %d, %f", ident_status, ident_duty_current);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void sendBattery(void* params){

  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void ident_mqtt(const char topic[], byte* payload, unsigned int length){
  ESP_LOGI(topic,"%s",topic);
  if(payload[1] == 0xA8 && strcmp(topic, "param") == 0){
      memcpy(&ident_duty_current, payload+2, sizeof(float));
      if(ident_duty_current > 100){
        ident_duty_current = 0;
        ident_status = 3;
      }
      mot2ident.rotateCW(ident_duty_current);
      ESP_LOGW(topic, "%s: %f",topic, ident_duty_current);
    }
}
// void ident_mqtt(const char topic[], byte* payload, unsigned int length){
//     if(strcmp(topic, "ident") == 0){
//       ident_status =  (int)payload[0] - 48; // 48 is ASCII code of 0
//       ESP_LOGW(topic, "%s: %d",topic, ident_status);
//     }
// }