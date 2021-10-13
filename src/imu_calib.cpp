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
#include "PubSubClient.h"
#include "esp_task.h"
#include "MPU9250.h"

const char *ssid = "OmniBot";
const char *password = "omnibot4";


// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient, 5);

Battery batt;
MPU9250 imu;

uint8_t ledpin = 12;

void updateBattery(void* params);
void sendBattery(void* params);

void ident_mqtt(const char topic[], byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  Serial.println("Identification start");

  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, HIGH);


  batt.init();
  
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

    if (!imu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // Builtin calibration
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    imu.verbose(true);
    delay(5000);
    imu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    imu.calibrateMag();

    print_calibration();
    imu.verbose(false);

    xTaskCreatePinnedToCore( sendBattery,
                           "Send Batt",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

  xTaskCreatePinnedToCore( updateBattery,
                           "Batt meas",
                           1000,
                           NULL,
                           2,
                           NULL,
                           1);
}

void loop() {

    mqttClient.loop();
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

void sendIMU(void* params){  
    while(true){
        imu.update();
        float data[3];
        data[0] = imu.getMagX();
        data[1] = imu.getMagY();
        data[2] = imu.getMagZ();
        mqttClient.publish3float(0x0B,data);
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
      
    }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(imu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(imu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(imu.getMagBiasX());
    Serial.print(", ");
    Serial.print(imu.getMagBiasY());
    Serial.print(", ");
    Serial.print(imu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(imu.getMagScaleX());
    Serial.print(", ");
    Serial.print(imu.getMagScaleY());
    Serial.print(", ");
    Serial.print(imu.getMagScaleZ());
    Serial.println();
}