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
#include "imu_settings.h"

#include "wheel_control.h"
#include "position_control.h"

const char *ssid = "OmniBot";
const char *password = "omnibot4";


// MQTT structures.
WiFiClient wifiClient;
OmniMQTTclient<2> mqttClient("192.168.4.2", 1883, wifiClient, 5);

Battery batt;
MPU9250 imu;

uint8_t ledpin = 12;

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


void updateBattery(void* params);
void sendBattery(void* params);
void sendIMU(void*params);
void print_calibration();
void param_handler(const char topic[], byte* payload, unsigned int length);
void sendSpeed(void* params);

void setup() {
  Serial.begin(115200);
  Serial.println("IMU calibration start");

  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, HIGH);


  batt.init();

  
  mot1.init();
  mot2.init();
  mot3.init();
  motors[0] = &mot1;
  motors[1] = &mot2;
  motors[2] = &mot3;

  enc1.init();
  enc2.init();
  enc3.init();
  encoders[0] = &enc1;
  encoders[1] = &enc2;
  encoders[2] = &enc3;
  
  WiFi.softAP(ssid, password, 1, false, 1);
  ESP_LOGI("wifi", "WiFi AP started");
  vTaskDelay(pdMS_TO_TICKS(100));
  
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

  Wire.begin();
  MPU9250Setting mpu_setting;
  mpu_setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  mpu_setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ;
  mpu_setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  mpu_setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
  mpu_setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  mpu_setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;

    if (!imu.setup(0x68, mpu_setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    ESP_LOGI("IMU", "IMU initialized");

    manual_ctrl_vec = true;

    // Builtin calibration
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    // imu.verbose(true);
    // delay(5000);
    // imu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    // imu.calibrateMag();

    // print_calibration();
    // imu.verbose(false);

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

  
  xTaskCreatePinnedToCore( sendIMU,
                           "IMU",
                           2000,
                           NULL,
                           2,
                           NULL,
                           0);

  ;xTaskCreatePinnedToCore( sendSpeed,
                           "SendSpeed",
                           2000,
                           NULL,
                           1,
                           NULL,
                           0);

  enc1.resume();
  enc2.resume();
  enc3.resume();
  xTaskCreatePinnedToCore( wheel_control,
                           "wheel control",
                           2000,
                           NULL,
                           2,
                           NULL,
                           1);

  // xTaskCreatePinnedToCore(position_control_task,
  //                         "post_control",
  //                         2000,
  //                         NULL,
  //                         2,
  //                         NULL,
  //                         1); 
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

void sendIMU(void* params){  
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    while(true){
        // taskENTER_CRITICAL();
        imu.update();
        imu.update();
        // taskEXIT_CRITICAL();
        float data[3];
        for(int i = 0; i < 3; i++){
        data[i] = imu.getMag(i);
        }
        //  ESP_LOGI("IMU","send mag %f\t%f\t%f", data[0],data[1],data[2]);
        mqttClient.publish3float(0x90,data);
        
        for(int i = 0; i < 3; i++){
        data[i] = imu.getAcc(i);
        }
        // ESP_LOGI("IMU","send acc %f\t%f\t%f", data[0],data[1],data[2]);
        mqttClient.publish3float(0x91,data);
        
        for(int i = 0; i < 3; i++){
        data[i] = imu.getGyro(i);
        }
        // ESP_LOGI("IMU","send gyro %f\t%f\t%f", data[0],data[1],data[2]);
        mqttClient.publish3float(0x92,data);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void param_handler(const char topic[], byte* payload, unsigned int length){
  ESP_LOGI(topic,"%s",topic);
  if(strcmp(topic,"param") == 0){
    // Wheel setpoint received
    switch(payload[1]){
      case 0xA4:
        memcpy(&speed_setpoints, payload + 2, 12);
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

void sendBattery(void* params){

  while(true){
    if(mqttClient.isConnected()){
      mqttClient.publishBattery(batt.getVoltage());
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
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