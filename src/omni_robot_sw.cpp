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
OmniMQTTclient<3> mqttClient("192.168.4.2", 1883, wifiClient);

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

MPU9250 imu;

void updateValuesSlow(void* params);

void communicationTask(void* params);

void sendSpeed(void* params);
void mqttLoop(void* params);

void param_handler(const char topic[], byte* payload, unsigned int length);
void camera_handler(const char topic[], byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");

  // Init components
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

  encoders[0] = &enc1;
  encoders[1] = &enc2;
  encoders[2] = &enc3;
  
Wire.begin();
WiFi.setSleep(false);
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
imu_fusion = &imu;
  ESP_LOGI("IMU", "IMU initialized");

  // Init communication
  WiFi.softAP(ssid, password, 1, false, 1);

  mqttClient.add_calback(param_handler);
  mqttClient.add_calback(camera_handler);
  mqttClient.init();
  ESP_LOGI("mqttc","MQTT initialized");
  while(!mqttClient.isConnected()){
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI("mqttc","MQTT connected");
  while(!mqttClient.subscribe("param")){
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  while(!mqttClient.subscribe("cps")){
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI("mqttc","MQTT subscribed");

  // Init robot state
  ESP_LOGI("init","Starting state initilaization");
  float init_meas[5][4];
  float cam_vec_prev[3] = {0,0,0};
  bool init_done = false;
  uint8_t measNo = 0;
  while(!init_done){
    copy(cam_vec_prev, cam_vec, 3);
    mqttClient.loop();

    if(cam_vec[0] != cam_vec_prev[0] || cam_vec[1] != cam_vec_prev[1] || cam_vec[2] != cam_vec_prev[2]){
      copy(init_meas[measNo], cam_vec, 3);
      imu.update();
      imu_corr();
      init_meas[measNo][3] = imu_head;

      measNo++;

      if(measNo == 5)
        init_done = true;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("Data collected");
  float init_state[4] = {0,0,0,0};
  for(uint8_t i = 0; i < 5; i++){
    for(uint8_t j = 0; j < 4; j++)
      init_state[j] += init_meas[i][j];
  }


  for(uint8_t j = 0; j < 4; j++)
    init_state[j] /= 5;

  init_fusion(init_state, init_state+2, init_state[3] - init_state[2]);
  init_position_control(init_state);

  ESP_LOGI("init","State initialized: \t%f\t%f\t%f", init_state[0],init_state[1],init_state[2]);

  manual_ctrl_vec = true;

  xTaskCreatePinnedToCore(mqttLoop,
                          "MQTT loop",
                          2000,
                          NULL,
                          3,
                          NULL,
                          0);

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

  xTaskCreatePinnedToCore(position_control_task,
                          "post_control",
                          5000,
                          NULL,
                          2,
                          NULL,
                          1); 

  xTaskCreatePinnedToCore(fusion_task,
                          "fusion",
                          5000,
                          NULL,
                          2,
                          NULL,
                          1); 

  ESP_LOGI("init","Tasks launched");

  ESP_LOGI("init","Setup finished");
}

void loop() {
  // put your main code here, to run repeatedly:
   //ESP_LOGW("ENC","Pulse cound: %d", ident_enc.getCountReset());
   vTaskDelay(pdMS_TO_TICKS(1000));
}

void mqttLoop(void *params){
  while(true){
    mqttClient.loop();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
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
  float temp[3];
  while(true){
    if(mqttClient.isConnected()){
      float temp[2];
      for(int i = 0; i < 3; i++){
        mqttClient.publishMotSpeed(i, current_speeds[i], speed_setpoints[i]);
        temp[0] = state_vec[i];
        temp[1] = ref_vec[i];
        mqttClient.publishNfloat(0x80+i, temp, 2);

        temp[0] = atan2(global_control_vec[1], global_control_vec[0]);
        temp[1] = atan2(control_vec[1], control_vec[0]);
        temp[2] = sqrt(control_vec[0]*control_vec[0] + control_vec[1]*control_vec[1]);
        mqttClient.publishNfloat(0x83, temp, 3);
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
      case 0x94:
        memcpy(&ref_vec, payload+2, 12);
        ESP_LOGW("wheel","Ref vec %f,/t%f,/t%f", ref_vec[0], ref_vec[1], ref_vec[2]);
        break;
      default:
        ESP_LOGW("MQTT", "Unknown type on params");
        break;
    }
  }
}

void camera_handler(const char topic[], byte* payload, unsigned int length){
  if(strcmp(topic, "cps") == 0){
    memcpy(cam_vec, &payload[2], 3*sizeof(float));
    memcpy(state_vec, &payload[2], 3*sizeof(float));
    // ESP_LOGW("cam", "%f,/t%f,/t%f", cam_vec[0], cam_vec[1], cam_vec[2]);
  }
}