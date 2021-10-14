#ifndef MYMQTT_H
#define MYMQTT_H
#include <WiFiClient.h>
#include "PubSubClient.h"

enum MyTopics
{
  telemetry,
  param,
  topics_no
};

enum TypeBytes
{
  batt_T = 0x01
};

extern const char *topicNames[];

template <int MAX_CB_NO>
class OmniMQTTclient
{
private:
  PubSubClient mqttClient;
  char *name = (char *)"OmniBot";
  char *username = (char *)"omnibot";
  char *pwd = (char *)"myomni";
  uint8_t status_led;

  void (*on_msg_callbacks[MAX_CB_NO])(const char[], byte *, unsigned int);
  int callback_no = 0;
  char topics[MAX_CB_NO][10];
  int topic_no = 0;
  SemaphoreHandle_t xSemaphore;

  void call_callbacks(const char topic[], byte *payload, unsigned int length);

public:
  OmniMQTTclient(const char *ipaddr, uint16_t port, Client &client, uint8_t led_pin = LED_BUILTIN);
  bool init(uint8_t core = 0, uint8_t priority = 2);
  bool isConnected();
  bool reconnect();
  void forceReconnectFromTask();
  static void keepConnectionTask(void *params);
  void publishBattery(const float voltage);
  void publish1float(const char *topic, uint8_t type_byte, float f);
  void publish_u16(const char *topic, uint8_t type_byte, uint16_t i);
  void publishData(const char topic[],const uint8_t type, const byte *payload, const uint8_t length);
  void publishMotSpeed(const uint8_t mot_idx, float current_speed, float setpoint);
  void publish3float(uint8_t type, const float[]);
  bool loop();

  bool add_calback(void (*on_msg_callback)(const char[], byte *, unsigned int));
  bool subscribe(const char topic[]);
};

const char *topicNames[] = {"tel", "param"};

/* Instantiate the client object
*/
template <int MAX_CB_NO>
OmniMQTTclient<MAX_CB_NO>::OmniMQTTclient(const char *ipaddr, uint16_t port, Client &client, uint8_t led_pin) : mqttClient(ipaddr, port, client), status_led(led_pin)
{
}

/* Initialize the client (connect and subscribe to topics)
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::init(uint8_t core, uint8_t priority)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  this->xSemaphore = xSemaphoreCreateMutex();

  mqttClient.setCallback(std::bind(&OmniMQTTclient<MAX_CB_NO>::call_callbacks, this, _1, _2, _3));

  while(!this->mqttClient.connect(this->name, this->username, this->pwd)){
  ESP_LOGI("sd","FUCKIN TRYING");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI("sd","FUCKIN HELL");

  xTaskCreatePinnedToCore(this->keepConnectionTask,
                          "MQTT keep connection",
                          2000,
                          this,
                          priority,
                          NULL,
                          core);

  return true;
}

/* Check if the client is connected to the MQTT server
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::isConnected()
{
  if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    bool isConn = this->mqttClient.connected();
    xSemaphoreGive(this->xSemaphore);
    //  Serial.println(isConn);
    return isConn;
  }
  else
  {
    return false;
  }
}

/* Try to connect to the MQTT broker once
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::reconnect()
{
  if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(10) == pdTRUE))
  {
    bool ret = this->mqttClient.connect(this->name, this->username, this->pwd);
    xSemaphoreGive(this->xSemaphore);
    return ret;
  }
  else
  {
    ESP_LOGD("reconnect","Connect timeout");
    return false;
  }
}

/* Try to reconnect until it is successful from a freeRTOS task
*/

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::forceReconnectFromTask()
{

  // Turn LED off.
  // TODO: outsource
  digitalWrite(status_led, LOW);

  //Try to connect.
  //  Serial.println("Trying to connect");
  while (!this->reconnect())
  {
    //    Serial.println("Trying to connect");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  ESP_LOGI("mqttConn","Connected");

  // Subscribe for topics
  for (int i = 0; i < topic_no; i++)
  {
    while (this->isConnected())
    {
      if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        ESP_LOGI("mqttConn", "Subscribing");
        if (mqttClient.subscribe(topics[i]))
        {
          xSemaphoreGive(this->xSemaphore);
          break;
        }

        xSemaphoreGive(this->xSemaphore);
      }else{
        ESP_LOGI("mqttConn","Subscription timeout");
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  ESP_LOGI("mqttConn","Subscription done");

  // Turn LED on.
  // TODO: outsource
  //  Serial.println("Connection successfull");
  digitalWrite(status_led, HIGH);
}

/* FreeRTOS task to keep the connection to the broker alive
*/

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::keepConnectionTask(void *params)
{
  OmniMQTTclient *self = (OmniMQTTclient *)params;
  while (1)
  {
    //    Serial.println("Check connection");
    if (!self->isConnected())
    {
      self->forceReconnectFromTask();
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publishBattery(const float voltage)
{
  publish1float(topicNames[telemetry], 0x01, voltage);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publish1float(const char *topic, uint8_t type, const float f)
{
  uint8_t float_size = sizeof(float);
  uint8_t payload[float_size + 2];
  memcpy(payload, &f, float_size);
  this->publishData(topic, type, payload, float_size);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publishData(const char topic[],const uint8_t type,const byte *payload,const uint8_t length)
{
  uint8_t bytes_to_send[length + 2];
  bytes_to_send[0] = length;
  bytes_to_send[1] = type;
  memcpy(bytes_to_send + 2, payload, length);
  if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(1)) == pdTRUE)
  {
    this->mqttClient.publish(topic, bytes_to_send, length + 2);
    xSemaphoreGive(this->xSemaphore);
  }
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::loop()
{
  if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(1)) == pdTRUE)
  {
    bool ret = this->mqttClient.loop();
    xSemaphoreGive(this->xSemaphore);
    return ret;
  }else{
    return false;
  }
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::add_calback(void (*on_msg_callback)(const char[], byte *, unsigned int))
{
  if (callback_no < MAX_CB_NO)
  {
    on_msg_callbacks[callback_no] = on_msg_callback;
    callback_no++;
    return true;
  }
  else
  {
    return false;
  }
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::call_callbacks(const char topic[], byte *payload, unsigned int length)
{
  for (int i = 0; i < callback_no; i++)
  {
    on_msg_callbacks[i](topic, payload, length);
  }
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::subscribe(const char topic[])
{
  for (int i = 0; i < topic_no; i++)
  {
    if (strcmp(topic, topics[i]) == 0)
      return true;
  }
  strcpy(topics[topic_no++], topic);

  if (xSemaphoreTake(this->xSemaphore, pdMS_TO_TICKS(1)) == pdTRUE)
  {
    bool ret = mqttClient.subscribe(topic);
    xSemaphoreGive(this->xSemaphore);
    return ret;
  }else{
    return false;
  }
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publish_u16(const char *topic, uint8_t type_byte, uint16_t i)
{
  uint8_t size = sizeof(uint16_t);
  uint8_t payload[size];
  memcpy(payload, &i, size);
  this->publishData(topic, type_byte, payload, size);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publishMotSpeed(const uint8_t mot_idx, float current_speed, float setpoint)
{
  uint8_t size = 2 * sizeof(float);
  uint8_t payload[size + 2];
  memcpy(payload, &current_speed, sizeof(float));
  memcpy(payload + sizeof(float), &setpoint, sizeof(float));
  this->publishData("tel", 0xA5 + mot_idx, payload, size);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publish3float(uint8_t type, const float floats[3]){
  uint8_t payload[3*sizeof(float)];
  memcpy(payload, floats, 3*sizeof(float));
  this->publishData("tel", type, payload, 3*sizeof(float));
}
#endif
