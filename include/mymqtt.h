#ifndef MYMQTT_H
#define MYMQTT_H
#include <WiFiClient.h>
#include "PubSubClient.h"


enum MyTopics{
  telemetry,
  topics_no
};

enum TypeBytes{
  batt_T = 0x01
};

extern const char* topicNames[];

template <int MAX_CB_NO>
class OmniMQTTclient{
  private:
    PubSubClient mqttClient;
    char* name = (char*)"OmniBot";
    char* username = (char*)"omnibot";
    char* pwd = (char*)"myomni";
    uint8_t status_led;

    void (*on_msg_callbacks[MAX_CB_NO])(const char[], byte*, unsigned int);
    int callback_no = 0;
    char topics[MAX_CB_NO][10];
    int topic_no = 0;

    void call_callbacks(const char topic[], byte* payload, unsigned int length);
  public:
    OmniMQTTclient(const char* ipaddr, uint16_t port, Client& client, uint8_t led_pin = LED_BUILTIN);
    bool init(uint8_t core = 0, uint8_t priority = 2);
    bool isConnected();
    bool reconnect();
    void forceReconnectFromTask();
    static void keepConnectionTask(void* params);
    void publishBattery(const float voltage);
    void publish1float(const char* topic, uint8_t type_byte, float f);
    void publish_u16(const char* topic, uint8_t type_byte, uint16_t i);
    void publishData(const char topic[], uint8_t type, byte* payload, uint8_t length);
    bool loop();
    
    bool add_calback(void(*on_msg_callback)(const char[], byte*, unsigned int));
    bool subscribe(const char topic[]);
    
};



const char* topicNames[] = {"tel"};

/* Instantiate the client object
*/
template <int MAX_CB_NO>
OmniMQTTclient<MAX_CB_NO>::OmniMQTTclient(const char* ipaddr, uint16_t port, Client& client, uint8_t led_pin):mqttClient(ipaddr, port, client),status_led(led_pin){
}

/* Initialize the client (connect and subscribe to topics)
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::init(uint8_t core, uint8_t priority){
  xTaskCreatePinnedToCore( this->keepConnectionTask,
                           "MQTT keep connection",
                           2000,
                           this,
                           priority,
                           NULL,
                           core);


  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  mqttClient.setCallback(std::bind(&OmniMQTTclient<MAX_CB_NO>::call_callbacks, this, _1, _2, _3));

  return true;
}

/* Check if the client is connected to the MQTT server
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::isConnected(){
  bool isConn = this->mqttClient.connected();
//  Serial.println(isConn);
  return isConn;
}

/* Try to connect to the MQTT broker once
*/
template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::reconnect(){
  return this->mqttClient.connect(this->name,this->username,this->pwd);
}

/* Try to reconnect until it is successful from a freeRTOS task
*/

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::forceReconnectFromTask(){
  
  // Turn LED off.
  // TODO: outsource
  digitalWrite(status_led, LOW);
  
  //Try to connect.
//  Serial.println("Trying to connect");
  while(!this->reconnect())
  {
//    Serial.println("Trying to connect");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // Subscribe for topics
  for(int i = 0; i < topic_no; i++){
    mqttClient.subscribe(topics[i]);
  }

  // Turn LED on.
  // TODO: outsource
//  Serial.println("Connection successfull");
  digitalWrite(status_led, HIGH);
}

/* FreeRTOS task to keep the connection to the broker alive
*/

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::keepConnectionTask(void* params){
  OmniMQTTclient* self = (OmniMQTTclient*)params;
  while(1){
//    Serial.println("Check connection");
    if(!self->isConnected()){
      self->forceReconnectFromTask();
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}


template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publishBattery(const float voltage){
  publish1float(topicNames[telemetry], 0x01, voltage);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publish1float(const char* topic, uint8_t type, const float f){
  uint8_t float_size = sizeof(float);
  uint8_t bytes_to_send[float_size+2];
  bytes_to_send[0] = float_size;
  bytes_to_send[1] = type;
  memcpy(bytes_to_send+2, &f, float_size);
  this->mqttClient.publish(topic, bytes_to_send,float_size+2);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publishData(const char topic[], uint8_t type, byte* payload, uint8_t length){
  uint8_t bytes_to_send[length+2];
  bytes_to_send[0] = length;
  bytes_to_send[1] = type;
  memcpy(bytes_to_send+2, payload, length);
  this->mqttClient.publish(topic, bytes_to_send, length+2);
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::loop(){
  return this->mqttClient.loop();
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::add_calback(void(*on_msg_callback)(const char[], byte*, unsigned int)){
  if(callback_no < MAX_CB_NO){
    on_msg_callbacks[callback_no] = on_msg_callback;
    callback_no++;
    return true;
  } else{
    return false;
  }
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::call_callbacks(const char topic[], byte* payload, unsigned int length){
    for(int i = 0; i < callback_no; i++){
      on_msg_callbacks[i](topic, payload, length);
    }
}

template <int MAX_CB_NO>
bool OmniMQTTclient<MAX_CB_NO>::subscribe(const char topic[]){
  for(int i = 0; i < topic_no; i++){
    if(strcmp(topic,topics[i]) == 0)
      return true;
  }
  strcpy(topics[topic_no++], topic);
  return mqttClient.subscribe(topic);
}

template <int MAX_CB_NO>
void OmniMQTTclient<MAX_CB_NO>::publish_u16(const char* topic, uint8_t type_byte, uint16_t i){
  uint8_t size = sizeof(uint16_t);
  uint8_t bytes_to_send[size+2];
  bytes_to_send[0] = size;
  bytes_to_send[1] = type_byte;
  memcpy(bytes_to_send+2, &i, size);
  this->mqttClient.publish(topic, bytes_to_send,size+2);
}
#endif

