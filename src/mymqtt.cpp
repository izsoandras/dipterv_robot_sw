#include "mymqtt.h"

/* Instantiate the client object
*/
OmniMQTTclient::OmniMQTTclient(const char* ipaddr, uint16_t port, Client& client):mqttClient{ipaddr, port, client}{}

/* Initialize the client (connect and subscribe to topics)
*/
bool OmniMQTTclient::init(uint8_t core, uint8_t priority){
  xTaskCreatePinnedToCore( this->keepConnectionTask,
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
bool OmniMQTTclient::isConnected(){
  bool isConn = this->mqttClient.connected();
//  Serial.println(isConn);
  return isConn;
}

/* Try to connect to the MQTT broker once
*/
bool OmniMQTTclient::reconnect(){
  return this->mqttClient.connect(this->name,this->username,this->pwd);
}

/* Try to reconnect until it is successful from a freeRTOS task
*/
void OmniMQTTclient::forceReconnectFromTask(){
  
  // Turn LED off.
  // TODO: outsource
  digitalWrite(LED_BUILTIN, LOW);
  
  //Try to connect.
//  Serial.println("Trying to connect");
  while(!this->reconnect())
  {
//    Serial.println("Trying to connect");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // Subscribe for topics
  // TODO: subscriptions
  //mqtt.subscribe("comm", 1);

  // Turn LED on.
  // TODO: outsource
//  Serial.println("Connection successfull");
  digitalWrite(LED_BUILTIN, HIGH);
}

/* FreeRTOS task to keep the connection to the broker alive
*/
void OmniMQTTclient::keepConnectionTask(void* params){
  OmniMQTTclient* self = (OmniMQTTclient*)params;
  while(1){
//    Serial.println("Check connection");
    if(!self->isConnected()){
      self->forceReconnectFromTask();
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}


void OmniMQTTclient::publishBattery(float voltage){
  uint8_t float_size = sizeof(float);
  uint8_t bytes_to_send[float_size];
  memcpy(bytes_to_send, &voltage, float_size);
  this->mqttClient.publish(topicNames[battery_topic].c_str(),bytes_to_send,4);
}

bool OmniMQTTclient::loop(){
  return this->mqttClient.loop();
}
