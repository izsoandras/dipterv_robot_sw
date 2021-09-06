#ifndef MYMQTT_H
#define MYMQTT_H
#include <WiFiClient.h>
#include "PubSubClient.h"

enum MyTopics{
  telemetry,
  topics_no
};

const String topicNames[topics_no] = {"tel"};

class OmniMQTTclient{
  private:
    PubSubClient mqttClient;
    char* name = (char*)"OmniBot";
    char* username = (char*)"omnibot";
    char* pwd = (char*)"myomni";

  public:
    OmniMQTTclient(const char* ipaddr, uint16_t port, Client& client);
    bool init(uint8_t core = 0, uint8_t priority = 2);
    bool isConnected();
    bool reconnect();
    void forceReconnectFromTask();
    static void keepConnectionTask(void* params);
    void publishBattery(float voltage);
    bool loop();
};

#endif
