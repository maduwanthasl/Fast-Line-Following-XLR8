#include <espnow.h>
#include <ESP8266WiFi.h>

uint8_t peerMac[] = { 0xD8, 0x3B, 0xDA, 0xA3, 0x88, 0x20 };

//receiving packet strucutre
typedef struct {
  float fwd_kp;
  float fwd_kd;
  float rot_kp;
  float rot_kd;
  float steering_kp;
  float steering_kd;
  int speed;
  int omega;
} ReceivePacket;

// Sending packet structure
typedef struct {
  float speed;
  float omega; 
  float distance;
  float angle;
} SendPacket;

ReceivePacket recPacket;  
SendPacket sendPacket;  


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  //0 is used instead of ESP_OK as apparently it isn't defined in the header
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  // Register the send and receive callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceived);

  // Add the peer device
  esp_now_add_peer(peerMac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
}

void loop() {

  if (Serial.available() > 0) {
    sendPacket.speed = Serial.parseFloat();
    sendPacket.omega = Serial.parseFloat();
    sendPacket.distance = Serial.parseFloat();
    sendPacket.angle = Serial.parseFloat();

    esp_now_send(peerMac, (uint8_t*)&sendPacket, sizeof(sendPacket));
  }
}

void OnDataReceived(uint8_t* mac, uint8_t* incomingData, uint8_t len) {
  memcpy(&recPacket, incomingData, sizeof(recPacket));

  Serial.println(recPacket.fwd_kp);
  Serial.println(recPacket.fwd_kd);
  Serial.println(recPacket.rot_kp);
  Serial.println(recPacket.rot_kd);
  Serial.println(recPacket.steering_kp);
  Serial.println(recPacket.steering_kd);
  Serial.println(recPacket.speed);
  Serial.println(recPacket.omega);
}

void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    Serial.println("SEND_OK");
  } else {
    Serial.println("SEND_FAIL");
  }
}