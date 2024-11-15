#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC ADDRESS OF YOUR ESP01S RECEIVER
uint8_t receiverMac[] = { 0xCC, 0x50, 0xE3, 0X41, 0x1C, 0xD6 };

// Structure to hold the data to be sent
typedef struct sendPacket {
  float fwd_kp;
  float fwd_kd;
  float rot_kp;
  float rot_kd;
  float steering_kp;
  float steering_kd;
  int speed;
  int omega;
} sendPacket;

typedef struct receivePacket {
  int speed;
  int omega;
  float distance;
  float angle;
} receivePacket;

sendPacket sendData;
receivePacket receiveData;

esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);

  // Set the device in station mode
  WiFi.mode(WIFI_STA);

  // Initalize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register the receive callback
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  if (Serial.available() > 0) {
    // Read the data from the serial port
    sendData.fwd_kp = Serial.parseFloat();
    sendData.fwd_kd = Serial.parseFloat();
    sendData.rot_kp = Serial.parseFloat();
    sendData.rot_kd = Serial.parseFloat();
    sendData.steering_kp = Serial.parseFloat();
    sendData.steering_kd = Serial.parseFloat();
    sendData.speed = Serial.parseInt();
    sendData.omega = Serial.parseInt();

    // Send the data via ESP-NOW
    esp_now_send(receiverMac, (uint8_t *)&sendData, sizeof(sendData));

  }

}

// Callback function for sent data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("SEND_OK");
  } else {
    Serial.println("SEND_FAIL");
  }
}

// Callback function for received data
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&receiveData, data, sizeof(receiveData));
  Serial.println(receiveData.speed);
  Serial.println(receiveData.omega);
}