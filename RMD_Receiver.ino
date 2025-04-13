#include <esp_now.h>
#include <WiFi.h>
#include <mcp_can.h>

#define CAN_INT 4

MCP_CAN CAN(5);
byte len = 8;
long unsigned int recvId = 0x241;
long unsigned int sendId = 0x141;
const float kt = 5;
const float r = 0.05;
const float g = 9.81;
const int timeout = 500;
const int maxRetries = 3;
volatile bool canReceived = false;


long position = 0;
bool incomingExerciseStarted = false;
float incomingResistance = 0;

struct CAN_Frame {
  uint32_t id;
  byte data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
};

typedef struct struct_message {
  float resistance;
  bool exerciseStarted;
} struct_message;

struct_message incomingData;

esp_now_peer_info_t peerInfo;

CAN_Frame createMessage(byte const &command, long const &data = 0) {
  CAN_Frame message;
  message.id = sendId;
  message.data[0] = command;
  message.data[1] = 0x00;
  message.data[2] = 0x00;
  message.data[3] = 0x00;
  message.data[4] = data & 0xFF;
  message.data[5] = (data >> 8) & 0xFF;
  message.data[6] = (data >> 16) & 0xFF;
  message.data[7] = (data >> 24) & 0xFF;

  return message;
}

bool awaitInterrupt() {
  unsigned long startTime = millis();
  canReceived = false;
  while (!canReceived) {
    if (millis() - startTime > timeout) {
      return false;
    }
    delay(1);
  }
  return true;
}

void IRAM_ATTR onCanInterrupt() {
  canReceived = true;
}

void resetMotor() {
  CAN_Frame resetMessage = createMessage(0x76);
  CAN.sendMsgBuf(resetMessage.id, 0, 8, resetMessage.data);
}

float resistanceToTorque(float resistance) {
  float torque = resistance * r * g;
  return torque;
}

void setResistance(float resistance) {
  int retries = 0;
  float torque = resistanceToTorque(resistance);
  float current = torque / kt;
  Serial.print("Left motor resistance: ");
  Serial.print(resistance);
  Serial.println(" kg");
  Serial.print("Left motor current: ");
  Serial.println(current);
  CAN_Frame torqueMessage = createMessage(0xA1, current * 100);
  CAN_Frame receivedMessage{ recvId };
  CAN.sendMsgBuf(torqueMessage.id, 0, 8, torqueMessage.data);
  long startTime = millis();
  while (!awaitInterrupt() && (retries < maxRetries)) {
    CAN.sendMsgBuf(torqueMessage.id, 0, 8, torqueMessage.data);
    retries++;
    delay(100);
  }
  if (retries >= maxRetries) {
    Serial.println("Max retries exceeded, exiting...");
    return;
  }
  CAN.readMsgBuf(&recvId, &len, receivedMessage.data);
}

void configureMotor(const float &resistance, const int &position) {
  int retries = 0;
  Serial.print("Configuring motor at position: ");
  Serial.print(position);
  Serial.print(" with resistance ");
  Serial.println(resistance);
  Serial.print("Current: ");
  float current = resistanceToTorque(resistance) / kt;
  Serial.println(current);
  Serial.print("Current percentage ");
  Serial.println((int)((current / 5.2) * 100));
  CAN_Frame configMessage = createMessage(0xA9, position);
  configMessage.data[1] = (int)((current / 5.2) * 100);
  configMessage.data[2] = (360 * 90) & 0xFF;
  configMessage.data[3] = ((360 * 90) >> 8) & 0xFF;
  CAN_Frame receivedMessage{ recvId };
  CAN.sendMsgBuf(configMessage.id, 0, 8, configMessage.data);
  long startTime = millis();
  while (!awaitInterrupt() && (retries < maxRetries)) {
    CAN.sendMsgBuf(configMessage.id, 0, 8, configMessage.data);
    delay(100);
    if (retries >= maxRetries) {
      Serial.println("Max retries exceeded, exiting...");
      return;
    }
    retries++;
  }
  long unsigned int localRecvId;
  Serial.println("Configure motor reply received");
  CAN.readMsgBuf(&localRecvId, &len, receivedMessage.data);
  if (localRecvId == 0x241) {
    float recvCurrent = (receivedMessage.data[2] | (receivedMessage.data[3] << 8)) / 100;
    uint32_t recvPosition = (uint32_t)receivedMessage.data[6] | (uint32_t)(receivedMessage.data[7] << 8);
    Serial.print("Received current: ");
    Serial.println(recvCurrent);
    Serial.print("configureMotor position: ");
    Serial.println(recvPosition);
  }
}

int getPosition() {
  uint32_t canId;
  CAN_Frame positionMessage = createMessage(0x92);
  CAN_Frame receivedMessage{ recvId };
  CAN.sendMsgBuf(positionMessage.id, 0, 8, positionMessage.data);
  int retries = 0;
  long startTime = millis();
  while (awaitInterrupt() && (retries < maxRetries)) {
    CAN.sendMsgBuf(positionMessage.id, 0, 8, positionMessage.data);
    retries++;
    delay(100);
  }
  if (retries >= maxRetries) {
    Serial.println("Max retries exceeded, exiting...");
    return -1;
  }
  Serial.println("Position reply received");
  memset(receivedMessage.data, 0, sizeof(receivedMessage.data));
  Serial.println("Reading CAN buffer...");
  CAN.readMsgBuf(&canId, &len, receivedMessage.data);
  Serial.print("CAN ID received: 0x");
  Serial.println(canId, HEX);
  if (canId != 0x241) {
    Serial.println("Unexpected CAN ID");
    return -1;
  }
  int32_t mPosition = (int32_t)(((uint32_t)receivedMessage.data[4]) | ((uint32_t)receivedMessage.data[5] << 8) | ((uint32_t)receivedMessage.data[6] << 16) | ((uint32_t)receivedMessage.data[7] << 24));
  Serial.print("Motor position: ");
  Serial.println(mPosition);
  return mPosition;
}

void onDataRecv(const uint8_t *mac, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  incomingExerciseStarted = incomingData.exerciseStarted;
  incomingResistance = incomingData.resistance;
  Serial.print("Exercise started: ");
  Serial.println(incomingExerciseStarted);
  Serial.print("Left motor resistance: ");
  Serial.println(incomingResistance);
  if (!incomingExerciseStarted) {
    setResistance(-4);
  }
}

void setup() {
  Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialised");
  } else {
    Serial.println("CAN initialisation failed");
  }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT), onCanInterrupt, FALLING);
  Serial.println("CAN set to normal mode");
  // resetMotor();
  setResistance(-4);
}

void loop() {
  if (incomingExerciseStarted) {
    configureMotor(incomingResistance, -10000000);
  }
  delay(100);
}
