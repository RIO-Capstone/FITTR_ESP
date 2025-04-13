#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <esp_now.h>

#define CAN_INT 4

#define SERVICE_UUID "4c72c9a7-69af-4a0b-8630-fab8f513fb9e"
#define RESISTANCE_L_CHARACTERISTIC_UUID "68ac7851-4e08-439d-8a3c-90baaa5f20d2"
#define RESISTANCE_R_CHARACTERISTIC_UUID "f5527fd1-bd21-4de3-a9b6-545b0559896c"
#define EXERCISE_CHARACTERISTIC_UUID "a57a73af-6259-46a6-a796-9bedc16ef857"
#define DUMMY_CHARACTERISTIC_UUID "e429a327-c1a4-4a25-956e-f3d632bdd63a"

uint8_t broadcastAddress[] = { 0xcc, 0xdb, 0xa7, 0x96, 0x69, 0x08 };

MCP_CAN CAN(5);
byte len = 8;
long unsigned int recvId = 0x241;
long unsigned int sendId = 0x141;
const float kt = 5;
const int timeout = 500;
const int maxRetries = 3;
const float r = 0.05;
const float g = 9.81;
volatile bool canReceived = false;


BLECharacteristic *resistanceLCharacteristic;
BLECharacteristic *resistanceRCharacteristic;
BLECharacteristic *exerciseCharacteristic;
BLECharacteristic *dummyCharacteristic;

bool deviceConnected = false;
float resistanceL = 0;
float resistanceR = 0;
bool exerciseStarted = false;
long position = 0;

typedef struct struct_message {
  float resistance;
  bool exerciseStarted;
} struct_message;

struct_message sentData{ 0, false };

esp_now_peer_info_t peerInfo;

struct CAN_Frame {
  uint32_t id;
  byte data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
};

void IRAM_ATTR onCanInterrupt() {
  canReceived = true;
}

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

float resistanceToTorque(float resistance) {
  float torque = resistance * r * g;
  return torque;
}

void resetMotor() {
  CAN_Frame resetMessage = createMessage(0x76);
  CAN.sendMsgBuf(resetMessage.id, 0, 8, resetMessage.data);
}

void setResistance(float resistance) {
  int retries = 0;
  float current = resistanceToTorque(resistance) / kt;
  Serial.print("Right motor resistance: ");
  Serial.print(resistance);
  Serial.println(" kg");
  Serial.print("Right motor current: ");
  Serial.println(current);
  long unsigned int localRecvId;
  CAN_Frame torqueMessage = createMessage(0xA1, current * 100);
  CAN_Frame receivedMessage{ recvId };
  CAN.sendMsgBuf(torqueMessage.id, 0, 8, torqueMessage.data);
  long startTime = millis();
  while (awaitInterrupt() && (retries < maxRetries)) {
    CAN.sendMsgBuf(torqueMessage.id, 0, 8, torqueMessage.data);
    retries++;
    delay(100);
  }
  if (retries >= maxRetries) {
    Serial.println("Max retries exceeded, exiting...");
    return;
  }
  CAN.readMsgBuf(&localRecvId, &len, receivedMessage.data);
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
  while (awaitInterrupt() && (retries < maxRetries)) {
    CAN.sendMsgBuf(configMessage.id, 0, 8, configMessage.data);
    retries++;
    delay(100);
  }
  if (retries >= maxRetries) {
    Serial.println("Max retries exceeded, exiting...");
    return;
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


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
};

class ResistanceLCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    resistanceL = value.toFloat();
    sentData.resistance = resistanceL;
    sendData();
    Serial.print("Left motor resistance set to ");
    Serial.print(resistanceL);
    Serial.println("kg");
  }
};

class ResistanceRCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    resistanceR = value.toFloat();
    Serial.print("Right motor resistance set to ");
    Serial.print(resistanceR);
    Serial.println("kg");
  }
};

void sendData() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sentData, sizeof(sentData));
  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}

class ExerciseCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    exerciseStarted = (value == "true");
    if (exerciseStarted) {
      sentData.exerciseStarted = true;
      setResistance(resistanceR);
      sendData();
      Serial.println("Exercise started");
    } else {
      sentData.exerciseStarted = false;
      setResistance(4);
      sendData();
      Serial.println("Exercise stopped");
    }
  }
};

class DummyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
  }
};

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  BLEDevice::init("FITTR");

  BLEServer *pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  resistanceLCharacteristic = pService->createCharacteristic(
    RESISTANCE_L_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  resistanceRCharacteristic = pService->createCharacteristic(
    RESISTANCE_R_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  exerciseCharacteristic = pService->createCharacteristic(
    EXERCISE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  dummyCharacteristic = pService->createCharacteristic(
    DUMMY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  resistanceLCharacteristic->setCallbacks(new ResistanceLCharacteristicCallbacks());
  resistanceRCharacteristic->setCallbacks(new ResistanceRCharacteristicCallbacks());
  exerciseCharacteristic->setCallbacks(new ExerciseCharacteristicCallbacks());
  dummyCharacteristic->setCallbacks(new DummyCharacteristicCallbacks());

  resistanceRCharacteristic->setValue("0");
  resistanceLCharacteristic->setValue("0");
  exerciseCharacteristic->setValue("false");
  dummyCharacteristic->setValue("sarang eats dick");

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("BLE server is now advertising...");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialised");
  } else {
    Serial.println("CAN initialisation failed");
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT), onCanInterrupt, FALLING);
  Serial.println("Setting CAN to normal mode");
  // resetMotor();
  setResistance(4);
}

void loop() {
  if (exerciseStarted) {
    configureMotor(resistanceR, 0);
  }
  delay(100);
}
