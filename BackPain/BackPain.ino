/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_LED_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TX_UUID "129eb0ef-737a-4fe1-b11b-43c4d7a3380d"


const int PINS[8] = {13, 12, 14, 27, 26, 25, 33, 32};
uint8_t LED_num = 0;

BLECharacteristic *pcharacteristicLED;
BLECharacteristic *pcharacteristicTX;

bool device_connected;
int tx_value;


void updateLEDs() {
  LED_num = *(pcharacteristicLED->getData());

  //LED_num = atoi(s.c_str());

  int i = 0;
  while (i < 9) {
    if (LED_num % 2 == 1) {
      digitalWrite(PINS[i], HIGH);
    }
    else {
      digitalWrite(PINS[i], LOW);
    }
    i++;
    LED_num /= 2;
  }
}



class ServerCallBacks : public BLEServerCallbacks {
  
  void onConnect(BLEServer *pServer) {
    device_connected = true;
  }
  void onDisconnect(BLEServer *pServer) {
    device_connected = false;
  }
};

class LEDCallBacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pcharacteristic) {
    if (pcharacteristic == pcharacteristicLED) {
      Serial.printf("data changed to %s \n", pcharacteristicLED->getValue());
      updateLEDs();
    }
  }
};





void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");


  for (int i = 0; i < 8; i++) {
    pinMode(PINS[i], OUTPUT);
  }


  BLEDevice::init("ESP-32 slave");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallBacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pcharacteristicLED = pService->createCharacteristic(CHARACTERISTIC_LED_UUID, BLECharacteristic::PROPERTY_WRITE);
  pcharacteristicLED->setCallbacks(new LEDCallBacks());

  pcharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pcharacteristicTX->addDescriptor(new BLE2902());


  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("began advertising");




}
void loop() {

  if (device_connected) {
    int r = random(-10, 10);
    tx_value = r;

    char tx_string[8];
    itoa(r, tx_string, 16);

    pcharacteristicTX->setValue(tx_string);
    pcharacteristicTX->notify();

    Serial.printf("sent value: %s\n", tx_string);
  
  }
  delay(1000);
}
