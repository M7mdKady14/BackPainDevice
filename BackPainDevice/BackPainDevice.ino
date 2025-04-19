/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


#include "driver/rtc_io.h"
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include <vector>
#include <queue>

#include <Update.h>


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
static BLEUUID BLE_DATA_SERVICE_UUID("82afce6c-9638-493b-9be0-d2aebc45f5af");


#define CHARACTERISTIC_CONTROL_UUID "dc2ddfce-07a1-4cd9-b53a-0c9461dd2e4d"
#define CHARACTERISTIC_DATA_UUID "2434c5cd-9af9-4c20-9f6c-ed569283adb5"
#define CHARACTERISTIC_LIVE_READING_UUID "2bc60ddf-0501-45a8-8d74-9cc60629d5fc"


#define FIRMWARE_VERSION "V0.1 after"



//-----------------CONFIG----------------

// MUST be a 6-digit number, no more, no less
const uint32_t PASSWORD = 123456;

// set to true to get simulated MPU data
const bool simulateMPU = false;

const bool calibrateMPU = false;

// whether to print readings to the serial or not
const bool printReadingsToSerial = true;

//whether to print history to the serial or not
const bool printHistoryToSerial = false;

Preferences prefs;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// the default interval between 2 data units in milliseconds
const int READING_INTERVAL_MS = 100;

// the array size of the score history
const int HISTORY_ARRAY_SIZE = 10;
const unsigned int MAX_BYTES_SENT_PER_PACKET = 160;
const unsigned int READING_BYTE_COUNT = 16;

// what's the maximum compression allowed before starting to overwrite past scores?
// set to -1 to disable overwriting and enable unlimited compression
const int MAX_COMPRESSIONS = 0;

const bool USE_DUMMY_VALUES_FOR_SENDING = false; // unused for now

// the pin that sends the esp to deep sleep and back
const uint8_t POWER_PIN = 14;

// the four pins that the mpu uses
const uint8_t MPU_VCC_PIN = 26;
const uint8_t MPU_GND_PIN = 25;
const uint8_t MPU_SCL_PIN = 33;
const uint8_t MPU_SDA_PIN = 32;
const uint8_t MPU_INT_PIN = 36;

//calibration for sensor readings
const int16_t X_ACCEL_MOD = -950;
const int16_t Y_ACCEL_MOD = -1000;
const int16_t Z_ACCEL_MOD = 1200;
const int16_t X_GYRO_MOD = 35;
const int16_t Y_GYRO_MOD = 15;
const int16_t Z_GYRO_MOD = -10;


// the two pins that the passive buzzer uses
const uint8_t BUZZER_GND_PIN = 18;
const uint8_t BUZZER_SIG_PIN = 21;


const unsigned int POWER_ON_NOTES[] = { 500, 0, 1000, 0, 1500 };
const unsigned int POWER_ON_TIMES[] = { 200, 100, 200, 100, 200 };
const unsigned int POWER_ON_NOTE_COUNT = 5;

const unsigned int POWER_OFF_NOTES[] = { 1500, 0, 1000, 0, 500 };
const unsigned int POWER_OFF_TIMES[] = { 200, 100, 200, 100, 200 };
const unsigned int POWER_OFF_NOTE_COUNT = 5;

const unsigned int CONNECTED_NOTES[] = { 1000, 0, 1200, 0, 1000 };
const unsigned int CONNECTED_TIMES[] = { 100, 50, 100, 50, 100 };
const unsigned int CONNECTED_NOTE_COUNT = 5;

const unsigned int DISCONNECTED_NOTES[] = { 800, 600, 400 };
const unsigned int DISCONNECTED_TIMES[] = { 100, 100, 100 };
const unsigned int DISCONNECTED_NOTE_COUNT = 3;





enum BUZZER_SOUNDS {
  BUZZER_POWER_ON,
  BUZZER_POWER_OFF,
  BUZZER_CONNECTED,
  BUZZER_DISCONNECTED,
};

enum MEASURED_POSITION {
  Z_POS_UP = 0,
  Z_NEG_UP = 1,
  X_POS_UP = 2,
  X_NEG_UP = 3,
  Y_POS_UP = 4,
  Y_NEG_UP = 5,

};


typedef struct __attribute__((packed)) {
  float theta;
  float x;
  float y;
  float z;
} Reading_t;

typedef struct __attribute__((packed)) {
  uint32_t readings_count;
  uint32_t millis_since_first_reading;
  uint32_t micros_between_two_readings;
} history_metadata_t;



//-----------------DEFINITIONS---------------------

void init_DeepSleep();
void ToggleDeepSleep();
void power_down();

void calibrate_sensor();

float getRoll(float x_axis, float y_axis, float z_axis);
float getPitch(float x_axis, float y_axis, float z_axis);

void init_BLEDataService();
void init_BLESecurity();
void init_BLE();
void DataServiceStart();
void DataServiceStop();

void updateFirmware();

void init_MPU6050();

void init_Buzzer();
void BuzzerPlay(BUZZER_SOUNDS sound, int step = 0);
void set_mute(bool status);
void giveWarning();
void stopWarning();
void sendHistory();

void updateTimer();


class Kalman;




//-----------------GLOBAL VARIABLES----------------
BLEServer *pServer;
BLEService *pService;


BLECharacteristic *pCharacteristicCtrl = nullptr;
BLECharacteristic *pCharacteristicData = nullptr;
BLECharacteristic *pCharacteristicLive = nullptr;

esp_bd_addr_t peerAddress;


// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


MPU6050 mpu;


float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;


uint8_t battery_level = 70;


// in milliseconds
unsigned long history_start_time = 0;



SemaphoreHandle_t buffer_mutex;
TaskHandle_t history_sending_task_handle;
// short term score buffer which will get averaged after it gets full


hw_timer_t *reading_timer;



//-----------------FLAGS----------------------

bool device_connected = false;
bool mpu_connected = false;
bool device_authenticated = false;
bool muted = false;
bool giving_warning = false;
bool update_started = false; // true after first packet is sent



#define DATA_NULL             0
#define DATA_FIRMWARE         1

int incoming_data = DATA_NULL;

int total_firmware_size = 0;

//-----------------CLASSES---------------------






//template <class T>
class CompressableArray {
private:
	Quaternion* data;
	size_t capacity;
	size_t startIndex = 0; // Circular buffer start index
  uint64_t firstElementDate = 0;

	void compress() {
		size /= 2;
		for (size_t i = 0; i < size; ++i) {
			data[i] = data[0].slerp(data[i*2], data[i*2+1], 0.5);
		}
		//data.reserve(capacity); // Ensure the maximum capacity remains the same
		compressionCount++;
    return;
	}

public:
	size_t compressionCount = 0;
	const size_t maxCompressions = MAX_COMPRESSIONS;
  size_t size = 0;

  
	CompressableArray(size_t initialCapacity) : capacity(initialCapacity) {
    data = new Quaternion[initialCapacity];
		//data.reserve(initialCapacity);
	}
  
  ~CompressableArray() {
    delete[] data;
  }


	void push(Quaternion value) {
    if (size== 0 ) firstElementDate = millis();
		if (compressionCount >= maxCompressions && size >= capacity) {
			// fully compressed, overwrite old data
			data[(startIndex + size) % capacity] = value;
			startIndex = (startIndex + 1) % capacity;
      firstElementDate += (READING_INTERVAL_MS) * (1 << maxCompressions);
		} else {
      // not fully compressed, puch back new data
			data[(startIndex + size++) % capacity]=value;
			if (size >= capacity && compressionCount < maxCompressions) {
				compress();
			}
		}
	}

  // get the value at index i (accounts for compression state)
	Quaternion at(size_t const i) {
		return data[(startIndex + i) % capacity];
	}

	bool isFullyCompressed() {
		return compressionCount >= maxCompressions;
	}

  // get interval between two elements in milli seconds
  uint32_t getInterval() {
    return (1 << compressionCount) * READING_INTERVAL_MS;
  }

  // the amount of time that has passed since the program started and the first reading in the array in milliseconds
  uint64_t getStartDate() {
    return firstElementDate;
  }


  void clear() {
    compressionCount = 0;
    startIndex = 0;
    size = 0;
  }
};


CompressableArray history(HISTORY_ARRAY_SIZE);


class ServerCallBacks : public BLEServerCallbacks {

  void onConnect(BLEServer *pServer) {
    Serial.println("connected to someone");
    device_authenticated = false;
    device_connected = true;
    BuzzerPlay(BUZZER_CONNECTED);
  }
  void onDisconnect(BLEServer *pServer) {
    device_authenticated = false;
    device_connected = false;
    Serial.println("disconnected from someone");
    DataServiceStop();
    pServer->getAdvertising()->start();
    BuzzerPlay(BUZZER_DISCONNECTED);
  }
};


class SecurityCallback : public BLESecurityCallbacks {

  uint32_t onPassKeyRequest() {
    Serial.println("key requested");
    return PASSWORD;
  }

  void onPassKeyNotify(uint32_t pass_key) {}

  bool onConfirmPIN(uint32_t pass_key) {
    Serial.println("on confirmPIN before");
    vTaskDelay(5000);
    Serial.println("on confirmPIN after");

    return pass_key == PASSWORD;
  }

  bool onSecurityRequest() {
    Serial.println("onSecurityRequest");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
    if (cmpl.success) {
      Serial.println("   - SecurityCallback - Authentication Success");
      device_authenticated = true;
      memcpy(peerAddress, cmpl.bd_addr, sizeof(esp_bd_addr_t));
      DataServiceStart();
      updateTimer();
    } else {
      Serial.println("   - SecurityCallback - Authentication Failure*");
      device_authenticated = false;
      pServer->removePeerDevice(pServer->getConnId(), true);
      DataServiceStop();
    }
  }
};



#define COMMAND_DEEP_SLEEP            0x00
#define COMMAND_ENABLE_MUTE           0x01
#define COMMAND_DISABLE_MUTE          0x02
#define COMMAND_TOGGLE_MUTE           0x03
#define COMMAND_GET_HISTORY           0x04
#define COMMAND_UPDATE_START          0x05
#define COMMAND_UPDATE_END            0x06
#define COMMAND_GET_FIRMWARE_VERSION  0x07
#define COMMAND_GET_BATTERY_LEVEL     0x08




class CommandCharacteristicCallBacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t* param) {

    uint8_t command_code = *(pCharacteristic->getData());

    //printf("command issued: %d\n", command_code);

    switch(command_code) {
      case COMMAND_DEEP_SLEEP: power_down(); break;
      case COMMAND_ENABLE_MUTE: set_mute(true); break;
      case COMMAND_DISABLE_MUTE: set_mute(false); break;
      case COMMAND_TOGGLE_MUTE: set_mute(!muted); break;
      case COMMAND_GET_HISTORY: sendHistory(); break;
      case COMMAND_UPDATE_START: {
        if (Update.isRunning()) {
          Update.end();
        }
        updateFirmware(); break;
      }
      case COMMAND_UPDATE_END: {
        if (Update.end(true)) {
          Serial.println("Update complete! Restarting...");
          delay(1000);
          ESP.restart();
        } else {
          Serial.printf("Update failed: %s\n", Update.errorString());
        }
        break;
      }
      case COMMAND_GET_FIRMWARE_VERSION: {
        pCharacteristicData->setValue(FIRMWARE_VERSION);
        break;
      }
      case COMMAND_GET_BATTERY_LEVEL: {
        pCharacteristicData->setValue(&battery_level, 1);
        break;
      }
    }


  }

};

class DataCharacteristicCallBacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t* param) {

    uint8_t* rx_data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (incoming_data == DATA_FIRMWARE) {
      if (!update_started) {
        Serial.println("Starting OTA update...");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Serial.printf("Update.begin failed: %s\n", Update.errorString());
          return;
        }
        update_started = true;
      }

      size_t written = Update.write(rx_data, length);
      total_firmware_size += written;
      Serial.printf("Received chunk: %d bytes (Total: %d)\n", written, total_firmware_size);

    }

  }
};



//-----------------DEEP SLEEP------------------

void init_DeepSleep() {
  rtc_gpio_pullup_en(gpio_num_t(POWER_PIN));
  rtc_gpio_pulldown_dis(gpio_num_t(POWER_PIN));
  delay(10);
  Serial.println("init deep sleep");
  attachInterrupt(POWER_PIN, ToggleDeepSleep, FALLING);
}

void IRAM_ATTR ToggleDeepSleep() {
  static int64_t lMillis = 0;

  if ((millis() - lMillis) < 5) return;

  lMillis = millis();
  disableInterrupt(POWER_PIN);

  Serial.println("going to sleep...");
  detachInterrupt(POWER_PIN);
  attachInterrupt(POWER_PIN, power_down, RISING);
}


void power_down() {
  Serial.println("now can enable again");
  detachInterrupt(POWER_PIN);

  //rtc_gpio_hold_en(gpio_num_t(POWER_PIN));

  esp_sleep_enable_ext0_wakeup(gpio_num_t(POWER_PIN), LOW);
  esp_deep_sleep_start();
}

//-----------------Calibration-----------------



//-----------------BLE SET UP-----------------

void init_BLEDataService() {
  pService = pServer->createService(BLE_DATA_SERVICE_UUID);


  pCharacteristicCtrl = pService->createCharacteristic(CHARACTERISTIC_CONTROL_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  
  pCharacteristicData = pService->createCharacteristic(CHARACTERISTIC_DATA_UUID, BLECharacteristic::PROPERTY_WRITE | 
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  
  pCharacteristicLive = pService->createCharacteristic(CHARACTERISTIC_LIVE_READING_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  //pCharacteristicCtrl->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  //pCharacteristicData->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  //pCharacteristicLive->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);

  pCharacteristicCtrl->setCallbacks(new CommandCharacteristicCallBacks());
  pCharacteristicData->setCallbacks(new DataCharacteristicCallBacks());


  BLEDescriptor *pDesciptorCtrl = new BLEDescriptor((uint16_t)0x2901);
  pDesciptorCtrl->setValue("control");
  BLEDescriptor *pDesciptorData = new BLEDescriptor((uint16_t)0x2901);
  pDesciptorData->setValue("data");
  BLEDescriptor *pDesciptorLive = new BLEDescriptor((uint16_t)0x2901);
  pDesciptorLive->setValue("live readings");

  pCharacteristicCtrl->addDescriptor(pDesciptorCtrl);
  pCharacteristicData->addDescriptor(pDesciptorData);
  pCharacteristicLive->addDescriptor(pDesciptorLive);

  pService->start();
}


void init_BLESecurity() {
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSWORD;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  Serial.println("init_BLESecurity");
}


void init_BLE() {
  BLEDevice::init("ESP-32 slave");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  BLEDevice::getAdvertising()->setMinPreferred(800);
  BLEDevice::getAdvertising()->setMaxPreferred(800);

  BLEDevice::setMTU(uint16_t (247));
  

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallBacks());

  init_BLEDataService();


  BLEAdvertising *pAdvertising = pServer->getAdvertising();


  //pServer->updateConnParams(uint8_t *remote_bda, uint16_t minInterval, uint16_t maxInterval, uint16_t latency, uint16_t timeout)

  //pAdvertising->set



  
  pAdvertising->start();

  Serial.println("began advertising");

  init_BLESecurity();
}


void DataServiceStart() {
  //pService->start();
}

void DataServiceStop() {
  //pService->stop();
}

//-----------------MPU-----------------

void init_MPU6050() {

  if (simulateMPU) return;

  pinMode(MPU_VCC_PIN, OUTPUT);
  pinMode(MPU_GND_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);

  digitalWrite(MPU_VCC_PIN, HIGH);
  digitalWrite(MPU_GND_PIN, LOW);
  

  if (!Wire.setPins(MPU_SDA_PIN, MPU_SCL_PIN)) {
    return;
  }


  Wire.begin();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();


  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    if (calibrateMPU) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);

      if (prefs.begin("mpu offsets", false)) {
        prefs.clear();
        prefs.putShort("AX", mpu.getXAccelOffset());
        prefs.putShort("AY", mpu.getYAccelOffset());
        prefs.putShort("AZ", mpu.getZAccelOffset());
        prefs.putShort("GX", mpu.getXGyroOffset());
        prefs.putShort("GY", mpu.getYGyroOffset());
        prefs.putShort("GZ", mpu.getZGyroOffset());

        prefs.end();
      }
    }
    else {
      if (prefs.begin("mpu offsets", true)) {
        
        mpu.setXAccelOffset(prefs.getShort("AX"));
        mpu.setYAccelOffset(prefs.getShort("AY"));
        mpu.setZAccelOffset(prefs.getShort("AZ"));
        mpu.setXGyroOffset(prefs.getShort("GX"));
        mpu.setYGyroOffset(prefs.getShort("GY"));
        mpu.setZGyroOffset(prefs.getShort("GZ"));
        
        prefs.end();
      }
    }
    Serial.print("active offsets: ");
    mpu.PrintActiveOffsets();
    
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MPU_INT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP init failed: ");
    Serial.println(devStatus);
  }
}






bool WithinThreshold(float score) {
  return score >= 0.5;

}

void sendReading(Reading_t reading) {
  printf("sent quaternion (%.2f, %.2f, %2.f, %2.f)\n", reading.theta, reading.x, reading.y, reading.z);

  uint8_t packet[sizeof(reading)];

  memcpy(packet, &reading, sizeof(packet));

  pCharacteristicLive->setValue(packet, sizeof(packet));
  pCharacteristicLive->notify();
}


void print_history() {


  
  // if the array is not compressed
  printf("\nscore history:\n");
  printf("size: %d\n", history.size);
  printf("last reading: %.2f\n", float(millis() - history.getStartDate())/1000.0);

  Quaternion quat = history.at(0);
  printf("[(%.2f, %.2f, %2.f, %2.f) ", quat.w, quat.x, quat.y, quat.z);

  for (int i = 1; i < history.size; i++) {
    quat = history.at(i);
    printf(", (%.2f, %.2f, %2.f, %2.f)", quat.w, quat.x, quat.y, quat.z);
  }

  printf("]\n");
  
  
}


//-----------------BUZZER SET UP-----------------
void init_Buzzer() {
  pinMode(BUZZER_GND_PIN, OUTPUT);
  pinMode(BUZZER_SIG_PIN, OUTPUT);

  digitalWrite(BUZZER_GND_PIN, LOW);
}

void BuzzerPlay(BUZZER_SOUNDS sound, int step) {
  if (muted) return;
  int freq = 0, time = 0, count = 0;
  switch (sound) {
    case BUZZER_POWER_ON:
      {
        freq = POWER_ON_NOTES[step];
        time = POWER_ON_TIMES[step];
        count = POWER_ON_NOTE_COUNT;
        break;
      }
    case BUZZER_POWER_OFF:
      {
        freq = POWER_OFF_NOTES[step];
        time = POWER_OFF_TIMES[step];
        count = POWER_OFF_NOTE_COUNT;
        break;
      }
    case BUZZER_CONNECTED:
      {
        freq = CONNECTED_NOTES[step];
        time = CONNECTED_TIMES[step];
        count = CONNECTED_NOTE_COUNT;
        break;
      }
    case BUZZER_DISCONNECTED:
      {
        freq = DISCONNECTED_NOTES[step];
        time = DISCONNECTED_TIMES[step];
        count = DISCONNECTED_NOTE_COUNT;
        break;
      }
  }

  if (step >= count) return;
  tone(BUZZER_SIG_PIN, freq, time);
  delay(time);
  BuzzerPlay(sound, step + 1);
}

void set_mute(bool status) {
  if (status == muted) return;

  muted = status;
  if (status == true) {
    noTone(BUZZER_SIG_PIN);
  }
}

void giveWarning() {
  printf("warning! bad posture!\n");
  if (!muted) {
    tone(BUZZER_SIG_PIN, 300);
  }
}

void stopWarning() {
  printf("good posture now\n");
  noTone(BUZZER_SIG_PIN);
}



// in micro seconds
void updateTimer() {
  uint64_t period;
  if (device_connected && device_authenticated) {
    period = uint64_t(READING_INTERVAL_MS * 1000);
  } else {
    period = uint64_t((1 << history.compressionCount) * (READING_INTERVAL_MS * 1000));
  }
  timerAlarm(reading_timer, period, true, 0);
  timerRestart(reading_timer);
  
}





volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

//-----------------INTERRUPTS-----------------


void sendHistory() {
  xTaskCreate(SendHistoryTask, "sending history task", 4096, NULL, 0, &history_sending_task_handle);
}

void SendHistoryTask(void *pvParameters) {
  
  
  // try not to print anything in this function please 🙏
  
  //printf("sending history!\n");

  // min = max = 100 ms
  // latency = 0
  // timeout = 2 seconds
  pServer->updateConnParams(peerAddress, 80, 80, 0, 200);

  int interval = 100;
  delay(interval*5);


  history_metadata_t history_metadata;

  history_metadata.readings_count = uint32_t(history.size);
  history_metadata.millis_since_first_reading = (millis() - history.getStartDate());
  history_metadata.micros_between_two_readings = history.getInterval();

  uint8_t metadata_packet[sizeof(history_metadata)];

  memcpy(metadata_packet, &history_metadata, sizeof(history_metadata));

  pCharacteristicData->setValue(metadata_packet, size_t(sizeof(metadata_packet)));
  pCharacteristicData->notify();
  delay(interval);
  
  
  int i = 0;
  while (i < history.size) {
    int packet_size = min(MAX_BYTES_SENT_PER_PACKET, (history.size - i) * READING_BYTE_COUNT) ;
    uint8_t packet[packet_size];

    for (int j = 0; j < packet_size; j += READING_BYTE_COUNT) {
      Quaternion quat = history.at(i);
      memcpy(packet + j + 0 * sizeof(float), &quat.w, sizeof(float));
      memcpy(packet + j + 1 * sizeof(float), &quat.x, sizeof(float));
      memcpy(packet + j + 2 * sizeof(float), &quat.y, sizeof(float));
      memcpy(packet + j + 3 * sizeof(float), &quat.z, sizeof(float));
    }

    // printf("sending byte array of size %d:\n", packet_size );
    // for (int j = 0 ; j < sizeof(packet); j++) {
    //   printf("%X", packet[i]);
    //   if (j % 4 == 0 && j > 0) printf(" ");
    // }
    // printf("\n");
      
    pCharacteristicData->setValue(packet, packet_size);
    pCharacteristicData->notify();
    delay(interval);
    
  }



  // min = max = 100 ms
  // latency = 20
  // timeout = 10 seconds
  pServer->updateConnParams(peerAddress, 800, 800, 20, 1000);


  history.clear();
  updateTimer();
  
  vTaskDelete(history_sending_task_handle);
}

void updateFirmware() {
  //pServer->updateConnParams(peerAddress, 80, 80, 0, 200);
  incoming_data = DATA_FIRMWARE;
}


TaskHandle_t readingTaskHandle;
void IRAM_ATTR onHandleReading() {
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Required for ISR safety
  xTaskNotifyFromISR(readingTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // Allow higher-priority task to execute immediately
}

void HandleReading(void *pvParameters) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for permission to start like a good boy
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) continue;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    if (printReadingsToSerial) {
      printf("\nreading acquired\n");

      #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        Serial.print("quat:\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler:\t");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr:\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal:\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld:\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
      #endif
    }

    if (device_connected && device_authenticated) {
      //Serial.println("send score");
      Reading_t reading;
      reading.theta = q.w;
      reading.x = q.x;
      reading.y = q.y;
      reading.z = q.z;
      sendReading(reading);
    } else {
      //Serial.println("append score to history");
      int previousCompression = history.compressionCount;
      
      history.push(q);
      
      if (history.compressionCount != previousCompression) {
        updateTimer();
      }

      if (printHistoryToSerial) print_history();
    }

    /*
    if (!WithinThreshold(reading.score) && !giving_warning) {
      giving_warning = true;
      giveWarning();
    }
    else if (WithinThreshold(reading.score) && giving_warning) {
      giving_warning = false;
      stopWarning();
    }
    */

    


  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  delay(50);
  //for test purposes, I'll use the button for deep sleep to make tests
  init_DeepSleep();

  init_Buzzer();
  BuzzerPlay(BUZZER_POWER_ON);

  init_MPU6050();
  init_BLE();

  reading_timer = timerBegin(uint32_t (1000000)); // 1MHz
  timerAttachInterrupt(reading_timer, &onHandleReading);
  timerAlarm(reading_timer, uint64_t(READING_INTERVAL_MS * 1000), true, 0);


  xTaskCreatePinnedToCore(HandleReading, "handle readings task", 4096, NULL, 2, &readingTaskHandle, 1);


  BuzzerPlay(BUZZER_POWER_OFF);


}


void loop() {

  delay(1000);
  return;

  //calibrate_sensor();
  //return;
  //printf("started loop\n");
  

   
    

  

}

