/* Includes ---------------------------------------------------------------- */
#include <alphabet-classification_inferencing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <ErriezBMX280.h>

/* Constant defines -------------------------------------------------------- */
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define BNO055_SAMPLERATE_DELAY_MS (10)

#define SEA_LEVEL_PRESSURE_HPA      1026.25

// BLE 통신용
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID (유아트 통신을 위해 정해진 ID)
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

ErriezBMX280 bmx280 = ErriezBMX280(0x76);

// 타이머용
bool prev = false;
bool curr = false;
unsigned long start_time = 0;   


// 터치 로직용
int threshold = 1500;   // ESP32S2 
bool touchDetected = false;
void gotTouch() {
  touchDetected = true;
}

/**
* @brief      Arduino setup function
*/

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: "); 
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};



void setup()
{
    delay(500);
    // put your setup code here, to run once:
    Serial.begin(115200);

    Serial.println(F("\nErriez BMP280/BMX280 example"));


    // BLE ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    // Create the BLE Device
      BLEDevice::init("SUJEONG");
      
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); // 콜백 함수 추가
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                        
    // 값을 지속적으로 받고 싶을때는(NOTIFY로 했을때는) Descriptor 추가해야 함!!! 
    pTxCharacteristic->addDescriptor(new BLE2902()); 

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");


    // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1)
        ;
    }

    // if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    //     ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    //     return;
    // }

    // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    // 온습도
    // Initialize I2C bus
    Wire.begin();
    Wire.setClock(400000);

    // Initialize sensor
    while (!bmx280.begin()) {
        Serial.println(F("Error: Could not detect sensor"));
    }

    // Print sensor type
    Serial.print(F("\nSensor type: "));
    switch (bmx280.getChipID()) {
        case CHIP_ID_BMP280:
            Serial.println(F("BMP280\n"));
            break;
        case CHIP_ID_BME280:
            Serial.println(F("BME280\n"));
            break;
        default:
            Serial.println(F("Unknown\n"));
            break;
    }

    bmx280.setSampling(BMX280_MODE_NORMAL,    // SLEEP, FORCED, NORMAL
                      BMX280_SAMPLING_X16,   // Temp:  NONE, X1, X2, X4, X8, X16
                      BMX280_SAMPLING_X16,   // Press: NONE, X1, X2, X4, X8, X16
                      BMX280_SAMPLING_X16,   // Hum:   NONE, X1, X2, X4, X8, X16 (BME280)
                      BMX280_FILTER_X16,     // OFF, X2, X4, X8, X16
                      BMX280_STANDBY_MS_500);// 0_5, 10, 20, 62_5, 125, 250, 500, 1000



// setUp    
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
unsigned long previousMillis = 0; // 이전 시간을 저장할 변수
const unsigned long interval = 2000; // 2초 간격


void loop() {
    unsigned long currentMillis = millis(); // 현재 시간 가져오기

    //temperature(); // 온습도 확인용

    // 2초가 지났는지 확인
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // 이전 시간을 현재 시간으로 갱신

        ei_printf("\nStarting inferencing in 2 seconds...\n");
        ei_printf("Sampling...\n");

        // Allocate a buffer here for the values we'll read from the IMU
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
            // Determine the next tick (and then sleep later)
            uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

            /* Display calibration status for each sensor. */
            uint8_t system, gyro, accel, mag = 0;
            bno.getCalibration(&system, &gyro, &accel, &mag);

            imu::Vector<3> gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            imu::Vector<3> laccVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

            buffer[ix] = gyroVector.x();
            buffer[ix + 1] = gyroVector.y();
            buffer[ix + 2] = gyroVector.z();
            buffer[ix + 3] = laccVector.x();
            buffer[ix + 4] = laccVector.y();
            buffer[ix + 5] = laccVector.z();
            delayMicroseconds(next_tick - micros());
        }

        // Turn the raw buffer into a signal which we can then classify
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        // Run the classifier
        ei_impulse_result_t result = {0};

        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        // Print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");

        bool check = false;

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (result.classification[ix].value > 0.8) {
                ei_printf("This Alphabet is %s", result.classification[ix].label);
                ei_printf("\n");
                check = true;

                // BLE 통신: 값을 BLE로 보내기
                if (deviceConnected) {
                    ei_printf("Connect Success!!!\n");

                    String message = "This Alphabet is " + String(result.classification[ix].label) + " !!!!";
                    pTxCharacteristic->setValue(message.c_str()); // BLE 값 설정
                    pTxCharacteristic->notify();                 // BLE 값 전송

                    if (String(result.classification[ix].label) == "I") {
                        String msg = "Temperature: " + String(bmx280.readTemperature()) + " C";
                        pTxCharacteristic->setValue(msg.c_str()); // BLE 값 설정
                        pTxCharacteristic->notify();
                    } else if (String(result.classification[ix].label) == "V") {
                        String msg = "V ^_^ V";
                        pTxCharacteristic->setValue(msg.c_str()); // BLE 값 설정
                        pTxCharacteristic->notify();
                    } else if (String(result.classification[ix].label) == "S") {
                      //ei_printf("Current value: %s\n", curr ? "true" : "false");
                      //ei_printf("Previous value: %s\n", prev ? "true" : "false");
                      
                      curr = true;
                      if (!prev) {
                        start_time = millis();
                      }
                      if (curr && prev) {
                       ei_printf("\n타이머 측정\n");
                        curr = false;
                        prev = false;

                        // 타이머 종료 및 경과 시간 계산
                        unsigned long elapsed_time = millis() - start_time; // 계산된 시간
                        ei_printf("Elapsed time: %lu\n", elapsed_time);

                        String msg = String(elapsed_time) + "초 입니다";
                        pTxCharacteristic->setValue(msg.c_str()); // BLE 값 설정
                        pTxCharacteristic->notify();

                      }

                    }
                }
            }
            prev = curr;
        }

        if (!check) {
            ei_printf("Please try again...\n");
            String msg = "Please try again...";
            pTxCharacteristic->setValue(msg.c_str()); // BLE 값 설정
            pTxCharacteristic->notify();
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
    }

    // 다른 작업 가능
    // 여기에서 추가 작업을 수행할 수 있습니다.
}



void temperature() {
    Serial.print(F("Temperature: "));
    Serial.print(bmx280.readTemperature());
    Serial.println(" C");

    if (bmx280.getChipID() == CHIP_ID_BME280) {
        Serial.print(F("Humidity:    "));
        Serial.print(bmx280.readHumidity());
        Serial.println(" %");
    }

    Serial.print(F("Pressure:    "));
    Serial.print(bmx280.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print(F("Altitude:    "));
    Serial.print(bmx280.readAltitude(SEA_LEVEL_PRESSURE_HPA));
    Serial.println(" m");

    Serial.println();

    delay(1000);
}
