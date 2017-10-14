#include <Adafruit_SSD1306.h>

#include <Adafruit_GFX.h>
#include <gfxfont.h>

#include <SoftwareSerial.h>

#include <Wire.h>

#include <SPI.h>
#include <SPI_registers.h>

#include <CurieBLE.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#define BUTTON1 4
#define BUTTON2 5
#define BUTTON3 6
#define BUTTON4 7

//sate define
#define STATE_STAND 0
#define STATE_UP 1
#define STATE_DOWN 2
#define STATE_LEFT 3
#define STATE_RIGHT 4

int oldButtonState1 = LOW;
int oldButtonState2 = LOW;
int oldButtonState3 = LOW;
int oldButtonState4 = LOW;

float oldRoll = 0;
float oldPitch = 0;
float oldYaw = 0;

int boadState;

//IMU
Madgwick filter;
unsigned long microsPerReading, microsPrevious; //
float accelScale, gyroScale;                    //
boolean calibrateOffsets = true;                //IMU를 보정 할지 안할지


//함수
void controlPeripheral(BLEDevice peripheral);  //해당하는 주변장치의 LED를 제어하는 함수 
void motionDetect();                           //모션인식 함수 
float convertRawGyro(int gRaw);                //자이로 센서값을 각속도로 변환 시켜줌 
float convertRawAcceleration(int aRaw);        //가속도 센서값을 mg단위로 변환 시켜줌

void setup() {
  //시리얼 포트 연결
  Serial.begin(9600); // initialize Serial communication
  //while (!Serial);    // wait for the serial port to open
  //Serial.println("serial port open");

  //PIN모드 세팅
  pinMode(BUTTON1,INPUT_PULLUP);
  pinMode(BUTTON2,INPUT_PULLUP);
  pinMode(BUTTON3,INPUT_PULLUP);
  pinMode(BUTTON4,INPUT_PULLUP);
  
  //IMU and filter initialize
  CurieIMU.begin();
  if (calibrateOffsets) {
  //자이로와 가속도센서의 오프셋을 설정하는 기능
  Serial.println("Internal sensor offsets BEFORE calibration…");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  
  // 올바로 오프셋이 설정되기 위해서는 제누이노보드가 수평으로 유지되어 있어야 한다.
  Serial.print("Starting Gyroscope calibration…");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration…");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");
  Serial.println("Internal sensor offsets AFTER calibration…");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
  filter.begin(25);
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  CurieIMU.setAccelerometerRange(2);    // Set the accelerometer range to 2G
  CurieIMU.setGyroRange(250);           // Set the gyroscope range to 250 degrees/second
  boadState = STATE_STAND; 
  
  //initialize the BLE hardware
  Serial.println("Initializing BLE device...");
  BLE.begin();
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");    // 해당 UUID를 가진 주변장치 검색

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;  //1초에 25번 (25Hz)
  microsPrevious = micros();
}

void loop() {
  
  BLEDevice peripheral = BLE.available();

  //해당 주변장치가 연결 되어있으면
  if (peripheral) {
    // 주변장치의 주소, 이름, 서비스 출력
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    
    BLE.stopScan();   //stop scanning

    controlPeripheral(peripheral); //해당 주변장치를 제어하는 함수 

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
    }
}


//모션인식
void motionDetect(BLECharacteristic ledCharacteristic){
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  //Serial.print("Orientation: ");
  //Serial.println(roll);
  //Serial.print(" ");
  //Serial.print(pitch);
  //Serial.print(" ");
  //Serial.print(yaw);
    
  if( boadState==STATE_STAND){
    if(pitch <= -30){
      ledCharacteristic.writeByte(0x01);  //car door open 
      Serial.println("car door open");
      boadState=STATE_UP;
    }else if(pitch >= 30){
      ledCharacteristic.writeByte(0x03);  //led2 on 
      boadState=STATE_DOWN;
    }else if(roll <= -30){
      ledCharacteristic.writeByte(0x05);  //led3 on 
      boadState=STATE_LEFT;
    }else if(roll >= 30){
      ledCharacteristic.writeByte(0x07);  //led4 on 
      boadState=STATE_RIGHT;
    }
  }else if(boadState==STATE_UP && (-5<pitch || pitch<5)){
    ledCharacteristic.writeByte(0x00);  //led1 door close
    Serial.println("car door close");
    boadState=STATE_STAND;
  }else if(boadState==STATE_DOWN && (-5<pitch || pitch<5)){
    ledCharacteristic.writeByte(0x02);  //led2 off
    boadState=STATE_STAND;
  }else if(boadState==STATE_LEFT && (-5<roll || roll<5)){
    ledCharacteristic.writeByte(0x04);  //led3 off
    boadState=STATE_STAND;
  }else if(boadState==STATE_RIGHT && (-5<roll || roll<5)){
    ledCharacteristic.writeByte(0x06);  //led4 off
    boadState=STATE_STAND;
  }
  
  //oldRoll = roll;
  //oldPitch = pitch;
  //oldYaw = yaw;
}

//해당 주변장치를 제어하는 함수 
void controlPeripheral(BLEDevice peripheral) {
  //주변장치 연결
  Serial.println("Connecting ...");
  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // 주변장치 특성 발견
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();  //실패하면 주변장치 연결 해제 PO    return;
  }

  //주변장치 서비스의 특성 검색
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    //주변장치가 연결되어 있는 동안
    //while the peripheral is connection
          // read the button pin
    int buttonState1 = digitalRead(BUTTON1);
    int buttonState2 = digitalRead(BUTTON2);
    int buttonState3 = digitalRead(BUTTON3);
    int buttonState4 = digitalRead(BUTTON4);

    //BUTTON1
    if (oldButtonState1 != buttonState1) {
      // button changed
      oldButtonState1 = buttonState1;

      if (!buttonState1) {
        Serial.println("button1 pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeByte(0x01);
      } else {
        Serial.println("button1 released");

        // button is released, write 0x00 to turn the LED of
        ledCharacteristic.writeByte(0x00);
      }
    }

    //BUTTON2
    if (oldButtonState2 != buttonState2) {
      // button changed
      oldButtonState2 = buttonState2;

      if (!buttonState2) {
        Serial.println("button2 pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeByte(0x03);
      } else {
        Serial.println("button2 released");

        // button is released, write 0x00 to turn the LED of
        ledCharacteristic.writeByte(0x02);
      }
    }

    //BUTTON3
    if (oldButtonState3 != buttonState3) {
      // button changed
      oldButtonState3 = buttonState3;

      if (!buttonState3) {
        Serial.println("button3 pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeByte(0x05);
      } else {
        Serial.println("button3 released");

        // button is released, write 0x00 to turn the LED of
        ledCharacteristic.writeByte(0x04);
      }
    }

    //BUTTON4
    if (oldButtonState4 != buttonState4) {
      // button changed
      oldButtonState4 = buttonState4;

      if (!buttonState4) {
        Serial.println("button4 pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeByte(0x07);
      } else {
        Serial.println("button4 released");

        // button is released, write 0x00 to turn the LED of
        ledCharacteristic.writeByte(0x06);
      }
    }

    unsigned long microsNow;

    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
      motionDetect(ledCharacteristic);
      microsPrevious += microsPerReading;    // increment previous time, so we keep proper pace
    }
  }

    
  Serial.println("Peripheral disconnected");
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * CurieIMU.getAccelerometerRange()) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * CurieIMU.getGyroRange()) / 32768.0;
  return g;
}

