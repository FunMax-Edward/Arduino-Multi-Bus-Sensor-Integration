#include "Wire.h"
#include "ICM42688.h"
#include "DFRobot_QMC5883.h"

// 创建两个Wire对象
TwoWire I2CBus1 = TwoWire(0);
TwoWire I2CBus2 = TwoWire(1);

// 传感器对象
ICM42688 IMU(I2CBus1, 0x69);
DFRobot_QMC5883 compass(&I2CBus2, 0x0D);  // 确保I2C地址正确

// 时间戳和时间片
unsigned long lastUpdateTime_IMU = 0;
unsigned long lastUpdateTime_Compass = 0;
unsigned long lastSecondTime = 0;
const unsigned long timeSlice_IMU = 10; // IMU时间片为100毫秒
const unsigned long timeSlice_Compass = 10; // Compass时间片为10毫秒

// 输出计数
int countUpdates_IMU = 0;
int countUpdates_Compass = 0;

void setup() {
  // 配置第一个I2C总线的引脚
  I2CBus1.begin(12, 13); // SDA, SCL

  // 配置第二个I2C总线的引脚
  I2CBus2.begin(6, 7); // SDA, SCL
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {}

  // 初始化IMU
  if (IMU.begin() < 0) {
    Serial.println("IMU initialization failed.");
    while(1) {}
  }

  // 初始化Compass
  if (!compass.begin()) {
    Serial.println("Could not find a valid Compass sensor, check wiring!");
  }
  lastSecondTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // 每秒输出更新次数
  if (currentTime - lastSecondTime >= 1000) {
    Serial.print("IMU Updates per second: ");
    Serial.println(countUpdates_IMU);
    Serial.print("Compass Outputs per second: ");
    Serial.println(countUpdates_Compass);
    countUpdates_IMU = 0;
    countUpdates_Compass = 0;
    lastSecondTime = currentTime;
  }

  // IMU数据更新
  if (currentTime - lastUpdateTime_IMU >= timeSlice_IMU) {
    lastUpdateTime_IMU = currentTime;
    countUpdates_IMU++;
    IMU.getAGT();
    Serial.print(IMU.accX(), 6);
    Serial.print("\t");
    Serial.print(IMU.accY(),6);
    Serial.print("\t");
    Serial.print(IMU.accZ(),6);
    Serial.print("\t");
    Serial.print(IMU.gyrX(),6);
    Serial.print("\t");
    Serial.print(IMU.gyrY(),6);
    Serial.print("\t");
    Serial.print(IMU.gyrZ(),6);
    Serial.print("\t");
    Serial.println(IMU.temp(),6);
  }

  // Compass数据更新
  if (currentTime - lastUpdateTime_Compass >= timeSlice_Compass) {
    lastUpdateTime_Compass = currentTime;
    countUpdates_Compass++;
    compass.setDeclinationAngle((4.0 + (26.0 / 60.0)) / (180 / PI));
    sVector_t mag = compass.readRaw();
    Serial.print("X: ");
    Serial.print(mag.XAxis);
    Serial.print(" Y:");
    Serial.print(mag.YAxis);
    Serial.print(" Z:");
    Serial.println(mag.ZAxis);
    Serial.print("Degress = ");
    Serial.println(mag.HeadingDegress);
  }
}
