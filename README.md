# Arduino-Multi-Bus-Sensor-Integration

# Arduino I2C Multi-Bus Sensor Integration

This project demonstrates how to integrate two different I2C sensors (ICM42688 IMU and DFRobot QMC5883 Compass) on an Arduino using multiple I2C buses. The project uses custom `TwoWire` objects to manage I2C communication for each sensor separately.

## Hardware Requirements

- Arduino board (e.g., Arduino Uno, Arduino Mega)
- ICM42688 IMU sensor
- DFRobot QMC5883 Compass sensor
- Connecting wires

## Wiring Diagram

| Arduino Pin | ICM42688 Pin | DFRobot QMC5883 Pin |
| ----------- | ------------ | ------------------- |
| 12 (SDA1)   | SDA          |                     |
| 13 (SCL1)   | SCL          |                     |
| 6 (SDA2)    |              | SDA                 |
| 7 (SCL2)    |              | SCL                 |
| GND         | GND          | GND                 |
| 5V          | VCC          | VCC                 |

## Software Requirements

- Arduino IDE
- `Wire.h` library (built-in with Arduino IDE)
- `ICM42688` library (provided in this repository)
- `DFRobot_QMC5883` library (provided in this repository)

## Installation

1. Download the repository and extract it.
2. Open the Arduino IDE.
3. Load the provided Arduino sketch file (`main.ino`).
4. Ensure you have the required libraries installed. You can install them using the Arduino Library Manager or manually by placing them in the `libraries` folder of your Arduino IDE installation.

## Usage

1. Connect the sensors to the Arduino board as per the wiring diagram.
2. Upload the sketch to the Arduino board.
3. Open the Serial Monitor (baud rate: 115200) to see the sensor data output.

## Code Explanation

The code involves the following steps:

1. **Include Libraries**
    ```cpp
    #include "Wire.h"
    #include "ICM42688.h"
    #include "DFRobot_QMC5883.h"
    ```

2. **Create TwoWire Objects**
    ```cpp
    TwoWire I2CBus1 = TwoWire(0);
    TwoWire I2CBus2 = TwoWire(1);
    ```

3. **Create Sensor Objects**
    ```cpp
    ICM42688 IMU(I2CBus1, 0x69);
    DFRobot_QMC5883 compass(&I2CBus2, 0x0D);
    ```

4. **Setup Function**
    - Configure I2C pins and initialize sensors.
    ```cpp
    void setup() {
      I2CBus1.begin(12, 13); // SDA, SCL for IMU
      I2CBus2.begin(6, 7); // SDA, SCL for Compass
    
      Serial.begin(115200);
      while (!Serial) {}
    
      if (IMU.begin() < 0) {
        Serial.println("IMU initialization failed.");
        while(1) {}
      }
    
      if (!compass.begin()) {
        Serial.println("Could not find a valid Compass sensor, check wiring!");
      }
      lastSecondTime = millis();
    }
    ```

5. **Loop Function**
    - Read and print sensor data periodically.
    ```cpp
    void loop() {
      unsigned long currentTime = millis();
    
      if (currentTime - lastSecondTime >= 1000) {
        Serial.print("IMU Updates per second: ");
        Serial.println(countUpdates_IMU);
        Serial.print("Compass Outputs per second: ");
        Serial.println(countUpdates_Compass);
        countUpdates_IMU = 0;
        countUpdates_Compass = 0;
        lastSecondTime = currentTime;
      }
    
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
        Serial.print("Degrees = ");
        Serial.println(mag.HeadingDegrees);
      }
    }
    ```

- 
