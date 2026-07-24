// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <cmath>
#include <Wire.h>

const double grav = 9.81;

int in1Pin = 18;
int in2Pin = 19;
int enaPin = 22;
int ena2Pin = 23;
int in3Pin = 17;
int in4Pin = 16;

//Global Variables
double throttle = 0.0;
double angle = 0.0;

double pitchAccel = 0.0;
double rollAccel = 0.0;


double heading = 0.0;       // Current integrated heading in degrees
double gyroYBias = 0.0;     // Gyro bias in degrees/second
uint32_t previousIMUTime = 0;


// Custom MPU6050 I2C driver using direct register access
// to initialize the IMU and acquire real-time sensor data.
constexpr uint8_t MPU_ADDRESS = 0x68;

int16_t accelXRaw = 0;
int16_t accelYRaw = 0;
int16_t accelZRaw = 0;

int16_t gyroXRaw = 0;
int16_t gyroYRaw = 0;
int16_t gyroZRaw = 0;

bool writeMPURegister(uint8_t registerAddress, uint8_t value) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(registerAddress);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool readMPURegisters(uint8_t startRegister, uint8_t* buffer, size_t length) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(startRegister);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t received = Wire.requestFrom(
        static_cast<uint8_t>(MPU_ADDRESS),
        length,
        true
    );

    if (received != length) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }

    return true;
}

bool initializeMPU6050() {
    // Wake the MPU6050 by clearing the sleep bit.
    if (!writeMPURegister(0x6B, 0x00)) {
        return false;
    }

    delay(100);

    // Gyroscope range: ±250 degrees/second.
    if (!writeMPURegister(0x1B, 0x00)) {
        return false;
    }

    // Accelerometer range: ±2 g.
    if (!writeMPURegister(0x1C, 0x00)) {
        return false;
    }

    return true;
}

bool readMPU6050() {
    uint8_t data[14];

    // Accelerometer data begins at register 0x3B.
    if (!readMPURegisters(0x3B, data, sizeof(data))) {
        return false;
    }

    accelXRaw = static_cast<int16_t>((data[0] << 8) | data[1]);
    accelYRaw = static_cast<int16_t>((data[2] << 8) | data[3]);
    accelZRaw = static_cast<int16_t>((data[4] << 8) | data[5]);

    // data[6] and data[7] contain temperature.

    gyroXRaw = static_cast<int16_t>((data[8] << 8) | data[9]);
    gyroYRaw = static_cast<int16_t>((data[10] << 8) | data[11]);
    gyroZRaw = static_cast<int16_t>((data[12] << 8) | data[13]);

    return true;
}

bool updateHeading() {
    if (!readMPU6050()) {
        return false;
    }

    uint32_t currentTime = micros();

    // On the first call, there is no previous time yet.
    if (previousIMUTime == 0) {
        previousIMUTime = currentTime;
        return true;
    }

    double dt =
        static_cast<double>(currentTime - previousIMUTime) / 1000000.0;

    previousIMUTime = currentTime;

    // Prevent a large timing gap from causing a large heading jump.
    if (dt <= 0.0 || dt > 0.1) {
        return false;
    }

    // ±250 deg/s range means 131 raw units per degree/second.
    double gyroYDegreesPerSecond =
        gyroYRaw / 131.0;

    double correctedGyroY =
        gyroYDegreesPerSecond - gyroYBias;

    // Ignore very small stationary gyro noise.
    if (fabs(correctedGyroY) < 0.3) {
        correctedGyroY = 0.0;
    }

    // Angular velocity × elapsed time = change in angle.
    heading += correctedGyroY * dt;

    // Keep the heading between -180 and +180 degrees.
    while (heading > 180.0) {
        heading -= 360.0;
    }

    while (heading < -180.0) {
        heading += 360.0;
    }

    return true;
}


bool calibrateGyroY() {
    constexpr int sampleCount = 1000;

    int validSamples = 0;
    int64_t rawTotal = 0;

    Console.println("Keep the robot completely still...");

    for (int i = 0; i < sampleCount; i++) {
        if (readMPU6050()) {
            rawTotal += gyroYRaw;
            validSamples++;
        }

        delay(2);
    }

    if (validSamples == 0) {
        Console.println("Gyro calibration failed");
        return false;
    }

    double averageRaw =
        static_cast<double>(rawTotal) / validSamples;

    gyroYBias = averageRaw / 131.0;

    heading = 0.0;
    previousIMUTime = micros();

    Console.printf(
        "Gyro Y bias: %.4f deg/s\n",
        gyroYBias
    );

    return true;
}


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons()  // bitmask of pressed "misc" buttons
    );
}



//Tells the motor to spin forward and backwards based off pwm values
void driveLeft(int speed) {


    if (speed >26) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(enaPin, abs(speed));
        //digitalWrite(in3Pin, HIGH);
        //digitalWrite(in4Pin, LOW);
        //analogWrite(ena2Pin, abs(speed));
    } else if (speed < -26) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(enaPin, abs(speed));
        //digitalWrite(in3Pin, LOW);
        //digitalWrite(in4Pin, HIGH);
        //analogWrite(ena2Pin, abs(speed));
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        //digitalWrite(in3Pin, LOW);
        //digitalWrite(in4Pin, LOW);
    }

}

void driveRight(int speed) {


    if (speed >26) {
        digitalWrite(in3Pin, HIGH);
        digitalWrite(in4Pin, LOW);
        analogWrite(ena2Pin, abs(speed));
        //digitalWrite(in3Pin, HIGH);
        //digitalWrite(in4Pin, LOW);
        //analogWrite(ena2Pin, abs(speed));
    } else if (speed < -26) {
        digitalWrite(in3Pin, LOW);
        digitalWrite(in4Pin, HIGH);
        analogWrite(ena2Pin, abs(speed));
        //digitalWrite(in3Pin, LOW);
        //digitalWrite(in4Pin, HIGH);
        //analogWrite(ena2Pin, abs(speed));
    } else {
        digitalWrite(in3Pin, LOW);
        digitalWrite(in4Pin, LOW);
        //digitalWrite(in3Pin, LOW);
        //digitalWrite(in4Pin, LOW);
    }

}

double usefulAngle(double Throttle, double Angle) {
        if (Throttle <10) {
            return 0.0f;
        }
    
        return Angle;
    }


void getPitchandRoll() {
    readMPU6050();

    double accelX = accelXRaw / 16384.0 * grav;
    double accelY = accelYRaw / 16384.0 * grav;
    double accelZ = accelZRaw / 16384.0 * grav;

    double gyroX = (gyroXRaw / 131.0) * PI / 180.0;
    double gyroY = (gyroYRaw / 131.0) * PI / 180.0;
    double gyroZ = (gyroZRaw / 131.0) * PI / 180.0;

     pitchAccel = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));
     rollAccel = atan2(accelY, accelZ);
}



void processGamepad(ControllerPtr ctl) {

    //Maps the joystick values to pwm values
    int motorSpeedY = map(ctl ->axisY(), -512, 511, 255, -255);
    int motorSpeedX = map(ctl ->axisX(), -512, 511, -255, 255);
    int forward{motorSpeedY};
    int turn{motorSpeedX};

    int rightMotorSpeed = constrain(forward + turn, -255, 255);
    int leftMotorSpeed = constrain(forward - turn, -255, 255);

driveLeft(leftMotorSpeed);
driveRight(rightMotorSpeed);


    //Uses pwm values to calculate a throttle percentage from -100 to 100
    double throttlePercentY = (motorSpeedY / 255.0) *100.0;
    double throttlePercentX = (motorSpeedX / 255.0) *100.0;


    //Pythagorean theorem to solve for force vector and theta and update global variable
     throttle = constrain(sqrt((throttlePercentX*throttlePercentX)+(throttlePercentY * throttlePercentY)), 0, 100);
     angle = atan2(throttlePercentY, throttlePercentX)*180/PI;
}



void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}


// Arduino setup function. Runs in CPU 1
void setup() {

    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enaPin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
    pinMode(ena2Pin, OUTPUT);

    Wire.begin(21, 4);

    //Initialize and calibrate mpu
    if (!initializeMPU6050()) {
    Console.println("MPU6050 initialization failed!");
} else {
    Console.println("MPU6050 initialized!");

    calibrateGyroY();
}

    //Initialize the Bluetooth
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());

    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
}

void updateControl(){
    bool dataUpdated = BP32.update();

    if (dataUpdated) {
        processControllers();
    }
}
// Arduino loop function. Runs in CPU 1.
void loop() {

        updateControl();
        getPitchandRoll();
        updateHeading();

        Console.printf( "Throttle: %5.1f%%   Angle: %6.1f°  |  Pitch: %7.2f°   Roll: %7.2f° | Heading: %7.2f\n",
        throttle,
        usefulAngle(throttle, angle),
        pitchAccel,
        rollAccel,
        heading
    );

    delay(20);
}
