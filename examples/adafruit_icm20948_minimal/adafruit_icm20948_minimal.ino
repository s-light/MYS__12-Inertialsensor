// adafruit_icm20948_minimal.ino
// gebe alle werte im *Arduino Serial-Monitor* aus.

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

void setup(void) {
    Serial.begin(115200);
    delay(10);

    Serial.println("MYS Inertialsensor (Nr 12) 9dof IMU (ICM20948)");

    // prüfe ob sensor angeschlossen ist
    if (!icm.begin_I2C()) {
        Serial.println("kein ICM20948 sensor gefunden. Prüfe Stecker!");
        while (1) {
            delay(10);
        }
    }
    Serial.println("ICM20948 gefunden!");
}

void loop() {

    // lese alle sensor daten
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    Serial.println();
    // Zeige Werte
    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" °C");

    // Beschleunigung
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    // Gyroskop
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");

    // Kompass
    Serial.print("\t\tMag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z);
    Serial.println(" uT");

    delay(100);
}