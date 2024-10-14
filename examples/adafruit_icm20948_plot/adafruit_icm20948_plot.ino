// adafruit_icm20948_plot.ino
// gebe alle werte in *Arduino Serial-Plotter* format aus.

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
    Serial.println("Bitte Arduino-Plotter starten.");
    Serial.println("(Werkzeuge - Serial-Plotter)");
    
    // wir geben den variablen namen:
    // https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter/
    Serial.print("temp:0,");
    Serial.print("accel_x:0,");
    Serial.print("accel_y:0,");
    Serial.print("accel_z:0,");
    Serial.print("gyro_x:0,");
    Serial.print("gyro_y:0,");
    Serial.print("gyro_z:0,");
    Serial.print("mag_x:0,");
    Serial.print("mag_y:0,");
    Serial.print("mag_z:0");
    Serial.println();
}

void loop() {

    // lese alle sensor daten
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    Serial.print(temp.temperature);

    Serial.print(",");

    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print(accel.acceleration.z);

    Serial.print(",");
    Serial.print(gyro.gyro.x);
    Serial.print(",");
    Serial.print(gyro.gyro.y);
    Serial.print(",");
    Serial.print(gyro.gyro.z);

    Serial.print(",");
    Serial.print(mag.magnetic.x);
    Serial.print(",");
    Serial.print(mag.magnetic.y);
    Serial.print(",");
    Serial.print(mag.magnetic.z);

    Serial.println();

    delay(1);
}