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
    Serial.println();

    // wir geben den variablen Namen:
    // https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter/
    // aktuell muss wie unten bei jedem datensatz die namen mit drin stehen.
    // dies ist ein bug im Serial-Plotter:
    // https://github.com/arduino/arduino-serial-plotter-webapp/issues/14
    Serial.print("temp,");
    Serial.print("accel_x,");
    Serial.print("accel_y,");
    Serial.print("accel_z,");
    Serial.print("gyro_x,");
    Serial.print("gyro_y,");
    Serial.print("gyro_z,");
    Serial.print("mag_x,");
    Serial.print("mag_y,");
    Serial.print("mag_z");
    Serial.println();
}

void loop() {

    // lese alle sensor daten
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    Serial.print("temp:");
    Serial.print(temp.temperature);
    Serial.print(",");

    Serial.print("accel_x:");
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print("accel_y:");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print("accel_z:");
    Serial.print(accel.acceleration.z);
    Serial.print(",");

    Serial.print("gyro_x:");
    Serial.print(gyro.gyro.x);
    Serial.print(",");
    Serial.print("gyro_y:");
    Serial.print(gyro.gyro.y);
    Serial.print(",");
    Serial.print("gyro_z:");
    Serial.print(gyro.gyro.z);
    Serial.print(",");

    Serial.print("mag_x:");
    Serial.print(mag.magnetic.x);
    Serial.print(",");
    Serial.print("mag_y:");
    Serial.print(mag.magnetic.y);
    Serial.print(",");
    Serial.print("mag_z");
    Serial.print(mag.magnetic.z);

    Serial.println();

    delay(100);
}
