# Inertialsensor (IMU)

Nr. 12

Hersteller Bezeichnungen

> Adafruit
> ICM-20948 9-DoF IMU
> Product ID: 4554
> TDK InvenSense ICM-20948

![IMU](https://makeyourschool.de/wp-content/uploads/2018/10/12_intertialsensor_rz-1024x1024.jpg)
Bildquelle: *Wissenschaft im Dialog*

## Beschreibung

Der ICM-20948 ist ein sogenannter neunachsiger Inertialsensor.
er besteht aus drei Sensoren:

-   Gyroskop
-   Beschleunigungssensor
-   Magnetometer

wenn man diese jeweils drei Messwerte geschickt kombiniert kann man die _relative_ Position _im Raum_ bestimmen.

Dieser Sensor eignet sich für alle Dinge die mit Bewegung und deren Erkennung zu tun haben.

## Anschlüsse

### Kommunikation mit Mikrocontroller

-   I2C

## Kurz-Datenblatt

-   Signal: 3-5V
-   Betriebsspannung: 3-5V
-   3-Axis Gyroscope FSR of ±250 dps, ±500 dps, ±1000 dps, and ±2000 dps
-   3-Axis Accelerometer FSR of ±2g, ±4g, ±8g, and ±16g
-   3-Axis Compass up to ±4900 µT

## Library

um dieses Bauteil zu benutzen, installiere bitte diese Library mit HIlfe des Arduino-Library-Managers: 
`Adafruit ICM20X`
(alle sub-libraries mit installieren.)

## Erste Schritte

schau dir das Minimal-Beispiel an:

```c++
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
```

### Anleitung

-   nimm dir
    -   ein Arduino Uno
    -   ein Grove-Shield
    -   ein Sensor-Board
    -   ein Adapterkabel `Grove to STEMMA QT`
-   Schließe den Sensor mit dem Adapterkabel an einen I2C Steckplatz an
-   kopiere / einfügen des Beispiel Code `adafruit_icm20948_minimal.ino`
-   Sketch Hochladen
-   Öffne den Serial-Monitor (Werkzeuge - Serial-Monitor)
-   Das Sollte nun passieren:
    - die aktuellen Werte des Sensors sollten dort angezeigt werden.
    - bewege den Sensor und schaue wie sich die Werte verändern

#### Output
![IDE output example](<extras/adafruit_icm20948_minimal IDE-output.png>)

## Nächste Schritte
um die Werte besser zu verstehen hilft eine Grafische Ausgabe:
kopiere diesen etwas abgewandelten Beispiel-Code `adafruit_icm20948_plot.ino`

```c++
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

```

und nach einem Hochladen Öffne nun unter `Werkzeuge` - `Serial-Plotter`
nun sollten die Werte in einer visuellen Diagram-Form angezeigt werden.

![plot](<extras/adafruit_icm20948_minimal IDE-plot.png>)



<style>
img {
    max-height: 35vh;
    max-width: 50vw;
}
</style>
