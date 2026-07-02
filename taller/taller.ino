#include "BluetoothSerial.h"
#include "Motors.h"

BluetoothSerial ControlBT;
Motors motores(27, 26, 25, 14, 12, 13, 1000, 10);

void setup() {
  Serial.begin(115200);

  delay(10);

  ControlBT.begin("tesi");  // Bluetooth device name
  Serial.println("Bluetooth device is ready to pair.");
  motores.speeds(200, 200);
}

void loop() {
  if (ControlBT.available()) {
    char caracter = ControlBT.read();  // Send data from Bluetooth to serial monitor

    if (caracter == 'f') {
      Serial.println("adelante");
    }
    if (caracter == 'b') {
      Serial.println("atrás");
    }
    if (caracter == 'l') {
      Serial.println("izquierda");
    }
    if (caracter == 'r') {
      Serial.println("derecha");
    }
    if (caracter == 's') {
      Serial.println("detener");
    }
  }
}
