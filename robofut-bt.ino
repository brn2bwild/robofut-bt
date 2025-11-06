#include "BluetoothSerial.h"
#include "Motors.h"

// #define DEBUG // Constante para debuggear

#define LED_BUILTIN 2  // Define led builtin pin esp32 dev module

#define LEFT_SPEED 180   // Const for left motor speed
#define RIGHT_SPEED 180  // Const for right motor speed

#define LED_BT_CLIENT_CONNECTED_DELAY 7
#define LED_BT_INITIALIZED_DELAY 1

BluetoothSerial SerialBT;
Motors motors;

char incoming_command;  // Variable for receive data from bt

unsigned long previous_led_millis = 0;
int leg_brightness = 0;
int brightness_step = 1;

int bt_status = 0;  // Variable for bt status

void setup() {
  // Initialize the output variable as output
  pinMode(LED_BUILTIN, OUTPUT);
  // Set the onboard LED to LOW (inverted logic)
  digitalWrite(LED_BUILTIN, !LOW);

  SerialBT.begin("Oveja");                      // Initialize bt with the name of the device
  SerialBT.register_callback(BT_EventHandler);  // Register callback function for BT events

#ifdef DEBUG
  // Initialize serial port
  Serial.begin(115200);
  delay(10);
  Serial.println("El dispositivo Bluetooth está listo para emparejarse.");
#endif
}

// Bluetooth Event Handler CallBack Function Definition
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_START_EVT) {
    bt_status = event;
    Serial.println("Initialized SPP");
  } else if (event == ESP_SPP_SRV_OPEN_EVT) {
    bt_status = event;
    Serial.println("Client connected");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    bt_status = event;
    Serial.println("Client disconnected");
  } else if (event == ESP_SPP_DATA_IND_EVT) {
    // Serial.println("Data received");
    if (SerialBT.available()) {
      incoming_command = SerialBT.read();
      set_direction();
    }
  }
}

void set_direction() {
#ifndef DEBUG
  switch (incoming_command) {
    case 'f':
      motors.motors(LEFT_SPEED, RIGHT_SPEED);
      break;
    case 'b':
      motors.motors(-LEFT_SPEED, -RIGHT_SPEED);
      break;
    case 'l':
      motors.motors(-LEFT_SPEED, RIGHT_SPEED);
      break;
    case 'r':
      motors.motors(LEFT_SPEED, -RIGHT_SPEED);
      break;
    case 's':
      motors.motors(0, 0);
      break;
  }
#endif

#ifdef DEBUG
  switch (incoming_command) {
    case 'f':
      Serial.println("go forward");
      break;
    case 'b':
      Serial.println("go back");
      break;
    case 'l':
      Serial.println("go left");
      break;
    case 'r':
      Serial.println("go right");
      break;
    case 's':
      Serial.println("stop");
      break;
  }
#endif
}

void loop() {
  if (millis() - previous_led_millis >= LED_BT_CLIENT_CONNECTED_DELAY && bt_status == ESP_SPP_SRV_OPEN_EVT) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }

  if (millis() - previous_led_millis >= LED_BT_INITIALIZED_DELAY && (bt_status == ESP_SPP_START_EVT || bt_status == ESP_SPP_CLOSE_EVT)) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }
}