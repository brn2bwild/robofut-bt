#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#include "BluetoothSerial.h"
#include "Motors.h"
// #include <WiFi.h>

#define DEBUG  // Constant for debug

const int LED_BUILTIN = 2;

const int LED_BT_CLIENT_CONNECTED_DELAY = 2000;
const int LED_BT_INITIALIZED_DELAY = 300;

const int MAX_LEFT_SPEED = 800;   // Const for constrains left motor speed
const int MAX_RIGHT_SPEED = 800;  // Const for constrains right motor speed

const int SPEED_VARIATION = 5;

BluetoothSerial SerialBT;
Motors motors(27, 26, 25, 14, 12, 13, 1000, 10);

// Variables for BT communication
int bt_status = 0, delayMillis = 0;  // Variable for bt status

unsigned long previous_led_millis = millis();

char aux_char;
String incoming_command;

int left_motor_speed = 0;   // Variable for left motor speed
int right_motor_speed = 0;  // Variable for right motor speed

int speeds_array[10] = { 0, 209, 225, 288, 471, 752, 935, 998, 1014, 1020 };
int speed_index = 0;

void setup() {
  // Initialize the output variable as output
  pinMode(LED_BUILTIN, OUTPUT);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);
  // Set the onboard LED to LOW (inverted logic)
  // digitalWrite(LED_BUILTIN, !LOW);

  // SerialBT.register_callback(BT_EventHandler);  // Register callback function for BT events
  SerialBT.begin("Prueba");  // Initialize bt with the name of the device

#ifdef DEBUG
  // Initialize serial port
  Serial.begin(115200);
  delay(10);
  Serial.println("El dispositivo Bluetooth está listo para emparejarse.");
#endif
}

// Bluetooth Event Handler CallBack Function Definition
// void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
//   // Serial.println(xPortGetCoreID());
//   bt_status = event;

//   if (event == ESP_SPP_START_EVT) {
//     Serial.println("Initialized SPP");
//   } else if (event == ESP_SPP_SRV_OPEN_EVT) {
//     Serial.println("Client connected");
//   } else if (event == ESP_SPP_CLOSE_EVT) {
//     Serial.println("Client disconnected");
//   } else if (event == ESP_SPP_DATA_IND_EVT) {
//     // Serial.println("Data received");

//     if (SerialBT.available()) {
//       incoming_command = SerialBT.readStringUntil('\n');
//       incoming_command.trim();
//       Serial.println(incoming_command);
//     }
//   }
// }

void set_direction() {
  // if (incoming_command == "f") {
  //   left_motor_speed += SPEED_VARIATION;
  //   right_motor_speed += SPEED_VARIATION;
  // } else if (incoming_command == "b") {
  //   left_motor_speed -= SPEED_VARIATION;
  //   right_motor_speed -= SPEED_VARIATION;
  // } else if (incoming_command == "l") {
  //   left_motor_speed -= SPEED_VARIATION;
  //   right_motor_speed += SPEED_VARIATION;
  // } else if (incoming_command == "r") {
  //   left_motor_speed += SPEED_VARIATION;
  //   right_motor_speed -= SPEED_VARIATION;
  // } else if (incoming_command == "s") {
  //   if (left_motor_speed > 0) left_motor_speed -= SPEED_VARIATION;
  //   else left_motor_speed += SPEED_VARIATION;

  //   if (right_motor_speed > 0) right_motor_speed -= SPEED_VARIATION;
  //   else right_motor_speed += SPEED_VARIATION;
  // }

  if (incoming_command != "s") {
    speed_index += 1;
  } else {
    speed_index -= 1;
  }

  if(speed_index >= 9 ) speed_index = 9;

  left_motor_speed = constrain(speeds_array[speed_index], -MAX_LEFT_SPEED, MAX_LEFT_SPEED);
  right_motor_speed = constrain(speeds_array[speed_index], -MAX_RIGHT_SPEED, MAX_RIGHT_SPEED);

  if (incoming_command == "l") {
    left_motor_speed = left_motor_speed * (-1);
  } else if (incoming_command == "r") {
    right_motor_speed = right_motor_speed * (-1);
  } else if (incoming_command == "b") {
    left_motor_speed = left_motor_speed * (-1);
    right_motor_speed = right_motor_speed * (-1);
  }

  motors.speeds(1020, 1020);
}

bool waitMillis(unsigned long* t, long timeMillis) {
  if ((millis() - *t) > timeMillis) {
    *t = millis();

    return true;
  }

  if (millis() < *t) *t = millis();

  return false;
}

void loop() {
  // motors.speeds(600, 600);

  if (SerialBT.available()) {
    incoming_command = SerialBT.readStringUntil('\n');
    incoming_command.trim();
    // Serial.println(incoming_command);
    set_direction();
  }

  if (waitMillis(&previous_led_millis, delayMillis)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // // switch (bt_status) {
  // //   case ESP_SPP_SRV_OPEN_EVT:
  // //   case ESP_SPP_DATA_IND_EVT:
  // //     delayMillis = LED_BT_CLIENT_CONNECTED_DELAY;
  // //     break;
  // //   default:
  // //     delayMillis = LED_BT_INITIALIZED_DELAY;
  // //     break;
  // // }
}