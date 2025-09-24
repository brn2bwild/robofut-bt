#include "BluetoothSerial.h"
#include "Motors.h"

#define LED_BUILTIN 2  // Define led builtin pin esp32 dev module

BluetoothSerial SerialBT;
Motors motors;

const unsigned long led_bt_client_connected_delay = 7;
const unsigned long led_bt_bt_initialized_delay = 1;

char incoming_command;  // Variable for receive data from bt

int speed = 180;  // Variable for speed

unsigned long previous_led_millis = 0;
int leg_brightness = 0;
int brightness_step = 1;

int bt_status = 0;  // Variable for bt status

void setup() {
  // Initialize the output variable as output
  pinMode(LED_BUILTIN, OUTPUT);
  // Set the onboard LED to LOW (inverted logic)
  digitalWrite(LED_BUILTIN, !LOW);

  // Initialize serial port
  Serial.begin(115200);
  delay(10);

  SerialBT.begin("prueba2");             // Initialize bt with the name of the device
  SerialBT.register_callback(BT_EventHandler);  // Register callback function for BT events
  Serial.println("El dispositivo Bluetooth está listo para emparejarse.");

  motors.begin(speed);
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
  switch (incoming_command) {
    case 'f':
      motors.go_forward();
      Serial.println("go forward");
      break;

    case 'b':
      motors.go_back();
      Serial.println("go back");
      break;

    case 'l':
      motors.go_left();
      Serial.println("go left");
      break;

    case 'r':
      motors.go_right();
      Serial.println("go right");
      break;

    case 's':
      motors.stop();
      Serial.println("stop");
      break;
  }
}

void loop() {
  if (millis() - previous_led_millis >= led_bt_client_connected_delay && bt_status == ESP_SPP_SRV_OPEN_EVT) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }

  if (millis() - previous_led_millis >= led_bt_bt_initialized_delay && (bt_status == ESP_SPP_START_EVT || bt_status == ESP_SPP_CLOSE_EVT)) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }
}