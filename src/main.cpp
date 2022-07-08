// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "eh_analog.h"
#include "eh_digital.h"
#include "eh_display.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/linear.h"

using namespace sensesp;

// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;
const int kOneWirePin = 4;
// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on SH-ESP32
const int kCANRxPin = GPIO_NUM_34;
const int kCANTxPin = GPIO_NUM_32;

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

const int kReadDelay = 2000;

// Test output pin configuration
//#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;

reactesp::ReactESP app;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Convenience function to print the addresses found on the I2C bus
void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

#ifdef ENABLE_TEST_OUTPUT_PIN
void ToggleTestOutputPin(void * parameter) {
  while (true) {
    digitalWrite(kTestOutputPin, !digitalRead(kTestOutputPin));
    delay(kTestOutputInterval);
  }
}
#endif

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ScanI2C(i2c);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

  // Initialize 1-wire bus
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(kOneWirePin);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  xTaskCreate(ToggleTestOutputPin, "toggler", 2048, NULL, 1, NULL);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engine-hat")
                    ->enable_ip_address_sensor()
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();


  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

//FloatProducer* ConnectVoltageSource(Adafruit_ADS1115* ads1115, int channel, String name, String sk_path)
  auto voltageA = ConnectVoltageSource(ads1115, 0, "port_oilpressure", "propulsion.port.oilpressure"); //A = yellow
  auto voltageB = ConnectVoltageSource(ads1115, 1, "startboard_oilpressure", "propulsion.starboard.oilpressure"); //B = red
  auto voltageC = ConnectVoltageSource(ads1115, 2, "port_voltage", "electrical.batteries.port.voltage"); //C = black
  auto voltageD = ConnectVoltageSource(ads1115, 3, "starboard_voltage", "electrical.batteries.starboard.voltage"); //D = green

  // Connect the tacho senders
  auto tacho_1_frequency = ConnectTachoSender(kDigitalInputPin1, "port");
  auto tacho_2_frequency = ConnectTachoSender(kDigitalInputPin2, "starboard");

  auto input_1 = new DigitalInputState(kDigitalInputPin3, INPUT);
  auto input_2 = new DigitalInputState(kDigitalInputPin4, INPUT);

  input_1->connect_to(new SKOutputBool("system.io1", "Digital/Input1"));
  input_2->connect_to(new SKOutputBool("system.io2", "Digital/Input2"));


  //initialize 1-wire sensors
   auto* first =
      new OneWireTemperature(dts, kReadDelay, "onewire_0");

  first->connect_to(new Linear(1.0, 0.0, "/onewire_0/linear"))
      ->connect_to(new SKOutputFloat("environment.first.temperature",
                                     "/onewire_0/skPath"));

  auto* second =
      new OneWireTemperature(dts, kReadDelay, "onewire_1");

  second->connect_to(new Linear(1.0, 0.0, "/onewire_1/linear"))
      ->connect_to(new SKOutputFloat("environment.second.temperature",
                                     "/onewire_1/skPath"));

  auto* third =
      new OneWireTemperature(dts, kReadDelay, "onewire_2");

  third->connect_to(new Linear(1.0, 0.0, "/onewire_2/linear"))
      ->connect_to(new SKOutputFloat("environment.third.temperature",
                                     "/onewire_2/skPath"));

  // Connect the alarm inputs
  //auto alarm_2_input = ConnectAlarmSender(kDigitalInputPin2, "2");
  //auto alarm_3_input = ConnectAlarmSender(kDigitalInputPin3, "3");
  //auto alarm_4_input = ConnectAlarmSender(kDigitalInputPin4, "4");

  // Update the alarm states based on the input value changes
  //alarm_2_input->connect_to(
  //    new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  //alarm_3_input->connect_to(
  //    new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  //alarm_4_input->connect_to(
  //    new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));



  // Connect the outputs to the display
  if (display_present) {
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    tacho_1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM 1", 60 * value); }));

    // Create a "christmas tree" display for the alarms
    app.onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
