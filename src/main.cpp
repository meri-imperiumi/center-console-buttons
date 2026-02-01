// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>

#include "sensesp.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_put_request.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include <sensesp/transforms/transform.h>
#include <sensesp/transforms/debounce.h> 
#include <sensesp/transforms/lambda_transform.h>
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

String default_pilot = "raymarineST";
String autopilot_state = "standby";

struct AutopilotButtonConfig {
  uint8_t pin;
  int value;
};

uint8_t autopilot_state_pin = 13;
std::vector<AutopilotButtonConfig> pilot_buttons = {
    {16, -10},
    {17, 10},
    {18, -1},
    {19, 1},
};
uint8_t decklight_pin = 21;
bool decklight_state = false;
uint8_t dodgerlight_pin = 22;
bool dodgerlight_state = false;

std::vector<std::shared_ptr<DigitalInputChange>> pilot_button_sensors; 

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("center-console-buttons")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.2.105", 80)
                    ->get_app();

  // Subscribe to autopilot buttons
  for (const auto& btn_cfg : pilot_buttons) {
    auto digital_input = std::make_shared<DigitalInputChange>(btn_cfg.pin, INPUT_PULLUP, CHANGE);
    pilot_button_sensors.push_back(digital_input);

    auto value_map = new LambdaTransform<bool, int>([btn_cfg](bool input) {
        // Only change on low
        if (input) {
          return 0;
        }
        return btn_cfg.value;
    });
    auto* debounced = new DebounceBool(100);
    auto* put_request = new SKPutRequest<int>(
        "steering.autopilot.actions.adjustHeading",
        "",
        false
    );
    digital_input
        ->connect_to(debounced)
        ->connect_to(value_map)
        ->connect_to(put_request);

    debugD("Subscribed to PIN: %d", btn_cfg.pin);
  }

  // Autopilot state handling
  auto* autopilot_state_listener = new StringSKListener("steering.autopilot.state");
  auto autopilot_state_listener_consumer = std::make_shared<LambdaConsumer<String>>([](String input) {
    debugD("Autopilot is set to: %s", input);
    autopilot_state = input;
  });
  autopilot_state_listener
    ->connect_to(autopilot_state_listener_consumer);

  // Button for switching autopilot state
  auto autopilot_state_input = std::make_shared<DigitalInputChange>(autopilot_state_pin, INPUT_PULLUP, CHANGE);
  auto autopilot_state_change = new LambdaTransform<bool, String>([](bool input) {
    if (autopilot_state == "standby") {
      if (input) {
        return "standby";
      }
      debugD("Setting autopilot to auto");
      autopilot_state = "auto";
      return "auto";
    }
    if (input) {
      return "auto";
    }
    // TODO: Do we want 'wind' mode also here?
    debugD("Setting autopilot to standby");
    autopilot_state = "standby";
    return "standby";
  });
  auto* autopilot_state_input_debounced = new DebounceBool(100);
  auto* autopilot_state_put_request = new SKPutRequest<String>(
    "steering.autopilot.state",
    "",
    false
  );
  autopilot_state_input
    ->connect_to(autopilot_state_input_debounced)
    ->connect_to(autopilot_state_change)
    ->connect_to(autopilot_state_put_request);

  // Button for switching deck light on/off. Every press should switch state
  auto decklight_input = std::make_shared<DigitalInputChange>(decklight_pin, INPUT_PULLUP, CHANGE);
  auto decklight_change = new LambdaTransform<bool, bool>([](bool input) {
    if (input) {
      return decklight_state;
    }
    if (decklight_state) {
      decklight_state = false;
      return decklight_state;
    }
    decklight_state = true;
    return decklight_state;
  });
  auto* decklight_input_debounced = new DebounceBool(100);
  decklight_input
    ->connect_to(decklight_input_debounced)
    ->connect_to(decklight_change)
    ->connect_to(new SKOutputBool(
      "electrical.switches.consoleA.state",              // Signal K path
      "/electrical/switches_consoleA/state",             // configuration path
      new SKMetadata("",                                // No units for boolean values
        "Electrical switch console A state")            // Value description
      ));

  auto dodgerlight_input = std::make_shared<DigitalInputChange>(dodgerlight_pin, INPUT_PULLUP, CHANGE);
  auto dodgerlight_change = new LambdaTransform<bool, bool>([](bool input) {
    if (input) {
      return dodgerlight_state;
    }
    if (dodgerlight_state) {
      dodgerlight_state = false;
      return dodgerlight_state;
    }
    dodgerlight_state = true;
    return dodgerlight_state;
  });
  auto* dodgerlight_input_debounced = new DebounceBool(100);
  dodgerlight_input
    ->connect_to(dodgerlight_input_debounced)
    ->connect_to(dodgerlight_change)
    ->connect_to(new SKOutputBool(
      "electrical.switches.consoleB.state",              // Signal K path
      "/electrical/switches_consoleB/state",             // configuration path
      new SKMetadata("",                                // No units for boolean values
        "Electrical switch console B state")            // Value description
      ));

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
