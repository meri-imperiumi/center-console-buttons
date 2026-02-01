Lille Ø Center Console Buttons
==============================

This is a [SensESP](https://signalk.org/SensESP/) project to connect the push buttons in Lille Ø's center console to Signal K with an ESP32 microcontroller. There is handling for the following buttons:

* Autopilot on/off (pin 13, Signal K path `steering.autopilot.state`))
* Autopilot controls (Signal K path `steering.autopilot.actions.adjustHeading`):
  - -10: (pin 16)
  - +10: (pin 17)
  - -1 (pin 18)
  - +1 (pin 19)
* Virtual light switch A (pin 21, Signal K path `electrical.switches.consoleA.state`)
* Virtual light switch B (pin 22, Signal K path `electrical.switches.consoleB.state`)

All digital I/O pins listed above are designed to be connected via a momentary switch to the `GND` pin on the microcontroller.
