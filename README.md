# esp32-pulse-sensor

## Introduction

This is an ESP32-compatible C component for interfacing with pulse sensors (e.g.
[Hall-effect](https://en.wikipedia.org/wiki/Hall_effect)-based flow-meters) via GPIO (interrupts).

Note, in this context, "pulses" assume quick changes in voltage on a wire response to an external
signal (e.g. water spinning a magnet around a sensor).

It was tested with v5.2 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment.

## Dependencies

* [GPIO ISR Service](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv424gpio_install_isr_servicei) (comes with ESP-IDF), must be started for the pulse sensor to work.

## Example

See [esp32-pulse-sensor-example](https://github.com/agargenta/esp32-pulse-sensor-example) for a complete example.

## Features

The features include:

* Cycle tracking with support for jitter-avoidance.
* Async notifications of cycle start/stop events via queues.
* Total pulse, duration, cycle tracking.
* Multiple pulse sensors can be managed at the same time (each on a different GPIO interrupt-capable input pin).

## Documentation

Automatically generated API documentation (doxygen) is available [here](https://agargenta.github.io/esp32-pulse-sensor/index.html).

## Source Code

The source is available from [GitHub](https://www.github.com/agargenta/esp32-pulse-sensor).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

* [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)
