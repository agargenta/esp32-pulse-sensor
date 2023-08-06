# esp32-pulse-sensor

## Introduction

This is an ESP32-compatible C component for interfacing with pulse (e.g. flow) sensors connected
via GPIO.

It supports multiple devices, each on a different GPIO pin.

It was tested with v5.2 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment.

## Dependencies

None.

## Example

Coming soon.

## Features

The features include:

* Cycle tracking with support for jitter-avoidance.
* Async notifications of cycle start/stop events via queues.
* Total pulse, duration, cycle tracking.
* Multiple pulse sensors can be managed at the same time.

## Documentation

Automatically generated API documentation (doxygen) is available [here](https://agargenta.github.io/esp32-pulse-sensor/index.html).

## Source Code

The source is available from [GitHub](https://www.github.com/agargenta/esp32-pulse-sensor).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

* [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)
