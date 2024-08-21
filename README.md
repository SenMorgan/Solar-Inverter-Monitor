# Solar Inverter Monitor

This project is a simple monitoring system for solar panels. It uses an ESP8266 microcontroller to read data from the INA226
current and voltage sensor and LED output of solar inverter board. The data is then sent to the Home Assistant using MQTT.<br>
The project was created in PlatformIO 15.05.2023

[![ESP8266](https://img.shields.io/badge/ESP-8266-000000.svg?longCache=true&style=flat&colorA=AA101F)](https://www.espressif.com/en/products/socs/esp8266)<br>
[![Build with PlatformIO](https://img.shields.io/badge/Build%20with-PlatformIO-orange)](https://platformio.org/)<br>
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

<br>

## Build and settings
1. Install (if you haven't already) [PlatformIO](https://platformio.org/) extension for VS Code.
2. Create a copy of `platformio_override.ini.example` and rename it to `platformio_override.ini`.
3. Set your own values in `platformio_override.ini`.
4. Update hardware and other settings in `lib/defs/def.h` file if needed.
5. Build and upload the project to your ESP8266 device using PlatformIO.

## Home Assistant YAML configuration
```yaml
sensor:
  - name: "Solar Panel Voltage"
    device_class: voltage
    state_topic: "/solar-inverter-monitor/state/volt"
    value_template: "{{ value | default(0) }}"
    unit_of_measurement: "V"
    state_class: measurement

  - name: "Solar Panel Current"
    device_class: current
    state_topic: "/solar-inverter-monitor/state/amp"
    value_template: "{{ value | default(0) }}"
    unit_of_measurement: "A"
    state_class: measurement

  - name: "Solar Panel Power"
    device_class: power
    state_topic: "/solar-inverter-monitor/state/watt"
    value_template: "{{ value | default(0) }}"
    unit_of_measurement: "W"
    state_class: measurement

  - name: "Solar Panel Energy"
    device_class: energy
    state_topic: "/solar-inverter-monitor/state/wh"
    value_template: "{{ value | default(0) }}"
    unit_of_measurement: "Wh"
    state_class: total_increasing

  - name: "Solar Panel Inverter Temperature"
    device_class: temperature
    state_topic: "/solar-inverter-monitor/state/temp"
    value_template: "{{ value | default(0) }}"
    unit_of_measurement: "°C"
    state_class: measurement

  - name: "Solar Panel State"
    state_topic: "/solar-inverter-monitor/state/mppt"
    value_template: "{{ value }}"

  - name: "Solar Panel Error"
    state_topic: "/solar-inverter-monitor/state/error"
    value_template: "{{ value }}"

  - name: "Solar Panel Signal Strength"
    state_topic: "/solar-inverter-monitor/state/signal-quality"
    value_template: "{{ value }}"
    unit_of_measurement: "%"

  - name: "Solar Panel Uptime"
    state_topic: "/solar-inverter-monitor/state/uptime"
    value_template: "{{ value | default(0) }}"
    state_class: total_increasing
    device_class: duration
    unit_of_measurement: s

button:
  - name: "Solar Inverter Monitor Reset"
    command_topic: "/solar-inverter-monitor/set/reset"
    payload_press: "1"
    entity_category: "config"
    device_class: "restart"
  - name: "Solar Inverter Monitor Reset Wh"
    command_topic: "/solar-inverter-monitor/set/wh-reset"
    payload_press: "1"
    entity_category: "config"
```

## Dependencies
All dependencies will be automatically installed by PlatformIO

## Copyright
Copyright (c) 2024 Sen Morgan. Licensed under the MIT license, see LICENSE.md