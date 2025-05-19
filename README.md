# Watering Controller
A tinkering project to automate and analyze watering plants in a greenhouse.
Also an exploration of using embedded rust, specificaly [embassy](https://embassy.dev/).
## Hardware
Per pot:
  - 1 STM32F103C8# 
  - capacitive moisture sensor
  - valve
  - mcp2551 can RX/TX
Central Controller:
  - Pico 2 W (pico 1 2 would be enough)
  - 2x16 LCD Display
  - MCP2515 for can communication.
  - UX to be determined

## System Design
The central controller communicates with each moisture sensor over CAN-Bus.
The Specs for this Communication can be found [here](/can_protocol.md). The implementation is found [here](/can_protocol).
The moisture sensors get a threshold and use their valves to hold that threshold, currently through a simple comparision(potential PID).

