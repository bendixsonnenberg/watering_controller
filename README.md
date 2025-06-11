# Watering Controller
A tinkering project to automate watering plants in a greenhouse and analyzing data afterwards.  
Current Progress can be seen in "Status" at the bottom.

Codewise: an exploration of using embedded rust, specificaly [embassy](https://embassy.dev/).

## Short Overview
### Hardware
Per pot:
  - 1 STM32F103C8# 
  - capacitive moisture sensor
  - valve
  - mcp2551 can RX/TX
  - Mosfet IRFB3006 (there are alternatives, this one worked for us)

Central Controller:
  - Pico 2 W (pico 1 2 would be enough)
  - 2x16 LCD Display
  - MCP2515 for can communication.
  - UX to be determined

Additional Hardware needed:
  - Some kind of reservoir
  - Silicone tubes for delivering water to valves

### System Design
The central controller communicates with each moisture sensor over CAN-Bus.
The Specs for this Communication can be found [here](/can_protocol.md). The implementation is found [here](/can_protocol).
The moisture sensors get a threshold and use their valves to hold that threshold, currently through a simple comparision (potential PID).

## Current Status
Two pot-sensors together with central controller, currently set up in greenhouse, with CAN-Network and watering function working as far as assessable.

## Story and some Context
editing pending...
## Milestones
June '25: working menu with individual IDs and live data
## Status History
editing pending...
