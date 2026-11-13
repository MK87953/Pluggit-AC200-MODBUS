# Pluggit-AC200-MODBUS
reading and writing values via modbus 


This script reads values from the Pluggit AC200 ventilation system over Modbus and propagate it over MQTT for Home Assistant
I use this level 
  - 0 for no person in houes 
  - 1 for winter
  - 2 for sommer

Requirement:
- It is necessary to close the jumper JP1. This disables the remote control!
- `sudo pip3 install -U minimalmodbus pymodbus paho-mqtt`
- start the script or create a service (see pluggit.service)

Parameter list are in the config file. 
Many unused option are comment out.
