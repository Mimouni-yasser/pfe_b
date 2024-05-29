# Project Name

the project in its most basic form is a smart IOT box that reads data from modular and configurable sensors and sends it through BLE with some custom GATT characteristics, included in the repo is the source code for the application with zephyr, the custom board dts based on an nrf52840 qfaa and altium design files for the custom pcb.

## content

- [src/](src/) this is the source for the zephyr app
- [altium designer files](Altium%20Designer%20files/) this folder is the root for an altium project, it contains the custom pcb design files and rules as well as some extras
- [boards/arm/smart_box](boards/arm/smart_box/) device tree for the custom pcb, still working out the kinks but so far the base funtionallities work.
