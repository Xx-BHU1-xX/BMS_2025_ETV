## Project archive for the 2025 ETV BMS
- Refer to the `hardware` section for design choices and PCB files.
- Refer to the `software` section for firmware dev.

## Yet to be implemented:
- Charging control and charger CAN comms
	- Control charging current and voltage according to INR1650 charge specs
	- J1939 CAN implementation for commanding charging current and voltage 
	- Should be relatively straightforard to implement with another task

- Inital SOH estimation
	- Detiled in the task description of `TaskUpdateSOC` 

- DCIR history
	- Store DCIR values in EEPROM for recovery in the next power cycle
