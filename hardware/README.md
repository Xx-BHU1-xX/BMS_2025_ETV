Architecture '24 : - LTC6804 (DC1894B) 6 slaves + Linduino (DC2026) master

Disadvantages: - atmega328p (2KB SRAM, barely sufficient for all 72x SOCs, CellVs…)

Architecture '25

Solution :- Offload processing to ESP32, retain slave comms with DC2026
ESP32 (referenced to GLV) <- Isolated UART -> Linduino (referenced to TS)

Additional features: - CAN comms, Redundant current sensor inputs (Hall (HBO300 fully differential) + Shunt (VAT4300 wireless))

![BMS architecture](images/BMS.jpeg)
