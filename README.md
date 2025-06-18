# T5 CAN Interpreter
Trionic5 CANBUS interface using the ASL CAN X2 Dev board. 
https://www.autosportlabs.com/product/esp32-can-x2-dual-can-bus-automotive-grade-development-board/

For creating an interface module to be able to read CAN data from Trionic 5 ECUs at their weird-ass bitrate of 615k

The ASL dev board has two CANBUS channels one using the onboard ESP32 module as well as an additional MCP2515 controller. This should allow translation of the CANBUS data to a more common standard for use in a range of commercial devices or to be able to build your own display.

The current state of the devices is to:

1. Change CANBUS controller bit rate to match Trionic5 ECUs using the MCP2515 controller (CAN2)
2. Send the SRAM data requests using the T5 messaging protocol and receive responses
3. Parse the data and convert to JSON string to send over the ESP-NOW protocol to act as a infomation hub for other ESP32 clients (Displays)
