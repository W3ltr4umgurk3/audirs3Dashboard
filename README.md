This project utilizes an ESP32 and a TFT display to visualize real-time vehicle data from an Audi RS3 (8V) equipped with the DAZA engine. 
The system communicates with an ELM327 Bluetooth adapter via the Serial Port Profile (SPP).

🚀 Features

Real-time Monitoring: Tracks 6 critical engine parameters simultaneously.
Specialized Oil Pressure Parser: Built to handle UDS protocols, specifically filtering "Pending Frames" (7F 22 78) and multi-line responses common in VAG control units.
Precise Calculations:
Oil Pressure: Displayed as absolute pressure (bar).Fuel Level: Converts percentage data into liters (based on a 55L tank capacity).
Temperatures: High-precision monitoring of Transmission Oil, Intake Air, and Coolant.
Advanced Visualization: 3x2 grid layout with dynamic progress bars and color-coded warnings (White to Red transition).
Diagnostic Logging: Serial output assigns every response to its specific PID for easy debugging.

📊 Monitored Parameters

ParameterUnitModePIDController 
(Header)
Oil Pressurebar (abs)
UDS (22)0x13F40x7E0 (ECU)
Fuel LevelLiters
OBDII (01)0x2F0x7DF (Broadcast)
Transmission°CUDS (22)0x21040x7E1 (TCU)
Intake Temp°COBDII (01)0x0F0x7DF (Broadcast)
Coolant Temp°COBDII (01)0x050x7DF (Broadcast)
BatteryVoltsOBDII (01)0x420x7DF (Broadcast)

🛠 Hardware Requirements

Microcontroller: ESP32 (e.g., DevKit V1)
Display: TFT with ILI9341 or compatible controller (320x240 resolution), driven by the TFT_eSPI library.
OBD2 Adapter: ELM327 Bluetooth (v1.5 or v2.1 recommended).

🔧 Software Setup

Libraries:TFT_eSPI: Must be configured for your specific display pinout in User_Setup.h.BluetoothSerial: Included in the standard ESP32 Arduino core.
Configuration:Enter the MAC address of your ELM327 adapter in the BT_MAC variable.
Adjust warning thresholds in the gauges array as needed.

📝 How the Parser Works

Because the DAZA ECU often sends "Busy/Wait" responses when queried for complex UDS data (like oil pressure), this code employs a "Last-Occurrence Parser". 
It scans the entire ELM327 buffer for the most recent valid data sequence (62 13 F4), ensuring that outdated or incomplete frames are ignored.

Log Format Example

Plaintext[PID 0x13F4]: 7E8037F22787E8056213F47BF5 
[PID 0x002F]: 7E803412FDB
In this example, the parser automatically identifies the value 7BF5 at the end of the first line, successfully bypassing the "Pending" signal sent by the ECU.
