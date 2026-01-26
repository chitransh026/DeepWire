# DeepWire

DEEPWIRE — Distributed Wireless Gas Monitoring System

DEEPWIRE is an open-innovation project designed to address the problem of undetected harmful gas release in mines, factories, and industrial workplaces. The system enables area-wide monitoring of gases such as CH₄, CO, CO₂, and H₂S, along with temperature and humidity, ensuring continuous safety coverage across the entire sector.

Technical Implementation:

Implemented a distributed sensor network using ESP32 microcontrollers as sensor nodes

Used ESP-NOW for low-latency, peer-to-peer wireless communication between nodes

Designed the system to be protocol-agnostic, allowing future integration with LoRa, Zigbee, or other wireless technologies

Sensor data from multiple nodes is aggregated at a central server node

Data Visualization & Alerts:

Implemented a separate server module to display real-time data on localhost dashboards or cloud platforms (e.g., ThingSpeak)

Enabled configurable data logging and monitoring based on deployment requirements

Key Focus Areas:

Distributed system design

Wireless communication reliability

Industrial safety monitoring

Scalable and modular architecture
