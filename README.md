#  DeepWire

> **Distributed Wireless Gas Monitoring System** — Real-time IoT safety architecture for industrial hazard detection using self-organizing mesh networks

---

##  Project Overview

DeepWire is an open-innovation embedded systems project that solves critical safety challenges in mines, factories, and industrial environments through **intelligent distributed gas sensing**. The system features a self-organizing mesh network of sensor nodes communicating via **ESP-NOW**, with autonomous node discovery, dynamic routing, and real-time threat detection.

### The Problem
- Undetected harmful gas releases in confined industrial spaces pose catastrophic risks
- Centralized monitoring systems have single-point-of-failure vulnerabilities
- Existing solutions lack scalability and rapid deployment capabilities

### The Solution
DeepWire deploys autonomous sensor nodes that form a **self-healing mesh network**, automatically discovering neighbors, establishing optimal routes to a central server, and aggregating environmental data with sub-100ms latency.

---

##  Core Technical Features

### **Distributed Sensor Architecture**
- **Multi-gas detection**: CH₄ (methane), CO (carbon monoxide), CO₂ (air quality), H₂S (hydrogen sulfide)
- **Environmental monitoring**: Temperature & humidity tracking via DHT11
- **Quad-sensor redundancy**: MQ-4, MQ-7, MQ-135, MQ-136 gas sensors with sensor-level fault detection
- **Autonomous data aggregation** at central server node

### **Advanced Wireless Communication**
- **ESP-NOW protocol**: Ultra-low-latency (100ms average latency) peer-to-peer wireless
- **Self-organizing mesh topology**: Nodes automatically discover neighbors and form network without pre-configuration
- **Intelligent routing algorithm**:
  - Hop-count optimization with RSSI weighting
  - Dynamic route selection based on link quality
  - Automatic failover to alternative routes on packet loss
- **Packet-level reliability**:
  - Sequence-based duplicate detection with 120-second cache window
  - ACK-based transmission confirmation with exponential backoff retry
  - TTL-based command forwarding to prevent infinite loops

### **Aggressive Auto-Discovery**
- **Initial discovery burst**: 8 aggressive HELLO packets (150ms intervals) during 30-second discovery phase
- **Smart adaptive timing**: Dynamic interval adjustment (1s-2.5s) based on network connectivity state
- **Neighbor state machine**: Active/inactive tracking with 20-second timeout windows
- **Server auto-detection**: Automatic server discovery via PKT_SERVER_ANNOUNCE packets

### **Real-Time Threat Detection**
- **Configurable threshold-based alerts**:
  - CH₄ danger: ≥1000 PPM | warning: ≥500 PPM
  - CO danger: ≥35 PPM | warning: ≥20 PPM
  - H₂S danger: ≥50 PPM | warning: ≥25 PPM
  - CO₂ poor air: ≥2000 PPM
  - Temperature: ≥40°C | Humidity: ≥80%
- **Multi-tier alarm system**: Buzzer patterns differentiate danger (10Hz pulse) vs. warning (3Hz pulse)
- **Sensor health monitoring**: Per-sensor connection detection with graceful degradation

### **Data Pipeline & Logging**
- **History buffering**: 50-sample circular buffers for each gas/environmental metric
- **Performance tracking**: Packet statistics (sent/received counters) logged every 30s
- **Network status reporting**: Neighbor table dumps every 60s with hop counts, RSSI, MAC addresses
- **Forwarder tracking**: Frame-by-frame logging of source→forwarder→destination chains

---

##  System Architecture

### **Hardware Configuration**
```
┌─ ESP32 Microcontroller (SoC)
├─ Quad-gas sensor array (MQ-4/7/135/136)
├─ DHT11 (Temperature/Humidity)
├─ 18650 Li-ion battery (3.7V, 2200mAh) + charging circuit
│  ├─ TP4056 (battery charger IC)
│  └─ MT3608 (boost converter for 5V rail)
├─ Piezo buzzer (alarm/alert signaling)
├─ Push button (manual data transmission trigger)
└─ PCB with analog-to-digital conditioning
```

### **Packet Format (esp_now_send_info_t)**
```c
struct Packet {
  uint8_t  type;        // PKT_HELLO, PKT_DATA, PKT_ACK, PKT_SERVER_ANNOUNCE, etc.
  uint8_t  sourceID;    // Originating node ID (2-250)
  uint8_t  destID;      // Destination node ID (server=1)
  uint16_t seq;         // Sequence number for duplicate detection
  uint8_t  hopCount;    // Hops traveled (0 = direct)
  int8_t   rssi;        // Signal strength (-100 to -20 dBm)
  uint8_t  ttl;         // Time-to-live (command forwarding)
  uint8_t  forwarderID; // Last relay node ID
  uint32_t timestamp;   // Transmission timestamp (ms)
  float    ch4, co, co2, h2s;           // Gas measurements (PPM)
  float    temp, humidity;               // Environmental data
  int      raw4, raw7, raw135, raw136;  // ADC raw values for debugging
} __attribute__((packed));  // 67 bytes total
```

### **State Machine: Node Discovery Flow**
```
STARTUP
  ↓
[Initialize sensors, warm up 10s]
  ↓
[Assign node ID from MAC hash: (hash % 248) + 2]
  ↓
[ESP-NOW init + register RX/TX callbacks]
  ↓
[AGGRESSIVE DISCOVERY PHASE: 30 seconds]
  ├─ Send 8 HELLO bursts (150ms apart)
  ├─ Collect neighbor advertisements
  └─ Update best hop-count to server
  ↓
[NORMAL OPERATION: Adaptive HELLO timing]
  ├─ 1.0s interval if 0 neighbors
  ├─ 1.5s interval if 1-2 neighbors
  ├─ 2.0s interval if 3+ neighbors
  ├─ 2.5s interval if server found + stable
  ↓
[ROUTE OPTIMIZATION + TRANSMISSION]
  ├─ Calculate scoring: -(hops × 100) + (rssi × 0.3)
  ├─ Select best next-hop neighbor
  └─ Transmit via optimal route with ACK tracking
```

---

##  Data Transmission Triggers

Data is transmitted to the server in these scenarios:

| Trigger | Frequency | Priority |
|---------|-----------|----------|
| **Gas threshold exceeded** | On detection | HIGH (immediate) |
| **Manual button press** | On demand | HIGH (immediate) |
| **Periodic update** | Every 300s (5 min) | LOW (best-effort) |
| **Server request** | On PKT_SERVER_REQUEST | MEDIUM |

All transmissions require:
- ✅ Node ID assigned
- ✅ Server discovered
- ✅ Active route to server

---

## 🔧 Key Code Components

### **1. Neighbor Discovery & Management**
```cpp
void updateNeighborEnhanced(const uint8_t *mac, uint8_t nodeID, 
                           uint8_t hops, int8_t rssi);
```
- Updates neighbor table with RSSI and hop count info
- Reactivates dormant neighbors on re-discovery
- Auto-detects server (NodeID=1) and stores server MAC

### **2. Intelligent Routing**
```cpp
Neighbor* getBestNextHop();
```
- Evaluates all active neighbors using scoring function
- Prioritizes lower hop counts with RSSI weighting
- Logs routing decision with candidate count

### **3. ACK-Based Reliability**
```cpp
void handleRetry();
```
- 800ms ACK timeout with 2 max retry attempts
- Automatic fallback to alternative routes on failure
- Detailed retry logging for debugging

### **4. Duplicate Detection**
```cpp
bool isDuplicateEnhanced(uint8_t src, uint16_t seq);
```
- Map-based cache (source_id || seq_num)
- 120-second cache window for old packets
- LRU-style cleanup when cache exceeds 100 entries

### **5. Packet Forwarding**
```cpp
void forwardPacket(Packet &pkt);
```
- Preserves original source ID through network hops
- Increments hop count at each relay
- Logs complete forwarding chain (src→forwarder→dest)

---

## 📁 Repository Structure

```
DeepWire/
├── Code/
│   └── sensor/
│       ├── sensor.ino          # Main firmware (1077 lines)
│       └── server/             # Server-side code
├── Hardware/
│   ├── Instruction.md          # Bill of materials
│   ├── sensor_node_pcb.jpeg    # PCB design
│   ├── sensor_node_schematic.jpeg
│   ├── sensor_pcb_layout.jpeg
│   ├── server_node_pcb.jpeg
│   ├── proteus_simulation.jpg  # Circuit simulation
│   └── sensor_node_photo.jpeg  # Physical prototype
├── Web Dashboard/              # Real-time monitoring UI
├── docs/
│   ├── Communication Flow.pptx # Protocol handshake diagrams
│   ├── ESP-NOW DataFrame Format.jpg
│   ├── Sensor_Node_Block_Diagram.pdf
│   ├── Instruction Manual.pdf
│   ├── communication_architecture.pdf
│   └── algorithm.png
└── README.md
```

---

##  Hardware BOM (Bill of Materials)

| Component | Spec | Role |
|-----------|------|------|
| **ESP32** | Dual-core Xtensa 240MHz | Core SoC, wireless stack |
| **MQ-4** | Methane sensor | CH₄ detection |
| **MQ-7** | Carbon monoxide sensor | CO detection |
| **MQ-135** | Air quality sensor | CO₂ monitoring |
| **MQ-136** | Hydrogen sulfide sensor | H₂S detection |
| **DHT11** | Temperature/humidity | Environmental data |
| **18650 Li-ion** | 3.7V, 2200mAh | Power supply |
| **TP4056** | Charging IC | Battery management |
| **MT3608** | Boost converter | 5V rail generation |
| **Piezo Buzzer** | 5V active | Alarm output |

---

##  Performance Characteristics

| Metric | Value |
|--------|-------|
| **Intra-network latency** | <100ms (ESP-NOW direct) |
| **Discovery time** | ~30s (aggressive phase) |
| **Packet success rate** | >95% (with ACK retry) |
| **Max network size** | ~12 direct neighbors per node |
| **Battery life** | ~8-12 hours (continuous operation) |
| **Mesh depth** | Unlimited (TTL-based forwarding) |

---

## 🔄 Packet Types & Handshakes

| Type | Source | Function |
|------|--------|----------|
| `PKT_HELLO` | Any node | Neighbor discovery broadcast |
| `PKT_DATA` | Sensor node | Gas readings transmission |
| `PKT_ACK` | Receiver | Acknowledge data receipt |
| `PKT_SERVER_ANNOUNCE` | Server | Server presence broadcast |
| `PKT_SERVER_REQUEST` | Server | Request node data/response |
| `PKT_ID_ANNOUNCE` | Sensor | Announce assigned node ID |
| `PKT_ID_CONFIRM` | Server | Confirm node ID assignment |

---

## 📡 Network Communication Flow

```
Node 2                    Node 3 (Forwarder)        Server (Node 1)
   |                           |                          |
   |--- PKT_DATA (Seq=42) ---->|                          |
   |                           |--- PKT_DATA (Seq=42) --->|
   |                           |<-- PKT_ACK (Seq=42) -----|
   |<-- PKT_ACK (Seq=42) ------|                          |
   |                                                       |
[Route: Node2 → Node3 → Server] [2 hops] [Success ✓]
```

---

## 🚀 Quick Start

### **Prerequisites**
- Arduino IDE with ESP32 board support
- Libraries: `MQUnifiedsensor`, `DHT`, `WiFi`, `esp_now`

### **Setup**
1. Clone repository
2. Configure sensor calibration in `sensor.ino` (lines 19-25)
3. Upload firmware to ESP32 nodes
4. Power on: nodes auto-discover network
5. Monitor via Serial (115200 baud) for status

### **Monitoring**
- Check Serial output for formatted status boxes
- Neighbor table dumps every 60s
- Alert boxes on gas threshold exceeded
- Network connectivity visual feedback

---

##  Educational Value

This project demonstrates:

✅ **Embedded Systems**: Real-time sensor I/O, PWM, ADC conditioning  
✅ **IoT Networking**: Self-organizing mesh, peer-to-peer protocols  
✅ **Data Structures**: Circular buffers, packet caches, neighbor tables  
✅ **Algorithms**: Routing optimization, duplicate detection, state machines  
✅ **Industrial Safety**: Multi-tier thresholds, alarm systems, fault tolerance  
✅ **Hardware Design**: PCB layout, power management, sensor integration  
✅ **Low-level Programming**: Interrupts, timers, wireless callbacks  

---

## 📄 License

MIT License — See LICENSE file

---

## 🤝 Contributing

Contributions welcome! Areas of interest:
- LoRa/Zigbee protocol integration
- Cloud dashboard (ThingSpeak/Azure IoT Hub)
- Machine learning for anomaly detection
- Extended sensor library support

---

**Built with 🔧 and ❤️ for safety engineering**
