#include <MQUnifiedsensor.h>
#include <DHT.h>
#include <WiFi.h>
#include <esp_now.h>
#include <vector>
#include <queue>
#include <map>

// ===== HARDWARE PIN DEFINITIONS =====
#define MQ4_PIN 33
#define MQ7_PIN 32  
#define MQ135_PIN 35
#define MQ136_PIN 34
#define DHT_PIN 21
#define BUTTON_PIN 22
#define BUZZER_PIN 23
#define DHT_TYPE DHT11

// ===== SENSOR CALIBRATION & THRESHOLDS =====
#define CO_DANGER 35
#define H2S_DANGER 50
#define CO2_POOR 2000
#define CH4_DANGER 1000
#define TEMP_DANGER 40
#define HUMIDITY_DANGER 80

// ===== OPTIMIZED DISCOVERY CONSTANTS =====
#define AUTO_READ_INTERVAL 300000
#define SENSOR_WARMUP_TIME 10000       // Reduced to 10 seconds
#define HELLO_INTERVAL_MS 2000         // Aggressive 2-second intervals
#define NEIGHBOR_TIMEOUT 20000         // 20-second timeout
#define ACK_TIMEOUT_MS 800
#define MAX_RETRY_ATTEMPTS 2
#define INITIAL_DISCOVERY_BURST 8      // More aggressive initial discovery
#define INITIAL_DISCOVERY_INTERVAL 150 // Faster initial bursts
#define DISCOVERY_PHASE_TIME 30000     // 30-second aggressive discovery phase

// ===== NETWORK CONSTANTS =====
#define MAX_NEIGHBORS 12
#define HISTORY_SIZE 40
#define RSSI_WEIGHT 0.3

// ===== SENSOR OBJECTS =====
MQUnifiedsensor MQ4("ESP32", 3.3, 12, MQ4_PIN, "MQ-4");
MQUnifiedsensor MQ7("ESP32", 3.3, 12, MQ7_PIN, "MQ-7");
MQUnifiedsensor MQ135("ESP32", 3.3, 12, MQ135_PIN, "MQ-135");
MQUnifiedsensor MQ136("ESP32", 3.3, 12, MQ136_PIN, "MQ-136");
DHT dht(DHT_PIN, DHT_TYPE);

// ===== PACKET TYPES & STRUCTURES =====
enum PacketType : uint8_t {
  PKT_HELLO = 0,
  PKT_DATA = 1,
  PKT_ACK = 2,
  PKT_SERVER_ANNOUNCE = 3,
  PKT_SERVER_REQUEST = 4,
  PKT_ID_ANNOUNCE = 5,
  PKT_ID_CONFIRM = 6
};

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t sourceID;
  uint8_t destID;
  uint16_t seq;
  uint8_t hopCount;
  int8_t rssi;
  uint8_t ttl;
  uint8_t forwarderID;
  uint32_t timestamp;
  float ch4;
  float co;
  float co2;
  float h2s;
  float temp;
  float humidity;
  int raw4;
  int raw7;
  int raw135;
  int raw136;
} Packet;

// ===== NETWORK DATA STRUCTURES =====
struct Neighbor {
  uint8_t mac[6];
  uint8_t nodeID;
  uint8_t hopsToServer;
  int8_t rssi;
  unsigned long lastSeen;
  bool isActive;
  unsigned long firstSeen;
};

struct PendingPacket {
  Packet pkt;
  uint8_t targetMAC[6];
  uint8_t retryCount;
  unsigned long sentTime;
  bool waitingForAck;
};

// ===== GLOBAL VARIABLES =====
struct SensorReadings {
  float ch4, co, co2, h2s, temp, humidity;
  int raw4, raw7, raw135, raw136;
  bool mq4_connected, mq7_connected, mq135_connected, mq136_connected, dht_connected;
};

// Timing variables
unsigned long rtcMillis = 0;
unsigned long lastAutoRead = 0;
bool firstRead = true;
bool discoveryPhase = true;
unsigned long discoveryPhaseStart = 0;

// Network variables
std::vector<Neighbor> neighbors;
uint8_t myNodeID = 0;
uint8_t myMAC[6];
bool nodeIDAssigned = false;
bool idAnnouncedToServer = false;
uint8_t serverMAC[6] = {0};
bool serverDiscovered = false;
uint16_t seqCounter = 0;
std::map<uint32_t, unsigned long> packetCache;
PendingPacket pendingPkt;
unsigned long lastHelloSent = 0;

// Performance monitoring
unsigned long packetsSent = 0;
unsigned long packetsReceived = 0;
unsigned long lastPerformanceLog = 0;

// Sensor data history
std::vector<float> ch4_history;
std::vector<float> co_history;
std::vector<float> co2_history;
std::vector<float> h2s_history;
std::vector<float> temp_history;
std::vector<float> humidity_history;
#define MAX_HISTORY 50

// ===== ENHANCED PRINTING FUNCTIONS =====
void printInfoBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printSuccessBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ ✅ " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printWarningBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ ⚠️  " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printErrorBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ ❌ " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printDangerBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 🚨 " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printNetworkBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 📡 " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

void printDataBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 📊 " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

// ===== ENHANCED LOGGING FUNCTIONS =====
void printNeighborsList() {
  Serial.println("📋 CURRENT NEIGHBORS LIST:");
  Serial.println("┌───────┬──────────┬──────────┬──────────┬────────────┐");
  Serial.println("│ NodeID│ Hops2Srv │   RSSI   │ Last Seen│    MAC     │");
  Serial.println("├───────┼──────────┼──────────┼──────────┼────────────┤");
  
  if (neighbors.empty()) {
    Serial.println("│              No neighbors found                │");
  } else {
    for (const auto &n : neighbors) {
      if (n.isActive) {
        unsigned long age = (millis() - n.lastSeen) / 1000;
        char macStr[14];
        snprintf(macStr, sizeof(macStr), "%02X%02X%02X", n.mac[3], n.mac[4], n.mac[5]);
        
        Serial.printf("│ %5d │ %8d │ %7d │ %8lu │ %10s │\n", 
                     n.nodeID, n.hopsToServer, n.rssi, age, macStr);
      }
    }
  }
  Serial.println("└───────┴──────────┴──────────┴──────────┴────────────┘");
  Serial.printf("Total active neighbors: %d\n", neighbors.size());
}

void printForwardingDetails(const uint8_t *targetMAC, uint8_t targetNodeID, uint8_t originalSource, uint16_t seq, uint8_t hops) {
  Serial.printf("🔄 FORWARDING DETAILS:\n");
  Serial.printf("   ├── Original Source: Node %d\n", originalSource);
  Serial.printf("   ├── Forwarder: Node %d (This Node)\n", myNodeID);
  Serial.printf("   ├── Next Hop: Node %d\n", targetNodeID);
  Serial.printf("   ├── Sequence: %u\n", seq);
  Serial.printf("   ├── Hop Count: %d\n", hops);
  Serial.printf("   └── Target MAC: ");
  for (int i = 3; i < 6; i++) {
    Serial.printf("%02X", targetMAC[i]);
    if(i < 5) Serial.print(":");
  }
  Serial.println();
}

// ===== SENSOR INITIALIZATION =====
void initializeSensors() {
  printInfoBox("Initializing sensors...");
  
  // Initialize MQ-4 (Methane)
  MQ4.init();
  MQ4.setRegressionMethod(1);
  MQ4.setR0(9.83);
  MQ4.setA(101.2); 
  MQ4.setB(-2.473);
  
  // Initialize MQ-7 (Carbon Monoxide)
  MQ7.init();
  MQ7.setRegressionMethod(1);
  MQ7.setR0(10.0);
  MQ7.setA(99.04); 
  MQ7.setB(-1.52);
  
  // Initialize MQ-135 (Air Quality)
  MQ135.init();
  MQ135.setRegressionMethod(1);
  MQ135.setR0(9.83);
  MQ135.setA(110.47); 
  MQ135.setB(-2.862);
  
  // Initialize MQ-136 (Hydrogen Sulfide)
  MQ136.init();
  MQ136.setRegressionMethod(1);
  MQ136.setR0(9.83);
  MQ136.setA(100.0); 
  MQ136.setB(-2.0);
  
  printSuccessBox("Sensors initialized successfully");
}

bool checkSensorConnection(int pin, String sensorName) {
  int value = analogRead(pin);
  if (value == 0 || value == 4095) {
    Serial.println("❌ " + sensorName + " not detected!");
    return false;
  }
  return true;
}

// ===== TIME MANAGEMENT =====
unsigned long getInternalRTCTime() {
  return rtcMillis + millis();
}

String formatTime(unsigned long timestamp) {
  unsigned long totalSeconds = timestamp / 1000;
  unsigned long hours = (totalSeconds / 3600) % 24;
  unsigned long minutes = (totalSeconds / 60) % 60;
  unsigned long seconds = totalSeconds % 60;
  
  char timeStr[9];
  snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(timeStr);
}

String formatUptime(unsigned long timestamp) {
  unsigned long totalSeconds = timestamp / 1000;
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  
  char uptimeStr[20];
  if (days > 0) {
    snprintf(uptimeStr, sizeof(uptimeStr), "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(uptimeStr);
}

// ===== AGGRESSIVE DISCOVERY FUNCTIONS =====
void aggressiveDiscovery() {
  printNetworkBox("Starting aggressive neighbor discovery");
  for(int i = 0; i < INITIAL_DISCOVERY_BURST; i++) {
    sendEnhancedHello();
    delay(INITIAL_DISCOVERY_INTERVAL);
    Serial.printf("🚀 Discovery burst %d/%d\n", i+1, INITIAL_DISCOVERY_BURST);
  }
  discoveryPhase = true;
  discoveryPhaseStart = millis();
}

void sendEnhancedHello() {
  Packet hello = {};
  hello.type = PKT_HELLO;
  hello.sourceID = myNodeID;
  hello.seq = ++seqCounter;
  hello.timestamp = millis();
  
  if (serverDiscovered) {
    Neighbor *best = getBestNextHop();
    hello.hopCount = best ? (best->hopsToServer + 1) : 255;
  } else {
    hello.hopCount = 255;
  }
  
  hello.rssi = -70;
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  sendPacket(broadcast, &hello);
}

void smartHelloTiming() {
  static unsigned long helloInterval = HELLO_INTERVAL_MS;
  unsigned long now = millis();
  
  if (discoveryPhase) {
    if (now - discoveryPhaseStart > DISCOVERY_PHASE_TIME) {
      discoveryPhase = false;
      printSuccessBox("Discovery phase completed");
    } else {
      helloInterval = 1000;
    }
  } else {
    if (neighbors.empty() && !serverDiscovered) {
      helloInterval = 1500;
    } else if (neighbors.size() < 3) {
      helloInterval = 2000;
    } else {
      helloInterval = HELLO_INTERVAL_MS;
    }
  }
  
  if (now - lastHelloSent > helloInterval) {
    sendEnhancedHello();
    lastHelloSent = now;
  }
}

// ===== ENHANCED DUPLICATE DETECTION =====
bool isDuplicateEnhanced(uint8_t src, uint16_t seq) {
  uint32_t key = (src << 16) | seq;
  auto it = packetCache.find(key);
  if (it != packetCache.end()) {
    return true;
  }
  
  packetCache[key] = millis();
  
  if (packetCache.size() > 100) {
    auto it = packetCache.begin();
    while (it != packetCache.end()) {
      if (millis() - it->second > 120000) {
        it = packetCache.erase(it);
      } else {
        ++it;
      }
    }
  }
  return false;
}

// ===== OPTIMIZED NEIGHBOR MANAGEMENT =====
void updateNeighborEnhanced(const uint8_t *mac, uint8_t nodeID, uint8_t hops, int8_t rssi) {
  for (auto &n : neighbors) {
    if (memcmp(n.mac, mac, 6) == 0) {
      bool wasActive = n.isActive;
      n.nodeID = nodeID;
      n.hopsToServer = hops;
      n.rssi = rssi;
      n.lastSeen = millis();
      n.isActive = true;
      
      // Enhanced logging
      if (!wasActive) {
        Serial.printf("🔄 Reactivated neighbor: Node %d (Hops=%d, RSSI=%d)\n", nodeID, hops, rssi);
      }
      
      if (nodeID == 1 && !serverDiscovered) {
        serverDiscovered = true;
        memcpy(serverMAC, mac, 6);
        printSuccessBox("Server discovered via neighbor update!");
      }
      return;
    }
  }
  
  if (neighbors.size() < MAX_NEIGHBORS) {
    Neighbor n;
    memcpy(n.mac, mac, 6);
    n.nodeID = nodeID;
    n.hopsToServer = hops;
    n.rssi = rssi;
    n.lastSeen = millis();
    n.firstSeen = millis();
    n.isActive = true;
    neighbors.push_back(n);
    
    Serial.printf("📝 New neighbor: Node %d (Hops=%d, RSSI=%d) - ", nodeID, hops, rssi);
    Serial.print("MAC: ");
    for (int i = 3; i < 6; i++) {
      Serial.printf("%02X", mac[i]);
      if(i < 5) Serial.print(":");
    }
    Serial.println();
    
    // Print updated neighbors list
    printNeighborsList();
    
    if (nodeID == 1) {
      serverDiscovered = true;
      memcpy(serverMAC, mac, 6);
      printSuccessBox("Server discovered through new neighbor!");
    }
  }
}

void cleanupNeighborsEnhanced() {
  unsigned long now = millis();
  int removedCount = 0;
  
  for (auto it = neighbors.begin(); it != neighbors.end(); ) {
    if (it->isActive && (now - it->lastSeen > NEIGHBOR_TIMEOUT)) {
      Serial.printf("🗑️  Neighbor timeout: Node %d (Last seen: %lu sec ago)\n", 
                    it->nodeID, (now - it->lastSeen) / 1000);
      it = neighbors.erase(it);
      removedCount++;
    } else {
      ++it;
    }
  }
  
  if (removedCount > 0) {
    Serial.printf("🧹 Cleaned up %d inactive neighbors. Remaining: %d\n", removedCount, neighbors.size());
    printNeighborsList();
  }
}

// ===== NETWORK FUNCTIONS =====
void ensurePeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to add peer");
    }
  }
}

bool macEqual(const uint8_t *mac1, const uint8_t *mac2) {
  return memcmp(mac1, mac2, 6) == 0;
}

bool sendPacket(const uint8_t *mac, Packet *p) {
  ensurePeer(mac);
  esp_err_t result = esp_now_send(mac, (uint8_t*)p, sizeof(*p));
  
  if (result == ESP_OK) {
    packetsSent++;
    return true;
  }
  return false;
}

void sendAck(const uint8_t *mac, uint16_t seq) {
  Packet ack = {};
  ack.type = PKT_ACK;
  ack.sourceID = myNodeID;
  ack.seq = seq;
  ack.timestamp = millis();
  sendPacket(mac, &ack);
}

void announceIDToServer() {
  if (!idAnnouncedToServer && serverDiscovered && nodeIDAssigned) {
    Packet idAnnounce = {};
    idAnnounce.type = PKT_ID_ANNOUNCE;
    idAnnounce.sourceID = myNodeID;
    idAnnounce.destID = 1;
    idAnnounce.seq = ++seqCounter;
    idAnnounce.hopCount = 0;
    idAnnounce.timestamp = millis();
    
    sendPacket(serverMAC, &idAnnounce);
    idAnnouncedToServer = true;
    Serial.printf("📢 Announced ID %d to server\n", myNodeID);
  }
}

// ===== ENHANCED NODE ID HANDLING =====
void handleNodeIDAssignment(uint8_t assignedID) {
  if (assignedID != 0 && assignedID != myNodeID) {
    myNodeID = assignedID;
    nodeIDAssigned = true;
    Serial.printf("✅ Node ID assigned by server: %d\n", myNodeID);
    
    // Immediately announce to server
    announceIDToServer();
  }
}

Neighbor* getBestNextHop() {
  Neighbor *best = nullptr;
  float bestScore = -999999;
  int candidateCount = 0;
  
  for (auto &n : neighbors) {
    if (!n.isActive || n.hopsToServer == 255) continue;
    
    candidateCount++;
    float score = -(n.hopsToServer * 100) + (n.rssi * RSSI_WEIGHT);
    
    if (score > bestScore) {
      bestScore = score;
      best = &n;
    }
  }
  
  // Log routing decision
  if (best) {
    Serial.printf("🎯 ROUTING: Selected Node %d (Score: %.1f, Hops: %d, RSSI: %d) from %d candidates\n",
                  best->nodeID, bestScore, best->hopsToServer, best->rssi, candidateCount);
  } else if (candidateCount == 0) {
    Serial.println("🎯 ROUTING: No active neighbors with route to server");
  } else {
    Serial.println("🎯 ROUTING: No suitable next hop found");
  }
  
  return best;
}

void handleRetry() {
  if (!pendingPkt.waitingForAck) return;
  
  unsigned long elapsed = millis() - pendingPkt.sentTime;
  
  if (elapsed < ACK_TIMEOUT_MS) return;
  
  pendingPkt.retryCount++;
  
  if (pendingPkt.retryCount < MAX_RETRY_ATTEMPTS) {
    Serial.printf("⏱ ACK timeout, retry %d/%d\n", pendingPkt.retryCount, MAX_RETRY_ATTEMPTS);
    pendingPkt.sentTime = millis();
    sendPacket(pendingPkt.targetMAC, &pendingPkt.pkt);
  } else {
    Serial.println("❌ Max retries reached, packet dropped");
    pendingPkt.waitingForAck = false;
    
    Neighbor *altRoute = getBestNextHop();
    if (altRoute && !macEqual(altRoute->mac, pendingPkt.targetMAC)) {
      Serial.println("🔄 Trying alternative route...");
      memcpy(pendingPkt.targetMAC, altRoute->mac, 6);
      pendingPkt.retryCount = 0;
      pendingPkt.sentTime = millis();
      pendingPkt.waitingForAck = true;
      sendPacket(altRoute->mac, &pendingPkt.pkt);
    }
  }
}

// ===== SENSOR READING FUNCTIONS =====
void takeReadings(bool forceTransmit) {
  SensorReadings readings;
  
  readings.mq4_connected = checkSensorConnection(MQ4_PIN, "MQ-4");
  readings.mq7_connected = checkSensorConnection(MQ7_PIN, "MQ-7");
  readings.mq135_connected = checkSensorConnection(MQ135_PIN, "MQ-135");
  readings.mq136_connected = checkSensorConnection(MQ136_PIN, "MQ-136");
  
  float tempCheck = dht.readTemperature();
  float humCheck = dht.readHumidity();
  readings.dht_connected = (!isnan(tempCheck) && !isnan(humCheck));
  if (!readings.dht_connected) {
    Serial.println("❌ DHT11 not detected!");
  }
  
  if (readings.mq4_connected) MQ4.update();
  if (readings.mq7_connected) MQ7.update();
  if (readings.mq135_connected) MQ135.update();
  if (readings.mq136_connected) MQ136.update();
  
  readings.raw4 = analogRead(MQ4_PIN);
  readings.raw7 = analogRead(MQ7_PIN);
  readings.raw135 = analogRead(MQ135_PIN);
  readings.raw136 = analogRead(MQ136_PIN);
  
  if (readings.mq4_connected) {
    readings.ch4 = MQ4.readSensor();
    ch4_history.push_back(readings.ch4);
    if (ch4_history.size() > MAX_HISTORY) ch4_history.erase(ch4_history.begin());
  } else {
    readings.ch4 = -1;
  }
  
  if (readings.mq7_connected) {
    readings.co = MQ7.readSensor();
    co_history.push_back(readings.co);
    if (co_history.size() > MAX_HISTORY) co_history.erase(co_history.begin());
  } else {
    readings.co = -1;
  }
  
  if (readings.mq135_connected) {
    readings.co2 = MQ135.readSensor();
    co2_history.push_back(readings.co2);
    if (co2_history.size() > MAX_HISTORY) co2_history.erase(co2_history.begin());
  } else {
    readings.co2 = -1;
  }
  
  if (readings.mq136_connected) {
    readings.h2s = MQ136.readSensor();
    h2s_history.push_back(readings.h2s);
    if (h2s_history.size() > MAX_HISTORY) h2s_history.erase(h2s_history.begin());
  } else {
    readings.h2s = -1;
  }
  
  if (readings.dht_connected) {
    readings.temp = dht.readTemperature();
    readings.humidity = dht.readHumidity();
    
    temp_history.push_back(readings.temp);
    humidity_history.push_back(readings.humidity);
    if (temp_history.size() > MAX_HISTORY) temp_history.erase(temp_history.begin());
    if (humidity_history.size() > MAX_HISTORY) humidity_history.erase(humidity_history.begin());
  } else {
    readings.temp = -1;
    readings.humidity = -1;
  }
  
  displayReadings(readings);
  checkDangerLevels(readings);
  
  bool thresholdExceeded = (readings.ch4 >= CH4_DANGER) || 
                          (readings.co >= CO_DANGER) || 
                          (readings.h2s >= H2S_DANGER);
  
  if ((thresholdExceeded || forceTransmit || (millis() % 60000 < 1000)) && nodeIDAssigned && serverDiscovered) {
    if (forceTransmit) {
      printDataBox("Button pressed - Transmitting data to server...");
    } else if (thresholdExceeded) {
      printDangerBox("Threshold exceeded - Transmitting data to server...");
    } else {
      printDataBox("Periodic update - Transmitting data to server...");
    }
    transmitSensorData(readings);
  } else if ((thresholdExceeded || forceTransmit) && !serverDiscovered) {
    printErrorBox("Cannot transmit: Server not discovered");
  } else if ((thresholdExceeded || forceTransmit) && !nodeIDAssigned) {
    printErrorBox("Cannot transmit: Node ID not assigned");
  }
}

void displayReadings(SensorReadings r) {
  unsigned long currentTime = getInternalRTCTime();
  
  Serial.println("┌─────────────────────────────────────────────────┐");
  Serial.println("│           GAS & ENVIRONMENT MONITOR            │");
  Serial.printf ("│            System Time: %-15s    │\n", formatTime(currentTime).c_str());
  Serial.printf ("│            Uptime: %-19s│\n", formatUptime(currentTime).c_str());
  Serial.printf ("│            Node ID: %-14d      │\n", myNodeID);
  Serial.println("├─────────────────────────────────────────────────┤");
  
  Serial.printf ("│ MQ-4 (CH4)  : %6.1f PPM %s        │\n", r.ch4, r.mq4_connected ? " " : "❌");
  Serial.printf ("│ MQ-7 (CO)   : %6.1f PPM %s        │\n", r.co, r.mq7_connected ? " " : "❌");
  Serial.printf ("│ MQ-135 (CO2): %6.1f PPM %s        │\n", r.co2, r.mq135_connected ? " " : "❌");
  Serial.printf ("│ MQ-136 (H2S): %6.1f PPM %s        │\n", r.h2s, r.mq136_connected ? " " : "❌");
  Serial.printf ("│ Temperature : %6.1f °C %s        │\n", r.temp, r.dht_connected ? " " : "❌");
  Serial.printf ("│ Humidity    : %6.1f %% %s         │\n", r.humidity, r.dht_connected ? " " : "❌");
  
  Serial.println("├─────────────────────────────────────────────────┤");
  Serial.printf ("│ Network: %-10s Neighbors: %-2d       │\n", 
                 serverDiscovered ? "Connected" : "Searching", neighbors.size());
  Serial.println("└─────────────────────────────────────────────────┘");
  
  Serial.print("Raw values - ");
  Serial.print("MQ4:"); Serial.print(r.raw4);
  Serial.print(" MQ7:"); Serial.print(r.raw7);
  Serial.print(" MQ135:"); Serial.print(r.raw135);
  Serial.print(" MQ136:"); Serial.println(r.raw136);
  Serial.println();
}

void checkDangerLevels(SensorReadings r) {
  bool danger = false;
  bool warning = false;
  
  if (r.mq4_connected && r.ch4 >= CH4_DANGER) {
    printDangerBox("High Methane (CH4) level detected!");
    danger = true;
  } else if (r.mq4_connected && r.ch4 >= 500) {
    printWarningBox("Elevated Methane (CH4) level detected!");
    warning = true;
  }
  
  if (r.mq7_connected && r.co >= CO_DANGER) {
    printDangerBox("High Carbon Monoxide (CO) level detected!");
    danger = true;
  } else if (r.mq7_connected && r.co >= 20) {
    printWarningBox("Elevated Carbon Monoxide (CO) level detected!");
    warning = true;
  }
  
  if (r.mq136_connected && r.h2s >= H2S_DANGER) {
    printDangerBox("High Hydrogen Sulfide (H2S) level detected!");
    danger = true;
  } else if (r.mq136_connected && r.h2s >= 25) {
    printWarningBox("Elevated Hydrogen Sulfide (H2S) level detected!");
    warning = true;
  }
  
  if (r.mq135_connected && r.co2 >= CO2_POOR) {
    printWarningBox("Poor air quality (High CO2) detected!");
    warning = true;
  }
  if (r.dht_connected && r.temp >= TEMP_DANGER) {
    printWarningBox("High temperature detected!");
    warning = true;
  }
  if (r.dht_connected && r.humidity >= HUMIDITY_DANGER) {
    printWarningBox("High humidity detected!");
    warning = true;
  }
  
  if (danger) {
    triggerAlarm();
  } else if (warning) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void triggerAlarm() {
  printDangerBox("DANGEROUS gas levels detected! Activating alarm!");
  
  for(int i = 0; i < 10; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

void transmitSensorData(SensorReadings readings) {
  if (!nodeIDAssigned) {
    printErrorBox("Cannot transmit: Node ID not assigned");
    return;
  }
  
  Neighbor *nextHop = getBestNextHop();
  
  if (nextHop == nullptr) {
    printErrorBox("No route to server available!");
    printNeighborsList(); // Show available neighbors
    return;
  }
  
  Serial.printf("🌐 TRANSMITTING: Node %d → Node %d (Hops: %d, RSSI: %d)\n", 
                myNodeID, nextHop->nodeID, nextHop->hopsToServer, nextHop->rssi);
  
  Packet sensorPkt = {};
  sensorPkt.type = PKT_DATA;
  sensorPkt.sourceID = myNodeID;
  sensorPkt.destID = 1;
  sensorPkt.seq = ++seqCounter;
  sensorPkt.hopCount = 0;
  sensorPkt.forwarderID = myNodeID;
  sensorPkt.timestamp = millis();
  
  sensorPkt.ch4 = readings.ch4;
  sensorPkt.co = readings.co;
  sensorPkt.co2 = readings.co2;
  sensorPkt.h2s = readings.h2s;
  sensorPkt.temp = readings.temp;
  sensorPkt.humidity = readings.humidity;
  sensorPkt.raw4 = readings.raw4;
  sensorPkt.raw7 = readings.raw7;
  sensorPkt.raw135 = readings.raw135;
  sensorPkt.raw136 = readings.raw136;
  
  pendingPkt.pkt = sensorPkt;
  memcpy(pendingPkt.targetMAC, nextHop->mac, 6);
  pendingPkt.retryCount = 0;
  pendingPkt.sentTime = millis();
  pendingPkt.waitingForAck = true;
  
  sendPacket(nextHop->mac, &sensorPkt);
}

// ===== ENHANCED FORWARDING FUNCTION =====
void forwardPacket(Packet &pkt) {
  // Preserve the original source information
  uint8_t originalSource = pkt.sourceID;
  
  pkt.hopCount++;
  pkt.forwarderID = myNodeID;
  
  Neighbor *nextHop = getBestNextHop();
  if (nextHop == nullptr) {
    Serial.println("❌ Cannot forward: No route to server");
    printNeighborsList(); // Show why no route available
    return;
  }
  
  // Enhanced forwarding logging
  Serial.printf("➡ FORWARDING: Node %d → Node %d → Node %d\n", 
                originalSource, myNodeID, nextHop->nodeID);
  
  printForwardingDetails(nextHop->mac, nextHop->nodeID, originalSource, pkt.seq, pkt.hopCount);
  
  sendPacket(nextHop->mac, &pkt);
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  dht.begin();
  initializeSensors();
  
  printInfoBox("ENHANCED GAS MONITOR - OPTIMIZED DISCOVERY");
  Serial.println("🔧 Initializing enhanced network stack...");
  
  Serial.println("🔥 Warming up sensors for 10 seconds...");
  unsigned long warmupStart = millis();
  while (millis() - warmupStart < SENSOR_WARMUP_TIME) {
    delay(1000);
    int remaining = (SENSOR_WARMUP_TIME - (millis() - warmupStart)) / 1000;
    Serial.printf("🔥 Warming up... %d seconds remaining\n", remaining);
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  WiFi.macAddress(myMAC);
  
  uint32_t macHash = 0;
  for (int i = 0; i < 6; i++) {
    macHash = (macHash << 5) + macHash + myMAC[i];
  }
  myNodeID = (macHash % 248) + 2;
  nodeIDAssigned = true;
  
  if (esp_now_init() != ESP_OK) {
    printErrorBox("ESP-NOW init failed! Restarting...");
    ESP.restart();
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ensurePeer(broadcast);
  
  printSuccessBox("Node initialized - Starting aggressive discovery");
  Serial.printf("🏷️ Node ID: %d | ", myNodeID);
  Serial.print("📡 MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", myMAC[i]);
    if(i < 5) Serial.print(":");
  }
  Serial.println();
  
  aggressiveDiscovery();
  takeReadings(false);
}

// ===== MAIN LOOP =====
void loop() {
  if (millis() < rtcMillis) {
    rtcMillis = millis();
  }
  
  unsigned long currentTime = getInternalRTCTime();
  
  if (firstRead || (currentTime - lastAutoRead > AUTO_READ_INTERVAL)) {
    takeReadings(false);
    lastAutoRead = currentTime;
    if (firstRead) firstRead = false;
  }
  
  static unsigned long lastButtonPress = 0;
  if(digitalRead(BUTTON_PIN) == LOW && (millis() - lastButtonPress > 1000)) {
    delay(50);
    if(digitalRead(BUTTON_PIN) == LOW) {
      lastButtonPress = millis();
      takeReadings(true);
    }
  }
  
  smartHelloTiming();
  
  if (nodeIDAssigned && !idAnnouncedToServer && serverDiscovered) {
    announceIDToServer();
  }
  
  handleRetry();
  cleanupNeighborsEnhanced();
  
  // Periodic neighbor status (every 60 seconds)
  static unsigned long lastNeighborStatus = 0;
  if (millis() - lastNeighborStatus > 60000) {
    Serial.println("\n📊 NETWORK STATUS UPDATE:");
    Serial.printf("   My Node ID: %d\n", myNodeID);
    Serial.printf("   Server Discovered: %s\n", serverDiscovered ? "YES" : "NO");
    Serial.printf("   Packets Sent: %lu, Received: %lu\n", packetsSent, packetsReceived);
    printNeighborsList();
    lastNeighborStatus = millis();
  }
  
  if (millis() - lastPerformanceLog > 30000) {
    Serial.printf("📊 Performance: Nodes=%d, Sent=%lu, Recv=%lu\n", 
                  neighbors.size(), packetsSent, packetsReceived);
    lastPerformanceLog = millis();
  }
  
  delay(10);
}

// ===== ESP-NOW CALLBACKS =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  
  Packet pkt;
  memcpy(&pkt, data, sizeof(pkt));
  
  int8_t rssi = info->rx_ctrl->rssi;
  pkt.rssi = rssi;
  packetsReceived++;
  
  // Enhanced reception logging
  Serial.printf("📥 INCOMING: From Node %d, Type: %d, Seq: %u, RSSI: %d\n", 
                pkt.sourceID, pkt.type, pkt.seq, rssi);
  
  if (isDuplicateEnhanced(pkt.sourceID, pkt.seq)) {
    Serial.printf("🔄 Duplicate packet from Node %d, Seq %u - ignoring\n", pkt.sourceID, pkt.seq);
    return;
  }
  
  switch (pkt.type) {
    case PKT_SERVER_ANNOUNCE: {
      if (!serverDiscovered) {
        memcpy(serverMAC, info->src_addr, 6);
        serverDiscovered = true;
        printSuccessBox("Server discovered via announcement!");
        updateNeighborEnhanced(info->src_addr, pkt.sourceID, pkt.hopCount, rssi);
      }
      break;
    }
    
    case PKT_HELLO: {
      if (pkt.sourceID != 0 && pkt.sourceID != myNodeID) {
        Serial.printf("👋 Hello from Node %d (Hops=%d, RSSI=%d)\n", pkt.sourceID, pkt.hopCount, rssi);
        updateNeighborEnhanced(info->src_addr, pkt.sourceID, pkt.hopCount, rssi);
      }
      break;
    }
    
    case PKT_DATA: {
      if (pkt.sourceID == myNodeID) {
        Serial.println("⚠ My own packet, ignoring");
        return;
      }
      
      Serial.printf("📦 Data packet from Node %d, forwarding...\n", pkt.sourceID);
      sendAck(info->src_addr, pkt.seq);
      forwardPacket(pkt);
      break;
    }
    
    case PKT_ACK: {
      if (pendingPkt.waitingForAck && pkt.seq == pendingPkt.pkt.seq &&  
          pendingPkt.pkt.sourceID == myNodeID) {
        Serial.printf("✅ ACK received for Seq %u from Node %d\n", pkt.seq, pkt.sourceID);
        pendingPkt.waitingForAck = false;
      }
      break;
    }
    
    case PKT_SERVER_REQUEST: {
      if (pkt.destID == myNodeID || pkt.destID == 0) {
        handleServerRequest(pkt);
      } else {
        forwardCommand(pkt);
      }
      break;
    }
    
    case PKT_ID_CONFIRM: {
      // Handle server confirmation of our node ID
      if (pkt.destID == myNodeID) {
        Serial.printf("✅ Server confirmed Node ID: %d\n", myNodeID);
        nodeIDAssigned = true;
        idAnnouncedToServer = true;
      }
      break;
    }
  }
}

void OnDataSent(const esp_now_send_info_t *send_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("❌ Send failed to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  send_info->des_addr[0], send_info->des_addr[1], send_info->des_addr[2],
                  send_info->des_addr[3], send_info->des_addr[4], send_info->des_addr[5]);
  }
}

void forwardCommand(Packet &cmd) {
  if (cmd.ttl <= 0) {
    Serial.println("🛑 Command TTL expired");
    return;
  }
  
  cmd.ttl--;
  Serial.printf("📣 Forwarding command [Seq=%u TTL=%d] from Node %d\n", cmd.seq, cmd.ttl, cmd.sourceID);
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ensurePeer(broadcast);
  sendPacket(broadcast, &cmd);
}

void handleServerRequest(Packet &request) {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.printf("║  📢 SERVER REQUEST (Seq: %u)                 ║\n", request.seq);
  Serial.println("╚════════════════════════════════════════════════╝");
  
  forwardCommand(request);
  
  Neighbor *route = getBestNextHop();
  uint8_t myHops = route ? (route->hopsToServer + 1) : 255;
  
  if (myHops == 255) {
    Serial.println("⚠ No route to server");
    return;
  }
  
  unsigned long responseDelay = (myHops * 2000) + 1000;
  Serial.printf("⏱ Waiting %lu ms before responding...\n", responseDelay);
  delay(responseDelay);
  
  takeReadings(true);
}