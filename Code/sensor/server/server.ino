#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>

// ===== OPTIMIZED NETWORK CONSTANTS =====
#define HELLO_INTERVAL_MS 1500
#define SERVER_ANNOUNCE_INTERVAL 3000
#define NODE_TIMEOUT 25000
#define MAX_NODES 25
#define SERVER_BUTTON_PIN 22
#define MAX_SENSOR_HISTORY 1000
#define INITIAL_ANNOUNCE_BURST 10

// ===== NETWORK CONFIGURATION =====
const char* sta_ssid = "IndustrialSensorNet";
const char* sta_password = "industry123";
IPAddress local_ip(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ===== PACKET TYPES & STRUCTURES =====
enum PacketType : uint8_t {
  PKT_HELLO = 0,
  PKT_DATA = 1,
  PKT_ACK = 2,
  PKT_SERVER_ANNOUNCE = 3,
  PKT_SERVER_REQUEST = 4,
  PKT_ID_ANNOUNCE = 5
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

// ===== NODE MANAGEMENT STRUCTURES =====
struct NodeInfo {
  uint8_t mac[6];
  uint8_t nodeID;
  uint8_t lastHopCount;
  int8_t lastRSSI;
  unsigned long lastSeen;
  unsigned long firstSeen;
  bool isActive;
  uint16_t packetCount;
  String nodeName;
  
  std::vector<float> ch4_history;
  std::vector<float> co_history;
  std::vector<float> co2_history;
  std::vector<float> h2s_history;
  std::vector<float> temp_history;
  std::vector<float> humidity_history;
  std::vector<unsigned long> timestamp_history;
};

struct PacketHistory {
  uint8_t sourceID;
  uint16_t seq;
};

// ===== GLOBAL VARIABLES =====
std::vector<NodeInfo> nodes;
uint8_t myMAC[6];
uint16_t seqCounter = 0;
unsigned long lastServerAnnounce = 0;
unsigned long lastHelloSent = 0;
PacketHistory history[50];
uint8_t historyIndex = 0;

bool lastButtonState = HIGH;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
#define DEBOUNCE_DELAY 50

std::map<uint8_t, String> nodeNames = {
  {1, "Control Room"},
  {2, "Production Area A"},
  {3, "Storage Tank 1"},
  {4, "Ventilation Shaft"},
  {5, "Laboratory"},
  {6, "Gate 1"},
  {7, "Compressor Room"},
  {8, "Loading Bay"},
  {9, "Main Corridor"},
  {10, "Emergency Exit"}
};

// ===== DANGER THRESHOLDS =====
#define CH4_DANGER 1000
#define CO_DANGER 35
#define H2S_DANGER 50
#define CO2_POOR 2000
#define TEMP_DANGER 40
#define HUMIDITY_DANGER 80

// ===== FUNCTION PROTOTYPES =====
void printMAC(const uint8_t *mac);
bool macEqual(const uint8_t *mac1, const uint8_t *mac2);
bool isDuplicate(uint8_t src, uint16_t seq);
void ensurePeer(const uint8_t *mac);
String getNodeName(uint8_t nodeID);
String formatTimestamp(unsigned long timestamp);
void setupIsolatedSTA();
void handleRoot();
void handleAPINodes();
void handleAPINodeData();
void handleAPIRequestReadings();
void handleNotFound();
void setupWebServer();
NodeInfo* findOrCreateNode(const uint8_t *mac, uint8_t nodeID, uint8_t originalSourceID = 0);
void updateNodeWithData(NodeInfo* node, Packet* pkt);
void cleanupInactiveNodes();
bool sendPacket(const uint8_t *mac, Packet *p);
void sendAck(const uint8_t *mac, uint16_t seq);
void sendServerAnnounce();
void sendEnhancedServerAnnounce();
void sendHello();
void sendCommandToNode(uint8_t nodeID);
void requestReadingsFromAllNodes();
void aggressiveServerAnnounce();
void handleServerButton();
void checkDangerLevels(NodeInfo* node);
void displayNodeData(NodeInfo* node, Packet* pkt);
void displayNetworkStatus();
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void OnDataSent(const esp_now_send_info_t *send_info, esp_now_send_status_t status);
void registerNodeWithID(const uint8_t *mac, uint8_t nodeID, const char* nodeType);
void checkNodeRegistration();

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

void printNetworkBox(const String& message) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 🌐 " + message + " ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

// ===== UTILITY FUNCTIONS =====
void printMAC(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

bool macEqual(const uint8_t *mac1, const uint8_t *mac2) {
  return memcmp(mac1, mac2, 6) == 0;
}

bool isDuplicate(uint8_t src, uint16_t seq) {
  for (int i = 0; i < 50; i++) {
    if (history[i].sourceID == src && history[i].seq == seq) {
      return true;
    }
  }
  history[historyIndex] = {src, seq};
  historyIndex = (historyIndex + 1) % 50;
  return false;
}

void ensurePeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

String getNodeName(uint8_t nodeID) {
  if (nodeNames.find(nodeID) != nodeNames.end()) {
    return nodeNames[nodeID];
  }
  return "Area " + String(nodeID);
}

String formatTimestamp(unsigned long timestamp) {
  unsigned long seconds = timestamp / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  
  hours = hours % 24;
  minutes = minutes % 60;
  seconds = seconds % 60;
  
  char buffer[20];
  if (days > 0) {
    snprintf(buffer, sizeof(buffer), "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buffer);
}

// ===== ISOLATED STA SETUP =====
void setupIsolatedSTA() {
  printNetworkBox("Setting up isolated STA network...");
  
  WiFi.mode(WIFI_AP_STA);
  
  if (!WiFi.config(local_ip, gateway, subnet)) {
    printErrorBox("STA Failed to configure");
    return;
  }
  
  WiFi.softAP(sta_ssid, sta_password, 1, 0, 4);
  
  printSuccessBox("Isolated STA Network Created Successfully");
  Serial.print("📡 STA SSID: "); Serial.println(sta_ssid);
  Serial.print("🌐 STA IP: "); Serial.println(WiFi.softAPIP());
}

// ===== AGGRESSIVE SERVER DISCOVERY =====
void aggressiveServerAnnounce() {
  printNetworkBox("Starting aggressive server announcement");
  for(int i = 0; i < INITIAL_ANNOUNCE_BURST; i++) {
    sendEnhancedServerAnnounce();
    delay(200);
    Serial.printf("📢 Server announce burst %d/%d\n", i+1, INITIAL_ANNOUNCE_BURST);
  }
}

void sendEnhancedServerAnnounce() {
  Packet announce = {};
  announce.type = PKT_SERVER_ANNOUNCE;
  announce.sourceID = 1;
  announce.seq = ++seqCounter;
  announce.hopCount = 0;
  announce.timestamp = millis();
  announce.rssi = -60;
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  sendPacket(broadcast, &announce);
}

void sendServerAnnounce() {
  Packet announce = {};
  announce.type = PKT_SERVER_ANNOUNCE;
  announce.sourceID = 1;
  announce.seq = ++seqCounter;
  announce.hopCount = 0;
  announce.timestamp = millis();
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  sendPacket(broadcast, &announce);
}

// ===== PACKET SENDING FUNCTIONS =====
bool sendPacket(const uint8_t *mac, Packet *p) {
  ensurePeer(mac);
  esp_err_t result = esp_now_send(mac, (uint8_t*)p, sizeof(*p));
  
  if (result == ESP_OK) {
    Serial.printf("📤 Server sent [Type=%d Seq=%u] to ", p->type, p->seq);
    printMAC(mac);
    Serial.println(" ✅");
    return true;
  }
  return false;
}

void sendAck(const uint8_t *mac, uint16_t seq) {
  Packet ack = {};
  ack.type = PKT_ACK;
  ack.sourceID = 1;
  ack.seq = seq;
  ack.timestamp = millis();
  sendPacket(mac, &ack);
}

void sendHello() {
  Packet hello = {};
  hello.type = PKT_HELLO;
  hello.sourceID = 1;
  hello.seq = ++seqCounter;
  hello.hopCount = 0;
  hello.timestamp = millis();
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ensurePeer(broadcast);
  sendPacket(broadcast, &hello);
}

void sendCommandToNode(uint8_t nodeID) {
  NodeInfo* targetNode = nullptr;
  for (auto &node : nodes) {
    if (node.nodeID == nodeID && node.isActive) {
      targetNode = &node;
      break;
    }
  }
  
  if (!targetNode) {
    Serial.printf("Node %d not found or inactive\n", nodeID);
    return;
  }
  
  Packet command = {};
  command.type = PKT_SERVER_REQUEST;
  command.sourceID = 1;
  command.destID = nodeID;
  command.seq = ++seqCounter;
  command.hopCount = 0;
  command.ttl = 10;
  command.timestamp = millis();
  
  Serial.printf("📡 Sending command to Node %d (%s)\n", nodeID, targetNode->nodeName.c_str());
  Serial.print("📍 MAC: "); printMAC(targetNode->mac); Serial.println();
  
  sendPacket(targetNode->mac, &command);
}

void requestReadingsFromAllNodes() {
  int activeCount = 0;
  
  Serial.println("Requesting readings from all active nodes...");
  
  for (auto &node : nodes) {
    if (node.isActive) {
      sendCommandToNode(node.nodeID);
      activeCount++;
      delay(50);
    }
  }
  
  if (activeCount == 0) {
    Serial.println("No active nodes found");
  } else {
    Serial.printf("Reading requests sent to %d active nodes\n", activeCount);
  }
}

// ===== FIXED NODE ID HANDLING =====
NodeInfo* findOrCreateNode(const uint8_t *mac, uint8_t nodeID, uint8_t originalSourceID) {
    // Use originalSourceID for multi-hop scenarios, otherwise use nodeID from packet
    uint8_t effectiveNodeID = (originalSourceID != 0) ? originalSourceID : nodeID;
    
    // **FIX: For forwarded packets, search by original nodeID first to avoid MAC overwriting**
    if (originalSourceID != 0 && effectiveNodeID != 0) {
        // First, try to find by original node ID (for forwarded packets)
        for (auto &node : nodes) {
            if (node.nodeID == effectiveNodeID) {
                // **FIX: Don't update MAC for forwarded packets - keep original node's MAC**
                node.lastSeen = millis();
                node.isActive = true;
                node.lastHopCount = 255; // This will be updated later with actual hop count
                return &node;
            }
        }
    }
    
    // For direct communications or new nodes, try to find by MAC address
    for (auto &node : nodes) {
        if (macEqual(node.mac, mac)) {
            node.lastSeen = millis();
            node.isActive = true;
            
            // Update nodeID if we have new information (only for direct communications)
            if (effectiveNodeID != 0 && node.nodeID != effectiveNodeID && originalSourceID == 0) {
                Serial.printf("Node MAC ");
                printMAC(mac);
                Serial.printf(" changed ID from %d to %d\n", node.nodeID, effectiveNodeID);
                node.nodeID = effectiveNodeID;
                node.nodeName = getNodeName(effectiveNodeID);
            }
            return &node;
        }
    }
    
    // If not found by MAC, try by nodeID (for cases where MAC changed but nodeID is same)
    if (effectiveNodeID != 0) {
        for (auto &node : nodes) {
            if (node.nodeID == effectiveNodeID) {
                // **FIX: Only update MAC if this is a direct communication, not forwarded**
                if (originalSourceID == 0) {
                    Serial.printf("Node ID %d updated MAC address from ", effectiveNodeID);
                    printMAC(node.mac);
                    Serial.print(" to ");
                    printMAC(mac);
                    Serial.println();
                    
                    memcpy(node.mac, mac, 6);
                }
                node.lastSeen = millis();
                node.isActive = true;
                return &node;
            }
        }
    }
    
    // Create new node with the effective node ID
    if (nodes.size() < MAX_NODES && effectiveNodeID != 0) {
        NodeInfo newNode;
        memcpy(newNode.mac, mac, 6);
        newNode.nodeID = effectiveNodeID;
        newNode.lastHopCount = 255;
        newNode.lastRSSI = 0;
        newNode.lastSeen = millis();
        newNode.firstSeen = millis();
        newNode.isActive = true;
        newNode.packetCount = 0;
        newNode.nodeName = getNodeName(effectiveNodeID);
        
        nodes.push_back(newNode);
        Serial.printf("📝 Registered new node: %s (ID:%d) with MAC ", newNode.nodeName.c_str(), effectiveNodeID);
        printMAC(mac);
        Serial.println();
        return &nodes.back();
    }
    
    Serial.printf("❌ Could not create node - MAC: ");
    printMAC(mac);
    Serial.printf(", NodeID: %d, EffectiveID: %d\n", nodeID, effectiveNodeID);
    return nullptr;
}

void updateNodeWithData(NodeInfo* node, Packet* pkt) {
  node->nodeID = pkt->sourceID;
  node->lastHopCount = pkt->hopCount;
  node->lastRSSI = pkt->rssi;
  node->lastSeen = millis();
  node->packetCount++;
  
  if (pkt->ch4 >= 0) {
    node->ch4_history.push_back(pkt->ch4);
    if (node->ch4_history.size() > MAX_SENSOR_HISTORY) {
      node->ch4_history.erase(node->ch4_history.begin());
    }
  }
  
  if (pkt->co >= 0) {
    node->co_history.push_back(pkt->co);
    if (node->co_history.size() > MAX_SENSOR_HISTORY) {
      node->co_history.erase(node->co_history.begin());
    }
  }
  
  if (pkt->co2 >= 0) {
    node->co2_history.push_back(pkt->co2);
    if (node->co2_history.size() > MAX_SENSOR_HISTORY) {
      node->co2_history.erase(node->co2_history.begin());
    }
  }
  
  if (pkt->h2s >= 0) {
    node->h2s_history.push_back(pkt->h2s);
    if (node->h2s_history.size() > MAX_SENSOR_HISTORY) {
      node->h2s_history.erase(node->h2s_history.begin());
    }
  }
  
  if (pkt->temp >= -40) {
    node->temp_history.push_back(pkt->temp);
    if (node->temp_history.size() > MAX_SENSOR_HISTORY) {
      node->temp_history.erase(node->temp_history.begin());
    }
  }
  
  if (pkt->humidity >= 0) {
    node->humidity_history.push_back(pkt->humidity);
    if (node->humidity_history.size() > MAX_SENSOR_HISTORY) {
      node->humidity_history.erase(node->humidity_history.begin());
    }
  }
  
  node->timestamp_history.push_back(pkt->timestamp);
  if (node->timestamp_history.size() > MAX_SENSOR_HISTORY) {
    node->timestamp_history.erase(node->timestamp_history.begin());
  }
}

void cleanupInactiveNodes() {
  unsigned long now = millis();
  for (auto it = nodes.begin(); it != nodes.end(); ) {
    if (it->isActive && (now - it->lastSeen > NODE_TIMEOUT)) {
      Serial.printf("Node %d marked inactive\n", it->nodeID);
      it->isActive = false;
      ++it;
    } else {
      ++it;
    }
  }
}

// Enhanced node registration for multi-hop scenarios
void registerNodeWithID(const uint8_t *mac, uint8_t nodeID, const char* nodeType) {
  NodeInfo* node = findOrCreateNode(mac, nodeID);
  if (node) {
    Serial.printf("✅ %s Node Registered: %s (ID:%d) - MAC: ", nodeType, node->nodeName.c_str(), nodeID);
    printMAC(mac);
    Serial.printf(" - RSSI: %d dBm\n", node->lastRSSI);
    
    // Send confirmation back to node
    Packet confirmPkt = {};
    confirmPkt.type = PKT_ID_ANNOUNCE;
    confirmPkt.sourceID = 1; // Server ID
    confirmPkt.destID = nodeID;
    confirmPkt.seq = ++seqCounter;
    confirmPkt.timestamp = millis();
    sendPacket(mac, &confirmPkt);
  }
}

// ===== BUTTON HANDLING =====
void handleServerButton() {
  bool reading = digitalRead(SERVER_BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && !buttonPressed) {
      buttonPressed = true;
      Serial.println("SERVER BUTTON PRESSED - Requesting readings from all nodes!");
      requestReadingsFromAllNodes();
    } else if (reading == HIGH && buttonPressed) {
      buttonPressed = false;
    }
  }
  
  lastButtonState = reading;
}

// ===== DATA PROCESSING & DISPLAY =====
void checkDangerLevels(NodeInfo* node) {
  bool danger = false;
  
  float last_ch4 = node->ch4_history.empty() ? -1 : node->ch4_history.back();
  float last_co = node->co_history.empty() ? -1 : node->co_history.back();
  float last_h2s = node->h2s_history.empty() ? -1 : node->h2s_history.back();
  
  if (last_ch4 >= CH4_DANGER) {
    Serial.printf("DANGER: %s - High Methane (CH4): %.1f PPM!\n", node->nodeName.c_str(), last_ch4);
    danger = true;
  }
  if (last_co >= CO_DANGER) {
    Serial.printf("DANGER: %s - High Carbon Monoxide (CO): %.1f PPM!\n", node->nodeName.c_str(), last_co);
    danger = true;
  }
  if (last_h2s >= H2S_DANGER) {
    Serial.printf("DANGER: %s - High Hydrogen Sulfide (H2S): %.1f PPM!\n", node->nodeName.c_str(), last_h2s);
    danger = true;
  }
  
  if (danger) {
    Serial.println("CRITICAL ALERT - DANGEROUS LEVELS DETECTED!");
  }
}

void displayNodeData(NodeInfo* node, Packet* pkt) {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 📊 NODE DATA RECEIVED                         ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.printf("🏭 Node: %s (ID: %d)\n", node->nodeName.c_str(), node->nodeID);
  Serial.print("📍 MAC: "); printMAC(node->mac); Serial.println();
  
  // **ENHANCED: Show forwarding information if applicable**
  if (pkt->hopCount > 0) {
    Serial.printf("🔄 Forwarded via %d hop(s) | ", pkt->hopCount);
  }
  
  Serial.printf("📡 RSSI: %d | Seq: %u\n", pkt->rssi, pkt->seq);
  Serial.printf("⏰ Time: %lu ms ago\n", millis() - pkt->timestamp);
  Serial.printf("💨 CH4: %.1f PPM %s\n", pkt->ch4, (pkt->ch4 >= CH4_DANGER) ? "🚨 DANGER" : "✅ OK");
  Serial.printf("☁️  CO: %.1f PPM %s\n", pkt->co, (pkt->co >= CO_DANGER) ? "🚨 DANGER" : "✅ OK");
  Serial.printf("💨 CO2: %.1f PPM %s\n", pkt->co2, (pkt->co2 >= CO2_POOR) ? "⚠️  POOR" : "✅ OK");
  Serial.printf("🧪 H2S: %.1f PPM %s\n", pkt->h2s, (pkt->h2s >= H2S_DANGER) ? "🚨 DANGER" : "✅ OK");
  Serial.printf("🌡️  Temp: %.1f C %s\n", pkt->temp, (pkt->temp >= TEMP_DANGER) ? "⚠️  HIGH" : "✅ OK");
  Serial.printf("💧 Humidity: %.1f %% %s\n", pkt->humidity, (pkt->humidity >= HUMIDITY_DANGER) ? "⚠️  HIGH" : "✅ OK");
  Serial.printf("🔧 Raw: MQ4:%d MQ7:%d MQ135:%d MQ136:%d\n", pkt->raw4, pkt->raw7, pkt->raw135, pkt->raw136);
  Serial.println();
}

void displayNetworkStatus() {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║ 🌐 NETWORK STATUS                              ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  
  int activeCount = 0;
  for (const auto &node : nodes) {
    if (node.isActive) activeCount++;
  }
  
  Serial.printf("📊 Active Nodes: %d/%d\n", activeCount, MAX_NODES);
  Serial.printf("📡 STA SSID: %s | IP: %s\n", sta_ssid, WiFi.softAPIP().toString().c_str());
  
  for (const auto &node : nodes) {
    if (node.isActive) {
      unsigned long age = (millis() - node.lastSeen) / 1000;
      String status = (age < 30) ? "🟢" : (age < 60) ? "🟡" : "🔴";
      Serial.printf("%s %s: Hops=%2d RSSI=%3d Age=%3lus Pkts=%3d\n", 
                    status.c_str(), node.nodeName.c_str(), node.lastHopCount, node.lastRSSI, age, node.packetCount);
    }
  }
  Serial.println();
}

// Update the main loop to periodically check node registration
void checkNodeRegistration() {
  static unsigned long lastRegCheck = 0;
  unsigned long now = millis();
  
  if (now - lastRegCheck > 30000) { // Every 30 seconds
    for (auto &node : nodes) {
      if (node.isActive && node.nodeID != 0) {
        // Verify node is still properly registered
        Serial.printf("📋 Node %d (%s) - Active: %s, Packets: %d, Last Seen: %lu sec ago\n",
                     node.nodeID, node.nodeName.c_str(), 
                     node.isActive ? "YES" : "NO",
                     node.packetCount,
                     (now - node.lastSeen) / 1000);
      }
    }
    lastRegCheck = now;
  }
}

// ===== FIXED ESP-NOW CALLBACKS =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  
  Packet pkt;
  memcpy(&pkt, data, sizeof(pkt));
  
  int8_t rssi = info->rx_ctrl->rssi;
  pkt.rssi = rssi;
  
  switch (pkt.type) {
    case PKT_HELLO: {
      // For hello packets, use the source ID directly
      NodeInfo* node = findOrCreateNode(info->src_addr, pkt.sourceID);
      if (node) {
        node->lastHopCount = pkt.hopCount;
        node->lastRSSI = rssi;
        Serial.printf("👋 Hello from %s (ID:%d, Hops=%d, RSSI=%d)\n", 
                      node->nodeName.c_str(), pkt.sourceID, pkt.hopCount, rssi);
      }
      break;
    }
    
    case PKT_DATA: {
      if (isDuplicate(pkt.sourceID, pkt.seq)) {
        Serial.printf("🔄 Duplicate packet from Node %d, Seq %u - ignoring\n", pkt.sourceID, pkt.seq);
        return;
      }
      
      // **ENHANCED: Log forwarding information for debugging**
      bool isForwarded = (pkt.hopCount > 0);
      if (isForwarded) {
        Serial.printf("📨 Forwarded packet detected: Original Source=%d, Forwarder MAC=", pkt.sourceID);
        printMAC(info->src_addr);
        Serial.printf(", Hops=%d\n", pkt.hopCount);
      }
      
      // For data packets in multi-hop, the sourceID in packet is the original sensor
      // The MAC is the immediate sender (could be intermediate node)
      NodeInfo* node = findOrCreateNode(info->src_addr, 0, pkt.sourceID); // Use original source ID
      
      if (node) {
        // **FIX: Update node information correctly**
        node->lastHopCount = pkt.hopCount;
        node->lastRSSI = rssi;
        node->lastSeen = millis();
        node->packetCount++;
        
        // Update with the actual source node's data
        updateNodeWithData(node, &pkt);
        
        // **ENHANCED: Display forwarding info in the data display**
        displayNodeData(node, &pkt);
        checkDangerLevels(node);
        sendAck(info->src_addr, pkt.seq);
        
        // Re-broadcast if needed (multi-hop) - only if we're not the ultimate destination
        if (pkt.destID != 1 && pkt.hopCount < 5) { // Limit hops to 5
          pkt.hopCount++;
          pkt.forwarderID = 1; // Server ID
          const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          sendPacket(broadcast, &pkt);
          Serial.printf("🔄 Multi-hop: Forwarding packet from Node %d (Hops=%d)\n", pkt.sourceID, pkt.hopCount);
        }
      }
      break;
    }
    
    case PKT_ID_ANNOUNCE: {
      // ID announce packets help with node registration
      NodeInfo* node = findOrCreateNode(info->src_addr, pkt.sourceID);
      if (node) {
        Serial.printf("📝 %s (ID:%d) registered with server via ID announce\n", 
                      node->nodeName.c_str(), pkt.sourceID);
      }
      break;
    }
  }
}

void OnDataSent(const esp_now_send_info_t *send_info, esp_now_send_status_t status) {
  Serial.printf("📤 Server packet sent - %s\n", status == ESP_NOW_SEND_SUCCESS ? "✅ OK" : "❌ FAIL");
}

// ===== OPTIMIZED WEB INTERFACE =====
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Industrial Sensor Network</title>
    <style>
        :root { --primary: #00d4ff; --secondary: #0088cc; --accent: #00b4d8; --success: #00cc88; --warning: #ffaa00; --danger: #ff4444; --dark: #0a0a0a; --darker: #050505; --light: #1a1a1a; --lighter: #2a2a2a; --text: #e0e0e0; --text-secondary: #a0a0a0; }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, var(--darker) 0%, var(--dark) 100%); min-height: 100vh; color: var(--text); overflow-x: hidden; }
        .dashboard { max-width: 1400px; margin: 0 auto; padding: 20px; }
        .header { background: rgba(10, 10, 10, 0.95); backdrop-filter: blur(20px); padding: 30px; border-radius: 15px; margin-bottom: 20px; border: 1px solid var(--lighter); box-shadow: 0 8px 32px rgba(0, 212, 255, 0.1); text-align: center; }
        .header h1 { background: linear-gradient(135deg, var(--primary), var(--accent)); -webkit-background-clip: text; -webkit-text-fill-color: transparent; font-size: 2.5em; margin-bottom: 10px; font-weight: 300; letter-spacing: 1px; }
        .header p { color: var(--text-secondary); font-size: 1.1em; font-weight: 300; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-bottom: 20px; }
        .card { background: rgba(20, 20, 20, 0.9); backdrop-filter: blur(15px); padding: 25px; border-radius: 12px; border: 1px solid var(--lighter); box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3); transition: all 0.3s ease; position: relative; overflow: hidden; }
        .card:hover { border-color: var(--primary); transform: translateY(-2px); box-shadow: 0 12px 40px rgba(0, 212, 255, 0.2); }
        .card h3 { color: var(--primary); margin-bottom: 20px; font-size: 1.3em; font-weight: 400; border-bottom: 1px solid var(--lighter); padding-bottom: 10px; }
        .stats-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; }
        .stat-card { background: linear-gradient(135deg, var(--light), var(--lighter)); padding: 18px; border-radius: 8px; text-align: center; border: 1px solid var(--lighter); transition: all 0.3s ease; position: relative; overflow: hidden; }
        .stat-card::before { content: ''; position: absolute; top: 0; left: 0; right: 0; height: 2px; background: linear-gradient(90deg, var(--primary), var(--accent)); }
        .stat-card:hover { transform: scale(1.02); border-color: var(--primary); }
        .stat-card.danger::before { background: linear-gradient(90deg, var(--danger), #ff6b6b); }
        .stat-card.warning::before { background: linear-gradient(90deg, var(--warning), #ffd93d); }
        .stat-card.success::before { background: linear-gradient(90deg, var(--success), #6bffb8); }
        .stat-number { font-size: 2em; font-weight: 300; margin-bottom: 5px; color: var(--text); }
        .stat-label { font-size: 0.85em; color: var(--text-secondary); font-weight: 300; }
        .btn { background: linear-gradient(135deg, var(--light), var(--lighter)); color: var(--text); border: 1px solid var(--lighter); padding: 12px 20px; border-radius: 6px; cursor: pointer; font-size: 0.95em; margin: 4px; transition: all 0.3s ease; font-weight: 400; backdrop-filter: blur(10px); }
        .btn:hover { border-color: var(--primary); background: linear-gradient(135deg, var(--lighter), var(--light)); transform: translateY(-1px); box-shadow: 0 4px 15px rgba(0, 212, 255, 0.2); }
        .btn-success { border-color: var(--success); color: var(--success); }
        .btn-success:hover { background: rgba(0, 204, 136, 0.1); box-shadow: 0 4px 15px rgba(0, 204, 136, 0.3); }
        .btn-warning { border-color: var(--warning); color: var(--warning); }
        .btn-warning:hover { background: rgba(255, 170, 0, 0.1); box-shadow: 0 4px 15px rgba(255, 170, 0, 0.3); }
        .node-card { background: rgba(25, 25, 25, 0.8); border-left: 3px solid var(--success); margin-bottom: 12px; padding: 18px; border-radius: 8px; transition: all 0.3s ease; border: 1px solid var(--lighter); }
        .node-card:hover { border-left-color: var(--primary); border-color: var(--primary); transform: translateX(4px); }
        .node-card.inactive { border-left-color: var(--text-secondary); opacity: 0.6; }
        .node-card.warning { border-left-color: var(--warning); }
        .node-card.danger { border-left-color: var(--danger); animation: pulse-danger 2s infinite; }
        .node-card h4 { color: var(--text); margin-bottom: 10px; display: flex; align-items: center; justify-content: space-between; font-weight: 400; }
        .live-badge { background: var(--success); color: var(--dark); padding: 4px 12px; border-radius: 12px; font-size: 0.75em; font-weight: 600; }
        .warning-badge { background: var(--warning); color: var(--dark); padding: 4px 12px; border-radius: 12px; font-size: 0.75em; font-weight: 600; }
        .danger-badge { background: var(--danger); color: white; padding: 4px 12px; border-radius: 12px; font-size: 0.75em; font-weight: 600; }
        .info-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 8px; margin-top: 12px; }
        .info-item { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px solid rgba(255,255,255,0.05); }
        .info-label { font-weight: 300; color: var(--text-secondary); font-size: 0.85em; }
        .info-value { color: var(--text); font-family: 'Courier New', monospace; font-size: 0.85em; font-weight: 500; }
        .alert-banner { background: linear-gradient(135deg, var(--danger), #ff6b6b); color: white; padding: 15px; border-radius: 8px; margin: 15px 0; text-align: center; font-weight: 500; border: 1px solid rgba(255,255,255,0.2); animation: pulse 2s infinite; }
        .tab-content { display: none; }
        .tab-content.active { display: block; }
        .form-control { width: 100%; padding: 10px; background: rgba(30, 30, 30, 0.8); border: 1px solid var(--lighter); border-radius: 6px; color: var(--text); font-size: 0.95em; transition: all 0.3s ease; }
        .form-control:focus { outline: none; border-color: var(--primary); box-shadow: 0 0 0 2px rgba(0, 212, 255, 0.2); }
        .sensor-reading { padding: 12px; margin: 6px 0; background: rgba(30, 30, 30, 0.6); border-radius: 6px; border-left: 3px solid var(--primary); border: 1px solid var(--lighter); }
        .reading-value { font-size: 1.3em; font-weight: 300; color: var(--text); }
        .reading-unit { color: var(--text-secondary); font-size: 0.85em; font-weight: 300; }
        .sensor-reading.danger { border-left: 3px solid var(--danger); background: rgba(255, 68, 68, 0.1); }
        .sensor-reading.warning { border-left: 3px solid var(--warning); background: rgba(255, 170, 0, 0.1); }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.8; } 100% { opacity: 1; } }
        @keyframes pulse-danger { 0% { box-shadow: 0 0 0 0 rgba(255, 68, 68, 0.4); } 70% { box-shadow: 0 0 0 10px rgba(255, 68, 68, 0); } 100% { box-shadow: 0 0 0 0 rgba(255, 68, 68, 0); } }
        @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } .stats-grid { grid-template-columns: 1fr; } .header h1 { font-size: 2em; } }
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="header">
            <h1>🏭DEEP WIRE🏭</h1>
            <p>Real-time Hazardous Gas Monitoring & Safety System</p>
            <div style="margin-top: 15px; font-size: 0.9em; color: var(--text-secondary);">
                <strong>Network:</strong> )rawliteral";
  html += sta_ssid;
  html += R"rawliteral( | <strong>IP:</strong> )rawliteral";
  html += WiFi.softAPIP().toString();
  html += R"rawliteral( | <strong>Uptime:</strong> <span id="uptime">00:00:00</span>
            </div>
        </div>

        <div id="alertBanner" class="alert-banner" style="display: none;">
            🚨 CRITICAL ALERT - DANGEROUS LEVELS DETECTED
        </div>

        <div class="grid">
            <div class="card">
                <h3>📊 System Overview</h3>
                <div class="stats-grid">
                    <div class="stat-card">
                        <div class="stat-number" id="totalNodes">0</div>
                        <div class="stat-label">Total Nodes</div>
                    </div>
                    <div class="stat-card success">
                        <div class="stat-number" id="activeNodes">0</div>
                        <div class="stat-label">Active Nodes</div>
                    </div>
                    <div class="stat-card warning">
                        <div class="stat-number" id="warningNodes">0</div>
                        <div class="stat-label">Warnings</div>
                    </div>
                    <div class="stat-card danger">
                        <div class="stat-number" id="dangerNodes">0</div>
                        <div class="stat-label">Critical Alerts</div>
                    </div>
                </div>
            </div>

            <div class="card">
                <h3>🎛️ Control Panel</h3>
                <button class="btn btn-success" onclick="requestReadings()">📡 Request Readings</button>
                <button class="btn" onclick="refreshData()">🔄 Refresh</button>
                <button class="btn btn-warning" onclick="toggleAutoRefresh()" id="autoRefreshBtn">⏰ Auto: OFF</button>
                <div style="margin-top: 15px;">
                    <button class="btn" onclick="showTab('nodes')">📋 Nodes</button>
                    <button class="btn" onclick="showTab('export')">📥 Export</button>
                </div>
            </div>
        </div>

        <!-- Nodes Tab -->
        <div id="nodesTab" class="tab-content active">
            <div class="card">
                <h3>📡 Connected Sensor Nodes</h3>
                <div id="nodesList">
                    <div style="text-align: center; padding: 40px; color: var(--text-secondary);">
                        <div style="font-size: 3em; margin-bottom: 10px;">🔍</div>
                        <p>Scanning for sensor nodes...</p>
                    </div>
                </div>
            </div>
        </div>

        <!-- Export Tab -->
        <div id="exportTab" class="tab-content">
            <div class="card">
                <h3>📥 Data Export</h3>
                <div class="form-group">
                    <label for="exportNodeSelect" style="color: var(--text-secondary);">Select Node:</label>
                    <select id="exportNodeSelect" class="form-control">
                        <option value="">Loading nodes...</option>
                    </select>
                </div>
                <div class="form-group">
                    <label for="exportDate" style="color: var(--text-secondary);">Select Date:</label>
                    <input type="date" id="exportDate" class="form-control">
                </div>
                <button class="btn btn-success" onclick="exportData()">📊 Export to Excel</button>
                <button class="btn" onclick="exportAllData()">💾 Export All Data</button>
                
                <div id="exportStatus" style="margin-top: 15px; padding: 10px; border-radius: 5px; display: none;"></div>
            </div>
        </div>

        <div class="card">
            <h3>📋 Recent Activity Log</h3>
            <div id="activityLog" style="max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 0.9em; background: var(--light); padding: 15px; border-radius: 8px; color: var(--text-secondary);">
                <div>System initialized at <span id="initTime"></span></div>
            </div>
        </div>
    </div>

    <script>
        let autoRefresh = false;
        let autoRefreshInterval;
        let startTime = Date.now();
        let currentNodes = [];

        document.getElementById('initTime').textContent = new Date().toLocaleString();
        setInterval(updateUptime, 1000);
        refreshData();

        function updateUptime() {
            const uptime = Math.floor((Date.now() - startTime) / 1000);
            const hours = Math.floor(uptime / 3600);
            const minutes = Math.floor((uptime % 3600) / 60);
            const seconds = uptime % 60;
            document.getElementById('uptime').textContent = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
        }

        function showTab(tabName) {
            document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
            document.getElementById(tabName + 'Tab').classList.add('active');
        }

        function requestReadings() {
            fetch('/api/request-readings', { method: 'POST' })
                .then(r => r.json())
                .then(data => showNotification('📡 Reading requests sent'))
                .catch(err => showNotification('❌ Failed to send requests'));
        }

        function refreshData() {
            fetch('/api/nodes')
                .then(r => r.json())
                .then(nodes => {
                    currentNodes = nodes;
                    updateNodesDisplay(nodes);
                    updateStats(nodes);
                    updateDropdowns(nodes);
                    checkAlerts(nodes);
                })
                .catch(err => {
                    console.error('Refresh error:', err);
                    showNotification('❌ Failed to refresh data');
                });
        }

        function updateNodesDisplay(nodes) {
            const nodesList = document.getElementById('nodesList');
            if (nodes.length === 0) {
                nodesList.innerHTML = '<div style="text-align: center; padding: 40px; color: var(--text-secondary);"><div style="font-size: 3em; margin-bottom: 10px;">🔍</div><p>No sensor nodes connected</p></div>';
                return;
            }
            nodesList.innerHTML = nodes.map(node => {
                const lastSeen = parseInt(node.lastSeen);
                const isRecentlyActive = lastSeen < 30000;
                const lastSeenSec = Math.floor(lastSeen / 1000);
                let status = 'normal', statusBadge = '';
                if (!node.isActive || lastSeen > 60000) status = 'inactive';
                else if (node.last_ch4 > 1000 || node.last_co > 35 || node.last_h2s > 50) { status = 'danger'; statusBadge = '<span class="danger-badge">DANGER</span>'; }
                else if (node.last_ch4 > 500 || node.last_co > 20 || node.last_h2s > 25) { status = 'warning'; statusBadge = '<span class="warning-badge">WARNING</span>'; }
                else if (isRecentlyActive) statusBadge = '<span class="live-badge">LIVE</span>';
                return `<div class="node-card ${status}">
                    <h4>${node.nodeName} (ID: ${node.nodeID}) ${statusBadge}</h4>
                    <div class="info-grid">
                        <div class="info-item"><span class="info-label">MAC:</span><span class="info-value" style="font-size: 0.75em;">${node.mac}</span></div>
                        <div class="info-item"><span class="info-label">Last Seen:</span><span class="info-value">${lastSeenSec}s ago</span></div>
                        <div class="info-item"><span class="info-label">Packets:</span><span class="info-value">${node.packetCount}</span></div>
                        <div class="info-item"><span class="info-label">Hops:</span><span class="info-value">${node.lastHopCount}</span></div>
                        <div class="info-item"><span class="info-label">RSSI:</span><span class="info-value">${node.lastRSSI} dBm</span></div>
                        <div class="info-item"><span class="info-label">Status:</span><span class="info-value">${node.isActive ? 'ACTIVE' : 'INACTIVE'}</span></div>
                    </div>
                    <div style="margin-top: 15px;">
                        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 12px;">
                            <div class="sensor-reading ${node.last_ch4 > 1000 ? 'danger' : node.last_ch4 > 500 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_ch4 >= 0 ? node.last_ch4.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">CH4 PPM</div>
                                ${node.last_ch4 > 1000 ? '🚨' : node.last_ch4 > 500 ? '⚠️' : '✅'}
                            </div>
                            <div class="sensor-reading ${node.last_co > 35 ? 'danger' : node.last_co > 20 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_co >= 0 ? node.last_co.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">CO PPM</div>
                                ${node.last_co > 35 ? '🚨' : node.last_co > 20 ? '⚠️' : '✅'}
                            </div>
                            <div class="sensor-reading ${node.last_co2 > 5000 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_co2 >= 0 ? node.last_co2.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">CO2 PPM</div>
                                ${node.last_co2 > 5000 ? '⚠️' : '✅'}
                            </div>
                            <div class="sensor-reading ${node.last_h2s > 50 ? 'danger' : node.last_h2s > 25 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_h2s >= 0 ? node.last_h2s.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">H2S PPM</div>
                                ${node.last_h2s > 50 ? '🚨' : node.last_h2s > 25 ? '⚠️' : '✅'}
                            </div>
                            <div class="sensor-reading ${node.last_temp > 40 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_temp >= 0 ? node.last_temp.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">°C</div>
                                ${node.last_temp > 40 ? '⚠️' : '✅'}
                            </div>
                            <div class="sensor-reading ${node.last_humidity > 80 ? 'warning' : ''}">
                                <div class="reading-value">${node.last_humidity >= 0 ? node.last_humidity.toFixed(1) : 'N/A'}</div>
                                <div class="reading-unit">% RH</div>
                                ${node.last_humidity > 80 ? '⚠️' : '✅'}
                            </div>
                        </div>
                    </div>
                </div>`;
            }).join('');
        }

        function updateStats(nodes) {
            const activeNodes = nodes.filter(n => n.isActive && parseInt(n.lastSeen) < 60000).length;
            const warningNodes = nodes.filter(n => n.isActive && parseInt(n.lastSeen) < 60000 && (n.last_ch4 > 500 || n.last_co > 20 || n.last_h2s > 25) && (n.last_ch4 <= 1000 && n.last_co <= 35 && n.last_h2s <= 50)).length;
            const dangerNodes = nodes.filter(n => n.isActive && parseInt(n.lastSeen) < 60000 && (n.last_ch4 > 1000 || n.last_co > 35 || n.last_h2s > 50)).length;
            document.getElementById('totalNodes').textContent = nodes.length;
            document.getElementById('activeNodes').textContent = activeNodes;
            document.getElementById('warningNodes').textContent = warningNodes;
            document.getElementById('dangerNodes').textContent = dangerNodes;
        }

        function updateDropdowns(nodes) {
            const exportSelect = document.getElementById('exportNodeSelect');
            const currentExport = exportSelect.value;
            exportSelect.innerHTML = '<option value="">Select a node...</option>';
            nodes.forEach(node => {
                exportSelect.innerHTML += `<option value="${node.nodeID}">${node.nodeName} (ID: ${node.nodeID})</option>`;
            });
            if (currentExport && nodes.some(n => n.nodeID == currentExport)) exportSelect.value = currentExport;
        }

        function checkAlerts(nodes) {
            const hasDanger = nodes.some(n => n.isActive && (n.last_ch4 > 1000 || n.last_co > 35 || n.last_h2s > 50));
            document.getElementById('alertBanner').style.display = hasDanger ? 'block' : 'none';
        }

   function exportData() {
    const nodeId = document.getElementById('exportNodeSelect').value;
    const date = document.getElementById('exportDate').value;
    
    if (!nodeId || !date) {
        showNotification('❌ Please select both node and date');
        return;
    }

    showNotification('📤 Fetching historical data for export...');
    
    const node = currentNodes.find(n => n.nodeID == nodeId);
    if (!node) {
        showNotification('❌ Node data not found');
        return;
    }

    // Fetch the actual historical data from the server
    fetch(`/api/node-data/${nodeId}`)
        .then(r => r.json())
        .then(nodeData => {
            try {
                // Create CSV content
                let csvContent = 'Industrial Sensor Network - Data Export\r\n';
                csvContent += `Node: ${node.nodeName} (ID: ${node.nodeID})\r\n`;
                csvContent += `Export Date: ${date}\r\n`;
                csvContent += `Generated: ${new Date().toLocaleString()}\r\n`;
                csvContent += `Total Readings: ${Math.max(
                    nodeData.ch4_history?.length || 0,
                    nodeData.co_history?.length || 0,
                    nodeData.co2_history?.length || 0,
                    nodeData.h2s_history?.length || 0,
                    nodeData.temp_history?.length || 0,
                    nodeData.humidity_history?.length || 0
                )}\r\n\r\n`;
                
                csvContent += 'Reading #,Timestamp,CH4 (PPM),CO (PPM),CO2 (PPM),H2S (PPM),Temperature (°C),Humidity (%),Status\r\n';

                // Get all sensor histories
                const ch4Data = nodeData.ch4_history || [];
                const coData = nodeData.co_history || [];
                const co2Data = nodeData.co2_history || [];
                const h2sData = nodeData.h2s_history || [];
                const tempData = nodeData.temp_history || [];
                const humidityData = nodeData.humidity_history || [];
                
                // Find the maximum length of all sensor arrays
                const maxReadings = Math.max(
                    ch4Data.length,
                    coData.length, 
                    co2Data.length,
                    h2sData.length,
                    tempData.length,
                    humidityData.length
                );

                let readingCount = 0;
                
                // Export ALL historical readings in reverse chronological order (newest first)
                for (let i = maxReadings - 1; i >= 0; i--) {
                    readingCount++;
                    
                    // Calculate timestamp (most recent reading is at the end of arrays)
                    const timeAgo = (maxReadings - i - 1) * 2; // Assuming 2-minute intervals between readings
                    const timestamp = new Date(Date.now() - timeAgo * 60000);
                    
                    // Get sensor values for this reading
                    const ch4 = i < ch4Data.length ? ch4Data[i] : 'N/A';
                    const co = i < coData.length ? coData[i] : 'N/A';
                    const co2 = i < co2Data.length ? co2Data[i] : 'N/A';
                    const h2s = i < h2sData.length ? h2sData[i] : 'N/A';
                    const temp = i < tempData.length ? tempData[i] : 'N/A';
                    const humidity = i < humidityData.length ? humidityData[i] : 'N/A';
                    
                    // Determine status based on safety thresholds
                    let status = 'NORMAL';
                    if (ch4 !== 'N/A' && co !== 'N/A' && h2s !== 'N/A') {
                        if (ch4 > 1000 || co > 35 || h2s > 50) {
                            status = 'DANGER';
                        } else if (ch4 > 500 || co > 20 || h2s > 25) {
                            status = 'WARNING';
                        }
                    }

                    csvContent += `${readingCount},`;
                    csvContent += `${timestamp.toLocaleString()},`;
                    csvContent += `${ch4 !== 'N/A' ? ch4.toFixed(2) : 'N/A'},`;
                    csvContent += `${co !== 'N/A' ? co.toFixed(2) : 'N/A'},`;
                    csvContent += `${co2 !== 'N/A' ? co2.toFixed(2) : 'N/A'},`;
                    csvContent += `${h2s !== 'N/A' ? h2s.toFixed(2) : 'N/A'},`;
                    csvContent += `${temp !== 'N/A' ? temp.toFixed(2) : 'N/A'},`;
                    csvContent += `${humidity !== 'N/A' ? humidity.toFixed(2) : 'N/A'},`;
                    csvContent += `${status}\r\n`;
                }

                // Add current readings summary
                csvContent += '\r\nCURRENT READINGS SUMMARY\r\n';
                csvContent += 'Parameter,Current Value,Unit,Status,Safety Threshold\r\n';
                csvContent += `Methane (CH4),${node.last_ch4 >= 0 ? node.last_ch4.toFixed(2) : 'N/A'},PPM,${node.last_ch4 > 1000 ? 'DANGER' : node.last_ch4 > 500 ? 'WARNING' : 'NORMAL'},500-1000 PPM\r\n`;
                csvContent += `Carbon Monoxide (CO),${node.last_co >= 0 ? node.last_co.toFixed(2) : 'N/A'},PPM,${node.last_co > 35 ? 'DANGER' : node.last_co > 20 ? 'WARNING' : 'NORMAL'},20-35 PPM\r\n`;
                csvContent += `Hydrogen Sulfide (H2S),${node.last_h2s >= 0 ? node.last_h2s.toFixed(2) : 'N/A'},PPM,${node.last_h2s > 50 ? 'DANGER' : node.last_h2s > 25 ? 'WARNING' : 'NORMAL'},25-50 PPM\r\n`;
                csvContent += `Temperature,${node.last_temp >= 0 ? node.last_temp.toFixed(2) : 'N/A'},°C,${node.last_temp > 40 ? 'HIGH' : 'NORMAL'},40°C\r\n`;
                csvContent += `Humidity,${node.last_humidity >= 0 ? node.last_humidity.toFixed(2) : 'N/A'},%,${node.last_humidity > 80 ? 'HIGH' : 'NORMAL'},80%\r\n`;

                // Add node information
                csvContent += '\r\nNODE INFORMATION\r\n';
                csvContent += `Node Name,${node.nodeName}\r\n`;
                csvContent += `Node ID,${node.nodeID}\r\n`;
                csvContent += `MAC Address,${node.mac}\r\n`;
                csvContent += `Last Seen,${Math.floor(parseInt(node.lastSeen) / 1000)} seconds ago\r\n`;
                csvContent += `Total Packets,${node.packetCount}\r\n`;
                csvContent += `Signal Strength,${node.lastRSSI} dBm\r\n`;
                csvContent += `Hop Count,${node.lastHopCount}\r\n`;

                // Create and download file
                const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
                const link = document.createElement('a');
                const url = URL.createObjectURL(blob);
                
                link.setAttribute('href', url);
                link.setAttribute('download', `sensor_data_${node.nodeName.replace(/\s+/g, '_')}_${date}.csv`);
                link.style.visibility = 'hidden';
                
                document.body.appendChild(link);
                link.click();
                document.body.removeChild(link);
                
                showNotification(`✅ ${readingCount} readings exported successfully!`);
                
            } catch (error) {
                console.error('Export processing error:', error);
                showNotification('❌ Export processing failed: ' + error.message);
            }
        })
        .catch(error => {
            console.error('API fetch error:', error);
            showNotification('❌ Failed to fetch historical data from server');
        });
}

function exportAllData() {
    showNotification('📤 Preparing complete data export...');
    
    try {
        let csvContent = 'Industrial Sensor Network - Complete Data Export\r\n';
        csvContent += `Export Date: ${new Date().toISOString().split('T')[0]}\r\n`;
        csvContent += `Generated: ${new Date().toLocaleString()}\r\n\r\n`;
        
        csvContent += 'Node Name,Node ID,Last Seen,Status,CH4 (PPM),CO (PPM),H2S (PPM),Temp (°C),Humidity (%),Packets,RSSI (dBm)\r\n';

        currentNodes.forEach(node => {
            const lastSeenMinutes = Math.floor(parseInt(node.lastSeen) / 60000);
            const status = node.isActive ? 
                (lastSeenMinutes < 5 ? 'ACTIVE' : 'INACTIVE') : 'OFFLINE';

            csvContent += `${node.nodeName},`;
            csvContent += `${node.nodeID},`;
            csvContent += `${lastSeenMinutes} min ago,`;
            csvContent += `${status},`;
            csvContent += `${node.last_ch4 >= 0 ? node.last_ch4.toFixed(2) : 'N/A'},`;
            csvContent += `${node.last_co >= 0 ? node.last_co.toFixed(2) : 'N/A'},`;
            csvContent += `${node.last_h2s >= 0 ? node.last_h2s.toFixed(2) : 'N/A'},`;
            csvContent += `${node.last_temp >= 0 ? node.last_temp.toFixed(2) : 'N/A'},`;
            csvContent += `${node.last_humidity >= 0 ? node.last_humidity.toFixed(2) : 'N/A'},`;
            csvContent += `${node.packetCount},`;
            csvContent += `${node.lastRSSI}\r\n`;
        });

        // Create and download file
        const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
        const link = document.createElement('a');
        const url = URL.createObjectURL(blob);
        
        link.setAttribute('href', url);
        link.setAttribute('download', `complete_sensor_network_${new Date().toISOString().split('T')[0]}.csv`);
        link.style.visibility = 'hidden';
        
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        
        showNotification('✅ Complete data exported successfully!');
        
    } catch (error) {
        console.error('Export error:', error);
        showNotification('❌ Export failed: ' + error.message);
    }
}

        function toggleAutoRefresh() {
            autoRefresh = !autoRefresh;
            const btn = document.getElementById('autoRefreshBtn');
            btn.textContent = `⏰ Auto: ${autoRefresh ? 'ON' : 'OFF'}`;
            btn.className = autoRefresh ? 'btn btn-success' : 'btn btn-warning';
            if (autoRefresh) {
                autoRefreshInterval = setInterval(refreshData, 5000);
            } else {
                clearInterval(autoRefreshInterval);
            }
        }

        function showNotification(message) {
            const toast = document.createElement('div');
            toast.style.cssText = 'position: fixed; top: 20px; right: 20px; background: linear-gradient(135deg, var(--success), #00ffaa); color: var(--dark); padding: 15px 25px; border-radius: 8px; box-shadow: 0 8px 25px rgba(0,0,0,0.3); z-index: 1000; font-family: inherit; max-width: 300px; animation: slideIn 0.3s ease; font-weight: 500; border: 1px solid rgba(255,255,255,0.2);';
            toast.textContent = message;
            document.body.appendChild(toast);
            setTimeout(() => {
                if (document.body.contains(toast)) {
                    toast.style.animation = 'slideOut 0.3s ease';
                    setTimeout(() => { if (document.body.contains(toast)) document.body.removeChild(toast); }, 300);
                }
            }, 3000);
        }

        const style = document.createElement('style');
        style.textContent = '@keyframes slideIn { from { transform: translateX(100%); opacity: 0; } to { transform: translateX(0); opacity: 1; } } @keyframes slideOut { from { transform: translateX(0); opacity: 1; } to { transform: translateX(100%); opacity: 0; } }';
        document.head.appendChild(style);
    </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// ===== WEB SERVER HANDLERS =====
void handleAPINodes() {
  DynamicJsonDocument doc(4096);
  JsonArray nodesArray = doc.to<JsonArray>();
  
  for (const auto& node : nodes) {
    JsonObject nodeObj = nodesArray.createNestedObject();
    nodeObj["nodeID"] = node.nodeID;
    nodeObj["nodeName"] = node.nodeName;
    
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             node.mac[0], node.mac[1], node.mac[2], node.mac[3], node.mac[4], node.mac[5]);
    nodeObj["mac"] = macStr;
    
    nodeObj["lastSeen"] = millis() - node.lastSeen;
    nodeObj["packetCount"] = node.packetCount;
    nodeObj["isActive"] = node.isActive;
    nodeObj["lastRSSI"] = node.lastRSSI;
    nodeObj["lastHopCount"] = node.lastHopCount;
    
    nodeObj["last_ch4"] = node.ch4_history.empty() ? -1 : node.ch4_history.back();
    nodeObj["last_co"] = node.co_history.empty() ? -1 : node.co_history.back();
    nodeObj["last_co2"] = node.co2_history.empty() ? -1 : node.co2_history.back();
    nodeObj["last_h2s"] = node.h2s_history.empty() ? -1 : node.h2s_history.back();
    nodeObj["last_temp"] = node.temp_history.empty() ? -1 : node.temp_history.back();
    nodeObj["last_humidity"] = node.humidity_history.empty() ? -1 : node.humidity_history.back();
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleAPINodeData() {
  String nodeIdStr = server.pathArg(0);
  uint8_t nodeId = nodeIdStr.toInt();
  
  for (const auto& node : nodes) {
    if (node.nodeID == nodeId) {
      DynamicJsonDocument doc(8192);
      doc["nodeID"] = node.nodeID;
      doc["nodeName"] = node.nodeName;
      doc["mac"] = node.mac;
      doc["lastSeen"] = millis() - node.lastSeen;
      doc["packetCount"] = node.packetCount;
      doc["isActive"] = node.isActive;
      doc["lastRSSI"] = node.lastRSSI;
      doc["lastHopCount"] = node.lastHopCount;
      
      // Current readings
      doc["last_ch4"] = node.ch4_history.empty() ? -1 : node.ch4_history.back();
      doc["last_co"] = node.co_history.empty() ? -1 : node.co_history.back();
      doc["last_co2"] = node.co2_history.empty() ? -1 : node.co2_history.back();
      doc["last_h2s"] = node.h2s_history.empty() ? -1 : node.h2s_history.back();
      doc["last_temp"] = node.temp_history.empty() ? -1 : node.temp_history.back();
      doc["last_humidity"] = node.humidity_history.empty() ? -1 : node.humidity_history.back();
      
      // Historical data
      JsonArray ch4Array = doc.createNestedArray("ch4_history");
      for (float val : node.ch4_history) ch4Array.add(val);
      
      JsonArray coArray = doc.createNestedArray("co_history");
      for (float val : node.co_history) coArray.add(val);
      
      JsonArray co2Array = doc.createNestedArray("co2_history");
      for (float val : node.co2_history) co2Array.add(val);
      
      JsonArray h2sArray = doc.createNestedArray("h2s_history");
      for (float val : node.h2s_history) h2sArray.add(val);
      
      JsonArray tempArray = doc.createNestedArray("temp_history");
      for (float val : node.temp_history) tempArray.add(val);
      
      JsonArray humidityArray = doc.createNestedArray("humidity_history");
      for (float val : node.humidity_history) humidityArray.add(val);
      
      String response;
      serializeJson(doc, response);
      server.send(200, "application/json", response);
      return;
    }
  }
  
  server.send(404, "application/json", "{\"error\": \"Node not found\"}");
}

void handleAPIRequestReadings() {
  requestReadingsFromAllNodes();
  server.send(200, "application/json", "{\"message\": \"Reading requests sent to all active nodes\"}");
}

void handleNotFound() {
  server.send(404, "application/json", "{\"error\": \"Endpoint not found\"}");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/nodes", handleAPINodes);
  server.on("/api/node-data/", handleAPINodeData);
  server.on("/api/node-data/{id}", handleAPINodeData);
  server.on("/api/request-readings", HTTP_POST, handleAPIRequestReadings);
  server.onNotFound(handleNotFound);
  
  server.begin();
  printSuccessBox("HTTP server started successfully");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  
  pinMode(SERVER_BUTTON_PIN, INPUT_PULLUP);
  
  setupIsolatedSTA();
  
  WiFi.macAddress(myMAC);
  
  printInfoBox("ENHANCED INDUSTRIAL SENSOR NETWORK SERVER");
  Serial.println("🌐 NETWORK CONFIGURATION:");
  Serial.printf("📡 STA SSID: %s\n", sta_ssid);
  Serial.printf("🌐 STA IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.print("📍 Server MAC: "); 
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", myMAC[i]);
    if(i < 5) Serial.print(":");
  }
  Serial.println();
  
  if (esp_now_init() != ESP_OK) {
    printErrorBox("ESP-NOW initialization failed! Restarting...");
    ESP.restart();
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ensurePeer(broadcast);
  
  setupWebServer();
  
  printSuccessBox("Server Initialized - Starting Aggressive Discovery");
  
  aggressiveServerAnnounce();
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long now = millis();
  
  handleServerButton();
  server.handleClient();
  
  static bool initialPhase = true;
  static unsigned long initialPhaseStart = millis();
  
  if (initialPhase) {
    if (now - initialPhaseStart < 30000) {
      if (now - lastServerAnnounce > 1000) {
        sendEnhancedServerAnnounce();
        lastServerAnnounce = now;
      }
    } else {
      initialPhase = false;
      printSuccessBox("Initial discovery phase completed");
    }
  } else {
    if (now - lastServerAnnounce > SERVER_ANNOUNCE_INTERVAL) {
      sendEnhancedServerAnnounce();
      lastServerAnnounce = now;
    }
  }
  
  if (now - lastHelloSent > HELLO_INTERVAL_MS) {
    sendHello();
    lastHelloSent = now;
  }
  
  static unsigned long lastCleanup = 0;
  if (now - lastCleanup > 20000) {
    cleanupInactiveNodes();
    lastCleanup = now;
  }
  
  static unsigned long lastStatusDisplay = 0;
  if (now - lastStatusDisplay > 45000) {
    displayNetworkStatus();
    lastStatusDisplay = now;
  }
  
  // Check node registration status
  checkNodeRegistration();
  
  delay(30);
}