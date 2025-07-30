#include <Arduino.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMessage.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi Access Point credentials
const char *ap_ssid = "OSC-to-CV";
const char *ap_password = "1234";

// OSC settings
WiFiUDP udp;
const int osc_port = 8000;

// MCP4822 DAC pins - you can modify these pin assignments
const int DAC_CS_PIN = 1;   // Chip Select
const int DAC_CS2_PIN = 2;  // Chip Select
const int DAC_SCK_PIN = 6;  // SPI Clock
const int DAC_MOSI_PIN = 4; // SPI MOSI
// const int DAC_LDAC_PIN = 19; // LDAC pin (optional, can be tied to ground)

// DAC settings
const int DAC_RESOLUTION = 4095; // 12-bit DAC (0-4095)
const float INPUT_MIN = 0.0;
const float INPUT_MAX = 1.0;

// Function declarations
void setupWiFiAP();
void setupDAC();
void writeDAC(uint8_t dac_chip, uint8_t channel, uint16_t value);
void handleOSCMessage();
uint16_t mapInputToDAC(float input);
void cvChannelA(OSCMessage &msg);
void cvChannelB(OSCMessage &msg);
void cvChannelC(OSCMessage &msg);
void cvChannelD(OSCMessage &msg);
void cvBothChannels(OSCMessage &msg);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 OSC to CV Converter Starting...");

  // Setup WiFi Access Point
  setupWiFiAP();
  delay(1000);

  // Setup DAC
  setupDAC();

  // Start UDP for OSC
  udp.begin(osc_port);
  Serial.printf("OSC server listening on port %d\n", osc_port);

  Serial.println("Setup complete!");
}

void loop() {
  // Handle OSC messages
  handleOSCMessage();

  // Small delay to prevent overwhelming the system
  delay(1);
}

void setupWiFiAP() {
  Serial.println("Setting up WiFi Access Point...");

  // Configure Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);

  // Wait for AP to start
  delay(1000);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);
  Serial.printf("SSID: %s\n", ap_ssid);
  Serial.printf("Password: %s\n", ap_password);
  Serial.printf("Connect to this network and send OSC to %s:%d\n",
                IP.toString().c_str(), osc_port);
}

void setupDAC() {
  Serial.println("Setting up MCP4822 DAC...");

  // Initialize CS pin
  pinMode(DAC_CS_PIN, OUTPUT);
  pinMode(DAC_CS2_PIN, OUTPUT);
  digitalWrite(DAC_CS_PIN, HIGH);  // Set CS high initially
  digitalWrite(DAC_CS2_PIN, HIGH); // Set CS high initially

  // Initialize SPI
  SPI.begin(DAC_SCK_PIN, -1, DAC_MOSI_PIN);
  delay(500); // Allow SPI to stabilize
  // SPI.setDataMode(SPI_MODE0);
  // SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Initialize all DAC channels to 0V
  writeDAC(1, 0, 0); // DAC1 Channel A
  writeDAC(1, 1, 0); // DAC1 Channel B
  writeDAC(2, 0, 0); // DAC2 Channel A (our Channel C)
  writeDAC(2, 1, 0); // DAC2 Channel B (our Channel D)

  Serial.println("Both DACs initialized");
}

void writeDAC(uint8_t dac_chip, uint8_t channel, uint16_t value) {
  // Ensure value is within 12-bit range
  value = constrain(value, 0, DAC_RESOLUTION);

  // MCP4822 command format:
  // Bit 15: Channel select (0 for A, 1 for B)
  // Bit 14: Unused (should be 0)
  // Bit 13: GA (Gain select: 1 = 1x, 0 = 2x)
  // Bit 12: SHDN (Shutdown: 1 = active, 0 = shutdown)
  // Bits 11-0: Data

  unsigned int command = channel ? 0x9000 : 0x1000; // Channel select + SHDN = 1
  int gain = 1;
  command |= gain ? 0x0000 : 0x2000; // GA bit
  command |= (value & 0x0FFF);       // 12-bit data

  // Select the appropriate CS pin based on DAC chip
  int cs_pin = (dac_chip == 1) ? DAC_CS_PIN : DAC_CS2_PIN;
  char dac_label =
      (dac_chip == 1) ? (channel ? 'B' : 'A') : (channel ? 'D' : 'C');

  Serial.printf("Writing to DAC%d Ch%c: %d (cmd: 0x%04X)\n", dac_chip,
                dac_label, value, command);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(1); // Small delay for setup time
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  delayMicroseconds(1); // Small delay before releasing CS
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();

  // Add a small delay to allow DAC to settle
  delayMicroseconds(10);
}

void handleOSCMessage() {
  OSCBundle bundle;
  int size = udp.parsePacket();

  if (size > 0) {
    while (size--) {
      bundle.fill(udp.read());
    }
    // Serial.println("Received OSC message:");
    if (!bundle.hasError()) {
      // Route OSC messages to appropriate handler functions
      bundle.dispatch("/cv/a", cvChannelA);
      bundle.dispatch("/cv/b", cvChannelB);
      bundle.dispatch("/cv/c", cvChannelC);
      bundle.dispatch("/cv/d", cvChannelD);
    } else {
      OSCErrorCode error = bundle.getError();
      Serial.print("OSC Error: ");
      Serial.println(error);
    }
  }
}

// OSC handler function for Channel A
void cvChannelA(OSCMessage &msg) {
  if (msg.isFloat(0)) {
    float value = msg.getFloat(0);
    uint16_t dacValue = mapInputToDAC(value);
    writeDAC(1, 0, dacValue); // DAC1, Channel A
    Serial.printf(">Channel A: %.3f\n", value);
  }
}

// OSC handler function for Channel B
void cvChannelB(OSCMessage &msg) {
  if (msg.isFloat(0)) {
    float value = msg.getFloat(0);
    uint16_t dacValue = mapInputToDAC(value);
    writeDAC(1, 1, dacValue); // DAC1, Channel B
    Serial.printf(">Channel B: %.3f\n", value);
  }
}

// OSC handler function for Channel C
void cvChannelC(OSCMessage &msg) {
  if (msg.isFloat(0)) {
    float value = msg.getFloat(0);
    uint16_t dacValue = mapInputToDAC(value);
    writeDAC(2, 0, dacValue); // DAC2, Channel A (our Channel C)
    Serial.printf(">Channel C: %.3f\n", value);
  }
}

// OSC handler function for Channel D
void cvChannelD(OSCMessage &msg) {
  if (msg.isFloat(0)) {
    float value = msg.getFloat(0);
    uint16_t dacValue = mapInputToDAC(value);
    writeDAC(2, 1, dacValue); // DAC2, Channel B (our Channel D)
    Serial.printf(">Channel D: %.3f\n", value);
  }
}

uint16_t mapInputToDAC(float input) {
  // Clamp input to expected range
  input = constrain(input, INPUT_MIN, INPUT_MAX);

  // Map from 0.0-1.0 to 0-4095 (12-bit DAC)
  return (uint16_t)(input * DAC_RESOLUTION);
}