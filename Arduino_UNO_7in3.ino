/***************************************************************************************
 * ESP32-C6 + 7.3inch E-Paper Display - Network Receiver
 * 
 * This version receives pre-converted pixel data from a Pi Zero server.
 * The Pi handles image conversion, this just displays.
 * 
 * Features:
 *   - WiFi connection with AP mode fallback for configuration
 *   - Receives display commands from Pi Zero
 *   - Minimal memory usage - streams data directly to display
 *   - Only updates screen on boot if WiFi fails or IP changes
 *   - NeoPixel LED control via web interface
 * 
 * Hardware Connection: SparkFun ESP32-C6 Thing Plus -> SeenGreat 7.3" E-Paper
 *   (Using Waveshare cable positions plugged into SeenGreat header)
 *      GPIO16 --> CS
 *      GPIO10 --> CLK (SCK)
 *      GPIO2  --> MOSI (DIN)
 *      GPIO1  --> DC
 *      GPIO3  --> BUSY
 *      GPIO4  --> RST
 *      GND    --> GND
 *      3.3V   --> VCC
 * 
 *      GPIO23 --> WS2812 NeoPixel LED (on-board)
 ***************************************************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Adafruit_NeoPixel.h>
#include <FS.h>
#include <SD.h>
#include <ctype.h>

// ============== Pin Definitions ==============
// E-Paper Display (SeenGreat 7.3" wired from Waveshare cable positions)
#define BUSY_Pin  3
#define RES_Pin   4
#define DC_Pin    1
#define CS_Pin    16
#define SCK_Pin   10
#define SDI_Pin   2   // MOSI/DIN

// SparkFun ESP32-C6 WS2812 NeoPixel LED on GPIO23
#define NEOPIXEL_PIN 23
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SD card (set CS pin for your board, or -1 to use default)
const int SD_CS_PIN = -1;  // TODO: set correct SD card CS pin if needed

// Battery monitoring (set pin/divider for your hardware)
const int BAT_ADC_PIN = -1;         // TODO: set ADC pin connected to battery divider
const float ADC_REF_V = 3.3f;
const int ADC_MAX = 4095;
const float BAT_DIVIDER_RATIO = 2.0f;  // Example: divider halves the voltage

// ============== Display Definitions ==============
#define EPD_WIDTH   800
#define EPD_HEIGHT  480
#define EPD_BUFFER_SIZE (EPD_WIDTH * EPD_HEIGHT / 2)  // 192000 bytes

// Color definitions (4-bit per pixel, packed as 2 pixels per byte)
#define Black   0x00
#define White   0x11
#define Green   0x66
#define Blue    0x55
#define Red     0x33
#define Yellow  0x22
#define Clean   0x77

// Single pixel color values
#define EPD_7IN3F_BLACK   0x0
#define EPD_7IN3F_WHITE   0x1
#define EPD_7IN3F_YELLOW  0x2
#define EPD_7IN3F_RED     0x3
#define EPD_7IN3F_BLUE    0x5
#define EPD_7IN3F_GREEN   0x6
#define EPD_7IN3F_CLEAN   0x7

// Commands
#define PSR         0x00
#define PWRR        0x01
#define POF         0x02
#define POFS        0x03
#define PON         0x04
#define BTST1       0x05
#define BTST2       0x06
#define DSLP        0x07
#define BTST3       0x08
#define DTM         0x10
#define DRF         0x12
#define PLL         0x30
#define CDI         0x50
#define TCON        0x60
#define TRES        0x61
#define REV         0x70
#define VDCS        0x82
#define T_VDCS      0x84
#define PWS         0xE3

// ============== WiFi Settings ==============
#define AP_SSID "EPaper-Setup"
#define AP_PASSWORD "epaper123"
#define WIFI_CONNECT_TIMEOUT 15000
#define DNS_PORT 53

// ============== Global Objects ==============
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;
File currentImageFile;

bool isAPMode = false;
String currentSSID = "";
String currentIP = "";
volatile bool displayBusy = false;
bool ledEnabled = false;  // LED state (false = off)
bool sdMounted = false;
String currentImageSaveName = "";

// ============== Function Declarations ==============
void SPI_Write(unsigned char value);
void testSPIPins(void);
void handlePinControl(void);
void handlePinTestPage(void);
void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char command);
void Epaper_READBUSY(void);
void EPD_Init(void);
void EPD_Clear(unsigned char color);
void EPD_DeepSleep(void);
void EPD_Display_6colors(void);
void displayTextScreen(const char* line1, const char* line2, const char* line3, const char* line4);
void displayAPInfo(void);
void displayConnectedInfo(void);
void setupWiFi(void);
void setupWebServer(void);
float readBatteryVoltage();
int batteryPercentFromVoltage(float v);
String sanitizeFileName(const String &name);
void handleUiPage(void);
void handleListImages(void);
void handleShowImage(void);

// ============== SPI Functions ==============
void SPI_Write(unsigned char value) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(SCK_Pin, LOW);
        if (value & 0x80)
            digitalWrite(SDI_Pin, HIGH);
        else
            digitalWrite(SDI_Pin, LOW);
        value = (value << 1);
        delayMicroseconds(2);
        digitalWrite(SCK_Pin, HIGH);
        delayMicroseconds(2);
    }
}

void Epaper_Write_Command(unsigned char command) {
    digitalWrite(CS_Pin, LOW);
    digitalWrite(DC_Pin, LOW);  // Command mode
    SPI_Write(command);
    digitalWrite(CS_Pin, HIGH);
}

void Epaper_Write_Data(unsigned char data) {
    digitalWrite(CS_Pin, LOW);
    digitalWrite(DC_Pin, HIGH);  // Data mode
    SPI_Write(data);
    digitalWrite(CS_Pin, HIGH);
}

// Test function to verify SPI pins are working
void testSPIPins(void) {
    Serial.println("\n=== SPI Pin Test ===");
    Serial.printf("Testing GPIO pins: SCK=%d, SDI=%d, CS=%d, DC=%d, RST=%d, BUSY=%d\n", 
                  SCK_Pin, SDI_Pin, CS_Pin, DC_Pin, RES_Pin, BUSY_Pin);
    
    // Test each pin can be toggled
    Serial.print("Toggling RST... ");
    digitalWrite(RES_Pin, LOW);
    delay(10);
    digitalWrite(RES_Pin, HIGH);
    Serial.println("OK");
    
    Serial.print("Toggling CS... ");
    digitalWrite(CS_Pin, LOW);
    delay(10);
    digitalWrite(CS_Pin, HIGH);
    Serial.println("OK");
    
    Serial.print("Toggling DC... ");
    digitalWrite(DC_Pin, LOW);
    delay(10);
    digitalWrite(DC_Pin, HIGH);
    Serial.println("OK");
    
    Serial.print("Toggling SCK... ");
    for(int i = 0; i < 8; i++) {
        digitalWrite(SCK_Pin, LOW);
        delayMicroseconds(100);
        digitalWrite(SCK_Pin, HIGH);
        delayMicroseconds(100);
    }
    Serial.println("OK");
    
    Serial.print("Toggling SDI (MOSI)... ");
    digitalWrite(SDI_Pin, HIGH);
    delay(10);
    digitalWrite(SDI_Pin, LOW);
    delay(10);
    int sdiRead = digitalRead(SDI_Pin);
    Serial.printf("wrote LOW, read back: %d\n", sdiRead);
    
    Serial.println("=== End SPI Pin Test ===\n");
}

// Manual pin test - holds each pin HIGH for 3 seconds for multimeter verification
// Now replaced by web interface control

void handlePinControl(void) {
    String pin = server.arg("pin");
    String state = server.arg("state");
    
    int pinNum = -1;
    String pinName = "";
    
    if (pin == "cs") { pinNum = CS_Pin; pinName = "CS"; }
    else if (pin == "sck") { pinNum = SCK_Pin; pinName = "SCK"; }
    else if (pin == "mosi") { pinNum = SDI_Pin; pinName = "MOSI"; }
    else if (pin == "dc") { pinNum = DC_Pin; pinName = "DC"; }
    else if (pin == "rst") { pinNum = RES_Pin; pinName = "RST"; }
    
    if (pinNum >= 0) {
        if (state == "high") {
            digitalWrite(pinNum, HIGH);
            Serial.printf("Pin %s (GPIO%d) set HIGH\n", pinName.c_str(), pinNum);
            server.send(200, "application/json", "{\"pin\":\"" + pinName + "\",\"gpio\":" + String(pinNum) + ",\"state\":\"HIGH\"}");
        } else {
            digitalWrite(pinNum, LOW);
            Serial.printf("Pin %s (GPIO%d) set LOW\n", pinName.c_str(), pinNum);
            server.send(200, "application/json", "{\"pin\":\"" + pinName + "\",\"gpio\":" + String(pinNum) + ",\"state\":\"LOW\"}");
        }
    } else if (pin == "busy") {
        int val = digitalRead(BUSY_Pin);
        Serial.printf("Pin BUSY (GPIO%d) reads: %d\n", BUSY_Pin, val);
        server.send(200, "application/json", "{\"pin\":\"BUSY\",\"gpio\":" + String(BUSY_Pin) + ",\"state\":\"" + String(val ? "HIGH" : "LOW") + "\"}");
    } else {
        server.send(400, "application/json", "{\"error\":\"Invalid pin\"}");
    }
}

const char PIN_TEST_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Pin Test</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
        .container { max-width: 600px; margin: 0 auto; padding: 20px; background: #16213e; border-radius: 10px; }
        h1 { color: #e94560; text-align: center; }
        .pin-row { 
            display: flex; align-items: center; justify-content: space-between;
            padding: 15px; margin: 10px 0; background: #0f3460; border-radius: 8px;
        }
        .pin-info { flex: 1; }
        .pin-name { font-weight: bold; font-size: 18px; color: #e94560; }
        .pin-gpio { color: #888; font-size: 14px; }
        .pin-status { 
            width: 80px; text-align: center; padding: 5px; border-radius: 4px; 
            font-weight: bold; margin: 0 15px;
        }
        .status-high { background: #27ae60; color: white; }
        .status-low { background: #555; color: #aaa; }
        .btn-group { display: flex; gap: 10px; }
        .btn { 
            padding: 10px 20px; border: none; border-radius: 5px; 
            cursor: pointer; font-size: 14px; font-weight: bold;
        }
        .btn-high { background: #27ae60; color: white; }
        .btn-high:hover { background: #2ecc71; }
        .btn-low { background: #c0392b; color: white; }
        .btn-low:hover { background: #e74c3c; }
        .btn-read { background: #3498db; color: white; }
        .btn-read:hover { background: #5dade2; }
        .busy-row { background: #1a3a5c; }
        .note { 
            margin-top: 20px; padding: 15px; background: #0a1628; 
            border-radius: 5px; font-size: 14px; color: #888;
        }
        .back-link { display: block; text-align: center; margin-top: 20px; color: #e94560; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔧 Pin Test</h1>
        <p style="text-align:center; color:#888;">Use multimeter to verify 3.3V (HIGH) or 0V (LOW)</p>
        
        <div class="pin-row">
            <div class="pin-info">
                <div class="pin-name">CS</div>
                <div class="pin-gpio">GPIO %CS_PIN%</div>
            </div>
            <div class="pin-status status-low" id="cs-status">LOW</div>
            <div class="btn-group">
                <button class="btn btn-high" onclick="setPin('cs','high')">HIGH</button>
                <button class="btn btn-low" onclick="setPin('cs','low')">LOW</button>
            </div>
        </div>
        
        <div class="pin-row">
            <div class="pin-info">
                <div class="pin-name">SCK (CLK)</div>
                <div class="pin-gpio">GPIO %SCK_PIN%</div>
            </div>
            <div class="pin-status status-low" id="sck-status">LOW</div>
            <div class="btn-group">
                <button class="btn btn-high" onclick="setPin('sck','high')">HIGH</button>
                <button class="btn btn-low" onclick="setPin('sck','low')">LOW</button>
            </div>
        </div>
        
        <div class="pin-row">
            <div class="pin-info">
                <div class="pin-name">MOSI (DIN)</div>
                <div class="pin-gpio">GPIO %MOSI_PIN%</div>
            </div>
            <div class="pin-status status-low" id="mosi-status">LOW</div>
            <div class="btn-group">
                <button class="btn btn-high" onclick="setPin('mosi','high')">HIGH</button>
                <button class="btn btn-low" onclick="setPin('mosi','low')">LOW</button>
            </div>
        </div>
        
        <div class="pin-row">
            <div class="pin-info">
                <div class="pin-name">DC</div>
                <div class="pin-gpio">GPIO %DC_PIN%</div>
            </div>
            <div class="pin-status status-low" id="dc-status">LOW</div>
            <div class="btn-group">
                <button class="btn btn-high" onclick="setPin('dc','high')">HIGH</button>
                <button class="btn btn-low" onclick="setPin('dc','low')">LOW</button>
            </div>
        </div>
        
        <div class="pin-row">
            <div class="pin-info">
                <div class="pin-name">RST</div>
                <div class="pin-gpio">GPIO %RST_PIN%</div>
            </div>
            <div class="pin-status status-high" id="rst-status">HIGH</div>
            <div class="btn-group">
                <button class="btn btn-high" onclick="setPin('rst','high')">HIGH</button>
                <button class="btn btn-low" onclick="setPin('rst','low')">LOW</button>
            </div>
        </div>
        
        <div class="pin-row busy-row">
            <div class="pin-info">
                <div class="pin-name">BUSY (Input)</div>
                <div class="pin-gpio">GPIO %BUSY_PIN%</div>
            </div>
            <div class="pin-status" id="busy-status">?</div>
            <div class="btn-group">
                <button class="btn btn-read" onclick="readBusy()">READ</button>
            </div>
        </div>
        
        <div class="note">
            <strong>How to test:</strong><br>
            1. Put multimeter on a pin at the display connector<br>
            2. Click HIGH - should read ~3.3V<br>
            3. Click LOW - should read ~0V<br>
            4. If a pin doesn't change, there's a wiring issue<br><br>
            <strong>BUSY</strong> is an input - click READ to see its state
        </div>
        
        <a href="/" class="back-link">← Back to main page</a>
    </div>
    
    <script>
        function setPin(pin, state) {
            fetch('/pin?pin=' + pin + '&state=' + state)
                .then(r => r.json())
                .then(data => {
                    const el = document.getElementById(pin + '-status');
                    el.textContent = data.state;
                    el.className = 'pin-status ' + (data.state === 'HIGH' ? 'status-high' : 'status-low');
                })
                .catch(e => alert('Error: ' + e));
        }
        
        function readBusy() {
            fetch('/pin?pin=busy')
                .then(r => r.json())
                .then(data => {
                    const el = document.getElementById('busy-status');
                    el.textContent = data.state;
                    el.className = 'pin-status ' + (data.state === 'HIGH' ? 'status-high' : 'status-low');
                })
                .catch(e => alert('Error: ' + e));
        }
        
        // Read BUSY on load
        readBusy();
    </script>
</body>
</html>
)rawliteral";

void handlePinTestPage(void) {
    String html = PIN_TEST_HTML;
    html.replace("%CS_PIN%", String(CS_Pin));
    html.replace("%SCK_PIN%", String(SCK_Pin));
    html.replace("%MOSI_PIN%", String(SDI_Pin));
    html.replace("%DC_PIN%", String(DC_Pin));
    html.replace("%RST_PIN%", String(RES_Pin));
    html.replace("%BUSY_PIN%", String(BUSY_Pin));
    server.send(200, "text/html", html);
}

void Epaper_READBUSY(void) {
    Serial.printf("Waiting for BUSY (GPIO%d)... current state: %d\n", BUSY_Pin, digitalRead(BUSY_Pin));
    unsigned long start = millis();
    while (!digitalRead(BUSY_Pin)) {
        delay(100);
        if (millis() - start > 30000) {  // 30 second timeout
            Serial.println("BUSY TIMEOUT! Display may not be connected properly.");
            return;
        }
        if ((millis() - start) % 5000 == 0) {
            Serial.printf("  Still waiting... BUSY=%d (%lu ms)\n", digitalRead(BUSY_Pin), millis() - start);
        }
    }
    Serial.printf("Display ready (%lu ms)\n", millis() - start);
}

// ============== EPD Functions ==============
void EPD_Init(void) {
    Serial.println("EPD Init starting...");
    
    // More aggressive reset sequence
    Serial.print("  Resetting display (RST pin)... ");
    digitalWrite(RES_Pin, HIGH);
    delay(50);
    digitalWrite(RES_Pin, LOW);
    delay(50);   // Hold reset low longer
    digitalWrite(RES_Pin, HIGH);
    delay(200);  // Wait longer after reset
    Serial.println("done");
    
    Serial.println("  Sending init commands...");

    Epaper_Write_Command(0xAA);
    Epaper_Write_Data(0x49);
    Epaper_Write_Data(0x55);
    Epaper_Write_Data(0x20);
    Epaper_Write_Data(0x08);
    Epaper_Write_Data(0x09);
    Epaper_Write_Data(0x18);

    Epaper_Write_Command(PWRR);
    Epaper_Write_Data(0x3F);

    Epaper_Write_Command(PSR);
    Epaper_Write_Data(0x5F);
    Epaper_Write_Data(0x69);

    Epaper_Write_Command(POFS);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x54);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x44);

    Epaper_Write_Command(BTST1);
    Epaper_Write_Data(0x40);
    Epaper_Write_Data(0x1F);
    Epaper_Write_Data(0x1F);
    Epaper_Write_Data(0x2C);

    Epaper_Write_Command(BTST2);
    Epaper_Write_Data(0x6F);
    Epaper_Write_Data(0x1F);
    Epaper_Write_Data(0x17);
    Epaper_Write_Data(0x49);

    Epaper_Write_Command(BTST3);
    Epaper_Write_Data(0x6F);
    Epaper_Write_Data(0x1F);
    Epaper_Write_Data(0x1F);
    Epaper_Write_Data(0x22);

    Epaper_Write_Command(PLL);
    Epaper_Write_Data(0x08);

    Epaper_Write_Command(CDI);
    Epaper_Write_Data(0x3F);

    Epaper_Write_Command(TCON);
    Epaper_Write_Data(0x02);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(TRES);
    Epaper_Write_Data(0x03);
    Epaper_Write_Data(0x20);
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0xE0);

    Epaper_Write_Command(T_VDCS);
    Epaper_Write_Data(0x01);

    Epaper_Write_Command(PWS);
    Epaper_Write_Data(0x2F);

    Serial.println("  Sending power on command (0x04)...");
    Epaper_Write_Command(0x04);
    Serial.println("EPD Init - waiting for busy...");
    Epaper_READBUSY();
}

void EPD_Clear(unsigned char color) {
    Epaper_Write_Command(DTM);
    for (int i = 0; i < EPD_WIDTH / 2; i++) {
        for (int j = 0; j < EPD_HEIGHT; j++) {
            Epaper_Write_Data((color << 4) | color);
        }
    }
    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
}

void EPD_Display_6colors(void) {
    unsigned long pixelsPerColor = 32000;
    
    Epaper_Write_Command(DTM);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(Black);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(White);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(Red);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(Green);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(Blue);
    for (unsigned long i = 0; i < pixelsPerColor; i++) Epaper_Write_Data(Yellow);
    
    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
}

void EPD_DeepSleep(void) {
    Epaper_Write_Command(0x02);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
}

// ============== Simple Text Display ==============
const unsigned char font5x7[][5] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x14, 0x14, 0x14, 0x14, 0x14}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
};

int getCharIndex(char c) {
    if (c == ' ') return 0;
    if (c >= 'A' && c <= 'Z') return c - 'A' + 1;
    if (c >= 'a' && c <= 'z') return c - 'a' + 1;
    if (c >= '0' && c <= '9') return c - '0' + 27;
    if (c == ':') return 37;
    if (c == '-') return 38;
    if (c == '.') return 39;
    if (c == '/') return 40;
    return 0;
}

void displayTextScreen(const char* line1, const char* line2, const char* line3, const char* line4) {
    EPD_Init();
    Epaper_Write_Command(DTM);
    
    int lineY[] = {160, 200, 240, 280};
    const char* lines[] = {line1, line2, line3, line4};
    
    for (int y = 0; y < EPD_HEIGHT; y++) {
        for (int x = 0; x < EPD_WIDTH / 2; x++) {
            unsigned char pixelPair = 0x11;
            
            for (int lineNum = 0; lineNum < 4; lineNum++) {
                if (lines[lineNum] == NULL) continue;
                
                int lineStartY = lineY[lineNum];
                int charHeight = 14;
                
                if (y >= lineStartY && y < lineStartY + charHeight) {
                    int textLen = strlen(lines[lineNum]);
                    int textWidthPixels = textLen * 12;
                    int startX = (EPD_WIDTH - textWidthPixels) / 2;
                    
                    int px1 = x * 2;
                    int px2 = x * 2 + 1;
                    
                    unsigned char c1 = EPD_7IN3F_WHITE;
                    unsigned char c2 = EPD_7IN3F_WHITE;
                    
                    if (px1 >= startX && px1 < startX + textWidthPixels) {
                        int charIdx = (px1 - startX) / 12;
                        int charCol = ((px1 - startX) % 12) / 2;
                        int charRow = (y - lineStartY) / 2;
                        
                        if (charIdx < textLen && charCol < 5 && charRow < 7) {
                            int fontIdx = getCharIndex(lines[lineNum][charIdx]);
                            unsigned char fontCol = pgm_read_byte(&font5x7[fontIdx][charCol]);
                            if (fontCol & (1 << charRow)) {
                                c1 = EPD_7IN3F_BLACK;
                            }
                        }
                    }
                    
                    if (px2 >= startX && px2 < startX + textWidthPixels) {
                        int charIdx = (px2 - startX) / 12;
                        int charCol = ((px2 - startX) % 12) / 2;
                        int charRow = (y - lineStartY) / 2;
                        
                        if (charIdx < textLen && charCol < 5 && charRow < 7) {
                            int fontIdx = getCharIndex(lines[lineNum][charIdx]);
                            unsigned char fontCol = pgm_read_byte(&font5x7[fontIdx][charCol]);
                            if (fontCol & (1 << charRow)) {
                                c2 = EPD_7IN3F_BLACK;
                            }
                        }
                    }
                    
                    pixelPair = (c1 << 4) | c2;
                    break;
                }
            }
            
            Epaper_Write_Data(pixelPair);
        }
    }
    
    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
    EPD_DeepSleep();
}

void displayAPInfo(void) {
    Serial.println("Displaying AP info on screen...");
    displayTextScreen(
        "SSID: " AP_SSID,
        "PASS: " AP_PASSWORD,
        "OPEN: 192.168.4.1",
        "TO CONFIGURE WIFI"
    );
}

void displayConnectedInfo(void) {
    Serial.println("Displaying connected info on screen...");
    char ipLine[32];
    snprintf(ipLine, sizeof(ipLine), "IP: %s", currentIP.c_str());
    
    displayTextScreen(
        "CONNECTED TO WIFI",
        currentSSID.c_str(),
        ipLine,
        "READY FOR IMAGES"
    );
}

// ============== Battery Helpers ==============
float readBatteryVoltage() {
    if (BAT_ADC_PIN < 0) {
        return 0.0f;
    }

    int raw = analogRead(BAT_ADC_PIN);
    float vIn = (raw * ADC_REF_V) / ADC_MAX;
    float vBat = vIn * BAT_DIVIDER_RATIO;
    return vBat;
}

int batteryPercentFromVoltage(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 4.20f) return 100;
    if (v <= 3.30f) return 5;

    // Piecewise linear approximation between key voltage points
    struct Point { float v; int pct; } points[] = {
        {4.20f, 100},
        {4.10f, 90},
        {3.90f, 70},
        {3.70f, 50},
        {3.50f, 25},
        {3.30f, 10},
    };

    for (size_t i = 0; i + 1 < sizeof(points) / sizeof(points[0]); i++) {
        if (v <= points[i].v && v >= points[i + 1].v) {
            float spanV = points[i].v - points[i + 1].v;
            int spanPct = points[i].pct - points[i + 1].pct;
            float ratio = (v - points[i + 1].v) / spanV;
            int pct = points[i + 1].pct + (int)(spanPct * ratio);
            return constrain(pct, 0, 100);
        }
    }

    return constrain((int)((v - 3.30f) * 100), 0, 100);
}

String sanitizeFileName(const String &name) {
    String clean = "";
    for (size_t i = 0; i < name.length(); i++) {
        char c = name.charAt(i);
        if (isalnum(c) || c == '_' || c == '-') {
            clean += c;
        }
    }

    if (clean.length() == 0) return clean;

    if (!clean.endsWith(".bin")) {
        clean += ".bin";
    }

    return clean;
}

// ============== WiFi Setup ==============
void setupWiFi(void) {
    preferences.begin("epaper", false);
    String savedSSID = preferences.getString("ssid", "");
    String savedPass = preferences.getString("password", "");
    String lastIP = preferences.getString("lastip", "");
    
    if (savedSSID.length() > 0) {
        Serial.printf("Attempting to connect to saved WiFi: %s\n", savedSSID.c_str());
        
        WiFi.mode(WIFI_STA);
        WiFi.begin(savedSSID.c_str(), savedPass.c_str());
        
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to WiFi!");
            currentSSID = savedSSID;
            currentIP = WiFi.localIP().toString();
            Serial.printf("IP Address: %s\n", currentIP.c_str());
            isAPMode = false;
            
            // Only update display if IP changed
            if (currentIP != lastIP) {
                Serial.println("IP changed - updating display");
                preferences.putString("lastip", currentIP);
                displayConnectedInfo();
            } else {
                Serial.println("IP unchanged - skipping display update");
            }
            
            preferences.end();
            return;
        }
        
        Serial.println("\nFailed to connect to saved WiFi.");
    }
    
    // Start AP Mode
    Serial.println("Starting Access Point mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    currentSSID = AP_SSID;
    currentIP = WiFi.softAPIP().toString();
    Serial.printf("AP Started. SSID: %s, IP: %s\n", AP_SSID, currentIP.c_str());
    isAPMode = true;
    
    // Always show AP info when entering AP mode
    preferences.putString("lastip", "");  // Clear last IP so we update when connected
    preferences.end();
    
    displayAPInfo();
}

// ============== HTML Pages ==============
const char AP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>E-Paper WiFi Setup</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
        .container { max-width: 400px; margin: 0 auto; padding: 20px; background: #16213e; border-radius: 10px; }
        h1 { color: #e94560; text-align: center; }
        input[type="text"], input[type="password"] { 
            width: 100%; padding: 12px; margin: 8px 0; box-sizing: border-box; 
            border: 2px solid #0f3460; border-radius: 5px; background: #1a1a2e; color: #eee;
        }
        input[type="submit"] { 
            width: 100%; padding: 14px; background: #e94560; color: white; 
            border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin-top: 10px;
        }
        input[type="submit"]:hover { background: #ff6b6b; }
        label { display: block; margin-top: 15px; color: #a2d2ff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>WiFi Setup</h1>
        <form action="/save" method="POST">
            <label for="ssid">WiFi Network Name (SSID):</label>
            <input type="text" id="ssid" name="ssid" required>
            <label for="password">WiFi Password:</label>
            <input type="password" id="password" name="password">
            <input type="submit" value="Connect">
        </form>
    </div>
</body>
</html>
)rawliteral";

const char STATUS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>E-Paper Display Status</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
        .container { max-width: 500px; margin: 0 auto; padding: 20px; background: #16213e; border-radius: 10px; }
        h1 { color: #e94560; text-align: center; }
        .status { text-align: center; color: #27ae60; margin: 20px 0; }
        .info { background: #0f3460; padding: 15px; border-radius: 5px; margin: 10px 0; }
        .btn { 
            display: block; width: 100%; padding: 15px; margin: 10px 0; 
            background: #0f3460; color: white; border: none; border-radius: 5px; 
            cursor: pointer; font-size: 16px; text-align: center; text-decoration: none;
        }
        .btn:hover { background: #e94560; }
        .btn-row { display: flex; gap: 10px; }
        .btn-row .btn { flex: 1; }
        .led-status { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .led-on { background: #f1c40f; box-shadow: 0 0 10px #f1c40f; }
        .led-off { background: #555; }
    </style>
</head>
<body>
    <div class="container">
        <h1>E-Paper Display</h1>
        <p class="status">Connected: %SSID%</p>
        <div class="info">
            <p><strong>IP Address:</strong> %IP%</p>
            <p><strong>Status:</strong> %STATUS%</p>
            <p><strong>Display:</strong> 800x480, 6-color</p>
            <p><strong>Battery:</strong> %BAT_PCT%% (%BAT_VOLT% V)</p>
            <p><strong>LED:</strong> <span class="led-status %LEDCLASS%"></span>%LEDSTATUS%</p>
        </div>
        <div class="info">
            <p><strong>How to use</strong></p>
            <ol>
                <li>Connect your phone or laptop to the same WiFi network as this device.</li>
                <li>Open the upload page below to send a photo directly from your camera roll.</li>
                <li>Tap "Display" on any saved item to show it on the e-paper screen.</li>
            </ol>
        </div>
        <a href="/ui" class="btn">Open Upload Page</a>
        <a href="/colortest" class="btn">6-Color Test</a>
        <a href="/clear" class="btn">Clear (White)</a>
        <div class="btn-row">
            <a href="/led/on" class="btn">LED On</a>
            <a href="/led/off" class="btn">LED Off</a>
        </div>
        <a href="/resetwifi" class="btn" onclick="return confirm('Reset WiFi?')">Reset WiFi</a>
    </div>
</body>
</html>
)rawliteral";

const char UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>E-Paper Image Upload</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 0; background: #0f172a; color: #e5e7eb; }
        .container { max-width: 900px; margin: 0 auto; padding: 20px; }
        h1 { text-align: center; color: #f97316; }
        .card { background: #111827; padding: 20px; border-radius: 10px; margin-bottom: 16px; box-shadow: 0 4px 10px rgba(0,0,0,0.25); }
        label { display: block; margin: 10px 0 6px; font-weight: bold; }
        input[type="file"], input[type="text"] { width: 100%; padding: 10px; border-radius: 6px; border: 1px solid #1f2937; background: #0b1220; color: #e5e7eb; }
        button { padding: 12px 18px; background: #2563eb; color: white; border: none; border-radius: 8px; cursor: pointer; font-size: 16px; }
        button:disabled { opacity: 0.6; cursor: not-allowed; }
        #status { background: #0b1220; padding: 12px; border-radius: 6px; min-height: 44px; white-space: pre-wrap; }
        .images { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; }
        .image-item { background: #0b1220; padding: 12px; border-radius: 6px; display: flex; justify-content: space-between; align-items: center; }
        .muted { color: #9ca3af; font-size: 14px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>E-Paper Image Upload</h1>
        <div class="card">
            <p class="muted" id="status">Ready.</p>
            <p class="muted" id="info">Loading status...</p>
        </div>
        <div class="card">
            <h3>Send photos from your phone</h3>
            <p class="muted">1) Stay on the same WiFi network as the e-paper display.<br>
            2) Tap "Choose image" to pick a photo or snap a new one with your camera.<br>
            3) Press "Send to Display" to convert and transmit. Optionally give it a name to keep it on the device.</p>
        </div>
        <div class="card">
            <label for="fileInput">Choose image</label>
            <input type="file" id="fileInput" accept="image/*">
            <label for="imageName">Optional name to save</label>
            <input type="text" id="imageName" placeholder="Optional name to save">
            <div style="margin-top: 12px; display: flex; gap: 10px; flex-wrap: wrap;">
                <button id="sendBtn">Send to Display</button>
                <button id="refreshBtn" type="button">Refresh Saved List</button>
            </div>
        </div>
        <div class="card">
            <h3>Saved Images</h3>
            <div id="savedImages" class="images"></div>
        </div>
    </div>

    <script>
    const statusEl = document.getElementById('status');
    const infoEl = document.getElementById('info');
    const savedContainer = document.getElementById('savedImages');
    const sendBtn = document.getElementById('sendBtn');
    const refreshBtn = document.getElementById('refreshBtn');

    function logStatus(msg) { statusEl.textContent = msg; }
    function logInfo(msg) { infoEl.textContent = msg; }

    async function fetchStatus() {
        try {
            const res = await fetch('/status');
            const data = await res.json();
            logInfo(`SSID: ${data.ssid} | IP: ${data.ip} | Battery: ${data.battery_percent}% (${data.battery_voltage} V)`);
        } catch (e) {
            logInfo('Unable to load status');
        }
    }

    async function fetchImages() {
        try {
            const res = await fetch('/images');
            const names = await res.json();
            savedContainer.innerHTML = '';
            if (!names || names.length === 0) {
                savedContainer.innerHTML = '<p class="muted">No saved images.</p>';
                return;
            }
            names.forEach(name => {
                const div = document.createElement('div');
                div.className = 'image-item';
                div.innerHTML = `<span>${name}</span>`;
                const btn = document.createElement('button');
                btn.textContent = 'Display';
                btn.onclick = async () => {
                    logStatus(`Displaying ${name}...`);
                    const res = await fetch(`/images/show?name=${encodeURIComponent(name)}`, { method: 'POST' });
                    const data = await res.json();
                    logStatus(data.message || 'Done');
                };
                div.appendChild(btn);
                savedContainer.appendChild(div);
            });
        } catch (e) {
            savedContainer.innerHTML = '<p class="muted">Unable to load images.</p>';
        }
    }

    const palette = [
        { code: 0x0, r: 0, g: 0, b: 0 },       // Black
        { code: 0x1, r: 255, g: 255, b: 255 }, // White
        { code: 0x2, r: 255, g: 255, b: 0 },   // Yellow
        { code: 0x3, r: 255, g: 0, b: 0 },     // Red
        { code: 0x5, r: 0, g: 0, b: 255 },     // Blue
        { code: 0x6, r: 0, g: 128, b: 0 },     // Green
        { code: 0x7, r: 255, g: 255, b: 255 }, // Clean treated as white
    ];

    function nearestPaletteColor(r, g, b) {
        let best = palette[0];
        let bestDist = Number.MAX_VALUE;
        for (const p of palette) {
            const dr = r - p.r, dg = g - p.g, db = b - p.b;
            const dist = dr * dr + dg * dg + db * db;
            if (dist < bestDist) { bestDist = dist; best = p; }
        }
        return best;
    }

    function nearestColor(r, g, b) {
        return nearestPaletteColor(r, g, b).code;
    }

    function clamp(val) {
        return Math.max(0, Math.min(255, val));
    }

    function applyFloydSteinbergDithering(imgData) {
        const { data, width, height } = imgData;
        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const idx = (y * width + x) * 4;
                const oldR = data[idx];
                const oldG = data[idx + 1];
                const oldB = data[idx + 2];

                const nearest = nearestPaletteColor(oldR, oldG, oldB);
                data[idx] = nearest.r;
                data[idx + 1] = nearest.g;
                data[idx + 2] = nearest.b;

                const errR = oldR - nearest.r;
                const errG = oldG - nearest.g;
                const errB = oldB - nearest.b;

                function scatter(nx, ny, factor) {
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) return;
                    const nIdx = (ny * width + nx) * 4;
                    data[nIdx] = clamp(data[nIdx] + errR * factor);
                    data[nIdx + 1] = clamp(data[nIdx + 1] + errG * factor);
                    data[nIdx + 2] = clamp(data[nIdx + 2] + errB * factor);
                }

                scatter(x + 1, y, 7 / 16);
                scatter(x - 1, y + 1, 3 / 16);
                scatter(x, y + 1, 5 / 16);
                scatter(x + 1, y + 1, 1 / 16);
            }
        }
    }

    function packImageData(imgData) {
        const packed = new Uint8Array(800 * 480 / 2);
        let p = 0;
        for (let i = 0; i < imgData.data.length; i += 8) {
            const r1 = imgData.data[i], g1 = imgData.data[i+1], b1 = imgData.data[i+2];
            const r2 = imgData.data[i+4], g2 = imgData.data[i+5], b2 = imgData.data[i+6];
            const c1 = nearestColor(r1, g1, b1);
            const c2 = nearestColor(r2, g2, b2);
            packed[p++] = (c1 << 4) | c2;
        }
        return packed;
    }

    async function sendToDisplay(bytes, saveName) {
        const query = saveName ? `?save=${encodeURIComponent(saveName)}` : '';
        logStatus('Starting upload...');
        let res = await fetch(`/display/start${query}`, { method: 'POST' });
        if (!res.ok) { logStatus('Failed to start transfer'); return; }

        const chunkSize = 4096;
        for (let offset = 0; offset < bytes.length; offset += chunkSize) {
            const chunk = bytes.slice(offset, offset + chunkSize);
            let hex = '';
            for (const b of chunk) hex += b.toString(16).padStart(2, '0');
            res = await fetch('/display/chunk', {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain' },
                body: hex
            });
            if (!res.ok) { logStatus('Chunk upload failed'); return; }
            logStatus(`Uploading... ${(offset + chunk.length)} / ${bytes.length}`);
        }

        res = await fetch('/display/end', { method: 'POST' });
        const data = await res.json();
        logStatus(data.message || 'Done');
    }

    function scaleAndConvert(image) {
        const canvas = document.createElement('canvas');
        canvas.width = 800; canvas.height = 480;
        const ctx = canvas.getContext('2d');
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        const portrait = image.height > image.width;

        if (portrait) {
            ctx.save();
            ctx.translate(canvas.width / 2, canvas.height / 2);
            ctx.rotate(-Math.PI / 2);
            const availableWidth = canvas.height;
            const availableHeight = canvas.width;
            const scale = Math.min(availableWidth / image.width, availableHeight / image.height);
            const drawW = image.width * scale;
            const drawH = image.height * scale;
            ctx.drawImage(image, -drawW / 2, -drawH / 2, drawW, drawH);
            ctx.restore();
        } else {
            const scale = Math.min(canvas.width / image.width, canvas.height / image.height);
            const drawW = image.width * scale;
            const drawH = image.height * scale;
            const dx = (canvas.width - drawW) / 2;
            const dy = (canvas.height - drawH) / 2;
            ctx.drawImage(image, dx, dy, drawW, drawH);
        }

        const imgData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        applyFloydSteinbergDithering(imgData);
        return packImageData(imgData);
    }

    async function handleSend() {
        const file = document.getElementById('fileInput').files[0];
        const saveName = document.getElementById('imageName').value.trim();
        if (!file) { logStatus('Select a file first.'); return; }
        sendBtn.disabled = true;
        logStatus('Loading image...');
        const reader = new FileReader();
        reader.onload = () => {
            const img = new Image();
            img.onload = async () => {
                const bytes = scaleAndConvert(img);
                await sendToDisplay(bytes, saveName);
                sendBtn.disabled = false;
                if (saveName) fetchImages();
            };
            img.onerror = () => { logStatus('Failed to load image.'); sendBtn.disabled = false; };
            img.src = reader.result;
        };
        reader.readAsDataURL(file);
    }

    sendBtn.onclick = handleSend;
    refreshBtn.onclick = fetchImages;
    fetchStatus();
    fetchImages();
    </script>
</body>
</html>
)rawliteral";

// ============== Web Server Handlers ==============
void handleRoot(void) {
    if (isAPMode) {
        server.send(200, "text/html", AP_HTML);
    } else {
        String html = STATUS_HTML;
        html.replace("%SSID%", currentSSID);
        html.replace("%IP%", currentIP);
        html.replace("%STATUS%", displayBusy ? "Busy (refreshing)" : "Ready");
        html.replace("%LEDCLASS%", ledEnabled ? "led-on" : "led-off");
        html.replace("%LEDSTATUS%", ledEnabled ? "On" : "Off");
        float vBat = readBatteryVoltage();
        int pct = batteryPercentFromVoltage(vBat);
        html.replace("%BAT_VOLT%", String(vBat, 2));
        html.replace("%BAT_PCT%", String(pct));
        server.send(200, "text/html", html);
    }
}

void handleSaveWiFi(void) {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    
    if (ssid.length() > 0) {
        preferences.begin("epaper", false);
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        preferences.end();
        
        server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#eee;font-family:Arial;text-align:center;padding:50px;'>"
                                       "<h1 style='color:#27ae60;'>WiFi Saved!</h1>"
                                       "<p>Restarting...</p></body></html>");
        delay(2000);
        ESP.restart();
    } else {
        server.send(400, "text/html", "Error: SSID required");
    }
}

void handleResetWiFi(void) {
    preferences.begin("epaper", false);
    preferences.clear();
    preferences.end();
    
    server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#eee;font-family:Arial;text-align:center;padding:50px;'>"
                                   "<h1 style='color:#e94560;'>WiFi Reset</h1>"
                                   "<p>Restarting in AP mode...</p></body></html>");
    delay(2000);
    ESP.restart();
}

void handleColorTest(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Running 6-color test\"}");
    
    displayBusy = true;
    Serial.println("Running 6-color test...");
    EPD_Init();
    EPD_Display_6colors();
    EPD_DeepSleep();
    displayBusy = false;
    Serial.println("6-color test complete.");
}

void handleClear(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Clearing display\"}");
    
    displayBusy = true;
    Serial.println("Clearing display...");
    EPD_Init();
    EPD_Clear(EPD_7IN3F_WHITE);
    EPD_DeepSleep();
    displayBusy = false;
    Serial.println("Clear complete.");
}

void handleSingleColor(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }
    
    String color = server.arg("color");
    unsigned char epdColor = EPD_7IN3F_WHITE;
    
    if (color == "black") epdColor = EPD_7IN3F_BLACK;
    else if (color == "white") epdColor = EPD_7IN3F_WHITE;
    else if (color == "red") epdColor = EPD_7IN3F_RED;
    else if (color == "green") epdColor = EPD_7IN3F_GREEN;
    else if (color == "blue") epdColor = EPD_7IN3F_BLUE;
    else if (color == "yellow") epdColor = EPD_7IN3F_YELLOW;
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Filling with " + color + "\"}");
    
    displayBusy = true;
    Serial.printf("Filling with %s...\n", color.c_str());
    EPD_Init();
    EPD_Clear(epdColor);
    EPD_DeepSleep();
    displayBusy = false;
    Serial.println("Fill complete.");
}

void handleStatus(void) {
    float vBat = readBatteryVoltage();
    int percent = batteryPercentFromVoltage(vBat);

    String json = "{";
    json += "\"busy\":" + String(displayBusy ? "true" : "false") + ",";
    json += "\"ip\":\"" + currentIP + "\",";
    json += "\"ssid\":\"" + currentSSID + "\",";
    json += "\"width\":800,";
    json += "\"height\":480,";
    json += "\"colors\":6,";
    json += "\"led\":" + String(ledEnabled ? "true" : "false") + ",";
    json += "\"battery_voltage\":" + String(vBat, 2) + ",";
    json += "\"battery_percent\":" + String(percent);
    json += "}";
    server.send(200, "application/json", json);
}

void handleUiPage(void) {
    server.send(200, "text/html", UI_HTML);
}

void handleListImages(void) {
    if (!sdMounted) {
        server.send(200, "application/json", "[]");
        return;
    }

    File root = SD.open("/");
    if (!root || !root.isDirectory()) {
        server.send(500, "application/json", "{\"error\":\"SD not ready\"}");
        return;
    }

    String json = "[";
    bool first = true;
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String name = file.name();
            if (name.endsWith(".bin")) {
                if (name.startsWith("/")) name = name.substring(1);
                if (!first) json += ",";
                json += "\"" + name + "\"";
                first = false;
            }
        }
        file = root.openNextFile();
    }
    root.close();
    json += "]";

    server.send(200, "application/json", json);
}

void handleShowImage(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }

    if (!sdMounted) {
        server.send(404, "application/json", "{\"error\":\"SD not available\"}");
        return;
    }

    String nameArg = server.arg("name");
    String clean = sanitizeFileName(nameArg);
    if (clean.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"Invalid name\"}");
        return;
    }

    String path = "/" + clean;
    File f = SD.open(path, FILE_READ);
    if (!f) {
        server.send(404, "application/json", "{\"error\":\"File not found\"}");
        return;
    }

    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Displaying " + clean + "\"}");

    displayBusy = true;
    Serial.printf("Displaying saved image: %s\n", clean.c_str());
    EPD_Init();
    Epaper_Write_Command(DTM);

    const size_t bufSize = 512;
    uint8_t buffer[bufSize];
    size_t total = 0;
    while (true) {
        size_t readBytes = f.read(buffer, bufSize);
        if (readBytes == 0) break;
        for (size_t i = 0; i < readBytes && total < EPD_BUFFER_SIZE; i++) {
            Epaper_Write_Data(buffer[i]);
            total++;
        }
        if (total >= EPD_BUFFER_SIZE) break;
    }

    while (total < EPD_BUFFER_SIZE) {
        Epaper_Write_Data(White);
        total++;
    }

    f.close();

    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
    EPD_DeepSleep();

    displayBusy = false;
    Serial.println("Saved image display complete.");
}

void handleLedOn(void) {
    ledEnabled = true;
    pixel.setPixelColor(0, pixel.Color(50, 50, 50));  // Dim white
    pixel.show();
    server.send(200, "application/json", "{\"status\":\"ok\",\"led\":true}");
    Serial.println("LED turned ON");
}

void handleLedOff(void) {
    ledEnabled = false;
    pixel.setPixelColor(0, 0);  // Off
    pixel.show();
    server.send(200, "application/json", "{\"status\":\"ok\",\"led\":false}");
    Serial.println("LED turned OFF");
}

void handleLedToggle(void) {
    ledEnabled = !ledEnabled;
    if (ledEnabled) {
        pixel.setPixelColor(0, pixel.Color(50, 50, 50));
    } else {
        pixel.setPixelColor(0, 0);
    }
    pixel.show();
    server.send(200, "application/json", "{\"status\":\"ok\",\"led\":" + String(ledEnabled ? "true" : "false") + "}");
    Serial.printf("LED toggled: %s\n", ledEnabled ? "ON" : "OFF");
}

// Set LED to a specific color (r, g, b values 0-255)
void handleLedColor(void) {
    int r = server.arg("r").toInt();
    int g = server.arg("g").toInt();
    int b = server.arg("b").toInt();
    
    // Clamp values
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
    
    ledEnabled = (r > 0 || g > 0 || b > 0);
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
    
    server.send(200, "application/json", 
        "{\"status\":\"ok\",\"r\":" + String(r) + ",\"g\":" + String(g) + ",\"b\":" + String(b) + "}");
    Serial.printf("LED color set to RGB(%d, %d, %d)\n", r, g, b);
}

// Stream image data directly to display - no buffering needed!
void handleDisplayImage(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }
    
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"No data received\"}");
        return;
    }
    
    String data = server.arg("plain");
    int expectedSize = EPD_BUFFER_SIZE;  // 192000 bytes
    
    if (data.length() != expectedSize) {
        Serial.printf("Error: Expected %d bytes, got %d\n", expectedSize, data.length());
        server.send(400, "application/json", "{\"error\":\"Invalid data size. Expected 192000 bytes.\"}");
        return;
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Displaying image\"}");
    
    displayBusy = true;
    Serial.println("Receiving and displaying image...");
    
    EPD_Init();
    Epaper_Write_Command(DTM);
    
    // Stream data directly to display
    for (int i = 0; i < expectedSize; i++) {
        Epaper_Write_Data((unsigned char)data[i]);
    }
    
    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
    EPD_DeepSleep();
    
    displayBusy = false;
    Serial.println("Image display complete.");
}

// Chunked upload for larger transfers
static size_t imageDataReceived = 0;
static bool imageInProgress = false;

void handleDisplayImageStart(void) {
    if (displayBusy) {
        server.send(503, "application/json", "{\"error\":\"Display busy\"}");
        return;
    }

    displayBusy = true;
    imageInProgress = true;
    imageDataReceived = 0;

    if (currentImageFile) {
        currentImageFile.close();
    }

    currentImageSaveName = "";
    if (sdMounted && server.hasArg("save")) {
        String requested = sanitizeFileName(server.arg("save"));
        if (requested.length() > 0) {
            currentImageSaveName = requested;
            String path = "/" + requested;
            currentImageFile = SD.open(path, FILE_WRITE);
            if (!currentImageFile) {
                Serial.println("Failed to open file for writing on SD.");
            } else {
                Serial.printf("Saving incoming image to %s\n", path.c_str());
            }
        }
    }

    Serial.println("Starting chunked image transfer...");
    EPD_Init();
    Epaper_Write_Command(DTM);

    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Ready for chunks\"}");
}

// Helper function to decode hex character to value
uint8_t hexCharToValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

void handleDisplayImageChunk(void) {
    if (!imageInProgress) {
        server.send(400, "application/json", "{\"error\":\"No transfer in progress. Call /display/start first.\"}");
        return;
    }
    
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"No data\"}");
        return;
    }
    
    String hexData = server.arg("plain");
    int bytesWritten = 0;
    
    // Decode hex pairs to bytes and write directly to display
    for (size_t i = 0; i + 1 < hexData.length() && imageDataReceived < EPD_BUFFER_SIZE; i += 2) {
        uint8_t highNibble = hexCharToValue(hexData[i]);
        uint8_t lowNibble = hexCharToValue(hexData[i + 1]);
        uint8_t byte = (highNibble << 4) | lowNibble;

        Epaper_Write_Data(byte);
        if (currentImageFile) {
            currentImageFile.write(byte);
        }
        imageDataReceived++;
        bytesWritten++;
    }
    
    Serial.printf("Received chunk: %d hex chars -> %d bytes (total: %d/%d)\n", 
                  hexData.length(), bytesWritten, imageDataReceived, EPD_BUFFER_SIZE);
    
    server.send(200, "application/json", 
                "{\"status\":\"ok\",\"received\":" + String(imageDataReceived) + "}");
}

void handleDisplayImageEnd(void) {
    if (!imageInProgress) {
        server.send(400, "application/json", "{\"error\":\"No transfer in progress\"}");
        return;
    }
    
    Serial.printf("Finalizing display. Total received: %d bytes\n", imageDataReceived);
    
    // Pad with white if we didn't receive enough data
    while (imageDataReceived < EPD_BUFFER_SIZE) {
        Epaper_Write_Data(0x11);  // White
        if (currentImageFile) {
            currentImageFile.write(0x11);
        }
        imageDataReceived++;
    }

    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
    EPD_DeepSleep();

    if (currentImageFile) {
        currentImageFile.close();
        Serial.printf("Saved image to SD as %s (%d bytes)\n", currentImageSaveName.c_str(), imageDataReceived);
    }

    currentImageSaveName = "";

    imageInProgress = false;
    displayBusy = false;
    
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Display complete\"}");
    Serial.println("Chunked image display complete.");
}

void setupWebServer(void) {
    // WiFi config endpoints
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSaveWiFi);
    server.on("/resetwifi", HTTP_GET, handleResetWiFi);
    
    // Display control endpoints (for Pi Zero to call)
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/ui", HTTP_GET, handleUiPage);
    server.on("/colortest", HTTP_GET, handleColorTest);
    server.on("/pintest", HTTP_GET, handlePinTestPage);
    server.on("/pin", HTTP_GET, handlePinControl);
    server.on("/clear", HTTP_GET, handleClear);
    server.on("/color", HTTP_GET, handleSingleColor);
    
    // LED control endpoints
    server.on("/led/on", HTTP_GET, handleLedOn);
    server.on("/led/off", HTTP_GET, handleLedOff);
    server.on("/led/toggle", HTTP_GET, handleLedToggle);
    server.on("/led/color", HTTP_GET, handleLedColor);
    
    // Single-shot image display (small images only due to HTTP body limits)
    server.on("/display", HTTP_POST, handleDisplayImage);
    
    // Chunked image display (for full-size images)
    server.on("/display/start", HTTP_POST, handleDisplayImageStart);
    server.on("/display/chunk", HTTP_POST, handleDisplayImageChunk);
    server.on("/display/end", HTTP_POST, handleDisplayImageEnd);

    // SD stored images
    server.on("/images", HTTP_GET, handleListImages);
    server.on("/images/show", HTTP_POST, handleShowImage);
    
    // Captive portal redirect
    server.onNotFound([]() {
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
    });
    
    server.begin();
    Serial.println("Web server started.");
}

// ============== Main Setup & Loop ==============
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== E-Paper Display Receiver (ESP32-C6) ===");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

    // Initialize SD card early
    if (SD_CS_PIN >= 0) {
        sdMounted = SD.begin(SD_CS_PIN);
    } else {
        sdMounted = SD.begin();
    }
    if (sdMounted) {
        Serial.println("SD card mounted.");
    } else {
        Serial.println("SD card not detected or failed to mount.");
    }

    // Initialize NeoPixel LED and turn it OFF
    pixel.begin();
    pixel.setPixelColor(0, 0);  // Off
    pixel.show();
    ledEnabled = false;
    Serial.println("NeoPixel LED initialized (OFF)");
    
    // Initialize display pins
    pinMode(BUSY_Pin, INPUT_PULLUP);  // Try with internal pull-up
    pinMode(RES_Pin, OUTPUT);
    pinMode(DC_Pin, OUTPUT);
    pinMode(CS_Pin, OUTPUT);
    pinMode(SCK_Pin, OUTPUT);
    pinMode(SDI_Pin, OUTPUT);
    
    // Set initial states
    digitalWrite(CS_Pin, HIGH);
    digitalWrite(DC_Pin, LOW);
    digitalWrite(SCK_Pin, LOW);
    digitalWrite(SDI_Pin, LOW);
    
    Serial.println("Pin states after init:");
    Serial.printf("  BUSY (GPIO%d): %d\n", BUSY_Pin, digitalRead(BUSY_Pin));
    Serial.printf("  RST  (GPIO%d): output\n", RES_Pin);
    Serial.printf("  DC   (GPIO%d): output\n", DC_Pin);
    Serial.printf("  CS   (GPIO%d): output\n", CS_Pin);
    Serial.printf("  SCK  (GPIO%d): output\n", SCK_Pin);
    Serial.printf("  SDI  (GPIO%d): output\n", SDI_Pin);
    
    // Run SPI pin test
    testSPIPins();
    
    // Note: We do NOT clear the display here on boot
    // Display will only update if WiFi fails or IP changes
    
    // Setup WiFi (will update display only if needed)
    setupWiFi();
    
    // Setup web server
    setupWebServer();
    
    Serial.println("Setup complete. Ready for connections.");
    Serial.println("\nAPI Endpoints:");
    Serial.println("  GET  /status              - Get display status");
    Serial.println("  GET  /colortest           - Run 6-color test");
    Serial.println("  GET  /pintest             - Pin test page (web UI)");
    Serial.println("  GET  /clear               - Clear to white");
    Serial.println("  GET  /color?color=X       - Fill with color");
    Serial.println("  GET  /ui                  - Browser upload UI");
    Serial.println("  GET  /led/on              - Turn LED on (dim white)");
    Serial.println("  GET  /led/off             - Turn LED off");
    Serial.println("  GET  /led/toggle          - Toggle LED");
    Serial.println("  GET  /led/color?r=X&g=X&b=X - Set LED color (0-255)");
    Serial.println("  POST /display/start       - Start chunked image transfer");
    Serial.println("  POST /display/chunk       - Send image data chunk");
    Serial.println("  POST /display/end         - Finish and refresh display");
    Serial.println("  GET  /images              - List saved images (SD)");
    Serial.println("  POST /images/show?name=X  - Display saved image");
}

void loop() {
    if (isAPMode) {
        dnsServer.processNextRequest();
    }
    
    server.handleClient();
    delay(10);
}