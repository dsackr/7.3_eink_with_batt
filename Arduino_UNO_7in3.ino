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

bool isAPMode = false;
String currentSSID = "";
String currentIP = "";
volatile bool displayBusy = false;
bool ledEnabled = false;  // LED state (false = off)

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
            <p><strong>LED:</strong> <span class="led-status %LEDCLASS%"></span>%LEDSTATUS%</p>
        </div>
        <p>This display is controlled by the Pi Zero server.<br>
        Use the Pi Zero web interface to upload images.</p>
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
    String json = "{";
    json += "\"busy\":" + String(displayBusy ? "true" : "false") + ",";
    json += "\"ip\":\"" + currentIP + "\",";
    json += "\"ssid\":\"" + currentSSID + "\",";
    json += "\"width\":800,";
    json += "\"height\":480,";
    json += "\"colors\":6,";
    json += "\"led\":" + String(ledEnabled ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
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
        imageDataReceived++;
    }
    
    Epaper_Write_Command(0x12);
    Epaper_Write_Data(0x00);
    Epaper_READBUSY();
    EPD_DeepSleep();
    
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
    Serial.println("  GET  /led/on              - Turn LED on (dim white)");
    Serial.println("  GET  /led/off             - Turn LED off");
    Serial.println("  GET  /led/toggle          - Toggle LED");
    Serial.println("  GET  /led/color?r=X&g=X&b=X - Set LED color (0-255)");
    Serial.println("  POST /display/start       - Start chunked image transfer");
    Serial.println("  POST /display/chunk       - Send image data chunk");
    Serial.println("  POST /display/end         - Finish and refresh display");
}

void loop() {
    if (isAPMode) {
        dnsServer.processNextRequest();
    }
    
    server.handleClient();
    delay(10);
}