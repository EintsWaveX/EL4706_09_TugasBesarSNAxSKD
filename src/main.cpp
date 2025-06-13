#include <Arduino.h>
#include <TFT_eSPI.h>
#include <DHT.h>
#include <PID_v1.h>
#include <vector>

#define MIN_TEMP 24
#define MAX_TEMP 32
#define WATER_SENSOR_PIN 34
#define WATER_ACTUATOR_IN1_PIN 32 // ADC pin
#define WATER_ACTUATOR_ENA_PIN 17 // ENA (PWM) pin
#define DHT22_SENSOR_PIN 13
#define FANBLOWER_ACTUATOR_PIN 16

TFT_eSPI tft = TFT_eSPI();
DHT dht(DHT22_SENSOR_PIN, DHT22);

unsigned long lastTouchTime = 0, lastGetData = 0;
const unsigned long debounceDelay = 50; // milliseconds
float multiplierSampleReadingTime = 1;
bool touchReleased = true;
int intPart = 1;
int fracPart = 0; // 0.0 to 0.9

float tempHistory[210], humiHistory[210];
double pidOutputHistory[210];
int dataCount = 0; // Keeps track of how many values are filled

static int lastSampleTimeMs = -1; // Declare this globally or static inside loop()
static bool lastSeeGraphInfo = false;
static bool firstEnter = true;  // Tracks first-time access
bool firstRun = false, dht22ErrorHandled = false, dht22Connected = true, seeGraphInfo = false, pauseReading = false, wasPaused = false;
uint64_t totalReadings = 0LL;
double totalTime = 0.0f;

int countGridGapXIndex = 5, countHistorySizeIndex = 5, waterPumpSpeedIndex = 0;
const int gridGapX[7] = {2, 4, 5, 7, 8, 10, 14};
const int historySize[7] = {10, 20, 30, 60, 100, 150, 210};
const float waterPumpSpeedList[5] = {0.5, 1.0, 2.0, 3.0, 4.0};
const float waterPumpPWMList[5] = {158.4, 166.8, 175.2, 183.6, 192.0};

int rawValue = 0;
float filtered = 0;
const float alpha = 0.1;
float waterPercent = 0.0;
const int sensorMaxValue = 1800; // Calibrate this for full water level
const int sensorMinValue = 0;  // Calibrate this for dry level
const int waterMinPercent = 30, waterMaxPercent = 60;
float waterSetpointPercent = 45;    // Default setpoint

const int pwmFrequency_WATERPUMP = 1000; // 1 kHz PWM frequency (standard for water pumps)
const int pwmFrequency_FANBLOWER = 25000; // 25 kHz PWM frequency (standard for 4-wire fans)
const int pwmChannel_WATERPUMP = 0; // PWM channel
const int pwmChannel_FANBLOWER = 1; // PWM channel
const int resolution_WATERPUMP = 10; // 8-bit resolution (0–255)
const int resolution_FANBLOWER = 8; // 8-bit resolution (0–255)

int setTemperature = 26;
float currentTemperature = 0, currentHumidity = 0;
float minTemp = 0, maxTemp = 0, avgTemp = 0;
float minHumi = 0, maxHumi = 0, avgHumi = 0;

// ============================== PID PID PID ==============================
double Kp = 100.0, Ki = 0.1, Kd = 5.0;
double Setpoint;     // Desired temperature
double Input;        // Current temperature from sensor
double Output;       // PWM value for fan
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
// ============================== PID PID PID ==============================
 
// Colors to note:
// #181818 (24, 24, 24)    >> Background Color
// #7fffd4 (127, 255, 212) >> Primary Color 1
// #64cdff (100, 205, 255) >> Primary Color 2
// #b2fefc (178, 254, 252) >> Secondary Color 1
// #fde5f5 (253, 229, 245) >> Secondary Color 2
// #555577 (85, 85, 119)   >> Filler Color

// Convert RGB888 to RGB565
uint16_t RGB565_U16(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t red   = (r >> 3) & 0x1F;  // 5 bits for red
    uint16_t green = (g >> 2) & 0x3F;  // 6 bits for green
    uint16_t blue  = (b >> 3) & 0x1F;  // 5 bits for blue

    // Combine into a single uint16_t
    return (red << 11) | (green << 5) | blue;
}

// Clamping the coordinate boundaries
int clamp(int val, int minVal, int maxVal) {
    return max(minVal, min(val, maxVal));
}
void drawQuadraticBezier(int x0, int y0, int cx, int cy, int x1, int y1, uint16_t color, int segments = 10) {
    float tStep = 1.0f / segments;
    float t = 0;
    int prevX = x0, prevY = y0;

    for (int i = 1; i <= segments; ++i) {
        t += tStep;
        float a = (1 - t) * (1 - t);
        float b = 2 * (1 - t) * t;
        float c = t * t;

        int x = a * x0 + b * cx + c * x1;
        int y = a * y0 + b * cy + c * y1;

        tft.drawLine(prevX, prevY, x, y, color);
        prevX = x;
        prevY = y;
    }
}

void updateHistory(float temp, float humi, double pidOutput) {
    if (dataCount < 210) {
        tempHistory[dataCount] = temp;
        humiHistory[dataCount] = humi;
        pidOutputHistory[dataCount] = pidOutput;
        dataCount++;
    } else {
        // Shift everything to the left (like a rolling buffer)
        for (int i = 0; i < 209; i++) {
            tempHistory[i] = tempHistory[i + 1];
            humiHistory[i] = humiHistory[i + 1];
            pidOutputHistory[i] = pidOutputHistory[i + 1];
        }
        tempHistory[209] = temp;
        humiHistory[209] = humi;
        pidOutputHistory[209] = pidOutput;
    }
}

float averageTemperature(float *temp) {
    float _temp = 0.0;
    for (int i = 0; i < historySize[countHistorySizeIndex]; i++) {
        _temp += temp[i];
    }
    
    return _temp / historySize[countHistorySizeIndex];
}
float averageHumidity(float *humi) {
    float _humi = 0.0;
    for (int i = 0; i < historySize[countHistorySizeIndex]; i++) {
        _humi += humi[i];
    }
    
    return _humi / historySize[countHistorySizeIndex];
}

typedef enum {
    SCREEN_MAIN,
    SCREEN_TemperatureSetpoint,
    SCREEN_TempGraph_MainOnly,
    SCREEN_HumiGraph_MainOnly,
    SCREEN_Settings,
    SCREEN_WaterLevelSetpoint,
    SCREEN_GraphConfiguration,
    SCREEN_PIDGraph_MainOnly,
    SCREEN_DHT22IsNan
} ScreenState;

typedef struct Button {
    int x, y, w, h;
    String label;
    uint16_t textColor, btnColor;
    bool isInverted = false;

    Button(int x, int y, int w, int h, String label, uint16_t textColor, uint16_t btnColor) :
        x(x), y(y), w(w), h(h), label(label), textColor(textColor), btnColor(btnColor), isInverted(false) {}

    // Break a label into lines based on available width
    std::vector<String> breakLabelIntoLines(String label, int maxWidth, uint8_t textSize) {
        std::vector<String> lines;
        String word, line;
        int spaceWidth = tft.textWidth(" ", textSize);
        int lineWidth = 0;

        for (int i = 0; i <= label.length(); ++i) {
            char c = label[i];
            if (c == ' ' || c == '\0') {
                if (tft.textWidth(line + word, textSize) <= maxWidth) {
                    if (line.length() > 0) line += ' ';
                    line += word;
                } else {
                    lines.push_back(line);
                    line = word;
                }
                word = "";
            } else {
                word += c;
            }
        }

        if (line.length() > 0) lines.push_back(line);
        return lines;
    }

    void draw(uint8_t textSize = 1) {
        uint16_t fg = isInverted ? btnColor : textColor;
        uint16_t bg = isInverted ? textColor : btnColor;
        tft.fillRoundRect(x, y, w, h, 8, bg);
        tft.setTextColor(fg, bg);
        tft.setTextSize(textSize);
        tft.setTextDatum(CC_DATUM);

        // Calculate lines to draw
        std::vector<String> lines = breakLabelIntoLines(label, w - 10, textSize);
        int lineHeight = 8 * textSize + 2;
        int totalHeight = lines.size() * lineHeight;

        // Center vertically
        int startY = y + (h - totalHeight) / 2 + lineHeight / 2;

        for (size_t i = 0; i < lines.size(); ++i) {
            tft.drawString(lines[i], x + w / 2, startY + i * lineHeight);
        }

        tft.setTextSize(1); // Reset
    }

    bool contains(int tx, int ty) {
        return tx >= x && tx <= x + w && ty >= y && ty <= y + h;
    }

    void setInverted(bool inverted, uint8_t textSize = 1) {
        if (isInverted != inverted) {
            isInverted = inverted;
            draw(textSize);
        }
    }
} Button;

void changeScreen(ScreenState next);
void clearButton(Button btn);
void drawButtonWithText(Button *btn, const String &text, uint8_t textSize);

// Draw button icon
void drawButtonWithText(Button *btn, const String &text, uint8_t textSize = 1) {
    btn->label = text;
    btn->draw(textSize);
}

// Important constants and enumerations
ScreenState currentScreen = SCREEN_MAIN, previousScreen = SCREEN_MAIN;
const uint16_t BACKGROUND_COLOR = RGB565_U16(24, 24, 24);
const uint16_t PRIMARY_COLOR_1 = RGB565_U16(127, 255, 212);
const uint16_t PRIMARY_COLOR_2 = RGB565_U16(100, 205, 255);
const uint16_t SECONDARY_COLOR_1 = RGB565_U16(178, 254, 252);
const uint16_t SECONDARY_COLOR_2 = RGB565_U16(253, 229, 245);
const uint16_t FILLER_COLOR = RGB565_U16(85, 85, 119);

// Buttons
int gap = 10;
const int boxW = 180;
const int boxH = 90;
const int boxX = (320 - boxW) / 2;
const int boxY = 70;
Button mainBtn = {20, 105, 280, 40, "Temperature Setpoint", FILLER_COLOR, PRIMARY_COLOR_1};
Button settingsBtn = {20, 105 + mainBtn.h + gap, 280, 40, "Settings", SECONDARY_COLOR_1, FILLER_COLOR};
Button backBtn = {10, 200, 75, 30, "Back", TFT_WHITE, TFT_RED};
Button incTempBtn = {320 - 10 - 75, 240 - backBtn.h - 20 - 45, 75, 45, "[+]", FILLER_COLOR, TFT_SKYBLUE};
Button decTempBtn = {10, 240 - backBtn.h - 20 - 45, 75, 45, "[-]", TFT_WHITE, TFT_MAROON};
Button incGridGapXBtn = {320 - 10 - 30, 240 - 10 - backBtn.h, 30, 30, ">", FILLER_COLOR, SECONDARY_COLOR_1};
Button decGridGapXBtn = {incGridGapXBtn.x - 10 - 30 - 45, incGridGapXBtn.y, 30, 30, "<", FILLER_COLOR, SECONDARY_COLOR_2};
Button resetDataCountBtn = {incGridGapXBtn.x - 5 - 45, incGridGapXBtn.y, 45, 30, "Reset", TFT_OLIVE, TFT_SILVER};
Button seeGraphInfoBtn = {backBtn.x + backBtn.w + 5, incGridGapXBtn.y, 100, 30, "See Graph Info", PRIMARY_COLOR_1, FILLER_COLOR};
Button closeGraphInfoBtn = {backBtn.x, incGridGapXBtn.y, 60, 30, "Close", FILLER_COLOR, TFT_GREENYELLOW};
Button togglePauseUPBtn = {320 - 10 - 30, 40, 30, 30, "||", FILLER_COLOR, TFT_WHITE}; // Default: ||;
Button togglePauseDOWNBtn = {320 - 10 - 30, 240 - 10 - 30, 30, 30, "||", FILLER_COLOR, TFT_WHITE}; // Default: ||;
Button graphBtn = {320 - 10 - 75, 200, 75, 30, "LIVE Graph", TFT_WHITE, TFT_MAGENTA};
Button showTempGraphBtn = {320 - 10 - 120, 40, 120, 30, "Show Temp. Graph", FILLER_COLOR, TFT_ORANGE};
Button showHumiGraphBtn = {320 - 10 - 120, 40, 120, 30, "Show Humi. Graph", FILLER_COLOR, TFT_CYAN};
Button waterLevelSetpointBtn = {30, 60, 125, 110, "MANUAL: \"Water\" Control Related", FILLER_COLOR, SECONDARY_COLOR_1};
Button graphRelatedSetpointBtn = {waterLevelSetpointBtn.x + waterLevelSetpointBtn.w + gap, 60, 125, 110, "MANUAL: Configure LIVE Graph", FILLER_COLOR, SECONDARY_COLOR_2};
Button incWLSBtn = {320 - 10 - 35, 75, 30, 30, "+", FILLER_COLOR, SECONDARY_COLOR_1};
Button decWLSBtn = {incWLSBtn.x - 10 - 30 - 75 + 1, incWLSBtn.y, 30, 30, "-", FILLER_COLOR, SECONDARY_COLOR_2};
Button incWSPWMBtn = {320 - 10 - 75, 240 - backBtn.h - 20 - 45, 75, 45, "[>]", FILLER_COLOR, TFT_SKYBLUE};
Button decWSPWMBtn = {10, 240 - backBtn.h - 20 - 45, 75, 45, "[<]", TFT_WHITE, TFT_MAROON};
Button btnIntUp = {boxX, boxY - 30, 85 + 2, 25, "^^^", FILLER_COLOR, SECONDARY_COLOR_1};
Button btnFracUp = {boxX + (85 + 2) + 5, boxY - 30, 85 + 2, 25, "^^^", FILLER_COLOR, SECONDARY_COLOR_1};
Button btnIntDown = {boxX, boxY + boxH + 5, 85 + 2, 25, "vvv", FILLER_COLOR, SECONDARY_COLOR_2};
Button btnFracDown = {boxX + (85 + 2) + 5, boxY + boxH + 5, 85 + 2, 25, "vvv", FILLER_COLOR, SECONDARY_COLOR_2};
Button showPIDGraphInfoBtn = {320 - 10 - 135, backBtn.y, 135, backBtn.h, "Show PID Graph Info", PRIMARY_COLOR_2, FILLER_COLOR};

void resetAllButtons() {
    mainBtn.setInverted(false, 2);
    settingsBtn.setInverted(false, 2);
    waterLevelSetpointBtn.setInverted(false, 2);
    graphRelatedSetpointBtn.setInverted(false, 2);

    incTempBtn.setInverted(false, 2);
    decTempBtn.setInverted(false, 2);
    incGridGapXBtn.setInverted(false, 2);
    decGridGapXBtn.setInverted(false, 2);
    resetDataCountBtn.setInverted(false);
    seeGraphInfoBtn.setInverted(false);
    closeGraphInfoBtn.setInverted(false);
    togglePauseUPBtn.setInverted(false, 2);
    togglePauseDOWNBtn.setInverted(false, 2);
    // pauseReadingBtn.setInverted(false, 2);
    // continueReadingBtn.setInverted(false, 2);
    
    backBtn.setInverted(false);
    graphBtn.setInverted(false);
    showTempGraphBtn.setInverted(false);
    showHumiGraphBtn.setInverted(false);
    incWLSBtn.setInverted(false, 2);
    decWLSBtn.setInverted(false, 2);
    incWSPWMBtn.setInverted(false, 2);
    decWSPWMBtn.setInverted(false, 2);
    btnIntUp.setInverted(false, 2);
    btnFracUp.setInverted(false, 2);
    btnIntDown.setInverted(false, 2);
    btnFracDown.setInverted(false, 2);
    showPIDGraphInfoBtn.setInverted(false);
}
void clearButton(Button btn) {
    tft.fillRoundRect(btn.x, btn.y, btn.w, btn.h, 8, BACKGROUND_COLOR);
}

void drawTopBar(String title) {
    tft.fillRect(0, 0, 320, 30, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.setTextSize(2);
    tft.setTextDatum(CL_DATUM);
    tft.drawString(title, 10, 15);
}

void drawMainScreen() {
    drawTopBar("HOME Menu");
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.setTextDatum(CC_DATUM);
    tft.setTextSize(2);
    tft.drawString("Hi and Welcome, User!", 160, 60);
    tft.setTextSize(1);
    tft.drawString("What would you like to do today?", 160, 75);
    // tft.drawString("Feel free to look around and see how things work...", 160, 85);

    mainBtn.draw(2);
    settingsBtn.draw(2);

    tft.setTextDatum(BL_DATUM);
    tft.setTextColor(SECONDARY_COLOR_2, BACKGROUND_COLOR);
    tft.drawString("Project Created by EL4705, Group No. 9", 0, 230);
    tft.drawString("Members: Immanuel, Danny, Zaki", 0, 240);
}

void drawDHT22IsNanScreen() {
    drawTopBar("ERROR! (see below...)");
    tft.setTextDatum(CC_DATUM);
    tft.setTextColor(TFT_RED, BACKGROUND_COLOR);
    tft.setTextSize(2);
    tft.drawString("ERROR:", 160, 60);
    tft.drawString("DHT22 Connection is Lost!", 160, 80);
    
    tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
    tft.setTextSize(1);
    tft.drawString("Please check that your DHT22 is properly", 160, 120);
    tft.drawString("connected into the board/circuit!", 160, 135);
    tft.drawString("You'll be brought back to the previous menu,", 160, 150);
    tft.drawString("ONLY after DHT22 is safely connected into the", 160, 165);
    tft.drawString("system...", 160, 180);
}

void drawSettingsScreen(const String &title) {
    drawTopBar(title);

    waterLevelSetpointBtn.draw(2);
    graphRelatedSetpointBtn.draw(2);
    backBtn.draw();
}

void drawWaterLevelSetpointScreen(const String &title) {
    drawTopBar(title);

    // ...
    char wpShow[4], wspShow[4], wpsShow[32], pwmIndexShow[4];
    snprintf(wpShow, sizeof(wpShow), "%03d", (int)waterPercent);
    snprintf(wspShow, sizeof(wspShow), "%02d", (int)waterSetpointPercent);
    snprintf(wpsShow, sizeof(wpsShow), "Speed INFO: %s",
        (waterPumpSpeedList[waterPumpSpeedIndex] == 0.5) ? "SLOW" :
        (waterPumpSpeedList[waterPumpSpeedIndex] == 1.0) ? "NORMAL" :
        (waterPumpSpeedList[waterPumpSpeedIndex] == 2.0) ? "QUICK" :
        (waterPumpSpeedList[waterPumpSpeedIndex] == 3.0) ? "FAST" :
        (waterPumpSpeedList[waterPumpSpeedIndex] == 4.0) ? "EXTREME" : "-");
    snprintf(pwmIndexShow, sizeof(pwmIndexShow), "%d/5", (waterPumpSpeedIndex + 1));

    int wYPos = 30;
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(PRIMARY_COLOR_1, BACKGROUND_COLOR);
    tft.drawString("Actual Water Level:", (tft.width() / 4), wYPos + 15);
    tft.setTextSize(4);
    tft.drawString(wpShow, (tft.width() / 4) - 12, wYPos + 60);
    tft.setTextSize(3);
    tft.drawString("%", (tft.width() / 4) + 35, wYPos + 45);

    tft.setTextSize(1);
    tft.setTextColor(SECONDARY_COLOR_1, BACKGROUND_COLOR);
    tft.drawString("Water Level Setpoint:", (320 - (tft.width() / 4)) - 4, wYPos + 15);
    tft.setTextSize(4);
    tft.drawString(wspShow, (320 - (tft.width() / 4)) - 15, wYPos + 60);
    tft.setTextSize(3);
    tft.drawString("%", (320 - (tft.width() / 4)) + 20, wYPos + 45);
    tft.setTextSize(1);
    tft.drawString("Min: 30%, Max: 60%", incWLSBtn.x - 42, incWLSBtn.y + 40);

    tft.fillRect(75 + 20, 240 - 20 - 75, 130, 85, TFT_BLACK);
    tft.drawRect(75 + 20, 240 - 20 - 75, 130, 85, TFT_WHITE);

    tft.setTextDatum(CC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Set Speed Control:", 105 + 55, 147 + 5);
    tft.setTextSize(4);

    switch (waterPumpSpeedIndex) {
        case 0: tft.setTextColor(TFT_ORANGE, TFT_BLACK); break;
        case 1: tft.setTextColor(TFT_SKYBLUE, TFT_BLACK); break;
        case 2: tft.setTextColor(TFT_GREEN, TFT_BLACK); break;
        case 3: tft.setTextColor(TFT_MAGENTA, TFT_BLACK); break;
        case 4: tft.setTextColor(TFT_RED, TFT_BLACK); break;
        default: break;
    }
    tft.drawFloat(waterPumpSpeedList[waterPumpSpeedIndex], 1, (320 / 2) - 10 + 2, 240 - ((130 / 2) - 15));
    tft.setTextSize(3);
    tft.drawString("x", (320 / 2) + 36 + 2, 240 - (130 / 2) + 2);
    tft.setTextSize(1);
    tft.drawString(wpsShow, 105 + 55, 242 - 20);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.drawString(pwmIndexShow, 320 - 45 - 2, 240 - 25);

    incWLSBtn.draw(2);
    decWLSBtn.draw(2);
    incWSPWMBtn.draw(2);
    decWSPWMBtn.draw(2);
    backBtn.draw();
}

void drawGraphConfigurationScreen(const String &title) {
    drawTopBar(title);

    // ...
    // Draw main box
    tft.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);
    tft.drawRect(boxX, boxY, boxW, boxH, TFT_WHITE);

    // Header and footer text
    tft.setTextSize(1);
    tft.setTextColor(SECONDARY_COLOR_2, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Set Reading Multiplier:", boxX + 6, boxY + 4);

    tft.setTextDatum(BL_DATUM);
    tft.drawString("Min: 0.1, Max: 9.9", boxX + 6, boxY + boxH - 4);

    // Draw multiplier
    tft.setTextSize(5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(CC_DATUM);

    String multiplierStr = String(intPart) + "." + String(fracPart);
    int textX = boxX + boxW / 2 - 10;
    int textY = boxY + boxH / 2;

    tft.drawString(multiplierStr, textX, textY);

    // Draw the raised 'x'
    tft.setTextSize(3);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("x", textX + 50, textY - 15);

    btnIntUp.draw(2);
    btnFracUp.draw(2);
    btnIntDown.draw(2);
    btnFracDown.draw(2);
    showPIDGraphInfoBtn.draw();
    backBtn.draw();
}

void drawSetTemperatureScreen(const String &title) {
    int xPos = 10, yPos = 90 + 5;
    char lowestTemp[50], averageTemp[50], highestTemp[50];
    char lowestHumidity[50], averageHumidity[50], highestHumidity[50];

    drawTopBar(title);
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.setTextDatum(CC_DATUM);

    // Temperature - Big number
    tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(4);
    tft.drawFloat(currentTemperature, 1, (tft.width() / 4) - 20, 60 + 5);  // Large temperature
    tft.setTextSize(3);
    tft.drawString("C", (tft.width() / 4) + 45, 45 + 5);  // Degree symbol and unit

    // Humidity - Big number
    tft.setTextColor(TFT_CYAN, BACKGROUND_COLOR);
    tft.setTextSize(4);
    tft.drawFloat(currentHumidity, 1, (320 - (tft.width() / 4)) - 5, 60 + 5);  // Humidity %
    tft.setTextSize(3);
    tft.drawString("%", (320 - (tft.width() / 4)) + 60, 45 + 5);

    // Min / Avg / Max Temp - Regular font
    tft.setTextSize(1);
    tft.setTextDatum(ML_DATUM);

    tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
    snprintf(lowestTemp, sizeof(lowestTemp), "Lowest  Temp.: %.1f C", minTemp);
    snprintf(averageTemp, sizeof(averageTemp), "Average Temp.: %.1f C", avgTemp);
    snprintf(highestTemp, sizeof(highestTemp), "Highest Temp.: %.1f C", maxTemp);
    tft.drawString(lowestTemp, xPos, yPos); yPos += 15;
    tft.drawString(averageTemp, xPos, yPos); yPos += 15;
    tft.drawString(highestTemp, xPos, yPos);

    xPos = 185; yPos = 90 + 5;
    tft.setTextColor(TFT_CYAN, BACKGROUND_COLOR);
    snprintf(lowestHumidity, sizeof(lowestHumidity), "Lowest  Humi.: %.1f %%", minHumi);
    snprintf(averageHumidity, sizeof(averageHumidity), "Average Humi.: %.1f %%", avgHumi);
    snprintf(highestHumidity, sizeof(highestHumidity), "Highest Humi.: %.1f %%", maxHumi);
    tft.drawString(lowestHumidity, xPos, yPos); yPos += 15;
    tft.drawString(averageHumidity, xPos, yPos); yPos += 15;
    tft.drawString(highestHumidity, xPos, yPos);

    incTempBtn.draw(2);
    decTempBtn.draw(2);
    backBtn.draw();
    graphBtn.draw();

    tft.fillRect(75 + 20, 240 - 20 - 75, 130, 85, TFT_BLACK);
    tft.drawRect(75 + 20, 240 - 20 - 75, 130, 85, TFT_WHITE);

    tft.setTextDatum(CC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Set Temperature:", 105 + 37 + 5, 147 + 5);
    tft.setTextSize(6);
    tft.drawNumber(setTemperature, (320 / 2) - 10 + 2, 240 - ((130 / 2) - 15));
    tft.setTextSize(3);
    tft.drawString("C", (320 / 2) + 36 + 2, 240 - (130 / 2) + 2);
    tft.setTextSize(1);
    tft.drawString("Min: 24, Max: 32", 105 + 37 + 5, 240 - 17);
}

void drawTempGraph(const String &title, int setGridGapX = 10) {
    drawTopBar(title);
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.setTextDatum(CC_DATUM);

    // --- Temperature Readout ---
    tft.setTextSize(3);
    tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
    tft.drawFloat(currentTemperature, 1, 65, 55);
    tft.setTextSize(2);
    tft.drawString("C", 110, 45);

    // --- Graph Constants ---
    const int labelX = 15;
    const int labelBaseY = 190;
    const int labelStepY = 27;
    const int graphX = 30, graphY = 80;
    const int graphWidth = 280, graphHeight = 110;
    const int gridLinesY = 4; // Matches number of gaps between 5 labels

    // --- Determine Temperature Scale Range ---
    float minTempToUse = MIN_TEMP;
    float maxTempToUse = MAX_TEMP;

    for (int i = 0; i < dataCount; ++i) {
        minTempToUse = min(minTempToUse, tempHistory[i]);
        maxTempToUse = max(maxTempToUse, tempHistory[i]);
    }

    // Add padding and align to 4°C steps
    minTempToUse = floor((minTempToUse - 1.0f) / 4.0f) * 4.0f;
    maxTempToUse = ceil((maxTempToUse + 1.0f) / 4.0f) * 4.0f;

    // Clamp bounds
    minTempToUse = max(0.0f, minTempToUse);
    maxTempToUse = min(100.0f, maxTempToUse);
    if (maxTempToUse - minTempToUse < 4.0f) maxTempToUse = minTempToUse + 4.0f;

    // --- Clamp to Prevent Overzooming ---
    minTempToUse = max(0.0f, minTempToUse);
    maxTempToUse = min(100.0f, maxTempToUse);
    if (maxTempToUse - minTempToUse < 4.0f) maxTempToUse = minTempToUse + 4.0f;

    // --- Temperature Labels (Always 5) ---
    float labelStep = (maxTempToUse - minTempToUse) / 4.0f;
    tft.setTextSize(1);
    for (int i = 0; i < 5; ++i) {
        float labelVal = minTempToUse + i * labelStep;
        tft.drawString(String((int)roundf(labelVal)), labelX, labelBaseY - labelStepY * i - (i > 0));
    }

    // --- Graph Container & Grid ---
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.fillRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_BLACK);
    tft.drawRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_WHITE);

    for (int i = 1; i <= setGridGapX; ++i) {
        int x = graphX + (graphWidth * i) / (setGridGapX + 1);
        tft.drawLine(x, graphY, x, graphY + graphHeight, TFT_DARKGREY);
    }
    for (int i = 1; i <= gridLinesY; ++i) {
        int y = graphY + (graphHeight * i) / (gridLinesY + 1);
        tft.drawLine(graphX, y, graphX + graphWidth, y, TFT_DARKGREY);
    }

    // --- Plot Temperature History ---
    int visibleSize = historySize[countHistorySizeIndex];
    int startIndex = max(0, dataCount - visibleSize);
    int endIndex = dataCount - 1;
    float stepX = (visibleSize > 1) ? (float)graphWidth / (visibleSize - 1) : 1;
    float scaleY = graphHeight / (maxTempToUse - minTempToUse);

    auto mapTemp = [&](float t) -> float {
        return (t - minTempToUse) * scaleY;
    };

    for (int i = startIndex + 1; i <= endIndex; ++i) {
        float yPrev = mapTemp(tempHistory[i - 1]);
        float yCurr = mapTemp(tempHistory[i]);

        int x0 = clamp(graphX + (i - startIndex - 1) * stepX, graphX, graphX + graphWidth);
        int x1 = clamp(graphX + (i - startIndex) * stepX, graphX, graphX + graphWidth);
        int y0 = clamp(graphY + graphHeight - yPrev, graphY, graphY + graphHeight);
        int y1 = clamp(graphY + graphHeight - yCurr, graphY, graphY + graphHeight);

        tft.drawLine(x0, y0, x1, y1, TFT_ORANGE);
    }

    // --- Set Temperature Reference Line ---
    float ySet = mapTemp(setTemperature);
    int ySetPx = clamp(graphY + graphHeight - ySet, graphY, graphY + graphHeight);
    tft.drawLine(graphX, ySetPx, graphX + graphWidth, ySetPx, TFT_MAGENTA);

    // --- Buttons ---
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    if (seeGraphInfo != lastSeeGraphInfo || firstEnter) {
        if (!seeGraphInfo) {
            tft.fillRect(115, 30, 205, 45, BACKGROUND_COLOR);
            tft.fillRect(closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 200, 240, 40, BACKGROUND_COLOR);

            clearButton(closeGraphInfoBtn);
            clearButton(togglePauseUPBtn);

            backBtn.draw();
            showHumiGraphBtn.draw();
            seeGraphInfoBtn.draw();
            incGridGapXBtn.draw(2);
            decGridGapXBtn.draw(2);
            resetDataCountBtn.draw();
        } else {
            clearButton(backBtn);
            clearButton(showHumiGraphBtn);
            clearButton(seeGraphInfoBtn);
            clearButton(incGridGapXBtn);
            clearButton(decGridGapXBtn);
            clearButton(resetDataCountBtn);

            closeGraphInfoBtn.draw();
            drawButtonWithText(&togglePauseUPBtn, pauseReading ? "|>" : "||", 2);
        }

        lastSeeGraphInfo = seeGraphInfo;
        firstEnter = false;
    }

    // --- Extra Graph Info ---
    if (seeGraphInfo) {
        char actualTemperature[16], sampleReading[32], graphScaleInfo[32], transcientInfo[32];
        snprintf(actualTemperature, sizeof(actualTemperature), "/%.1f", (float)setTemperature);
        snprintf(sampleReading, sizeof(sampleReading), "%" PRIu64 " reading(s) per %.3fs", totalReadings, totalTime);
        snprintf(graphScaleInfo, sizeof(graphScaleInfo), "X-axis (Time) Scale: %dx", gridGapX[countGridGapXIndex]);
        // snprintf(transcientInfo, sizeof(transcientInfo), "Transcient Response: %.3fs", 0.0); // placeholder

        tft.setTextSize(3);
        tft.setTextColor(TFT_MAGENTA, BACKGROUND_COLOR);
        tft.drawString(actualTemperature, 160, 55);
        tft.setTextSize(2);
        tft.drawString("C", 215 - 1, 45);

        tft.setTextSize(1);
        tft.setTextDatum(BL_DATUM);
        tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
        tft.drawString(sampleReading, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 25);
        tft.setTextColor(SECONDARY_COLOR_1, BACKGROUND_COLOR);
        tft.drawString(graphScaleInfo, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 15);
        // tft.setTextColor(SECONDARY_COLOR_2, BACKGROUND_COLOR);
        // tft.drawString(transcientInfo, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 10);
    }
}

void drawHumiGraph(const String &title, int setGridGapX = 10) {
    drawTopBar(title);
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.setTextDatum(CC_DATUM);

    // Y-axis labels
    tft.setTextSize(1);
    for (int i = 0; i <= 5; i++) {
        int y = 190 - (22 * i);
        tft.drawString(String(i * 20), 15, y);
    }

    // Current humidity display
    tft.setTextSize(3);
    tft.setTextColor(TFT_CYAN, BACKGROUND_COLOR);
    tft.drawFloat(currentHumidity, 1, 65, 55);
    tft.setTextSize(2);
    tft.drawString("%", 110, 45);

    // Graph settings
    const int graphX = 30, graphY = 80;
    const int graphWidth = 280, graphHeight = 110;
    const int gridGapY = 4;

    // Graph background
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    tft.fillRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_BLACK);
    tft.drawRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_WHITE);

    // Grid lines
    for (int i = 1; i <= setGridGapX; i++) {
        int x = graphX + (graphWidth * i) / (setGridGapX + 1);
        tft.drawLine(x, graphY, x, graphY + graphHeight, TFT_DARKGREY);
    }
    for (int i = 1; i <= gridGapY; i++) {
        int y = 190 - (22 * i);
        tft.drawLine(graphX, y, graphX + graphWidth, y, TFT_DARKGREY);
    }

    // Graph data
    int visibleSize = historySize[countHistorySizeIndex];
    if (visibleSize < 2) return; // nothing to draw

    int startIndex = max(0, dataCount - visibleSize);
    int endIndex = dataCount - 1;

    float minVal = 0, maxVal = 100;
    float scaleY = graphHeight / (maxVal - minVal);
    float stepX = (float)graphWidth / (visibleSize - 1);

    for (int i = startIndex + 1; i <= endIndex; i++) {
        float valPrev = humiHistory[i - 1];
        float valCurr = humiHistory[i];

        int x0 = graphX + round(stepX * (i - startIndex - 1));
        int x1 = graphX + round(stepX * (i - startIndex));
        int y0 = graphY + graphHeight - (valPrev - minVal) * scaleY;
        int y1 = graphY + graphHeight - (valCurr - minVal) * scaleY;

        // Clamp to bounds to avoid overflow
        x0 = constrain(x0, graphX, graphX + graphWidth);
        x1 = constrain(x1, graphX, graphX + graphWidth);
        y0 = constrain(y0, graphY, graphY + graphHeight);
        y1 = constrain(y1, graphY, graphY + graphHeight);

        tft.drawLine(x0, y0, x1, y1, TFT_CYAN);
    }

    // --- Buttons ---
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    if (seeGraphInfo != lastSeeGraphInfo || firstEnter) {
        if (!seeGraphInfo) {
            // Clear info texts that were drawn in graph info mode
            // Match these to the actual area covered by the texts
            tft.fillRect(115, 30, 205, 45, BACKGROUND_COLOR);  // actualTemperature + "C"
            tft.fillRect(closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 200, 240, 40, BACKGROUND_COLOR);  // sampleReading, graphScaleInfo, transcientInfo

            clearButton(closeGraphInfoBtn);
            clearButton(togglePauseUPBtn);

            backBtn.draw();
            showHumiGraphBtn.draw();
            seeGraphInfoBtn.draw();
            incGridGapXBtn.draw(2);
            decGridGapXBtn.draw(2);
            resetDataCountBtn.draw();
        } else {
            clearButton(backBtn);
            clearButton(showHumiGraphBtn);
            clearButton(seeGraphInfoBtn);
            clearButton(incGridGapXBtn);
            clearButton(decGridGapXBtn);
            clearButton(resetDataCountBtn);

            closeGraphInfoBtn.draw();
            drawButtonWithText(&togglePauseUPBtn, pauseReading ? "|>" : "||", 2);
        }

        lastSeeGraphInfo = seeGraphInfo;
        firstEnter = false;  // Reset after initial draw
    }
    
    if (seeGraphInfo) {
        char actualTemperature[16], sampleReading[32], graphScaleInfo[32], transcientInfo[32];
        snprintf(actualTemperature, sizeof(actualTemperature), "/%.1f", (float)setTemperature);
        snprintf(sampleReading, sizeof(sampleReading), "%" PRIu64 " reading(s) per %.3fs", totalReadings, totalTime);
        snprintf(graphScaleInfo, sizeof(graphScaleInfo), "X-axis (Time) Scale: %dx", gridGapX[countGridGapXIndex]);
        // snprintf(transcientInfo, sizeof(transcientInfo), "Transcient Response: %.3fs", 0.0); // still need to find the way here

        // tft.setTextSize(3);
        // tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
        // tft.drawString(actualTemperature, 160, 55);
        // tft.setTextSize(2);
        // tft.drawString("C", 215, 45);

        tft.setTextSize(1);
        tft.setTextDatum(BL_DATUM);
        tft.setTextColor(TFT_CYAN, BACKGROUND_COLOR);
        tft.drawString(sampleReading, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 25);
        tft.setTextColor(SECONDARY_COLOR_1, BACKGROUND_COLOR);
        tft.drawString(graphScaleInfo, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 15);
        // tft.setTextColor(SECONDARY_COLOR_2, BACKGROUND_COLOR);
        // tft.drawString(transcientInfo, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 10);
    }
}

void drawPIDGraph(const String &title, int setGridGapX = 10) {
    drawTopBar(title);

    // --- Graph Layout Constants ---
    const int labelX = 5;
    const int graphX = 30, graphY = 40;
    const int graphWidth = 280, graphHeight = 150;
    const int gridLinesY = 7;
    const int labelStepY = graphHeight / 8;

    // --- Calculate Dynamic Max Output ---
    const int minOutput = 0;
    int visibleSize = historySize[countHistorySizeIndex];
    int startIndex = max(0, dataCount - visibleSize);
    int endIndex = dataCount - 1;

    int maxOutputSeen = 0;
    for (int i = startIndex; i <= endIndex; ++i) {
        maxOutputSeen = max(maxOutputSeen, (int)pidOutputHistory[i]);
    }

    // Snap top Y value to nearest multiple of 32 (then subtract 1 to avoid hitting the max)
    int yAxisTopValue = ((int(maxOutputSeen) + 31) / 32) * 32 - 1;
    yAxisTopValue = max(yAxisTopValue, 127);  // Ensure minimum visible scale
    yAxisTopValue = min(yAxisTopValue, 255);  // Clamp to 255 for safety

    float scaleY = graphHeight / float(yAxisTopValue - minOutput);

    // --- Draw Y-axis Labels dynamically ---
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);

    char valueStr[4];
    for (int i = 0; i <= 8; ++i) {
        int value = (i > 0) ? (i * (yAxisTopValue + 1) / 8) - 1 : 0;
        int y = graphY + graphHeight - i * labelStepY;
        snprintf(valueStr, sizeof(valueStr), "%03d", value);
        tft.drawString(valueStr, labelX, y - 3);
    }

    // --- Draw Graph Container ---
    tft.fillRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_BLACK);
    tft.drawRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, TFT_WHITE);

    // --- Grid Lines ---
    for (int i = 1; i <= setGridGapX; i++) {
        int x = graphX + (graphWidth * i) / (setGridGapX + 1);
        tft.drawLine(x, graphY, x, graphY + graphHeight, TFT_DARKGREY);
    }
    for (int i = 1; i <= gridLinesY; ++i) {
        int y = graphY + (graphHeight * i) / 8;
        tft.drawLine(graphX, y, graphX + graphWidth, y, TFT_DARKGREY);
    }

    // --- Map value to Y coordinate dynamically ---
    auto mapToY = [&](float value) -> int {
        return graphY + graphHeight - clamp((value - minOutput) * scaleY, 0.0f, float(graphHeight));
    };

    float stepX = (visibleSize > 1) ? (float)graphWidth / (visibleSize - 1) : 1;

    // --- Plot PID Output and Temperature ---
    for (int i = startIndex + 1; i <= endIndex; ++i) {
        int idx0 = i - 1;
        int idx1 = i;

        float x0 = graphX + (idx0 - startIndex) * stepX;
        float x1 = graphX + (idx1 - startIndex) * stepX;

        // Output (CYAN)
        int y0_out = mapToY(pidOutputHistory[idx0]);
        int y1_out = mapToY(pidOutputHistory[idx1]);
        tft.drawLine(x0, y0_out, x1, y1_out, TFT_CYAN);

        // Input / Temperature (ORANGE)
        int y0_in = mapToY(tempHistory[idx0]);
        int y1_in = mapToY(tempHistory[idx1]);
        tft.drawLine(x0, y0_in, x1, y1_in, TFT_ORANGE);
    }

    // --- Draw Setpoint Line (MAGENTA) ---
    int setY = mapToY(setTemperature);
    tft.drawLine(graphX, setY, graphX + graphWidth, setY, TFT_PINK);

    // --- Dynamic Buttons / Graph Info Display ---
    tft.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    if (seeGraphInfo != lastSeeGraphInfo || firstEnter) {
        if (!seeGraphInfo) {
            tft.fillRect(closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 200, 240, 40, BACKGROUND_COLOR);

            clearButton(closeGraphInfoBtn);
            clearButton(togglePauseDOWNBtn);

            seeGraphInfoBtn.draw();
            incGridGapXBtn.draw(2);
            decGridGapXBtn.draw(2);
            resetDataCountBtn.draw();
            backBtn.draw();
        } else {
            clearButton(backBtn);
            clearButton(seeGraphInfoBtn);
            clearButton(incGridGapXBtn);
            clearButton(decGridGapXBtn);
            clearButton(resetDataCountBtn);

            closeGraphInfoBtn.draw();
            drawButtonWithText(&togglePauseDOWNBtn, pauseReading ? "|>" : "||", 2);
        }

        lastSeeGraphInfo = seeGraphInfo;
        firstEnter = false;
    }

    // --- PID Graph Text Info ---
    if (seeGraphInfo) {
        tft.setTextDatum(BL_DATUM);
        tft.setTextSize(1);

        char inputBuf[32], outputBuf[32], setpointBuf[32];
        snprintf(inputBuf, sizeof(inputBuf), "Input:    %.2f", currentTemperature);
        snprintf(outputBuf, sizeof(outputBuf), "Output:   %.2lf", Output);
        snprintf(setpointBuf, sizeof(setpointBuf), "Setpoint: %.2f", (float)setTemperature);

        tft.setTextColor(TFT_ORANGE, BACKGROUND_COLOR);
        tft.drawString(inputBuf, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 30);
        tft.setTextColor(TFT_CYAN, BACKGROUND_COLOR);
        tft.drawString(outputBuf, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 20);
        tft.setTextColor(TFT_PINK, BACKGROUND_COLOR);
        tft.drawString(setpointBuf, closeGraphInfoBtn.x + closeGraphInfoBtn.w + 10, 240 - 10);
    }
}

void changeScreen(ScreenState next) {
    previousScreen = currentScreen;
    currentScreen = next;

    resetAllButtons();  // Clear visual states
    tft.fillScreen(BACKGROUND_COLOR);
    switch (currentScreen) {
        case SCREEN_MAIN: drawMainScreen(); break;
        case SCREEN_TemperatureSetpoint: drawSetTemperatureScreen("Temperature Setpoint"); break;
        case SCREEN_TempGraph_MainOnly: drawTempGraph("LIVE Graph: Temperature", gridGapX[countGridGapXIndex]); break;
        case SCREEN_HumiGraph_MainOnly: drawHumiGraph("LIVE Graph: Humidity", gridGapX[countGridGapXIndex]); break;
        case SCREEN_Settings: drawSettingsScreen("Settings"); break;
        case SCREEN_WaterLevelSetpoint: drawWaterLevelSetpointScreen("\"Water\" Control Related"); break;
        case SCREEN_GraphConfiguration: drawGraphConfigurationScreen("Configure LIVE Graph"); break;
        case SCREEN_PIDGraph_MainOnly: drawPIDGraph("LIVE Graph: PID", gridGapX[countGridGapXIndex]); break;
        case SCREEN_DHT22IsNan: drawDHT22IsNanScreen(); break;
        default: /* optional fallback or logging */ break;
    }

    firstEnter = true;
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    tft.init();
    tft.setRotation(1);
    tft.setTextFont(1);

    Setpoint = setTemperature; // Target temperature in °C
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255); // Assuming 8-bit PWM for fan

    pinMode(WATER_SENSOR_PIN, INPUT);
    pinMode(WATER_ACTUATOR_IN1_PIN, OUTPUT);
    pinMode(WATER_ACTUATOR_ENA_PIN, OUTPUT);
    // pinMode(ACTUATOR_PIN, OUTPUT);
    digitalWrite(WATER_ACTUATOR_IN1_PIN, HIGH);
    // digitalWrite(ACTUATOR_PIN, LOW); // default OFF

    // 1. Water Pump
    ledcSetup(pwmChannel_WATERPUMP, pwmFrequency_WATERPUMP, resolution_WATERPUMP); // Setup PWM
    ledcAttachPin(WATER_ACTUATOR_ENA_PIN, pwmChannel_WATERPUMP);         // Attach pin to channel

    // 2. Fan Blower
    ledcSetup(pwmChannel_FANBLOWER, pwmFrequency_FANBLOWER, resolution_FANBLOWER); // Setup PWM
    ledcAttachPin(FANBLOWER_ACTUATOR_PIN, pwmChannel_FANBLOWER);         // Attach pin to channel

    changeScreen(SCREEN_MAIN);
}

char buf[32];
void loop() {
    uint16_t x, y;
    bool touched = tft.getTouch(&x, &y);
    y = 240 - y;

    // snprintf(buf, sizeof(buf), "%lf | %lf | %lf", Input, Output, Setpoint);
    // Serial.println(buf);

    unsigned long now = millis();
    int newSampleTimeMs = debounceDelay * multiplierSampleReadingTime;

    if (multiplierSampleReadingTime < 0.1) multiplierSampleReadingTime = 0.1;
    if (pauseReading) wasPaused = true;
    else {
        if (wasPaused) {
            lastGetData = now;
            wasPaused = false;
        }
    
        if (now - lastGetData > newSampleTimeMs) {
            float tempHumidity = dht.readHumidity();
            currentTemperature = dht.readTemperature();
            currentHumidity = (tempHumidity >= 99.9) ? 99.9 : tempHumidity;
    
            if (!isnan(currentTemperature) || !isnan(currentHumidity)) {
                if (!firstRun) {
                    minTemp = currentTemperature;
                    maxTemp = currentTemperature;
                    minHumi = currentHumidity;
                    maxHumi = currentHumidity;
                    firstRun = true;
                }

                updateHistory(currentTemperature, currentHumidity, Output);
                avgTemp = averageTemperature(tempHistory);
                avgHumi = averageHumidity(humiHistory);

                Input = currentTemperature;
                float tempDifference = abs(Input - Setpoint);
                static float lastOutput = 0;
                if (tempDifference < 0.2) Output = lastOutput;
                else {
                    myPID.Compute();
                    lastOutput = Output;
                }
                ledcWrite(pwmChannel_FANBLOWER, (int)Output);

                if (minTemp > currentTemperature) minTemp = currentTemperature;
                if (maxTemp < currentTemperature) maxTemp = currentTemperature;
                if (minHumi > currentHumidity) minHumi = currentHumidity;
                if (maxHumi < currentHumidity) maxHumi = currentHumidity;
    
                totalReadings++;
                totalTime += (now - lastGetData) / 1e3;
                dht22Connected = true;
                dht22ErrorHandled = false;
            } else {
                dht22Connected = false;
                if (!dht22ErrorHandled) {
                    changeScreen(SCREEN_DHT22IsNan);
                    dht22ErrorHandled = true;
                }
            }
    
            lastGetData = now;
        }
    }

    // Water level sensor module reading and mapping
    rawValue = analogRead(WATER_SENSOR_PIN);
    filtered = alpha * rawValue + (1 - alpha) * filtered;
    float norm = filtered / sensorMaxValue;
    float adjusted = pow(norm, 2.0);
    waterPercent = constrain((int)(adjusted * 100.0), 0, 100);
    if (waterPercent > waterSetpointPercent) {
        ledcWrite(pwmChannel_WATERPUMP, waterPumpPWMList[waterPumpSpeedIndex]);
    } else {
        ledcWrite(pwmChannel_WATERPUMP, 0);
    }

    if (currentScreen == SCREEN_TemperatureSetpoint) drawSetTemperatureScreen("Temperature Setpoint");
    else if (currentScreen == SCREEN_TempGraph_MainOnly) drawTempGraph("LIVE Graph: Temperature", gridGapX[countGridGapXIndex]);
    else if (currentScreen == SCREEN_HumiGraph_MainOnly) drawHumiGraph("LIVE Graph: Humidity", gridGapX[countGridGapXIndex]);
    else if (currentScreen == SCREEN_WaterLevelSetpoint) drawWaterLevelSetpointScreen("\"Water\" Control Related");
    else if (currentScreen == SCREEN_GraphConfiguration) drawGraphConfigurationScreen("Configure LIVE Graph");
    else if (currentScreen == SCREEN_PIDGraph_MainOnly) drawPIDGraph("LIVE Graph: PID", gridGapX[countGridGapXIndex]);
    else if (currentScreen == SCREEN_DHT22IsNan && dht22Connected) changeScreen(previousScreen);

    if (touched) {
        // Only handle input if finger was previously lifted and debounce time passed
        if (touchReleased && now - lastTouchTime > debounceDelay) {
        touchReleased = false;
        lastTouchTime = now;

        switch (currentScreen) {
            case SCREEN_MAIN:
                mainBtn.setInverted(mainBtn.contains(x, y), 2);
                settingsBtn.setInverted(settingsBtn.contains(x, y), 2);
                break;
            case SCREEN_TemperatureSetpoint:
                graphBtn.setInverted(graphBtn.contains(x, y));
                incTempBtn.setInverted(incTempBtn.contains(x, y), 2);
                decTempBtn.setInverted(decTempBtn.contains(x, y), 2);
                backBtn.setInverted(backBtn.contains(x, y));
                break;
            case SCREEN_TempGraph_MainOnly:
                if (!seeGraphInfo) {
                    showHumiGraphBtn.setInverted(showHumiGraphBtn.contains(x, y));
                    seeGraphInfoBtn.setInverted(seeGraphInfoBtn.contains(x, y));
                    incGridGapXBtn.setInverted(incGridGapXBtn.contains(x, y), 2);
                    decGridGapXBtn.setInverted(decGridGapXBtn.contains(x, y), 2);
                    resetDataCountBtn.setInverted(resetDataCountBtn.contains(x, y));
                    backBtn.setInverted(backBtn.contains(x, y));
                } else {
                    closeGraphInfoBtn.setInverted(closeGraphInfoBtn.contains(x, y));
                    togglePauseUPBtn.setInverted(togglePauseUPBtn.contains(x, y), 2);
                }
                break;
            case SCREEN_HumiGraph_MainOnly:
                if (!seeGraphInfo) {
                    showTempGraphBtn.setInverted(showTempGraphBtn.contains(x, y));
                    seeGraphInfoBtn.setInverted(seeGraphInfoBtn.contains(x, y));
                    incGridGapXBtn.setInverted(incGridGapXBtn.contains(x, y), 2);
                    decGridGapXBtn.setInverted(decGridGapXBtn.contains(x, y), 2);
                    resetDataCountBtn.setInverted(resetDataCountBtn.contains(x, y));
                    backBtn.setInverted(backBtn.contains(x, y));
                } else {
                    closeGraphInfoBtn.setInverted(closeGraphInfoBtn.contains(x, y));
                    togglePauseUPBtn.setInverted(togglePauseUPBtn.contains(x, y), 2);
                }
                break;
            case SCREEN_Settings:
                waterLevelSetpointBtn.setInverted(waterLevelSetpointBtn.contains(x, y), 2);
                graphRelatedSetpointBtn.setInverted(graphRelatedSetpointBtn.contains(x, y), 2);
                backBtn.setInverted(backBtn.contains(x, y));
                break;
            case SCREEN_WaterLevelSetpoint:
                incWLSBtn.setInverted(incWLSBtn.contains(x, y), 2);
                decWLSBtn.setInverted(decWLSBtn.contains(x, y), 2);
                incWSPWMBtn.setInverted(incWSPWMBtn.contains(x, y), 2);
                decWSPWMBtn.setInverted(decWSPWMBtn.contains(x, y), 2);
                backBtn.setInverted(backBtn.contains(x, y));
                break;
            case SCREEN_GraphConfiguration:
                btnIntUp.setInverted(btnIntUp.contains(x, y), 2);
                btnFracUp.setInverted(btnFracUp.contains(x, y), 2);
                btnIntDown.setInverted(btnIntDown.contains(x, y), 2);
                btnFracDown.setInverted(btnFracDown.contains(x, y), 2);
                showPIDGraphInfoBtn.setInverted(showPIDGraphInfoBtn.contains(x, y));
                backBtn.setInverted(backBtn.contains(x, y));
                break;
            case SCREEN_PIDGraph_MainOnly:
                if (!seeGraphInfo) {
                    seeGraphInfoBtn.setInverted(seeGraphInfoBtn.contains(x, y));
                    incGridGapXBtn.setInverted(incGridGapXBtn.contains(x, y), 2);
                    decGridGapXBtn.setInverted(decGridGapXBtn.contains(x, y), 2);
                    resetDataCountBtn.setInverted(resetDataCountBtn.contains(x, y));
                    backBtn.setInverted(backBtn.contains(x, y));
                } else {
                    closeGraphInfoBtn.setInverted(closeGraphInfoBtn.contains(x, y));
                    togglePauseDOWNBtn.setInverted(togglePauseDOWNBtn.contains(x, y), 2);
                }
                break;
            default:
                backBtn.setInverted(backBtn.contains(x, y));
                break;
            }
        }
    } else {
        if (!touchReleased) {
            // Finger was lifted — process click action
            touchReleased = true;
            lastTouchTime = now;

            // Detect actions based on previously pressed buttons
            if (currentScreen == SCREEN_MAIN) {
                if (mainBtn.isInverted) changeScreen(SCREEN_TemperatureSetpoint);
                if (settingsBtn.isInverted) changeScreen(SCREEN_Settings);
            } else if (currentScreen == SCREEN_Settings) {
                if (waterLevelSetpointBtn.isInverted) changeScreen(SCREEN_WaterLevelSetpoint);
                if (graphRelatedSetpointBtn.isInverted) changeScreen(SCREEN_GraphConfiguration);
                if (backBtn.isInverted) changeScreen(SCREEN_MAIN);
            } else {
                if (currentScreen == SCREEN_TemperatureSetpoint && graphBtn.isInverted) changeScreen(SCREEN_TempGraph_MainOnly);
                if (currentScreen == SCREEN_TemperatureSetpoint) {
                    if (incTempBtn.isInverted && setTemperature < MAX_TEMP) setTemperature++;
                    if (decTempBtn.isInverted && setTemperature > MIN_TEMP) setTemperature--;
                    Setpoint = setTemperature; // Target temperature in °C
                }
                
                if (currentScreen == SCREEN_TempGraph_MainOnly && showHumiGraphBtn.isInverted) {
                    seeGraphInfo = false;
                    changeScreen(SCREEN_HumiGraph_MainOnly);
                } if (currentScreen == SCREEN_HumiGraph_MainOnly && showTempGraphBtn.isInverted) {
                    seeGraphInfo = false;
                    changeScreen(SCREEN_TempGraph_MainOnly);
                }
                
                if ((currentScreen == SCREEN_TempGraph_MainOnly || currentScreen == SCREEN_HumiGraph_MainOnly) && seeGraphInfo && togglePauseUPBtn.isInverted) {
                    pauseReading = !pauseReading;
                    drawButtonWithText(&togglePauseUPBtn, pauseReading ? "|>" : "||", 2);
                } if (currentScreen == SCREEN_PIDGraph_MainOnly && seeGraphInfo && togglePauseDOWNBtn.isInverted) {
                    pauseReading = !pauseReading;
                    drawButtonWithText(&togglePauseDOWNBtn, pauseReading ? "|>" : "||", 2);
                } if ((currentScreen == SCREEN_TempGraph_MainOnly || currentScreen == SCREEN_HumiGraph_MainOnly || currentScreen == SCREEN_PIDGraph_MainOnly) && !seeGraphInfo && seeGraphInfoBtn.isInverted) {
                    seeGraphInfo = true;
                } if ((currentScreen == SCREEN_TempGraph_MainOnly || currentScreen == SCREEN_HumiGraph_MainOnly || currentScreen == SCREEN_PIDGraph_MainOnly) && seeGraphInfo && closeGraphInfoBtn.isInverted) {
                    seeGraphInfo = false;
                }
                
                if ((currentScreen == SCREEN_TempGraph_MainOnly || currentScreen == SCREEN_HumiGraph_MainOnly || currentScreen == SCREEN_PIDGraph_MainOnly) && !seeGraphInfo) {
                    if (incGridGapXBtn.isInverted) {
                        if (countGridGapXIndex < 6 && countHistorySizeIndex < 6 && countGridGapXIndex == countHistorySizeIndex) {
                            countGridGapXIndex++;
                            countHistorySizeIndex++;
                        }
                    }
                    if (decGridGapXBtn.isInverted) {
                        if (countGridGapXIndex > 0 && countHistorySizeIndex > 0 && countGridGapXIndex == countHistorySizeIndex) {
                            countGridGapXIndex--;
                            countHistorySizeIndex--;
                        }
                    }
                    if (resetDataCountBtn.isInverted) {
                        Output = 0;
                        myPID.SetMode(MANUAL);      // Stop PID temporarily
                        Input = currentTemperature; // or whatever your latest input is
                        myPID.SetMode(AUTOMATIC);   // Restart PID fresh

                        dataCount = 0;
                        countGridGapXIndex = 5;
                        countHistorySizeIndex = 5;
                        totalReadings = 0ULL;
                        totalTime = 0.0f;
                    }
                }

                if (currentScreen == SCREEN_WaterLevelSetpoint) {
                    if (incWLSBtn.isInverted && waterSetpointPercent < waterMaxPercent) waterSetpointPercent += 1.0;
                    if (decWLSBtn.isInverted && waterSetpointPercent > waterMinPercent) waterSetpointPercent -= 1.0;
                    if (incWSPWMBtn.isInverted && waterPumpSpeedIndex < 4) waterPumpSpeedIndex++;
                    if (decWSPWMBtn.isInverted && waterPumpSpeedIndex > 0) waterPumpSpeedIndex--;
                }

                if (currentScreen == SCREEN_GraphConfiguration) {
                    if (showPIDGraphInfoBtn.isInverted) changeScreen(SCREEN_PIDGraph_MainOnly);

                    // INT UP
                    if (btnIntUp.isInverted) {
                        if (intPart < 9) intPart++;
                    }

                    // INT DOWN — ensures minimum value is 0.1
                    if (btnIntDown.isInverted) {
                        if (intPart > 1) {
                            intPart--;
                        } else if (intPart == 1 && fracPart == 0) {
                            intPart = 0;
                            fracPart = 1;
                        } else if (intPart == 1 && fracPart >= 1) {
                            intPart = 0;
                        } else if (intPart == 0 && fracPart > 0) {
                            fracPart = 1;
                        }
                    }

                    // FRAC UP
                    if (btnFracUp.isInverted) {
                        if (fracPart < 9) {
                            fracPart++;
                        } else {
                            fracPart = 0;
                            if (intPart < 9) intPart++;
                        }
                    }

                    // FRAC DOWN — ensures minimum is 0.1
                    if (btnFracDown.isInverted) {
                        if (intPart == 0 && fracPart == 1) {
                            // Prevent going below 0.1
                        } else if (fracPart > 0) {
                            fracPart--;
                        } else {
                            if (intPart > 0) {
                                intPart--;
                                fracPart = 9;
                            }
                        }
                    }

                    multiplierSampleReadingTime = intPart + fracPart * 0.1;

                    Output = 0;
                    myPID.SetMode(MANUAL);      // Stop PID temporarily
                    Input = currentTemperature; // or whatever your latest input is
                    myPID.SetMode(AUTOMATIC);   // Restart PID fresh
                    myPID.SetSampleTime(multiplierSampleReadingTime);
                }

                if ((currentScreen == SCREEN_WaterLevelSetpoint || currentScreen == SCREEN_GraphConfiguration) && backBtn.isInverted) changeScreen(SCREEN_Settings);
                if (currentScreen == SCREEN_PIDGraph_MainOnly && backBtn.isInverted) changeScreen(SCREEN_GraphConfiguration);
                if (currentScreen == SCREEN_TemperatureSetpoint && backBtn.isInverted) changeScreen(SCREEN_MAIN);
                if ((currentScreen == SCREEN_TempGraph_MainOnly || currentScreen == SCREEN_HumiGraph_MainOnly) && backBtn.isInverted) changeScreen(SCREEN_TemperatureSetpoint);
            }

            // Reset all buttons' visual state after touch release
            resetAllButtons();
        }
    }
}
