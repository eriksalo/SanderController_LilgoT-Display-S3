// Belt Sander VFD Controller — LilyGO T-Display-S3
// Two-button speed control with bar graph display

#include <Arduino.h>
#include <TFT_eSPI.h>

// ── Pin assignments ────────────────────────────────────────────
constexpr uint8_t BTN_UP    = 21;   // short = speed up, long = motor ON
constexpr uint8_t BTN_DOWN  = 16;   // short = speed down, long = motor OFF
constexpr uint8_t PWM_OUT   = 17;   // LEDC PWM → RC filter → VFD analog in
constexpr uint8_t LCD_POWER = 15;   // HIGH to enable display
constexpr uint8_t BACKLIGHT = 38;   // HIGH to enable backlight

// ── Timing constants (ms) ──────────────────────────────────────
constexpr uint32_t DEBOUNCE_MS   = 50;
constexpr uint32_t LONG_PRESS_MS = 800;

// ── Speed / PWM ────────────────────────────────────────────────
constexpr int SPEED_MIN   = 1;
constexpr int SPEED_MAX   = 10;
constexpr int SPEED_INIT  = 5;
constexpr uint32_t PWM_FREQ = 5000;   // 5 kHz
constexpr uint8_t  PWM_RES  = 12;     // 12-bit → 0-4095

// ── Display geometry (170×320 portrait, rotation 0) ────────────
constexpr int SCREEN_W = 170;
constexpr int SCREEN_H = 320;

constexpr int BAR_X      = 30;               // left edge of bars
constexpr int BAR_W      = 130;              // bar width
constexpr int BAR_H      = 20;               // bar height
constexpr int BAR_GAP    = 2;                // gap between bars
constexpr int BAR_BOTTOM = 298;              // y of bottom edge of segment 1
constexpr int BAR_STEP   = BAR_H + BAR_GAP;  // vertical pitch

constexpr int STATUS_Y   = 8;                // top status line y
constexpr int SPEED_NUM_X = 70;              // large speed number x
constexpr int SPEED_NUM_Y = 30;              // large speed number y
constexpr int BADGE_X    = 120;              // ON/OFF badge x
constexpr int BADGE_Y    = 8;                // ON/OFF badge y
constexpr int BADGE_W    = 44;
constexpr int BADGE_H    = 22;

// ── Colors ─────────────────────────────────────────────────────
constexpr uint16_t COL_BG       = TFT_BLACK;
constexpr uint16_t COL_BAR_ON   = TFT_GREEN;
constexpr uint16_t COL_BAR_OFF  = 0x2104;    // dark gray
constexpr uint16_t COL_LABEL    = TFT_WHITE;
constexpr uint16_t COL_ON_BG    = TFT_GREEN;
constexpr uint16_t COL_ON_FG    = TFT_BLACK;
constexpr uint16_t COL_OFF_FG   = TFT_RED;

// ── Button event types ─────────────────────────────────────────
enum class BtnEvent { NONE, SHORT, LONG };

// ── Button state ───────────────────────────────────────────────
struct Button {
    uint8_t   pin;
    bool      lastReading   = true;   // pull-up → HIGH idle
    bool      stable        = true;
    uint32_t  debounceTime  = 0;
    bool      pressed       = false;
    uint32_t  pressStart    = 0;
    bool      longFired     = false;
};

// ── Global objects / state ─────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();

Button btnUp   { BTN_UP };
Button btnDown { BTN_DOWN };

bool motorOn      = false;
int  speedLevel   = SPEED_INIT;
bool displayDirty = true;
bool fullRedraw   = true;         // force complete screen repaint

// ── Previous display state (for partial redraws) ───────────────
int  prevSpeed    = -1;
bool prevMotorOn  = false;

// ── readButton: debounce + detect short/long press ─────────────
BtnEvent readButton(Button &b) {
    bool reading = digitalRead(b.pin);
    uint32_t now = millis();

    // Debounce
    if (reading != b.lastReading) {
        b.debounceTime = now;
    }
    b.lastReading = reading;

    if ((now - b.debounceTime) < DEBOUNCE_MS) return BtnEvent::NONE;

    bool newStable = reading;

    // Falling edge → press start
    if (b.stable && !newStable) {
        b.pressed   = true;
        b.pressStart = now;
        b.longFired  = false;
    }

    // While held → check long press
    if (b.pressed && !newStable && !b.longFired) {
        if ((now - b.pressStart) >= LONG_PRESS_MS) {
            b.longFired = true;
            b.stable = newStable;
            return BtnEvent::LONG;
        }
    }

    // Rising edge → release
    if (!b.stable && newStable && b.pressed) {
        b.pressed = false;
        b.stable  = newStable;
        if (!b.longFired) return BtnEvent::SHORT;
        return BtnEvent::NONE;
    }

    b.stable = newStable;
    return BtnEvent::NONE;
}

// ── handleButtons: map events to state changes ─────────────────
void handleButtons() {
    BtnEvent upEvt   = readButton(btnUp);
    BtnEvent downEvt = readButton(btnDown);

    if (upEvt == BtnEvent::LONG) {
        motorOn = true;
        displayDirty = true;
        Serial.println("BTN_UP long  → Motor ON");
    } else if (upEvt == BtnEvent::SHORT && motorOn) {
        if (speedLevel < SPEED_MAX) {
            speedLevel++;
            displayDirty = true;
            Serial.printf("BTN_UP short → Speed %d\n", speedLevel);
        }
    }

    if (downEvt == BtnEvent::LONG) {
        motorOn = false;
        displayDirty = true;
        Serial.println("BTN_DOWN long → Motor OFF");
    } else if (downEvt == BtnEvent::SHORT && motorOn) {
        if (speedLevel > SPEED_MIN) {
            speedLevel--;
            displayDirty = true;
            Serial.printf("BTN_DOWN short → Speed %d\n", speedLevel);
        }
    }
}

// ── updatePWM: output duty cycle based on motor state + speed ──
void updatePWM() {
    if (!motorOn) {
        ledcWrite(PWM_OUT, 0);
        return;
    }
    // Map speed 1-10 → duty 0-4095 linearly
    uint32_t duty = map(speedLevel, SPEED_MIN, SPEED_MAX, 0, 4095);
    ledcWrite(PWM_OUT, duty);
}

// ── drawSplash: startup screen ─────────────────────────────────
void drawSplash() {
    tft.fillScreen(COL_BG);
    tft.setTextDatum(TC_DATUM);

    tft.setTextColor(TFT_CYAN, COL_BG);
    tft.setTextSize(3);
    tft.drawString("Erik Salo", SCREEN_W / 2, 120);

    tft.setTextColor(TFT_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.drawString("Porter Cable G8", SCREEN_W / 2, 165);
    tft.drawString("Controller", SCREEN_W / 2, 180);

    delay(2000);
}

// ── drawOnOffIndicator: green filled "ON" or red outline "OFF" ─
void drawOnOffIndicator() {
    // Clear badge area
    tft.fillRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, COL_BG);

    if (motorOn) {
        tft.fillRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 4, COL_ON_BG);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(COL_ON_FG, COL_ON_BG);
        tft.setTextSize(1);
        tft.drawString("ON", BADGE_X + BADGE_W / 2, BADGE_Y + BADGE_H / 2);
    } else {
        tft.drawRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 4, COL_OFF_FG);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(COL_OFF_FG, COL_BG);
        tft.setTextSize(1);
        tft.drawString("OFF", BADGE_X + BADGE_W / 2, BADGE_Y + BADGE_H / 2);
    }
}

// ── drawSpeedNumber: large speed digit in status area ──────────
void drawSpeedNumber() {
    // Clear area behind number
    tft.fillRect(SPEED_NUM_X - 20, SPEED_NUM_Y, 50, 30, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.setTextSize(3);
    tft.drawString(String(speedLevel), SPEED_NUM_X, SPEED_NUM_Y);
}

// ── drawBarGraph: 10 horizontal segments, filled from bottom ───
void drawBarGraph() {
    for (int i = 1; i <= SPEED_MAX; i++) {
        int segY = BAR_BOTTOM - (i - 1) * BAR_STEP;
        bool active = motorOn && (i <= speedLevel);
        uint16_t col = active ? COL_BAR_ON : COL_BAR_OFF;

        tft.fillRect(BAR_X, segY - BAR_H, BAR_W, BAR_H, col);

        // Label on the left
        tft.setTextDatum(MR_DATUM);
        tft.setTextColor(COL_LABEL, COL_BG);
        tft.setTextSize(1);
        tft.drawString(String(i), BAR_X - 4, segY - BAR_H / 2);
    }
}

// ── drawMainScreen: full repaint ───────────────────────────────
void drawMainScreen() {
    tft.fillScreen(COL_BG);

    // "SPEED" label
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.setTextSize(2);
    tft.drawString("SPD", 4, STATUS_Y);

    drawSpeedNumber();
    drawOnOffIndicator();
    drawBarGraph();

    prevSpeed   = speedLevel;
    prevMotorOn = motorOn;
}

// ── updateDisplay: partial redraws via dirty flag ──────────────
void updateDisplay() {
    if (!displayDirty) return;
    displayDirty = false;

    if (fullRedraw) {
        fullRedraw = false;
        drawMainScreen();
        return;
    }

    // Partial updates
    if (speedLevel != prevSpeed) {
        drawSpeedNumber();
        drawBarGraph();
        prevSpeed = speedLevel;
    }
    if (motorOn != prevMotorOn) {
        drawOnOffIndicator();
        drawBarGraph();        // bar colors depend on motorOn
        prevMotorOn = motorOn;
    }
}

// ── setup ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n== Belt Sander Controller ==");

    // Buttons
    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    // Display power
    pinMode(LCD_POWER, OUTPUT);
    digitalWrite(LCD_POWER, HIGH);
    pinMode(BACKLIGHT, OUTPUT);
    digitalWrite(BACKLIGHT, HIGH);

    // PWM output (ESP32 Arduino v3 API)
    ledcAttach(PWM_OUT, PWM_FREQ, PWM_RES);
    ledcWrite(PWM_OUT, 0);

    // TFT
    tft.init();
    tft.setRotation(0);   // portrait 170×320
    tft.fillScreen(COL_BG);

    drawSplash();
    drawMainScreen();

    displayDirty = false;
    fullRedraw   = false;

    Serial.printf("Ready — speed=%d motor=%s\n", speedLevel, motorOn ? "ON" : "OFF");
}

// ── loop ───────────────────────────────────────────────────────
void loop() {
    handleButtons();
    updatePWM();
    updateDisplay();
}
