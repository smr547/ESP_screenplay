#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <stdio.h>

#define CMD_X 0b10010000   // X position
#define CMD_Y 0b11010000   // Y position
#define CMD_Z1 0b10110000  // Z1 (pressure-related)
#define CMD_Z2 0b11000000  // Z2 (optional if you want fancier pressure)

#define PENIRQ_EN 26

uint32_t screenWidth, screenHeight;
volatile bool touchArmed = true;  // re-armed when pen lifted
volatile bool penDownFlag = false;

void disablePen_IRQ() {
    digitalWrite(PENIRQ_EN, LOW);
    detachInterrupt(digitalPinToInterrupt(TOUCH_IRQ));
    touchArmed = false;
}

void IRAM_ATTR penIRQ_ISR() {
    disablePen_IRQ();
    penDownFlag = true;
}

void enablePen_IRQ() {
    digitalWrite(PENIRQ_EN, HIGH);
    touchArmed = true;
    attachInterrupt(digitalPinToInterrupt(TOUCH_IRQ), penIRQ_ISR, FALLING);
}

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h;

#define TFT_GREY 0xBDF7
#define VERY_LIGHT_GREY 0xF7BE

// TFT and XPT2046 share the same bus but operate at different rates

SPISettings tftSPI(40000000, MSBFIRST, SPI_MODE0);  // example
SPISettings touchSPI(100000, MSBFIRST, SPI_MODE0);  // conservative & clean

uint16_t xpt2046_read(uint8_t cmd) {
    uint16_t result = 0;

    tft.getSPIinstance().beginTransaction(touchSPI);
    digitalWrite(TOUCH_CS, LOW);

    tft.getSPIinstance().transfer(cmd);
    uint8_t hi = tft.getSPIinstance().transfer(0x00);
    uint8_t lo = tft.getSPIinstance().transfer(0x00);

    digitalWrite(TOUCH_CS, HIGH);
    tft.getSPIinstance().endTransaction();

    Serial.println(cmd, HEX);
    Serial.println(hi, BIN);
    Serial.println(lo, BIN);

    result = ((hi << 8) | lo) >> 4;  // 12-bit value

    Serial.println(result, HEX);
    return result & 0x0FFF;
}

// Assume global TFT object, e.g. TFT_eSPI tft;

class Tile {
   public:
    Tile() = default;  // needed for array
    Tile(int16_t x0, int16_t y0, int16_t w0, int16_t h0)
        : x{x0}, y{y0}, w{w0}, h{h0} {}

    void init(int16_t x0, int16_t y0, int16_t w0, int16_t h0) {
        x = x0;
        y = y0;
        w = w0;
        h = h0;
        covered_ = true;
    }

    void redraw() {
        if (covered_) {
            uint32_t b = TILE_BEVEL;
            tft.fillRect(x + b, y + b, w - 2 * b, h - 2 * b, TFT_GREY);
            // tft.drawLine(x0_ + w_, y0_, x0_ + w_, y0_ + h_, TFT_BLACK);
            // tft.drawLine(x0_, y0_ + h_, x0_ + w_, y0_ + h_, TFT_BLACK);
            // tft.drawRect(x0_, y0_, w_, h_, TFT_BLACK);

            // draw the bevels
            tft.fillRect(x, y + b, b, h - b, TFT_DARKGREY);   // LHS
            tft.fillRect(x, y + h - b, w, b, TFT_DARKGREY);   // bottom
            tft.fillRect(x + b, y, w - b, b, TFT_WHITE);      // top
            tft.fillRect(x + w - b, y, b, h - b, TFT_WHITE);  // RHS
            tft.fillTriangle(x, y, x + b, y, x + b, y + b,
                             TFT_WHITE);  // top LHC
            tft.fillTriangle(x + w - b, y + h - b, x + w, y + h - b, x + w,
                             y + h,
                             TFT_WHITE);  // bottom RHC
        } else {
            // uncovered
            tft.fillRect(x, y, w, h, VERY_LIGHT_GREY);
            tft.drawLine(x, y, x + w, y, TFT_LIGHTGREY);          // top
            tft.drawLine(x, y + h, x + w, y + h, TFT_LIGHTGREY);  // bollom
            tft.drawLine(x, y, x, y + h, TFT_LIGHTGREY);          // LHS
            tft.drawLine(x + w, y, x + w, y + h, TFT_LIGHTGREY);
        }
    }

    void toggle() {
        covered_ = !covered_;
        redraw();
    }

    bool isCovered() const { return covered_; }

   private:
    int16_t x{0};
    int16_t y{0};
    int16_t w{0};
    int16_t h{0};
    bool covered_{true};
};

// array of tiles represents the screen

// #define TILES_X 16
// #define TILES_Y 12

constexpr int16_t tilesX = TILES_X;
constexpr int16_t tilesY = TILES_Y;

Tile tiles[tilesY][tilesX];

// ---- Simple touch gating: one toggle per pen-down ----
void handleTouch(uint16_t rawX, uint16_t rawY) {
    // Gate repeated toggles while the pen is held down
    // (release is detected when PENIRQ goes HIGH again)

    // static uint32_t lastTouchMs = 0;

    Serial.print("handleTouch at x=");
    Serial.print(rawX);
    Serial.print(", y=");
    Serial.print(rawY);
    Serial.println("");

    // If pen lifted, re-arm and exit (nothing to do)
    if (digitalRead(TOUCH_IRQ) == HIGH) {
        Serial.println(
            "Pen lifted while in handleTouch() -- ignoring interrupt");
        enablePen_IRQ();
        return;
    }

    // Optional: debounce the IRQ / panel contact a bit
    // uint32_t now = millis();
    /*
    if (!touchArmed) {
        Serial.println("pen down already handled, ignoring");
        return;  // already handled this press
    }

    if (now - lastTouchMs < 60) {
        Serial.println("bounce of pen down ignored");
        return;  // simple debounce window (tune)
    }
    */

    // Sanity check raw range (reject obvious noise / misreads)
    // if (rawX < 50 || rawX > 4095 || rawY < 50 || rawY > 4095) {
    //  return;
    //}

    // Map raw ADC -> screen coordinates
    // Note: Depending on your wiring/orientation, X/Y may be swapped or
    // inverted. Start with this and adjust later.
    int sx = (int)(((int32_t)(rawX - X_MIN) * screenWidth) / (X_MAX - X_MIN));
    int sy = (int)(((int32_t)(rawY - Y_MIN) * screenHeight) / (Y_MAX - Y_MIN));

    // Clamp
    if (sx < 0) sx = 0;
    if (sx >= screenWidth) sx = screenWidth - 1;
    if (sy < 0) sy = 0;
    if (sy >= screenHeight) sy = screenHeight - 1;

    // Convert to tile coordinates
    int tx = sx / TILE_SIZE;
    int ty = sy / TILE_SIZE;

    // Clamp tile indices (in case of rounding edge cases)
    if (tx < 0) tx = 0;
    if (tx >= TILES_X) tx = TILES_X - 1;
    if (ty < 0) ty = 0;
    if (ty >= TILES_Y) ty = TILES_Y - 1;

    // Toggle + redraw exactly one tile
    tiles[ty][tx].toggle();

    // Disarm until pen-up
    // lastTouchMs = now;
    enablePen_IRQ();
}

void setup() {
    Serial.begin(115200);
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);

    tft.init();
    uint8_t rotation = TFT_ROTATION;
    tft.setRotation(rotation);

    screenWidth = tft.width();
    screenHeight = tft.height();

    int maxRow = int(screenHeight / TILE_SIZE);
    int maxCol = int(screenWidth / TILE_SIZE);

    for (int row = 0; row < maxRow; row++) {
        for (int col = 0; col < maxCol; col++) {
            tiles[row][col].init(col * TILE_SIZE, row * TILE_SIZE, TILE_SIZE,
                                 TILE_SIZE);
            tiles[row][col].redraw();
        }
    }

    // add the pen interrupt handler
    pinMode(TOUCH_IRQ, INPUT_PULLUP);
    pinMode(PENIRQ_EN, OUTPUT);
    enablePen_IRQ();
}

void loop() {
    // 1. Touch event?
    if (penDownFlag) {
        Serial.println("pen down event");
        penDownFlag = false;

        if (digitalRead(TOUCH_IRQ) == LOW) {      // still touching
            uint16_t rawX = xpt2046_read(CMD_X);  // 0x90
            uint16_t rawY = xpt2046_read(CMD_Y);  // 0xD0

            // Map raw -> screen -> tile
            handleTouch(rawX, rawY);
        } else {
            Serial.println("pen down no longer present -- ignore interrupt");
            enablePen_IRQ();
        }
        /*
            } else {
                if (digitalRead(TOUCH_IRQ) == HIGH) {
                    enablePen_IRQ();
                }
                */
    }
}
