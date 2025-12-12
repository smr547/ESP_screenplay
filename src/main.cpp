#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <stdio.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7

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
        on_ = false;
    }

    void redraw() {
        uint32_t b = TILE_BEVEL;
        uint16_t color = on_ ? TFT_GREY : TFT_DARKGREY;
        tft.fillRect(x + b, y + b, w - 2 * b, h - 2 * b, TFT_GREY);
        // tft.drawLine(x0_ + w_, y0_, x0_ + w_, y0_ + h_, TFT_BLACK);
        // tft.drawLine(x0_, y0_ + h_, x0_ + w_, y0_ + h_, TFT_BLACK);
        // tft.drawRect(x0_, y0_, w_, h_, TFT_BLACK);

        // draw the bevels
        tft.fillRect(x, y + b, b, h - b, TFT_DARKGREY);  // LHS
        tft.fillRect(x, y + h - b, w, b, TFT_DARKGREY);  // bottom
        tft.fillRect(x + b, y, w - b, b, TFT_WHITE);     // top

        tft.fillRect(x + w - b, y, b, h - b, TFT_WHITE);  // RHS

        tft.fillTriangle(x, y, x + b, y, x + b, y + b, TFT_WHITE);  // top LHC
        tft.fillTriangle(x + w - b, y + h - b, x + w, y + h - b, x + w, y + h,
                         TFT_WHITE);  // bottom RHC
    }

    void toggle() {
        on_ = !on_;
        redraw();
    }

    bool isOn() const { return on_; }

   private:
    int16_t x{0};
    int16_t y{0};
    int16_t w{0};
    int16_t h{0};
    bool on_{false};
};

// array of tiles represents the screen

#define TILES_X 16
#define TILES_Y 12

constexpr int16_t tilesX = TILES_X;
constexpr int16_t tilesY = TILES_Y;

Tile tiles[tilesY][tilesX];

void setup() {
    Serial.begin(115200);

    tft.init();
    uint8_t rotation = TFT_ROTATION;
    tft.setRotation(rotation);
}

void loop() {
    // put your main code here, to run repeatedly:

    int maxRow = int(tft.height() / TILE_SIZE);
    int maxCol = int(tft.width() / TILE_SIZE);

    for (int row = 0; row < maxRow; row++) {
        for (int col = 0; col < maxCol; col++) {
            tiles[row][col].init(col * TILE_SIZE, row * TILE_SIZE, TILE_SIZE,
                                 TILE_SIZE);
            tiles[row][col].redraw();
        }
    }
    /*
        char buf[64];
        snprintf(buf, sizeof(buf), "Rotation=%d\n", tft.getRotation());
        Serial.print(buf);
        snprintf(buf, sizeof(buf), "width=%d\n", tft.width());
        Serial.print(buf);
        snprintf(buf, sizeof(buf), "height=%d\n", tft.height());
        Serial.print(buf);

    tft.fillScreen(TFT_GREY);

    // Set "cursor" at top left corner of display (0,0) and select font 2
    // (cursor will move to next line automatically during printing with
    // 'tft.println'
    //  or stay on the line is there is room for the text with tft.print)
    tft.setCursor(0, 0, 2);
    // Set the font colour to be white with a black background, set text size
    // multiplier to 1
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);

    // line at top of screen

    int32_t x0, x1, x2, y0, y1, y2;

    // top

    y1 = y2 = 5;
    y0 = x2 = 0;
    x1 = 5;

    for (x0 = 2; x0 < tft.width(); x0 += 5) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
        x1 += 5;
        x2 += 5;
    }

    // bottom

    x2 = 5;
    y1 = y2 = tft.height() - 5;
    y0 = tft.height() - 1;
    x1 = 0;

    for (x0 = 2; x0 < tft.width(); x0 += 5) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_BLUE);
        x1 += 5;
        x2 += 5;
    }

    // LHS

    x1 = x2 = 5;
    x0 = y1 = 0;
    y2 = 5;

    for (y0 = 2; y0 < tft.height(); y0 += 5) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
        y1 += 5;
        y2 += 5;
    }

    // RHS

    x0 = tft.width() - 1;
    x1 = x2 = x0 - 5;
    y1 = 0;
    y2 = 5;

    for (y0 = 2; y0 < tft.height(); y0 += 5) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_CYAN);
        y1 += 5;
        y2 += 5;
    }
*/
    delay(10000);
}
