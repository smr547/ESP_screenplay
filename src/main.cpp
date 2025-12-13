#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <stdio.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7
#define VERY_LIGHT_GREY 0xF7BE

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

void setup() {
    Serial.begin(115200);

    tft.init();
    uint8_t rotation = TFT_ROTATION;
    tft.setRotation(rotation);

    int maxRow = int(tft.height() / TILE_SIZE);
    int maxCol = int(tft.width() / TILE_SIZE);

    for (int row = 0; row < maxRow; row++) {
        for (int col = 0; col < maxCol; col++) {
            tiles[row][col].init(col * TILE_SIZE, row * TILE_SIZE, TILE_SIZE,
                                 TILE_SIZE);
            tiles[row][col].redraw();
        }
    }

    randomSeed(analogRead(A0));
}

void loop() {
    // put your main code here, to run repeatedly:
    int maxRow = int(tft.height() / TILE_SIZE);
    int maxCol = int(tft.width() / TILE_SIZE);

    int row = random(maxRow);
    int col = random(maxCol);

    tiles[row][col].toggle();

    delay(200);
}
