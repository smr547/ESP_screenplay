#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <stdio.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7

void setup() {
    Serial.begin(115200);

    tft.init();
    uint8_t rotation = TFT_ROTATION;
    tft.setRotation(rotation);
}

void loop() {
    // put your main code here, to run repeatedly:
    /*
        char buf[64];
        snprintf(buf, sizeof(buf), "Rotation=%d\n", tft.getRotation());
        Serial.print(buf);
        snprintf(buf, sizeof(buf), "width=%d\n", tft.width());
        Serial.print(buf);
        snprintf(buf, sizeof(buf), "height=%d\n", tft.height());
        Serial.print(buf);
        */
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

    delay(10000);
}
