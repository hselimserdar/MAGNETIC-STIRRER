#include "Arduino.h"

const uint8_t stir1 [] PROGMEM = {25, 24,
                                  0x00, 0xC0, 0x20, 0x10, 0x88, 0x84, 0x82, 0x82, 0x01, 0x01, 0x01, 0x79, 0xF9, 0xFD, 0xFD, 0xF9,
                                  0xF9, 0xFA, 0xF2, 0xF4, 0xC8, 0x10, 0x20, 0xC0, 0x00, 0xFF, 0x00, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF,
                                  0xFF, 0xFF, 0x7F, 0x86, 0xAA, 0x83, 0xAB, 0xC7, 0xFF, 0xCF, 0xCF, 0x87, 0x87, 0x81, 0x00, 0x00,
                                  0x00, 0xFF, 0x01, 0x06, 0x08, 0x11, 0x23, 0x43, 0x83, 0x81, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x7F,
                                  0x7F, 0x7F, 0x3F, 0xBF, 0x9F, 0x4F, 0x27, 0x10, 0x08, 0x06, 0x01, 0x00, 0x00, 0x00,
                                 };

const uint8_t stir2 [] PROGMEM = {25, 24,
                                  0x00, 0xC0, 0x20, 0x10, 0x08, 0x04, 0x02, 0x72, 0xF9, 0xF9, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
                                  0xF9, 0xF2, 0x02, 0x04, 0x08, 0x10, 0x20, 0xC0, 0x00, 0xFF, 0x00, 0xE0, 0xF0, 0xF8, 0xF8, 0xFC,
                                  0xFC, 0xFC, 0xFD, 0xC3, 0xAB, 0x83, 0xAB, 0xC7, 0xFF, 0xF3, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0x60,
                                  0x00, 0xFF, 0x01, 0x06, 0x08, 0x13, 0x27, 0x4F, 0x9F, 0x9F, 0x1F, 0x1F, 0x03, 0x00, 0x00, 0x03,
                                  0x0F, 0x0F, 0x1F, 0x9F, 0x9F, 0x4F, 0x2F, 0x13, 0x08, 0x06, 0x01, 0x00, 0x00, 0x00,
                                 };

const uint8_t stir3 [] PROGMEM = {25, 24,
                                  0x00, 0xC0, 0x20, 0x10, 0xC8, 0xE4, 0xF2, 0xFA, 0xF9, 0xFD, 0xFD, 0xFD, 0xF9, 0xF1, 0xC1, 0x01,
                                  0x01, 0x02, 0x82, 0x84, 0x88, 0x10, 0x20, 0xC0, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x03, 0xC3, 0xC3,
                                  0xE7, 0xE7, 0xFF, 0xC7, 0xAB, 0x83, 0xAB, 0xC3, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
                                  0x00, 0xFF, 0x01, 0x06, 0x08, 0x10, 0x27, 0x5F, 0x9F, 0xBF, 0x3F, 0x3F, 0x7F, 0x7F, 0x3F, 0x3C,
                                  0x00, 0x01, 0x01, 0x83, 0x83, 0x43, 0x23, 0x11, 0x08, 0x06, 0x01, 0x00, 0x00, 0x00,
                                 };

const uint8_t stir4 [] PROGMEM = {25, 24,
                                  0x00, 0xC0, 0x20, 0x90, 0xE8, 0xE4, 0xF2, 0xF2, 0xF1, 0xE1, 0xE1, 0x81, 0x01, 0x01, 0x81, 0xF1,
                                  0xF1, 0xF2, 0xF2, 0xE4, 0xC8, 0x90, 0x20, 0xC0, 0x00, 0xFF, 0x00, 0x0C, 0x3F, 0x3F, 0x3F, 0x3F,
                                  0x1F, 0x9F, 0xFF, 0xC7, 0xAB, 0x82, 0xAA, 0x87, 0x7F, 0x7F, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x0E,
                                  0x00, 0xFF, 0x01, 0x06, 0x08, 0x10, 0x20, 0x40, 0x80, 0x9E, 0x3F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F,
                                  0x7F, 0x3F, 0x3E, 0x9C, 0x80, 0x40, 0x20, 0x10, 0x08, 0x06, 0x01, 0x00, 0x08, 0x54,
                                 };
