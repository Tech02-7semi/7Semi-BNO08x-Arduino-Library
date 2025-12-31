/******************************************************************************
 * File    : BnoI2CBus.h
 * Module  : 7Semi BNO08x — I2C Transport Adapter
 * Version : 0.1.0
 * License : MIT
 *  
 * Summary
 * -------
 * I2C transport for the BNO08x SHTP protocol. Presents the uniform BnoBus
 * interface (begin/tx/rx) so the higher-level driver stays transport-agnostic.
 *
 * Address Notes
 * -------------
 * - Default 7-bit address is 0x4B (0x4A if AD0 pulled low).
 ******************************************************************************/

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "BnoBus.h"

#if defined(ARDUINO_ARCH_AVR)
#define BNO_WIRE_CHUNK 32
#else
#define BNO_WIRE_CHUNK 128
#endif

/**
 * BnoI2CBus — I²C transport for BNO08x SHTP
 * - Optional custom SDA/SCL pins where supported (ESP32/ESP8266/RP2040/STM32)
 * - Default device address: 0x4B (many boards); 0x4A on some
 * - Implements the unified BnoBus interface (begin/tx/rx)
 */
struct BnoI2CBus : public BnoBus {
  TwoWire* w;
  int sda;       // use int so -1 sentinel works
  int scl;       // use int so -1 sentinel works
  uint8_t addr;  // 7-bit I2C address (0x4B typical)
  uint32_t clk;  // I2C clock (Hz)

  /**
   * ctor
   * - wire    : TwoWire instance (default Wire)
   * - sdaPin  : SDA pin (or -1 to use core default)
   * - sclPin  : SCL pin (or -1 to use core default)
   * - i2cAddr : device address (0x4B typical)
   * - clock   : bus speed (400 kHz default)
   */
  BnoI2CBus(TwoWire& wire = Wire,
            int sdaPin = -1,
            int sclPin = -1,
            uint8_t i2cAddr = 0x4B,
            uint32_t clock = 400000)
    : w(&wire), sda(sdaPin), scl(sclPin), addr(i2cAddr), clk(clock) {}

  /**
   * Initialize I²C bus (with optional custom pins where supported).
   */
  bool begin() override {
    if (!w) return false;

      // Some cores allow custom SDA/SCL (order = SDA, SCL). AVR ignores them.
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
    if (sda >= 0 && scl >= 0) {
      w->begin(sda, scl);
    } else {
      w->begin();
    }
#else
    (void)sda;
    (void)scl;
    w->begin();
#endif

    // Set clock if supported by the core
#if defined(TWBR) || defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
    w->setClock(clk);
#endif
    return true;
  }
  /**
   * Transmit a complete SHTP frame (header+payload) in one transaction.
   */
  bool tx(const uint8_t* data, size_t n) override {
    if (!w || !data || n == 0) return false;
    w->beginTransmission(addr);
    w->write(data, n);
    return w->endTransmission();
  }

  /**
   * Receive one SHTP frame.
   * - Reads 4-byte header to obtain total length L (incl. header).
   * - Copies up to 'cap' bytes into 'buf' (partial copy allowed).
   * - Drains any excess bytes to keep the device FIFO aligned.
   */
  int rx(uint8_t* buf, size_t cap) override {
    w->requestFrom((int)addr, cap);
    uint8_t len = 0;
    while (w->available() && len < cap)
      buf[len++] = w->read();
    return len;
  }
};
