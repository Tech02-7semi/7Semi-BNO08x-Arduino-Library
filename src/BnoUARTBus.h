// /******************************************************************************
//  * File    : BnoUARTBus.h
//  * Module  : 7Semi BNO08x — UART Transport Adapter (HDLC-like framing)
//  * Version : 0.1.0
//  * License : MIT
//  *
//  * Summary
//  * -------
//  * UART transport for the BNO08x SHTP protocol as seen on some modules that
//  * encapsulate SHTP frames in an HDLC-like envelope:
//  *
//  *   0x7E <PID=0x01> <escaped SHTP bytes> 0x7E
//  *
//  * Escaping: each 0x7E or 0x7D in the SHTP stream is sent as 0x7D, (byte ^ 0x20).
//  *
//  * Responsibilities
//  * ----------------
//  * - begin(): initialize the UART (with optional RX/TX pin selection on ESP32)
//  * - tx():    frame + escape SHTP bytes and send
//  * - rx():    deframe + unescape a single SHTP frame into caller buffer
//  *
//  * IMPORTANT
//  * ---------
//  * Some BNO08x boards expose *raw SHTP over UART* (no 0x7E/0x7D framing). If
//  * yours does, adapt tx()/rx() accordingly (remove the HDLC bits).
//  *
//  * SHTP Receive Contract (BnoBus::rx)
//  * ----------------------------------
//  * int rx(uint8_t* buf, size_t cap, uint16_t& len)
//  * - On success:
//  *     - 'len' = FULL SHTP length (header included; from the SHTP 4-byte header)
//  *     - returns number of bytes copied into 'buf'
//  * - If 'len' > cap:
//  *     - the function *drains the rest of the UART frame* to keep the link aligned
//  *     - returns 0 (caller should provide a larger buffer)
//  * - On timeout or malformed frame: returns 0 (len may remain 0)
//  *
//  * ESP32 Pin Note
//  * --------------
//  * If you need custom RX/TX pins on ESP32, pass them to the constructor. This
//  * adapter will call:
//  *
//  *   serial.begin(baud, SERIAL_8N1, rxPin, txPin)   // when pins are provided
//  *
//  ******************************************************************************/

// #pragma once
// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include "BnoBus.h"

// struct BnoUARTBus : public BnoBus {
//   HardwareSerial* ser;
//   uint32_t        baud;
//   int             rxPin;
//   int             txPin;

//   /**
//    * ctor
//    * - serial : HardwareSerial instance (e.g. Serial1)
//    * - _baud  : UART speed
//    * - _rxPin : ESP32 RX pin (pass -1 to use port defaults)
//    * - _txPin : ESP32 TX pin (pass -1 to use port defaults)
//    */
//   BnoUARTBus(HardwareSerial& serial = Serial1,
//              uint32_t _baud = 1000000,
//              int _rxPin = -1,
//              int _txPin = -1)
//   : ser(&serial), baud(_baud), rxPin(_rxPin), txPin(_txPin) {}

//   /**
//    * Initialize UART.
//    * - On ESP32, uses custom pins if provided.
//    */
//   bool begin() override {
//     if (!ser) return false;
// #if defined(ESP32)
//     if (rxPin >= 0 || txPin >= 0) {
//       ser->begin(baud, SERIAL_8N1, rxPin < 0 ? -1 : rxPin, txPin < 0 ? -1 : txPin);
//     } else {
//       ser->begin(baud);
//     }
// #else
//     ser->begin(baud);
// #endif
//     return true;
//   }

//   /**
//    * Transmit one HDLC-like framed SHTP packet.
//    * - Adds 0x7E start/end flags
//    * - Inserts PID (0x01)
//    * - Escapes 0x7E/0x7D as 0x7D, (byte ^ 0x20)
//    */
//   bool tx(const uint8_t* data, size_t n) override {
//     if (!ser || !data || n == 0) return false;

//     ser->write(0x7E);
//     ser->write(0x01); // PID

//     for (size_t i = 0; i < n; ++i) {
//       uint8_t b = data[i];
//       if (b == 0x7E || b == 0x7D) {
//         ser->write(0x7D);
//         ser->write(uint8_t(b ^ 0x20));
//       } else {
//         ser->write(b);
//       }
//     }

//     ser->write(0x7E);
//     ser->flush();
//     pump_(5);
//     return true;
//   }
//   void pump_(uint32_t ms) {
//   uint8_t pkt[256];
//   const uint32_t t0 = millis();
//   while (millis() - t0 < ms) {
//     uint16_t L = 0;
//     rx(pkt, sizeof(pkt));
//     yield();
//   }
// }
//   /**
//    * Receive one HDLC-like framed SHTP packet and unescape it.
//    * - Waits for 0x7E, then expects PID=0x01
//    * - Reads bytes until the next 0x7E (end flag), handling 0x7D escapes
//    * - Validates SHTP header (length >= 4, sane upper bound)
//    * - If length L > cap, drains the rest of the frame and returns 0
//    *
//    * Returns:
//    * - >0 : bytes copied into 'buf' (== L) and sets 'len = L'
//    * -  0 : on timeout, malformed header, or when L > cap (drained)
//    */
//   int rx(uint8_t* buf, size_t cap) override {
//     uint16_t len = 0;
//     if (!ser || !buf || cap < 4) return 0;

//     // Tunable timeouts (ms)
//     const unsigned long T_FIND    = 10;  // find start + PID
//     const unsigned long T_PAYLOAD = 50;  // read payload window

//     // 1) Find start flag 0x7E
//     unsigned long t0 = millis();
//     bool sawStart = false;
//     while ((millis() - t0) < T_FIND) {
//       if (ser->available()) {
//         int c = ser->read();
//         if (c == 0x7E) { sawStart = true; break; }
//       }
//       yield();
//     }
//     if (!sawStart) return 0;

//     // 2) Expect PID=0x01 (skip possible extra 0x7E noise)
//     int pid = -1;
//     t0 = millis();
//     while ((millis() - t0) < T_FIND) {
//       if (ser->available()) {
//         int c = ser->read();
//         if (c == 0x7E) continue; // tolerate consecutive flags
//         pid = c;
//         break;
//       }
//       yield();
//     }
//     if (pid != 0x01) return 0;

//     // 3) Read/unescape until next 0x7E. First recover SHTP header (4 bytes).
//     uint8_t hdr[4];
//     size_t  hdrCnt = 0;

//     // We copy only if the total SHTP length fits into 'cap'.
//     // We'll decide this after we have the header.
//     uint16_t L = 0;
//     bool lengthKnown = false;

//     size_t copied = 0;  // bytes copied into 'buf'
//     bool   copyOK = true; // becomes false if L > cap (then we drain and return 0 later)

//     t0 = millis();
//     while ((millis() - t0) < T_PAYLOAD) {
//       if (!ser->available()) { yield(); continue; }

//       int b = ser->read();
//       if (b < 0) continue;

//       // End flag => end of this HDLC frame
//       if (b == 0x7E) break;

//       // Unescape
//       if (b == 0x7D) {
//         // Wait for the next byte (with timeout)
//         unsigned long tEsc = millis();
//         while (!ser->available() && (millis() - tEsc) < T_PAYLOAD) yield();
//         if (!ser->available()) return 0; // unexpected end
//         b = ser->read();
//         if (b < 0) return 0;
//         b ^= 0x20;
//       }

//       // Accumulate header first
//       if (hdrCnt < 4) {
//         hdr[hdrCnt++] = uint8_t(b);
//         if (hdrCnt == 4) {
//           // Compute SHTP length (mask bit15 continuation)
//           L = (uint16_t(hdr[0]) | (uint16_t(hdr[1]) << 8)) & 0x7FFF;
//           lengthKnown = true;

//           // Validate length
//           if (L < 4 || L > 2048) {
//             // Malformed; drain rest of this frame
//             if (!drainToEndFlag_(T_PAYLOAD)) {}
//             return 0;
//           }

//           // If it won't fit, drain full frame and return 0
//           if (L > cap) {
//             copyOK = false;
//           }

//           // If we can copy, start by copying header
//           if (copyOK) {
//             memcpy(buf, hdr, 4);
//             copied = 4;
//           }
//         }
//         continue;
//       }

//       // Past the header: SHTP payload bytes
//       if (!lengthKnown) continue; // defensive (should not happen)

//       // We should consume exactly (L - 4) payload bytes;
//       // however, some modules may append CRC/padding before 0x7E.
//       // We'll copy only up to L and ignore everything beyond until 0x7E.
//       if (copied < L && copyOK) {
//         buf[copied++] = uint8_t(b);
//       }
//     }

//     // Must have a valid header
//     if (!lengthKnown) return 0;

//     // If we couldn't copy (L > cap), ensure the frame was fully drained above
//     if (!copyOK) {
//       // We already kept reading till 0x7E; return 0 to signal "buffer too small"
//       return 0;
//     }

//     // We may have read fewer than L bytes if the sender added extra bytes *after* L
//     // but before 0x7E; ensure we at least got the SHTP-declared length.
//     if (copied < L) return 0;

//     len = L;
//     return int(copied);
//   }

// private:
//   // Drain until we see an end flag 0x7E or hit timeout; best-effort cleanup
//   bool drainToEndFlag_(unsigned long timeoutMs) {
//     unsigned long t = millis();
//     while ((millis() - t) < timeoutMs) {
//       if (ser->available()) {
//         if (ser->read() == 0x7E) return true;
//       } else {
//         yield();
//       }
//     }
//     return false;
//   }
// };
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "BnoBus.h"

struct BnoUARTBus : public BnoBus {
  HardwareSerial* ser;
  uint32_t        baud;
  int             rxPin;
  int             txPin;

  BnoUARTBus(HardwareSerial& serial,
             uint32_t _baud = 1000000,
             int _rxPin = -1,
             int _txPin = -1)
  : ser(&serial), baud(_baud), rxPin(_rxPin), txPin(_txPin) {}

  bool begin() override {
    if (!ser) return false;
#if defined(ESP32)
    if (rxPin >= 0 || txPin >= 0) {
      ser->begin(baud, SERIAL_8N1,
                 rxPin < 0 ? -1 : rxPin,
                 txPin < 0 ? -1 : txPin);
    } else {
      ser->begin(baud);
    }
#else
    ser->begin(baud);
#endif
    return true;
  }

  bool tx(const uint8_t* data, size_t n) override {
    if (!ser || !data || n == 0) return false;

    ser->write(0x7E);
    ser->write(0x01);

    for (size_t i = 0; i < n; ++i) {
      uint8_t b = data[i];
      if (b == 0x7E || b == 0x7D) {
        ser->write(0x7D);
        ser->write(uint8_t(b ^ 0x20));
      } else {
        ser->write(b);
      }
    }

    ser->write(0x7E);
    ser->flush();
    pump_(5);
    return true;
  }

  void pump_(uint32_t ms) {
    uint8_t pkt[256];
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      rx(pkt, sizeof(pkt));
      yield();
    }
  }

  int rx(uint8_t* buf, size_t cap) override {
    if (!ser || !buf || cap < 4) return 0;

    const unsigned long T_FIND = 10;
    const unsigned long T_PAYLOAD = 50;

    // Find start flag
    unsigned long t0 = millis();
    bool sawStart = false;
    while ((millis() - t0) < T_FIND) {
      if (ser->available()) {
        int c = ser->read();
        if (c == 0x7E) { sawStart = true; break; }
      }
      yield();
    }
    if (!sawStart) return 0;

    // Expect PID
    int pid = -1;
    t0 = millis();
    while ((millis() - t0) < T_FIND) {
      if (ser->available()) {
        int c = ser->read();
        if (c == 0x7E) continue;
        pid = c;
        break;
      }
      yield();
    }
    if (pid != 0x01) return 0;

    uint8_t hdr[4];
    size_t hdrCnt = 0;
    uint16_t L = 0;
    bool lengthKnown = false;
    size_t copied = 0;
    bool copyOK = true;

    t0 = millis();
    while ((millis() - t0) < T_PAYLOAD) {
      if (!ser->available()) { yield(); continue; }
      int b = ser->read();
      if (b < 0) continue;

      if (b == 0x7E) break;
      if (b == 0x7D) {
        unsigned long tEsc = millis();
        while (!ser->available() && (millis() - tEsc) < T_PAYLOAD) yield();
        if (!ser->available()) return 0;
        b = ser->read();
        if (b < 0) return 0;
        b ^= 0x20;
      }

      if (hdrCnt < 4) {
        hdr[hdrCnt++] = uint8_t(b);
        if (hdrCnt == 4) {
          L = (uint16_t(hdr[0]) | (uint16_t(hdr[1]) << 8)) & 0x7FFF;
          lengthKnown = true;
          if (L < 4 || L > 2048) { drainToEndFlag_(T_PAYLOAD); return 0; }
          if (L > cap) copyOK = false;
          if (copyOK) { memcpy(buf, hdr, 4); copied = 4; }
        }
        continue;
      }

      if (!lengthKnown) continue;
      if (copied < L && copyOK) buf[copied++] = uint8_t(b);
    }

    if (!lengthKnown) return 0;
    if (!copyOK) return 0;
    if (copied < L) return 0;

    return int(copied);
  }

private:
  bool drainToEndFlag_(unsigned long timeoutMs) {
    unsigned long t = millis();
    while ((millis() - t) < timeoutMs) {
      if (ser->available()) {
        if (ser->read() == 0x7E) return true;
      } else yield();
    }
    return false;
  }
};
