/*
 * MCP_CAN hardware loopback self-test for the corrected library.
 * Arduino Nano/Uno: CS=D10, MOSI=D11, MISO=D12, SCK=D13, common GND.
 * Match CAN_CLOCK to the crystal on the MCP2515 module, not the Arduino CPU.
 * Serial Monitor: 115200 baud. Press RESET to run the suite again.
 *
 * Uses internal loopback only; no second CAN node is required. Run on a
 * standalone module with CANH/CANL disconnected. This replaces the sketch
 * currently on the Arduino when you upload it.
 *
 * This is a hardware smoke test, not a renamed copy of the host regression
 * suite. See README.md for coverage and limitations.
 */
#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"

const byte CAN_CS_PIN = 10;
const byte CAN_CLOCK = MCP_8MHZ;
const byte CAN_BITRATE = CAN_50KBPS;
const unsigned long RX_TIMEOUT_MS = 100;

MCP_CAN CAN0(CAN_CS_PIN);
unsigned int passed = 0;
unsigned int failed = 0;

bool check(bool condition, const __FlashStringHelper *label) {
  if (condition) {
    ++passed;
    Serial.print(F("PASS: "));
  } else {
    ++failed;
    Serial.print(F("FAIL: "));
  }
  Serial.println(label);
  return condition;
}

bool statusIs(byte actual, byte expected, const __FlashStringHelper *label) {
  if (actual != expected) {
    Serial.print(F("  status="));
    Serial.print(actual);
    Serial.print(F(" expected="));
    Serial.println(expected);
  }
  return check(actual == expected, label);
}

bool waitForReceive() {
  const unsigned long start = millis();
  do {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) return true;
  } while ((unsigned long)(millis() - start) < RX_TIMEOUT_MS);
  return false;
}

bool receiveMatches(unsigned long expectedId, byte expectedLen,
                    const byte *expectedData, bool separateExt) {
  if (!waitForReceive()) {
    Serial.println(F("  No loopback frame before RX deadline."));
    return false;
  }

  unsigned long id = 0;
  byte ext = 0, len = 0;
  byte data[8] = {};
  byte status;
  if (separateExt) {
    status = CAN0.readMsgBuf(&id, &ext, &len, data);
    if (ext) id |= 0x80000000UL;
  } else {
    status = CAN0.readMsgBuf(&id, &len, data);
  }

  bool matches = status == CAN_OK && id == expectedId && len == expectedLen;
  if (matches && !(expectedId & 0x40000000UL)) {
    // RTR frames carry no payload. Check only their ID, flags and DLC.
    for (byte i = 0; i < expectedLen; ++i) {
      if (data[i] != expectedData[i]) matches = false;
    }
  }
  if (!matches) {
    Serial.print(F("  RX status=")); Serial.print(status);
    Serial.print(F(" ID=0x")); Serial.print(id, HEX);
    Serial.print(F(" DLC=")); Serial.print(len);
    Serial.print(F(" expected ID=0x")); Serial.print(expectedId, HEX);
    Serial.print(F(" DLC=")); Serial.println(expectedLen);
  }
  return matches;
}

bool roundTrip(unsigned long id, byte len, byte *data,
               bool packedTx, bool separateRx) {
  Serial.print(F("  ID=0x")); Serial.print(id, HEX);
  Serial.print(F(" DLC=")); Serial.print(len);
  Serial.print(F(" TX=")); Serial.print(packedTx ? F("packed") : F("explicit EXT"));
  Serial.print(F(" RX=")); Serial.println(separateRx ? F("separate EXT") : F("packed"));

  byte status;
  if (packedTx) {
    status = CAN0.sendMsgBuf(id, len, data);
  } else {
    status = CAN0.sendMsgBuf(id & 0x1FFFFFFFUL,
                           (id & 0x80000000UL) ? 1 : 0, len, data);
  }
  if (status != CAN_OK) {
    Serial.print(F("  TX status=")); Serial.println(status);
    return check(false, F("Loopback transmission"));
  }
  return check(receiveMatches(id, len, data, separateRx), F("Loopback ID, flags, DLC and data"));
}

bool emptyReadPreservesOutputs(bool separateExt) {
  unsigned long id = 0x12345678UL;
  byte ext = 0xA5, len = 0xA5;
  byte data[8];
  for (byte i = 0; i < 8; ++i) data[i] = 0xA5;
  const byte status = separateExt ? CAN0.readMsgBuf(&id, &ext, &len, data)
                                  : CAN0.readMsgBuf(&id, &len, data);
  bool unchanged = status == CAN_NOMSG && id == 0x12345678UL && len == 0xA5 && ext == 0xA5;
  for (byte i = 0; i < 8; ++i) unchanged = unchanged && data[i] == 0xA5;
  return check(unchanged, F("Empty read preserves output values"));
}

bool testDataFrames() {
  byte payload[8];
  for (byte format = 0; format < 2; ++format) {
    const unsigned long id = format ? 0x81ABCDE3UL : 0x123UL;
    for (byte overload = 0; overload < 2; ++overload) {
      for (byte len = 0; len <= 8; ++len) {
        for (byte i = 0; i < 8; ++i) payload[i] = 0x30 + i + len * 3 + format + overload;
        if (!roundTrip(id, len, len ? payload : 0, overload != 0, overload != 0)) return false;
      }
    }
  }

  // Boundaries and a genuinely short input array, not just an 8-byte array
  // with a smaller DLC. Memory safety itself is checked by the host suite.
  byte shortPayload[3] = {0x12, 0xA5, 0x5A};
  if (!roundTrip(0x000UL, 3, shortPayload, false, false)) return false;
  if (!roundTrip(0x7FFUL, 3, shortPayload, true, true)) return false;
  if (!roundTrip(0x80000000UL, 3, shortPayload, true, false)) return false;
  if (!roundTrip(0x9FFFFFFFUL, 3, shortPayload, false, true)) return false;

  // Nonzero RTR DLC still requires a data pointer under this library's API.
  byte dummy[8] = {};
  if (!roundTrip(0x40000321UL, 0, 0, true, false)) return false;
  if (!roundTrip(0x40000321UL, 8, dummy, true, false)) return false;
  if (!roundTrip(0xC01ABCDEUL, 0, 0, true, false)) return false;
  if (!roundTrip(0xC01ABCDEUL, 8, dummy, true, false)) return false;
  return emptyReadPreservesOutputs(false) && emptyReadPreservesOutputs(true);
}

bool testRejectedArguments() {
  byte oversized[9] = {};
  if (!statusIs(CAN0.sendMsgBuf(0x123, 0, 9, oversized), CAN_FAILTX, F("Reject TX length 9, explicit EXT"))) return false;
  if (!statusIs(CAN0.sendMsgBuf(0x123UL, 9, oversized), CAN_FAILTX, F("Reject TX length 9, packed ID"))) return false;
  if (!statusIs(CAN0.sendMsgBuf(0x123, 0, 1, 0), CAN_FAILTX, F("Reject null TX data, explicit EXT"))) return false;
  if (!statusIs(CAN0.sendMsgBuf(0x123UL, 1, 0), CAN_FAILTX, F("Reject null TX data, packed ID"))) return false;
  if (!statusIs(CAN0.init_Mask(2, 0, 0UL), MCP2515_FAIL, F("Reject mask index 2, explicit EXT"))) return false;
  if (!statusIs(CAN0.init_Mask(2, 0UL), MCP2515_FAIL, F("Reject mask index 2, packed ID"))) return false;
  if (!statusIs(CAN0.init_Filt(6, 0, 0UL), MCP2515_FAIL, F("Reject filter index 6, explicit EXT"))) return false;
  if (!statusIs(CAN0.init_Filt(6, 0UL), MCP2515_FAIL, F("Reject filter index 6, packed ID"))) return false;
  if (!statusIs(CAN0.setMode(0x21), MCP2515_FAIL, F("Reject invalid mode 0x21"))) return false;
  if (!statusIs(CAN0.setMode(0xFF), MCP2515_FAIL, F("Reject invalid mode 0xFF"))) return false;
  if (!check(!waitForReceive(), F("Rejected calls did not produce an RX frame"))) return false;
  byte payload[1] = {0xA5};
  // Do not restore loopback here: this also detects an invalid call changing it.
  return roundTrip(0x456UL, 1, payload, false, false);
}

bool testSleepWakeAndAbort() {
  byte payload[1] = {0x55};
  for (byte wakeEnabled = 0; wakeEnabled <= 1; ++wakeEnabled) {
    CAN0.setSleepWakeup(wakeEnabled);
    if (!statusIs(CAN0.setMode(MCP_SLEEP), MCP2515_OK, F("Enter SLEEP"))) return false;
    if (!statusIs(CAN0.setMode(MCP_LOOPBACK), MCP2515_OK, F("Wake into LOOPBACK"))) return false;
    if (!roundTrip(0x234UL, 1, payload, true, false)) return false;
  }
  CAN0.setSleepWakeup(0);
  if (!statusIs(CAN0.enOneShotTX(), CAN_OK, F("Enable one-shot TX"))) return false;
  if (!roundTrip(0x235UL, 1, payload, true, false)) return false;
  if (!statusIs(CAN0.disOneShotTX(), CAN_OK, F("Disable one-shot TX"))) return false;
  // Public synchronous sends have already completed; this checks idle abort
  // and a subsequent send, not cancellation of an in-flight CAN-bus frame.
  if (!statusIs(CAN0.abortTX(), CAN_OK, F("Abort with no pending TX"))) return false;
  return roundTrip(0x236UL, 1, payload, true, false);
}

bool testOverflowReset() {
  CAN0.resetOverflowErrors();
  if (!check((CAN0.getError() & 0xC0) == 0, F("Overflow flags initially clear"))) return false;
  byte payload[1] = {0x77};
  // Fill both RX buffers, then intentionally overflow them with a third frame.
  for (byte i = 0; i < 3; ++i) {
    if (!statusIs(CAN0.sendMsgBuf(0x500UL + i, 0, 1, payload), CAN_OK, F("TX for RX overflow test"))) return false;
  }
  const byte before = CAN0.getError();
  if (!check((before & 0xC0) != 0, F("RX overflow detected"))) return false;
  CAN0.resetOverflowErrors();
  if (!check(CAN0.getError() == (before & 0x3F), F("Clear only RX overflow flags"))) return false;
  // Do not send between these reads: RX0/RX1 are not a general FIFO.
  if (!check(receiveMatches(0x500UL, 1, payload, false), F("First queued frame survives overflow reset"))) return false;
  if (!check(receiveMatches(0x501UL, 1, payload, false), F("Second queued frame survives overflow reset"))) return false;
  if (!emptyReadPreservesOutputs(false)) return false;
  return roundTrip(0x503UL, 1, payload, false, false);
}

bool testStandardFilter() {
  // Both masks and all six filters are configured so no other filter can
  // accept the ID that is supposed to be rejected. UL is essential on AVR.
  if (!statusIs(CAN0.init_Mask(0, 0, 0x7FFUL << 16), CAN_OK, F("Exact standard mask 0"))) return false;
  if (!statusIs(CAN0.init_Mask(1, 0x7FFUL << 16), CAN_OK, F("Exact standard mask 1"))) return false;
  for (byte i = 0; i < 6; ++i) {
    if (!statusIs(CAN0.init_Filt(i, 0, 0x123UL << 16), CAN_OK, F("Exact standard filter"))) return false;
  }
  byte payload[2] = {0xAA, 0x55};
  if (!roundTrip(0x123UL, 2, payload, false, false)) return false;
  // A filter rejects reception, not transmission; sendMsgBuf should succeed.
  if (!statusIs(CAN0.sendMsgBuf(0x124, 0, 2, payload), CAN_OK, F("Transmit nonmatching ID internally"))) return false;
  return check(!waitForReceive(), F("Nonmatching ID rejected by RX filters"));
}

bool runSelfTest() {
  Serial.println(F("\nMCP_CAN loopback self-test - 115200 baud"));
  Serial.println(F("CS=D10 by default. Use the module's actual crystal setting."));
  Serial.println(F("The next unsupported-rate rejection is intentional."));
  if (!statusIs(CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ), CAN_FAILINIT,
                F("Reject unsupported 8 MHz / 1 Mbit/s"))) return false;
  if (!statusIs(CAN0.begin(MCP_STDEXT, CAN_BITRATE, CAN_CLOCK), CAN_OK, F("Initialize controller"))) return false;
  if (!statusIs(CAN0.setMode(MCP_LOOPBACK), MCP2515_OK, F("Select internal LOOPBACK"))) return false;
  if (!statusIs(CAN0.init_Mask(0, 0, 0UL), CAN_OK, F("Zero mask 0"))) return false;
  if (!statusIs(CAN0.init_Mask(1, 0UL), CAN_OK, F("Zero mask 1"))) return false;
  if (!testDataFrames()) return false;
  if (!testRejectedArguments()) return false;
  if (!testSleepWakeAndAbort()) return false;
  if (!testOverflowReset()) return false;
  if (!testStandardFilter()) return false;
  return check(CAN0.checkError() == CAN_OK && CAN0.getError() == 0, F("No controller error flags at end"));
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  const bool completed = runSelfTest();
  Serial.println(F("\n--- RESULT ---"));
  Serial.print(F("PASS=")); Serial.print(passed);
  Serial.print(F(" FAIL=")); Serial.println(failed);
  if (completed && failed == 0) {
    Serial.println(F("ALL CHECKS PASSED (internal loopback only)."));
  } else {
    // Stop on the first failure so later tests do not hide the original cause.
    Serial.println(F("STOPPED AT FIRST FAILURE. Remaining checks were not run."));
    Serial.print(F("EFLG=0x")); Serial.print(CAN0.getError(), HEX);
    Serial.print(F(" REC=")); Serial.print(CAN0.errorCountRX());
    Serial.print(F(" TEC=")); Serial.println(CAN0.errorCountTX());
    CAN0.abortTX();
  }
  Serial.println(F("Press RESET to repeat. External CAN wiring and bus timing are not tested."));
}

void loop() {
  // Run once per reset; leave the controller in its final test state.
}
