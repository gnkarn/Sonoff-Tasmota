/*
 * IRremoteESP8266: IRrecvDumpV2 - dump details of IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 *
 * Copyright 2009 Ken Shirriff, http://arcfn.com
 * Copyright 2017 David Conran
 *
 * Example circuit diagram:
 *  https://github.com/markszabo/IRremoteESP8266/wiki#ir-receiving
 *
 * Changes:
 *   Version 0.3 November, 2017
 *     - Support for A/C decoding for some protcols.
 *   Version 0.2 April, 2017
 *     - Decode from a copy of the data so we can start capturing faster thus
 *       reduce the likelihood of miscaptures.
 * Based on Ken Shirriff's IrsendDemo Version 0.1 July, 2009,
 * Modified by gnk for mpx alarm zones detection feb2018-
 *  added all ir codes on kIrRemoteProtocols[]
 */

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#if DECODE_AC
#include <ir_Daikin.h>
#include <ir_Fujitsu.h>
#include <ir_Kelvinator.h>
#include <ir_Midea.h>
#include <ir_Toshiba.h>
#endif  // DECODE_AC

// ==================== start of TUNEABLE PARAMETERS ====================
// An IR detector/demodulator is connected to GPIO pin 04
// e.g. D2 on a NodeMCU board.
#define RECV_PIN 4  //D2 gpio04

// The Serial connection baud rate.
// i.e. Status message will be sent to the PC at this baud rate.
// Try to avoid slow speeds like 9600, as you will miss messages and
// cause other problems. 115200 (or faster) is recommended.
// NOTE: Make sure you set your Serial Monitor to the same speed.
// #define BAUD_RATE 115200

// As this program is a special purpose capture/decoder, let us use a larger
// than normal buffer so we can handle Air Conditioner remote codes.
#define CAPTURE_BUFFER_SIZE 128

//decode_results results;  // Somewhere to store the results , aca funciona sacando dumpACInfo

// TIMEOUT is the Nr. of milli-Seconds of no-more-data before we consider a
// message ended.
// This parameter is an interesting trade-off. The longer the timeout, the more
// complex a message it can capture. e.g. Some device protocols will send
// multiple message packets in quick succession, like Air Conditioner remotes.
// Air Coniditioner protocols often have a considerable gap (20-40+ms) between
// packets.
// The downside of a large timeout value is a lot of less complex protocols
// send multiple messages when the remote's button is held down. The gap between
// them is often also around 20+ms. This can result in the raw data be 2-3+
// times larger than needed as it has captured 2-3+ messages in a single
// capture. Setting a low timeout value can resolve this.
// So, choosing the best TIMEOUT value for your use particular case is
// quite nuanced. Good luck and happy hunting.
// NOTE: Don't exceed MAX_TIMEOUT_MS. Typically 130ms.
#if DECODE_AC
#define TIMEOUT 50U  // Some A/C units have gaps in their protocols of ~40ms.
                     // e.g. Kelvinator
                     // A value this large may swallow repeats of some protocols
#else  // DECODE_AC
#define TIMEOUT 15U  // Suits most messages, while not swallowing many repeats.
#endif  // DECODE_AC
// Alternatives:
// #define TIMEOUT 90U  // Suits messages with big gaps like XMP-1 & some aircon
                        // units, but can accidentally swallow repeated messages
                        // in the rawData[] output.
// #define TIMEOUT MAX_TIMEOUT_MS  // This will set it to our currently allowed
                                   // maximum. Values this high are problematic
                                   // because it is roughly the typical boundary
                                   // where most messages repeat.
                                   // e.g. It will stop decoding a message and
                                   //   start sending it to serial at precisely
                                   //   the time when the next message is likely
                                   //   to be transmitted, and may miss it.

// Set the smallest sized "UNKNOWN" message packets we actually care about.
// This value helps reduce the false-positive detection rate of IR background
// noise as real messages. The chances of background IR noise getting detected
// as a message increases with the length of the TIMEOUT value. (See above)
// The downside of setting this message too large is you can miss some valid
// short messages for protocols that this library doesn't yet decode.
//
// Set higher if you get lots of random short UNKNOWN messages when nothing
// should be sending a message.
// Set lower if you are sure your setup is working, but it doesn't see messages
// from your device. (e.g. Other IR remotes work.)
// NOTE: Set this value very high to effectively turn off UNKNOWN detection.
#define MIN_UNKNOWN_SIZE 12
// ==================== end of TUNEABLE PARAMETERS ====================

//
// pin definitions for Node Mcu numbers are Gpio
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

//decode_results results;  // Somewhere to store the results

// void dumpACInfo(decode_results *results) {
//   String description = "";
// // #if DECODE_DAIKIN
// //   if (results->decode_type == DAIKIN) {
// //     IRDaikinESP ac(0);
// //     ac.setRaw(results->state);
// //     description = ac.toString();
// //   }
// // #endif  // DECODE_DAIKIN
// // #if DECODE_FUJITSU_AC
// //   if (results->decode_type == FUJITSU_AC) {
// //     IRFujitsuAC ac(0);
// //     ac.setRaw(results->state, results->bits / 8);
// //     description = ac.toString();
// //   }
// // #endif  // DECODE_FUJITSU_AC
// // #if DECODE_KELVINATOR
// //   if (results->decode_type == KELVINATOR) {
// //     IRKelvinatorAC ac(0);
// //     ac.setRaw(results->state);
// //     description = ac.toString();
// //   }
// // #endif  // DECODE_KELVINATOR
// // #if DECODE_TOSHIBA_AC
// //   if (results->decode_type == TOSHIBA_AC) {
// //     IRToshibaAC ac(0);
// //     ac.setRaw(results->state);
// //     description = ac.toString();
// //   }
// // #endif  // DECODE_TOSHIBA_AC
// // #if DECODE_MIDEA
// //   if (results->decode_type == MIDEA) {
// //     IRMideaAC ac(0);
// //     ac.setRaw(results->value);  // Midea uses value instead of state.
// //     description = ac.toString();
// //   }
// // #endif  // DECODE_MIDEA
//   // If we got a human-readable description of the message, display it.
//   if (description != "")  Serial.println("Mesg Desc.: " + description);
// } // end dumpACInfo

//#define LED     D0        // Led in NodeMCU at pin GPIO16 (D0)
#define LED     D4        // Led in ESP12F at pin GPIO02 (D4)
unsigned long previousLedMillis = 0;        // will store last time LED was updated
unsigned long currentLedMillis = 0; // last time when led was turned on
bool ledState = 0; // led Status
// constants won't change:
const long lediInterval = 500;           // interval at which to blink (milliseconds)

// if the code corresponds to an alarm zone return zone number
uint8_t DetectAlarmZone(uint64_t zone){
  if (zone == 0x9665) return 6; // dormitorio
  if (zone == 0x9653) return 5; // escritorio
  if (zone == 0x1540) return 4; // matrim
  if (zone == 0x9630) return 3; // liv/coc
  //if (zone == 0x9653) return 2; // patio
  if (zone == 0x1615) return 1; // entrada

  return 0 ;
}

// The section of code run only once at start-up.
// void Mpx_setup() {
//   pinMode(LED, OUTPUT);   // LED pin as output.
//   //Serial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_TX_ONLY);
//   digitalWrite(LED, LOW);
//   delay(500);  // Wait a bit for the serial connection to be establised.
//   digitalWrite(LED, HIGH);
//
// #if DECODE_HASH
//   // Ignore messages with less than minimum on or off pulses.
//   irrecv.setUnknownThreshold(MIN_UNKNOWN_SIZE);
// #endif  // DECODE_HASH
// // Use turn on the save buffer feature for more complete capture coverage.
//   IRrecv irrecv(RECV_PIN, CAPTURE_BUFFER_SIZE, TIMEOUT, true);
//   irrecv.enableIRIn();  // Start the receiver
// }
