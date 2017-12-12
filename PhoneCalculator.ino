/*************************************************
 *  SimpleIO.ino                                 *
 *  Example from the ArTICL library              *
 *           Created by Christopher Mitchell,    *
 *           2011-2016, all rights reserved.     *
 *           
 *  This sketch was originally SimpleIO.ino, the example sketch but
 *  has been modified to work with a SIM5320E module in order to
 *  send and recieve phone calls with a TI84 calculator. This
 *  modification was made by myself (Scott Howie) and other projects
 *  can be found on my github page github.com/BoomBrush
 */

#include "CBL2.h"
#include "TIVar.h"
#include <SoftwareSerial.h>

CBL2 cbl;
const int lineRed = 2;
const int lineWhite = 3;

#if defined(__MSP432P401R__)    // MSP432 target
#define BUTTON_PIN 73
#define SWITCH_PIN 12
#else                           // Arduino target
#define BUTTON_PIN 3
#define SWITCH_PIN 2
#endif
// Lists are 2 + (9 * dimension) bytes,
// so incidentally a 255-byte max data length
// limits this demo's lists to 28 elements.
#define MAXDATALEN 255

uint8_t header[16];
uint8_t data[MAXDATALEN];

char c;

int onSendAsCBL2(uint8_t type, enum Endpoint model, int* headerlen,
                 int* datalen, data_callback* data_callback);
int onGetAsCBL2(uint8_t type, enum Endpoint model, int datalen);

SoftwareSerial GSM(7, 6); // RX, TX

void setup() {
  Serial.begin(9600);                           // Used for debugging
  GSM.begin(9600);
  cbl.setLines(lineRed, lineWhite);
  cbl.resetLines();
  // cbl.setVerbosity(true, &Serial);      // Comment this in for verbose message information
  
  // The following registers buffers for exchanging data, the maximum
  // allowed data length, and functions to call on Get() and Send().
  cbl.setupCallbacks(header, data, MAXDATALEN,
                      onGetAsCBL2, onSendAsCBL2);
                      
}

// Repeatedly check to see if the calculator has initiated a Get()
// or a Send() operation yet. If it has, then onGetAsCBL2() or
// onSendAsCBL2() will be invoked, since they were registered in
// setup() above.
void loop() {
  int rval;
  rval = cbl.eventLoopTick();
  if (rval && rval != ERR_READ_TIMEOUT) {
    Serial.print("Failed to run eventLoopTick: code ");
    Serial.println(rval);
  }

  // This just allows you to communicate with the SIM module in the event you need to plug the arduino into
  // a computer and manually execute AT commands.
  if (GSM.available()) {
    Serial.write(GSM.read());
  }
  if (Serial.available()) {
    c = Serial.read();
    if (c == '*') GSM.write(0x1A);
    else GSM.write(c);
  }
}

// Callback when the CBL2 class has successfully received a variable
// from the attached calculator.
int onGetAsCBL2(uint8_t type, enum Endpoint model, int datalen) {
  Serial.print("Got variable of type ");
  Serial.print(type);
  Serial.print(" from endpoint of type ");
  Serial.println((int)model);
  
  // We only want to handle lists.
  if (type != VarTypes82::VarRList && type != VarTypes82::VarURList &&
      type != VarTypes84PCSE::VarRList)
  {
    return -1;
  }

  // Turn the LEDs and motor on or off
  uint16_t list_len = TIVar::sizeWordToInt(&(data[0]));    // Convert 2-byte size word to int
  if (list_len == 3) {
    // It is indeed a 5-element list
    int size_of_real = TIVar::sizeOfReal(model);
    int first   = TIVar::realToLong8x(&data[size_of_real * 0 + 2], model); // First list element starts after 2-byte size word
    int second = TIVar::realToLong8x(&data[size_of_real * 1 + 2], model);
    int third  = TIVar::realToLong8x(&data[size_of_real * 2 + 2], model);

    GSM.print("ATD+61");
    GSM.print(first);
    for (int i=0;i<(3-String(second).length());i++) GSM.print("0");
    GSM.print(second);
    for (int i=0;i<(3-String(third).length());i++) GSM.print("0");
    GSM.print(third);
    GSM.println(";");
    
  }
  if (list_len == 1) {
    int size_of_real = TIVar::sizeOfReal(model);
    int first   = TIVar::realToLong8x(&data[size_of_real * 0 + 2], model);
    if (first == 0) GSM.println("AT+CHUP"); //hang up
    if (first == 1) GSM.println("ATA"); //answer

    
    
  }
  return 0;
}

// Callback when the CBL2 class notices the attached calculator
// wants to start a Get() exchange. The CBL2 class needs to get
// any data to send before continuing the exchange.
int onSendAsCBL2(uint8_t type, enum Endpoint model, int* headerlen,
                 int* datalen, data_callback* data_callback)
{
  Serial.print("Got request for variable of type ");
  Serial.print(type);
  Serial.print(" from endpoint of type ");
  Serial.println((int)model);
  
  if (type != VarTypes82::VarRList)
    return -1;
  
  // Compose the VAR header
  *datalen = 2 + TIVar::sizeOfReal(model) * 2;
  TIVar::intToSizeWord(*datalen, &header[0]);  // Two bytes for the element count, ANALOG_PIN_COUNT Reals
                                                // This sets header[0] and header[1]
  header[2] = VarTypes85::VarRList;             // RealList (if you're a TI-85. Bleh.)
  header[3] = 0x01;                // Name length
  header[4] = 0x41;                // "A", as per "standard" See http://www.cemetech.net/forum/viewtopic.php?p=224739#224739
  header[5] = 0x00;                // Zero terminator (remainder of header is ignored)
  *headerlen = 11;
  
  // Compose the body of the variable
  data[0] = 2;   // Little-endian word for number of
  data[1] = 0;            // elements in this list
  int offset = 2;         // Offset past the count word

  // Convert the value, get the length of the inserted data or -1 for failure
  int rval;
  rval = TIVar::longToReal8x(digitalRead(BUTTON_PIN), &data[offset], model);
  if (rval < 0) {
    return -1;
  }
  offset += rval;
  rval = TIVar::longToReal8x(digitalRead(SWITCH_PIN), &data[offset], model);
  if (rval < 0) {
    return -1;
  }
  offset += rval;

  return 0;
}
