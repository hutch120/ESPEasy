#include "../PluginStructs/P008_data_struct.h"

#ifdef USES_P008

/**
 * Convert/cast a hexadecimal input to a decimal representation, so 0x1234 (= 4660) comes out as 1234.
 *
 * //FIXME Move to a more global place to also be used elsewhere?
 */
uint64_t P008_data_struct::castHexAsDec(uint64_t hexValue) {
  uint64_t result = 0;
  uint8_t  digit;

  uint64_t factor = 1;

  for (int i = 0; i < 8; i++) {
    digit = (hexValue & 0x0000000F);

    if (digit > 10) {
      digit = 0;     // Cast by dropping any non-decimal input
    }

    if (digit > 0) {
      result += (digit * factor);
    }
    hexValue >>= 4;

    if (hexValue == 0) {
      break; // Stop when no more to process
    }
    factor *= 10;
  }
  return result;
}

/**************************************************************************
* Constructor
**************************************************************************/
P008_data_struct::P008_data_struct(struct EventStruct *event) {
  _pin1 = CONFIG_PIN1;
  _pin2 = CONFIG_PIN2;
}

/*****************************************************
* Destructor
*****************************************************/
P008_data_struct::~P008_data_struct() {
  detachInterrupt(digitalPinToInterrupt(_pin1));
  detachInterrupt(digitalPinToInterrupt(_pin2));
}

/**************************************************************************
* plugin_init Initialize interrupt handling
**************************************************************************/
bool P008_data_struct::plugin_init(struct EventStruct *event) {
  pinMode(_pin1, INPUT_PULLUP);
  pinMode(_pin2, INPUT_PULLUP);

  if (P008_COMPATIBILITY == 0) { // Keep 'old' setting for backward compatibility
    attachInterruptArg(digitalPinToInterrupt(_pin1),
                       reinterpret_cast<void (*)(void *)>(Plugin_008_interrupt1),
                       this,
                       FALLING);
    attachInterruptArg(digitalPinToInterrupt(_pin2),
                       reinterpret_cast<void (*)(void *)>(Plugin_008_interrupt2),
                       this,
                       FALLING);
  } else {
    attachInterruptArg(digitalPinToInterrupt(_pin1),
                       reinterpret_cast<void (*)(void *)>(Plugin_008_interrupt2),
                       this,
                       FALLING);
    attachInterruptArg(digitalPinToInterrupt(_pin2),
                       reinterpret_cast<void (*)(void *)>(Plugin_008_interrupt1),
                       this,
                       FALLING);
  }

  // >>> Write Wiegand
  pinMode(_pinOutD0, OUTPUT);  
  pinMode(_pinOutD1, OUTPUT);
  digitalWrite(_pinOutD0, 1); // set line to IDLE state
  digitalWrite(_pinOutD1, 1); // "             "
  got_line = false;

  String log = F("Wiegand Init. D0 Pin ");
  log += _pinOutD0;
  log += F(", D1 Pin ");
  log += _pinOutD1;
  addLogMove(LOG_LEVEL_INFO, log);
  // Write Wiegand <<<

  initialised = true;
  return initialised;
}

/*****************************************************
* plugin_once_a_second
*****************************************************/
bool P008_data_struct::plugin_once_a_second(struct EventStruct *event) {
  bool success = false;

  if (initialised) {
    // >>> Write Wiegand
    _count++;
    if (_count > 10) {
      _count = 0;
      // Must be a 24 bit number max. 16777215
      //                              10000001
      uint32_t c = 10000001 + random(1, 500);
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("Wiegand Write card ");
        log += c;
        addLogMove(LOG_LEVEL_INFO, log);
      }
      outwieg26(c);
      // Write Wiegand <<<
    }
    
    if (bitCount > 0) {
      success = true; // Let's assume read succeeded
      uint64_t keyMask = 0ull;

      if ((bitCount % 4 == 0) && ((keyBuffer & 0xF) == 11)) {
        // a number of keys were pressed and finished by #
        keyBuffer = keyBuffer >> 4;                 // Strip #
      } else if (bitCount == P008_DATA_BITS) {
        // read a tag
        keyBuffer = keyBuffer >> 1;                 // Strip leading and trailing parity bits from the keyBuffer

        keyMask = (0x1ull << (P008_DATA_BITS - 2)); // Shift in 1 just past the number of remaining bits
        keyMask--;                                  // Decrement by 1 to get 0xFFFFFFFFFFFF...
        keyBuffer &= keyMask;
      } else {
        // not enough bits, maybe next time
        timeoutCount++;

        if (timeoutCount > P008_TIMEOUT_LIMIT) {
          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("RFID : reset bits: ");
            log += bitCount;
            addLogMove(LOG_LEVEL_INFO, log);
          }

          // reset after ~5 sec
          keyBuffer    = 0ull;
          bitCount     = 0u;
          timeoutCount = 0u;
        }
        success = false;
      }

      if (success) {
        uint64_t old_key = UserVar.getSensorTypeLong(event->TaskIndex);
        bool     new_key = false;

        if (P008_HEX_AS_DEC == 1) {
          keyBuffer = castHexAsDec(keyBuffer);
        }

        if (old_key != keyBuffer) {
          UserVar.setSensorTypeLong(event->TaskIndex, keyBuffer);
          new_key = true;
        }

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          // write log
          String log = F("RFID : ");

          if (new_key) {
            log += F("New Tag: ");
          } else {
            log += F("Old Tag: ");
          }
          log += (unsigned long)keyBuffer;
          log += F(", 0x");
          log += ull2String(keyBuffer, 16);
          log += F(", mask: 0x");
          log += ull2String(keyMask, 16);
          log += F(" Bits: ");
          log += bitCount;
          addLogMove(LOG_LEVEL_INFO, log);
        }

        // reset everything
        keyBuffer    = 0ull;
        bitCount     = 0u;
        timeoutCount = 0u;

        if (new_key) { sendData(event); }
        uint32_t resetTimer = P008_REMOVE_TIMEOUT;

        if (resetTimer < 250) { resetTimer = 250; }
        Scheduler.setPluginTaskTimer(resetTimer, event->TaskIndex, event->Par1);

        // Used during debugging
        // String   info;
        // uint64_t invalue  = 0x1234;
        // uint64_t outvalue = castHexAsDec(invalue);
        // info.reserve(40);
        // info += F("Test castHexAsDec(");
        // info += (double)invalue;
        // info += F(") => ");
        // info += (double)outvalue;
        // addLog(LOG_LEVEL_INFO, info);
      }
    }
  }
  return success;
}

/*****************************************************
* plugin_timer_in
*****************************************************/
bool P008_data_struct::plugin_timer_in(struct EventStruct *event) {
  if (initialised && (P008_AUTO_REMOVE == 0)) { // P008_AUTO_REMOVE check uses inversed logic!
    // Reset card id on timeout
    UserVar.setSensorTypeLong(event->TaskIndex, P008_REMOVE_VALUE);
    addLog(LOG_LEVEL_INFO, F("RFID : Removed Tag"));

    if (P008_REMOVE_EVENT == 1) {
      sendData(event);
    }
    return true;
  }
  return false;
}

/***********************************************************************
 * shift_bit_in_buffer
 **********************************************************************/
void IRAM_ATTR P008_data_struct::Plugin_008_shift_bit_in_buffer(P008_data_struct *self,
                                                                uint8_t           bit) {
  self->keyBuffer = self->keyBuffer << 1; // Left shift the number (effectively multiplying by 2)

  if (bit) { self->keyBuffer |= 1ull; }   // Add the 1 (not necessary for the zeroes)
  self->bitCount++;                       // Increment the bit count
}

/*********************************************************************
* Interrupt 1 : Handle 1 bits
*********************************************************************/
void IRAM_ATTR P008_data_struct::Plugin_008_interrupt1(P008_data_struct *self) {
  // We've received a 1 bit. (bit 0 = high, bit 1 = low)
  String log = F("1 ");
  log += millis();
  addLog(LOG_LEVEL_INFO, log);
  Plugin_008_shift_bit_in_buffer(self, 1); // Shift in a 1
}

/*********************************************************************
* Interrupt 2 : Handle 0 bits
*********************************************************************/
void IRAM_ATTR P008_data_struct::Plugin_008_interrupt2(P008_data_struct *self) {
  // We've received a 0 bit. (bit 0 = low, bit 1 = high)
  String log = F("0 ");
  log += millis();
  addLog(LOG_LEVEL_INFO, log);
  Plugin_008_shift_bit_in_buffer(self, 0); // Shift in a 0
}


// outputs ONE Wiegand bit
void P008_data_struct::outwiegbit(unsigned int b)
{
  int sel = b == 0 ? _pinOutD0 : _pinOutD1;
  digitalWrite(sel, 0);
  //String log = F("Low ");
  //log += millis();
  //addLog(LOG_LEVEL_INFO, log);
  delayMicroseconds(50);
  digitalWrite(sel, 1);
  //String log2 = F("High ");
  //log2 += millis();
  //addLog(LOG_LEVEL_INFO, log2);
  delayMicroseconds(250);
  // Removing this log message seems to break the system?? WTF?? 
  // Setting larger delays doesnt do anything.
  String log3 = F("Next ");
  log3 += millis();
  addLog(LOG_LEVEL_INFO, log3);
}

// outputs a 26 bit Wiegand code
// u32 is actually the 24-bit numeric code
void P008_data_struct::outwieg26(uint32_t u32)
{
  String binLog = F("Binary Number ");

  uint32_t tmp = u32;
  unsigned int p_even = 0;
  unsigned int p_odd = 1;
  // compute parity on trailing group of bits 
  for (int n=0; n<12; ++n)
  {
    p_odd ^= (tmp & 1);
    tmp >>= 1;
  }
  // compute parity on heading group of bits
  for (int n=12; n<24; ++n)
  {
    p_even ^= (tmp & 1);
    tmp >>= 1;
  }
  // now output data bits framed by parity ones
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    //String log = F("Start parity bit ");
    //log += p_even;
    //binLog += p_even;
    //addLog(LOG_LEVEL_INFO, log);
  }
  
  outwiegbit(p_even);
  for (int n=0; n<24; ++n)
  {
    unsigned int nextBit = (u32 >> (23-n)) & 1;
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      //String log = F("");
      //log += nextBit;    
      //binLog += nextBit;
      //addLog(LOG_LEVEL_INFO, log);
    }
    outwiegbit(nextBit);
  }
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    //String log = F("End parity bit ");
    //log += p_odd;
    //binLog += p_odd;
    //addLog(LOG_LEVEL_INFO, log);
  }
  outwiegbit(p_odd);  

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    //addLog(LOG_LEVEL_INFO, binLog);
  }
}

// output just 'meaningful' numbers
/*
void P008_data_struct::process_line(const char str[])
{
  char msg[64];
  long l = atol(str);
  if (l < 0 || l > 0xFFFFFF)
  {
    Serial.write("ERROR\n");
    return;
  }
  sprintf(msg, "OK: %ld (0x%06lX)\n", l, l);
  Serial.write(msg);
  outwieg26((unsigned long) l);
  delay(1000); // << !!! DO NOT Use delay THIS IN THIS FRAMEWORK. !!!
}
*/


#endif // ifdef USES_P008
