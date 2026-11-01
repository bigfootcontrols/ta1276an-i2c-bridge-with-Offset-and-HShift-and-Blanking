/* TA1276AN I2C Bridge w Contrast, Color, and Position Offsets, and Blanking
 *
 * Based on bsafta's fork of TA1276AN I2C Bridge by
 * Martin Hejnfelt (2019) (martin@hejnfelt.com)
 * basfta fork: https://github.com/bsafta/ta1276an-i2c-bridge 
 * original: https://github.com/skumlos/ta1276an-i2c-bridge 
 * 
    * Original Description (my changes described under v1.4):
    *
    * For use with monitors that use the TA1276AN jungle IC.
    * This changes the I2C communication from monitor MCUs
    * that doesn't treat RGB like it should, to something
    * that does. Basically it ensures that RGB Brightness
    * and RGB Contrast are in sync with the "other"
    * brightness and contrast values, plus kills the "HI BRT"
    * bit which otherwise causes a somewhat dull picture (or
    * close to black image when 75 Ohm terminations resistors
    * are used (which they should).
    * Connect the main I2C line of the monitors PCB to the
    * hardware I2C (TWI) pins and then specify the desired
    * secondary I2C pins for the communication to the jungle.
    * A speedy Arduino is recommended, for now only tested on:
    * - 20MHz atmega2560 clone
    * - 16MHz Nano v3.0 clone
    * - 16MHz Mini Pro
    *
    * Requires the SoftI2CMaster library
    * https://github.com/felias-fogg/SoftI2CMaster
    *
    * Monitors tested:
    * JVC DT-V100CG (initial version)
    * JVC TM-H1375SU (version 1.1)
    * JVC TM-1011G (version 1.3)
    * JVC TM-H140PN (improved stability with 1.2, credit to okgrak for testing)
    *
    *  Version 1.3 - Tested on TM-1011G
    *  RGB contrast and brightness can be adjusted independently in the TA1276 ADJ.
    *  service menu as an offset to composite adjustments.
    *  [TM-1011G] RGB brightness is adjustable in the TA1276 ADJ. T[A06]
    *  [TM-1011G] RGB contrast is adjustable in the TA1276 ADJ. T[A07]
 *
 *  Version 1.4 - Tested on RCA MM36110 with Arduino Nano v3
 *  - Substantially refactored the code to significantly speed up Interrupt Service Routine (ISR) functions.
 *    Specifically, under the prior versions, almost all calculations/modifications and writes to the jungle chip
 *    occur DURING writeRequest(). But my specific monitor's MCU is EXTREMELY picky (not sure if it even supports 
 *    clock stretching) and it will freak out and shut the set down anytime anything remotely unexpected happens. 
 *    Doing all the work during writeRequest() increases both the chance of the Arduino taking too long to 
 *    communicate with the MCU, and the chance of ISR functions conflicting with other stuff, i.e. the functions 
 *    called from loop(). Now, writeRequest() literally just reads from the MCU and stores the values in a ring buffer.
 *    The ring buffer is then processed in loop().
 *  - Changed bsafta's offset functionality such that offsets are user-defined in the code
 *   using arbitrary signed integer values instead of needing to use values set in service menu (which
 *   may be problematic on some units including MM36110 depending on service menu setting availability)
 *  - Added offset for horizontal position. On many sets, running RGB sync through existing factory
 *   inputs will cause a horizontal shift that needs corrected. Some people do it in the service menu,
 *   but then your factory video inputs will be off the other direction. This allows an offset only
 *   while mod is active. HOWEVER, I ended up finding out that this doesn't work on the MM36110 as the jungle's
 *  deflection controls are not used. I've commented-out the code so it doesn't use resources, but it could
 *  be commented back in. 
 *  - Added the ability to turn the mod on and off using a specified pin. Pin needs supplied with 5v. 
 *   You can either use a switch from vcc, or some other method. I personally use another arduino running
 *   IR receiver software, and that sends out an enabling signal when a certain button is long pressed.
 *  - Added blanking output. Arduino will send a blanking signal so you don't need a blanking switch. 
 *  - Added functionality to turn velocity scan modulation off if a pin is set high. Independently checks
 *    for this regardless of whether RGB mod pin is high. 
 *
 */
 //Version 1.4

// THE FOLLOWING THREE VALUES SHOULD BE SET BASED ON YOUR PERSONAL PREFERENCES FOR YOUR MONITOR.
// THESE ARE SET TO WHAT WORKED FOR ME ON MY RCA MM36110, YOUR MILEAGE MAY VARY
// 0 = no offset, positive integers = positive offset, negative = negative

const int8_t rgbOffsetBrightness = 10; //Tells the jungle how much to offset RGB brightness from regular brightness. 
const int8_t rgbOffsetContrast = 10; //Tells the jungle how much to offset Contrast from regular brightness.
const int8_t hPosOffset = 0; //Tells the jungle how much to offset horizontal position. 

// Values that tell functions whether RGB and VM mods are running or not. Don't mess with this except for testing purposes.
volatile bool RGBModEnabled = false;
volatile bool VSMDisabled = false;

//this indicates if an I2C write is in process (also used to indicate if other critical stuff in process)
volatile bool wireActive = false;

// Both of these may need adjusted depending on monitor in question. 
#define READ_DELAY 100 // how often we read from the jungle chip
#define CHECK_DELAY 250 // how often we check for status of the mod pins

// Some setup values for SoftI2CMaster. Probably don't need to adjust these unless you have
// an unusual problem.
#define I2C_TIMEOUT 10000
#define I2C_PULLUP 0 //don't use internal pullups. You need external pullups to VCC on pins 4/5
#define I2C_FASTMODE 1

// These work for Nano 3.0
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5

// Note: the following pins are activated and deactivated via direct port manipulation
// in this version of the code to save clock time. If you change the pins, you have to make sure the implementation
// in loop() and setup() is still correct. 
#define RGB_PIN 6 // = PD6 - this is the pin that will be used to activate the mod - use a 10k pulldown to ground
#define BLANKING_PIN 7 // = PD7 - this pin will supply blanking voltage
#define VSM_PIN 8 //PD8 - this pin when high will disable VSM - use a 10k pulldown to ground

#include <SoftI2CMaster.h>
#include <Wire.h>
// Address in the datasheet is said to be 0x88 for write
// and 0x89 for read. That is somewhat of a "mistake" as
// i2c uses 7 bit addressing and the least significant bit
// is read (1) or write (0). Thus the address is shifted once
// to the right to get the "real" address which is then 44h/68
#define TA1276AN_ADDR (68)

#define REG_UNICOLOR (0x00) // Regular contrast register
#define REG_BRIGHTNESS (0x01) // Regular brightness register
#define REG_RGB_BRIGHTNESS (0x05) // RGB brightness register
#define REG_RGB_CONTRAST (0x06) // RGB contrast register
#define REG_H_POS (0x0B) // Horizontal Position Register
#define REG_VM (0x11) // Velocity Modulation Register

uint8_t brightness = 0x80;
uint8_t contrast = 0x80;

// --- Ring buffer for deferring I2C writes out of ISR context ---
// Each entry stores one complete I2C write transaction received
// from the monitor MCU: a starting register address followed by
// one or more data bytes.

#define MSG_MAX_LEN  25   // max payload per message (reg + up to 24 data bytes - TA1276AN has 24 registers)
#define RING_BUF_SIZE 8   // number of message slots (must be power of 2)

struct I2CMessage {
  uint8_t data[MSG_MAX_LEN];
  uint8_t len;
};

static volatile I2CMessage ringBuf[RING_BUF_SIZE];
static volatile uint8_t rbHead = 0; // written by ISR
static volatile uint8_t rbTail = 0; // read by loop()

// --- Helper: enqueue a message (called from ISR) ---
static inline bool ringBufPut(const uint8_t *buf, uint8_t len) {
  uint8_t nextHead = (rbHead + 1) & (RING_BUF_SIZE - 1);
  if (nextHead == rbTail) {
    return false; // buffer full – drop message
  }
  volatile I2CMessage *slot = &ringBuf[rbHead];
  for (uint8_t i = 0; i < len; i++) {
    slot->data[i] = buf[i];
  }
  slot->len = len;
  rbHead = nextHead;
  return true;
}

// --- Helper: dequeue a message (called from loop) ---
static inline bool ringBufGet(uint8_t *buf, uint8_t *len) {
  if (rbTail == rbHead) {
    return false; // nothing pending
  }
  volatile I2CMessage *slot = &ringBuf[rbTail];
  *len = slot->len;
  for (uint8_t i = 0; i < *len; i++) {
    buf[i] = slot->data[i];
  }
  rbTail = (rbTail + 1) & (RING_BUF_SIZE - 1);
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.print("TA1276AN I2C Bridge\n");

  //configure pins
  pinMode(RGB_PIN, INPUT); //configure switch pin as input 
  pinMode(VSM_PIN, INPUT); //configure VM pin as input
  pinMode(BLANKING_PIN, OUTPUT); //configure blanking pin as output

  //I2C startup
  bool iicinit = i2c_init();
  Wire.begin(TA1276AN_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  if(!iicinit) Serial.println("I2C init failed");

  //Set initial states
  RGBModEnabled = (PIND & (1 << RGB_PIN)); //direct port manipulation for increased speed
  VSMDisabled = (PINB & (1 << (VSM_PIN - 8))); 
  if (RGBModEnabled) {
        PORTD |= (1 << BLANKING_PIN);   // set PD7 HIGH
      } else {
        PORTD &= ~(1 << BLANKING_PIN);  // set PD7 LOW
      }
}

// Called from other functions, it's a shortcut for writing values without needing to
// specify all the steps.
void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TA1276AN_ADDR<<1)|I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop(); 
}


// --- ISR callback: just capture bytes into the ring buffer, don't process ---
void writeRequest(int byteCount) {
  wireActive = true;
  uint8_t tmp[MSG_MAX_LEN];
  uint8_t len = 0;
  while (Wire.available() && len < MSG_MAX_LEN) {
    tmp[len++] = Wire.read();
  }
  // Drain anything that didn't fit (shouldn't happen)
  while (Wire.available()) {
    Wire.read();
  }
  if (len >= 2) {
    ringBufPut(tmp, len);
  }
  wireActive = false;
}

// --- Process a single queued message (called from loop) ---
// This contains the same register-manipulation logic that
// was previously inside the ISR.
void processMessage(const uint8_t *buf, uint8_t len) {
  uint8_t reg = buf[0];
  uint8_t byteCount = len; // total bytes including register byte

  if (byteCount > 2) {
    // Multi-byte (burst) write: register + N data bytes
    i2c_start((TA1276AN_ADDR<<1)|I2C_WRITE);
    i2c_write(reg);
    for (uint8_t i = 1; i < byteCount; i++) {
      uint8_t val = buf[i];
      //If mod is activated, processes commands as follows, otherwise val passes unchanged
      if (RGBModEnabled == true) { 
        switch (reg) {
          case REG_UNICOLOR:
            contrast = val;
            break;
          case REG_BRIGHTNESS:
            brightness = val;
            break;
          case REG_RGB_BRIGHTNESS:
            val = getRgbBrightnessRegValue(brightness);
            break;
          case REG_RGB_CONTRAST:
            val = getRgbContrastRegValue(contrast);
            break;
          /*case REG_H_POS:
            val = getHPosOffsetValue(val);
            break;*/
          default:
            break;
        }
      }
      //Turn VSM off if VSM-disable mod is activated
      if (reg == REG_VM) {
        if (VSMDisabled == true) {
          val = val | 0b00110000; //set bits 5 and 6 to 1 to set VSM gain to "off"
        }
      }
      i2c_write(val);
      ++reg;
    }
    i2c_stop();
  } else {
    // Single register write: register + 1 data byte
    uint8_t val = buf[1];
    // apply modifications if mod enabled, otherwise leave val unchanged
    switch (reg) {
      case REG_UNICOLOR:
        contrast = val;
        writeRegister(reg, val);
        // REG_RGB_CONTRAST needs to be set when REG_UNICOLOR changes, but only if mod enabled
        if (RGBModEnabled == true) {writeRegister(REG_RGB_CONTRAST, getRgbContrastRegValue(contrast));}
        break;
      case REG_BRIGHTNESS:
        brightness = val;
        writeRegister(reg, val);
        // REG_RGB_BRIGHTNESS needs to be set when REG_BRIGHTNESS changes, but only if mod enabled
        if (RGBModEnabled == true) {writeRegister(REG_RGB_BRIGHTNESS, getRgbBrightnessRegValue(brightness));}
        break;
      case REG_RGB_BRIGHTNESS:
        // do nothing. Already set last time MCU did single-register write to brightness.
        break;
      case REG_RGB_CONTRAST:
        //do nothing. Already set last time MCU did single-register write to unicolor.
        break;
      /*case REG_H_POS:
        //offset h position, but only if mod enabled.
       if (RGBModEnabled == true) {
          val = getHPosOffsetValue(val);
       }
       writeRegister(reg, val);
        break;*/
      case REG_VM:
        if (VSMDisabled == true) {
          val = val | 0b00110000; //set bits 5 and 6 to "11" to set VSM gain to "off", leave rest of bits unchanged
          Serial.println("vm disabled");
        }
        writeRegister(reg, val);
        break;
      default:
        writeRegister(reg, val);
        break;
      }
  }
}

// Offsets standard contrast value by user-defined offset to arrive at RGB contrast value
uint8_t getRgbContrastRegValue(uint8_t contrastVal) {
  uint8_t rgbContrast = add_with_signed_offset(contrastVal, rgbOffsetContrast);
  // Write the RGB Contrast register (0x06) value
  // MSB controls HI BRT (High-bright color) where 0 = OFF, 1 = ON
  // Hejnfelt said to make sure to kill most significant bit
  // otherwise picture gets dark. On my particular set Hi Brt has little
  // to no effect one way or the other, so I left this mod in. 
  uint8_t regValue = rgbContrast & 0x7F;

  //for debugging, DO NOT uncomment unless you need to, slows down operation
  /*Serial.print("Original Contrast: "); 
  Serial.print(contrastVal, BIN);
  Serial.print(";   RGB Contrast: ");
  Serial.print(regValue, BIN);
  Serial.print("\n\n");*/

  return regValue;
}

// Offsets standard brightness value by user-defined offset to arrive at RGB contrast value
uint8_t getRgbBrightnessRegValue(uint8_t brightnessVal) {
  uint8_t rgbBrightness = add_with_signed_offset(brightnessVal, rgbOffsetBrightness << 1); //shift offset 1 to the left as the LSB is a separate setting
  // LSB controls WPSL (White Peak Suppression Level) where 0 = 130IRE,
  // 1 = 110IRE
  // this version clears that bit to set it to 0, YMMV
  uint8_t regValue = (rgbBrightness>>1)<<1; 
  
  //for debugging, don't uncomment unless you need it
  /*Serial.print("Original Brightness: "); 
  Serial.print(brightnessVal, BIN);
  Serial.print(";   RGB Brightness: ");
  Serial.print(regValue, BIN);
  Serial.print("\n\n");*/

  return regValue;
}

// Offsets horizontal position by user-defined value
//uint8_t getHPosOffsetValue(uint8_t hPos) {
  //uint8_t regValue = add_with_signed_offset(hPos, hPosOffset << 3); //shift offset 3 to the left because rightmost bits control other things we don't want to mess with

  // For debugging, don't uncomment unless you need it
  /*Serial.print("Original HPOS: "); 
  Serial.print(hPos, BIN);
  Serial.print(";   Modified HPOS: ");
  Serial.print(regValue, BIN);
  Serial.print("\n\n");*/

 // return regValue;
//}

// Math logic used by each of the above offset functions, can accept negative values
uint8_t add_with_signed_offset(uint8_t base, int8_t offset) {
  int16_t sum = (int16_t)base + (int16_t)offset;
  if (sum > 255) return 255;
  if (sum < 0) return 0;
  return (uint8_t)sum;
}

// Set up two-byte array to hold values read from TA1276AN
uint8_t r[2] = {0, 0};

// Called by MCU read requests
void readRequest() {
  Wire.write(r,2);
}

// Called from loop, this occasionally fills the two-byte array defined above. 
void read() {
  if (wireActive) return; // A slave transaction is in progress, skip

  if (!i2c_start((TA1276AN_ADDR << 1) | I2C_READ)) {
    i2c_stop();
    return;
  }
  uint8_t tmp0 = i2c_read(false); //read byte and proceed to next
  uint8_t tmp1 = i2c_read(true);  // send stop signal
  i2c_stop();

  noInterrupts();
  r[0] = tmp0; //write to array
  r[1] = tmp1;
  interrupts();
}

void loop() {
  static unsigned long lastRead = 0;
  static unsigned long lastPinCheck = 0;
  unsigned long now = millis();

  // Drain the ring buffer continuously
  uint8_t msg[MSG_MAX_LEN];
  uint8_t msgLen;
  while (ringBufGet(msg, &msgLen)) {
    processMessage(msg, msgLen);
  }

  // Periodic read from the TA1276AN
  if (now - lastRead >= READ_DELAY) {
    lastRead = now;
    read();
  }

  // Check mod status pins periodically
  if (now - lastPinCheck >= CHECK_DELAY) {
    lastPinCheck = now;
    if (!wireActive) {
      RGBModEnabled = (PIND & (1 << RGB_PIN)); //direct port manipulation for increased speed
      VSMDisabled = (PINB & (1 << (VSM_PIN - 8))); // pin 8 = PB0
      if (RGBModEnabled) {
        PORTD |= (1 << BLANKING_PIN);   // set PD7 HIGH
      } else {
        PORTD &= ~(1 << BLANKING_PIN);  // set PD7 LOW
      }
    }
  }
}
