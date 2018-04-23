/***************************************************
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Battery Cavity:
//  2.5" deep
//  2" wide
//  0.75" tall

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include "Adafruit_VS1053.h"
#include <LowPower.h>
#include <avr/power.h>
#include <avr/sleep.h>

//#define NDEBUG 0
#define DEV_BOARD 0


#if DEV_BOARD == 0
// These are the pins used for the music maker breakout
#  define VS1053bRESET  8      // VS1053 reset pin
#  define VS1053bCS     7      // VS1053 chip select pin (output)
#  define VS1053bDCS    4      // VS1053 Data/command select pin (output)
#  define VS1053bPOWER  9      // PFET that disables VCC to shield.

// These are common pins between breakout and shield
#  define CARDCS 6     // Card chip select pin

// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#  define DREQ 2       // VS1053 Data request, ideally an Interrupt pin

// AMP Controls.
#  define AMP_POWER  3         // Turn on and off the AMP before cutting power to stop "pop"
#  define AMP_SHUTDOWN_US 1000  // Microsends post shutdown switch to stablize amp.

#else 
// These are the pins used for the music maker shield
#  define VS1053bRESET  -1      // VS1053 reset pin
#  define VS1053bCS     7      // VS1053 chip select pin (output)
#  define VS1053bDCS    6      // VS1053 Data/command select pin (output)
#  define VS1053bPOWER  8      // PFET that disables VCC to shield.

// These are common pins between breakout and shield
#  define CARDCS 4     // Card chip select pin

// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#  define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

#  define AMP_POWER  2         // Turn on and off the AMP before cutting power to stop "pop"
#  define AMP_SHUTDOWN_US 1000  // Microsends post shutdown switch to stablize amp.
#endif


#ifdef NDEBUG
#  define DEBUG_PRINT(x, ...)
#  define DEBUG_PRINTLN(x, ...)
#else
#  define DEBUG_PRINT(x, args...) Serial.print(x, ##args)
#  define DEBUG_PRINTLN(x, args...) Serial.println(x, ##args)
#endif

Adafruit_VS1053_FilePlayer musicPlayer =
  Adafruit_VS1053_FilePlayer(VS1053bRESET, VS1053bCS, VS1053bDCS, DREQ, CARDCS);
static boolean isDspOn = false;
static volatile boolean isSdInit = false;

#define NUM_SONGS 6
static const char* files[NUM_SONGS] = {
  "p.mp3",
  "b.mp3",
  "o.mp3",
  "y.mp3",
  "r.mp3",
  "g.mp3",
};

// Button control inputs
#define RED_BUTTON A5
#define GREEN_BUTTON A4
#define YELLOW_BUTTON A3
#define PURPLE_BUTTON A2
#define BLUE_BUTTON A1
#define ORANGE_BUTTON A0

// LED map, cause I wired one of the harnesses wrong. grr. argh.
const int buttonLedMap[6] = {
  0,  // Red
  1,  // Green
  2,  // Yellow
  3,  // Orange
  5,  // Blue
  4,  // Purple
};


enum PLayCommand {
  NO_COMMAND = 0,
  PLAY_SONG = 1,
};
static volatile byte playCommand = PLAY_SONG;
static volatile unsigned char commandCount = 0;
static volatile byte buttonState = 0;

ISR(PCINT1_vect) {
  byte newButtonState = PINC;
  // Interrupts are triggered on down and up edges. Ignore when everything is up.
  if ((newButtonState & 0x3F) != 0x3F) {
    playCommand = PLAY_SONG;
    buttonState = newButtonState;
    commandCount = commandCount + 1;
  }
  PCICR &= ~_BV(1);
}

//// VS1053b

#define SKIP_PLUGIN_VARNAME
const PROGMEM uint16_t vs1053b_patches[] = { 
//#include "/Users/awong/Desktop/vlsi/awong-ledtest/ledtest-plugin.h"
#include "/Users/awong/Desktop/vlsi/may-music/may-music-vs1053b-plugin.h"
};
#undef SKIP_PLUGIN_VARNAME

////

static void ErrorSleepForever(void);

static void inline PowerOnDsp(void) {
  isDspOn = true;
  
  digitalWrite(AMP_POWER, HIGH);
  delayMicroseconds(AMP_SHUTDOWN_US);
  DEBUG_PRINTLN(F("AMP Powered On"));

  digitalWrite(VS1053bPOWER, LOW);
  // Per VS1053 10.2, Hardware Reset, DREQ is low for about 22000 clocks.
  // At 12Mhz, that's 1.8ms. Give 4 just for good measure.
  delay(4);
  DEBUG_PRINTLN(F("DSP Powered On"));

  initMp3Shield();
}

static void inline PowerOffDsp(void) {
  DEBUG_PRINTLN("DSP Powered Off");
  /* Turn on MP3 shield */
  detachInterrupt(DREQ);
  digitalWrite(AMP_POWER, LOW);
  delayMicroseconds(AMP_SHUTDOWN_US);
  digitalWrite(VS1053bPOWER, HIGH);
  isDspOn = false;
}

static void initMp3Shield() {
  // initialise the music player
  if (!musicPlayer.begin()) {
    DEBUG_PRINTLN(F("Couldn't find VS1053, do you have the right pins defined?"));
    while(1);
    ErrorSleepForever();
  }
  DEBUG_PRINTLN(F("VS1053 found"));

  // Set volume for left, right channels. lower numbers == louder volume!
  // 90 seems like a good amplified volume.
  enum {
    VOL_INSANE = 10, // 70mA during playback.  .21mA sleeping.
    VOL_HIGH = 20,
    VOL_NORMAL = 25,  // Emprical testing for "loud enough"
    VOL_MED = 90,   // Seems to be about 2mA less than HIGH ode.
    VOL_LOW = 120,
  };
  musicPlayer.setVolume(VOL_NORMAL, VOL_NORMAL);

  DEBUG_PRINT(F("isSdInit: "));
  DEBUG_PRINTLN(isSdInit, BIN);
  if (!SD.begin(CARDCS)) {
    if (!isSdInit) {
      DEBUG_PRINTLN(F("SD failed, or not present"));
      DEBUG_PRINTLN(F("SD failed, or not present"));
      DEBUG_PRINTLN(F("SD failed, or not present"));
      DEBUG_PRINTLN(F("SD failed, or not present"));
      ErrorSleepForever();
    }
    // Second inits always generates an error from the library, but it is required.
  } else {
    isSdInit = true;
  }
  DEBUG_PRINTLN(F("SD OK!"));

  // This option uses a pin interrupt. No timers required! But DREQ
  // must be on an interrupt pin. For Uno/Duemilanove/Diecimilla
  // that's Digital #2 or #3
  // See http://arduino.cc/en/Reference/attachInterrupt for other pins
  // *** This method is preferred
  if (!musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    DEBUG_PRINTLN(F("DREQ pin is not an interrupt pin"));
    ErrorSleepForever();
  }

  musicPlayer.applyPatch(vs1053b_patches, PLUGIN_SIZE);
  musicPlayer.sciWrite(VS1053_SCI_AIADDR, 0x50);
  delay(1);    while (! musicPlayer.readyForData() );
}

static void ErrorSleepForever(void) {
  DEBUG_PRINTLN(F("Error state. Sleep Forever"));
  PowerOffDsp();
  while(1) {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}


//// Power Code.

static void sleepTillInt()         // here we put the arduino to sleep
{
#ifndef NDEBUG
  // Clear the Serial buffer so there isn't crap.
  Serial.flush();

  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
    UCSR0A |= 1 << TXC0;  // mark transmission not complete
  while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
#endif

  //power_spi_disable();
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  
  //power_spi_enable();
}

void setup() {
#ifndef NDEBUG
  Serial.begin(9600);
  DEBUG_PRINTLN(F("May Music Box"));
#endif

  // Start with DSP powered down. It is powered up on button push.
  pinMode(VS1053bPOWER, OUTPUT);
  pinMode(AMP_POWER, OUTPUT);
  PowerOffDsp();

  SPI.begin();

  // Shut down bunches of modules we don't use.
  // See PRR register 14.12.3.
  power_adc_disable();
#ifdef NDEBUG
  // No UART for console outside of debug mode.
  power_usart0_disable();
#endif
  power_twi_disable();
  DEBUG_PRINTLN("Unused Arduino Modules disabled");

  /*** Setup the control buttons here ***/
  pinMode(RED_BUTTON, INPUT_PULLUP);
  pinMode(GREEN_BUTTON, INPUT_PULLUP);
  pinMode(YELLOW_BUTTON, INPUT_PULLUP);
  pinMode(PURPLE_BUTTON, INPUT_PULLUP);
  pinMode(BLUE_BUTTON, INPUT_PULLUP);
  pinMode(ORANGE_BUTTON, INPUT_PULLUP);
  delay(30);  // Give 30 ms for the debounce capacitors to charge. 1uF + 30k at 3.3v should charge in 23ms.

  // Ensure all unused pins are in output state.
  // TODO(awong): Fix this.

  // Clear and enable Pinchange Interrupt Vector 1 for the analog pins A0-A5.
  PCIFR |= _BV(1);
  PCICR |= _BV(1);
  PCMSK1 |= 0x3F;
  DEBUG_PRINTLN(F("Button interrupts configured"));
}

void loop() {
  boolean hasNewSong = false;

  // Read the command.
  switch (playCommand) {
    case PLAY_SONG:
      hasNewSong = true;
      playCommand = NO_COMMAND;
      break;

    default:
      // Do nothing.
      break;
  }

  // Start playing a file, then we can do stuff while waiting for it to finish
  PCICR |= _BV(1);
  unsigned char lastButtonState = buttonState;
  if (hasNewSong && lastButtonState) {
    DEBUG_PRINTLN(F("Playing Song"));
    if (!isDspOn) {
      PowerOnDsp();
    }
    DEBUG_PRINTLN(commandCount);

    if (musicPlayer.playingMusic) {
      musicPlayer.stopPlaying();
    }

    DEBUG_PRINT(F("Button state:"));
    DEBUG_PRINT(lastButtonState, BIN);
    DEBUG_PRINT(F(", "));
    int curSong = NUM_SONGS - 1;
    while (lastButtonState & 0x1) {
      lastButtonState >>= 1;
      --curSong;
    }

    DEBUG_PRINT(F("Cur Song: "));
    DEBUG_PRINTLN(curSong);
    if (curSong < NUM_SONGS) {
      const char* currentFile = files[curSong];

      DEBUG_PRINT(F("Current file: "));
      DEBUG_PRINTLN(currentFile);
      int led_value = buttonLedMap[5 - curSong];
      DEBUG_PRINT(F("Led Value: "));
      DEBUG_PRINTLN(led_value);
      musicPlayer.sciWrite(VS1053_SCI_AICTRL0, led_value);
      if (! musicPlayer.startPlayingFile(currentFile)) {
        DEBUG_PRINT(F("Could not open file "));
      }
    
      DEBUG_PRINTLN(F("Started playing"));
    } else {
      DEBUG_PRINT(F("!! Out or range song: "));
      DEBUG_PRINTLN(curSong);
    }
  }

  if (musicPlayer.playingMusic) {
#ifndef NDEBUG
    static unsigned int counter = 0;
    if (counter++ % 50 == 0) {
      DEBUG_PRINT(F("."));
    }
#endif
  } else if (isDspOn) {
#ifndef NDEBUG
    static unsigned int counter = 0;
    if (counter++ % 50 == 0) {
      DEBUG_PRINT(F("_s_"));
    }
#endif
    PowerOffDsp();
  }

  // Wait until something wakes us.
  sleepTillInt();
}
