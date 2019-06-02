#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "Adafruit_MPR121.h"
#include "BluefruitConfig.h"
#include "pitchToNote.h"

#define FACTORYRESET_ENABLE       0 //If 1, the BLE device will factory reset, including any custom device name
#define MINIMUM_FIRMWARE_VERSION  "0.7.0"
#define NUM_CAP_PADS              12

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// Which note should be played for each capacitive pad

//C Major
//const byte note_pitches[NUM_CAP_PADS] = {pitchC3, pitchD3, pitchE3, pitchF3, pitchG3, pitchA3, pitchB3, pitchC4, pitchD4, pitchE4, pitchF4, pitchG4};

//Major pentatonic, white-key transposed
const byte note_pitches[NUM_CAP_PADS] = {pitchC4, pitchD4, pitchE4, pitchG4, pitchA4, pitchC5, pitchD5, pitchE5, pitchG5, pitchA5, pitchC6, pitchD6};

//Egyptian, suspended, white-key transposed
//const byte note_pitches[NUM_CAP_PADS] = {pitchG4, pitchA4, pitchC5, pitchD5, pitchF5, pitchG5, pitchA5, pitchC6, pitchD6, pitchF6, pitchG6, pitchA6};

//#define CAP_DEBUG 1 //Comment this entire line out to turn off cap debug

uint16_t max_filtered_vals[NUM_CAP_PADS];
uint16_t filtered_data[NUM_CAP_PADS];

#ifdef CAP_DEBUG
uint16_t baseline_data[NUM_CAP_PADS];
#endif

bool isConnected = false;
int current_note = note_pitches[0];

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  isConnected = true;
  Serial.println(F("BLE connected."));
  delay(1000);
}

void disconnected(void)
{
  Serial.println("BLE disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midi.send(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midi.send(0x80 | channel, pitch, velocity);
}

void setup_ble_midi()
{
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("Bluefruit found.") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enabling MIDI."));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.println(F("Waiting for BLE connection..."));
}

void setup_cap()
{
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A))
  {
    while (1)
    {
      Serial.println("Capacitive touch sensor not found. I can't continue.");
      delay(2000);
    }
  }
  Serial.println("Capacitive touch sensor found.");

  for (uint8_t i = 0; i < NUM_CAP_PADS; i++)
  {
    max_filtered_vals[i] = 0;
  }
}

//Manual calculation of is-touched bitmask. Call it "touched" if the value is below the max by at least 10%
uint16_t detect_touched()
{
  uint16_t touched_mask = 0;
  for (uint8_t i = 0; i < NUM_CAP_PADS; i++) 
  {
    filtered_data[i] = cap.filteredData(i);
    if (filtered_data[i] > max_filtered_vals[i])
    {
      max_filtered_vals[i] = filtered_data[i];
    }

    if (filtered_data[i] < (uint16_t)(0.9 * (float)max_filtered_vals[i]))
    {
      touched_mask |= _BV(i);
    }
  }
  return touched_mask;
}

void setup(void)
{
  delay(500);

  Serial.begin(115200);
  setup_ble_midi();
  setup_cap();
}

void loop(void)
{
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  // bail if not connected
  if (!isConnected)
  {
    //Serial.println(F("Waiting for BLE connection..."));
    delay(1000); //This might not work. It may block future connections. Test disconnecting and reconnecting to see if this needs more work.
  }

  // Get the currently touched pads
  //currtouched = cap.touched();
  currtouched = detect_touched();
  
#ifdef CAP_DEBUG
  for (uint8_t i = 0; i < NUM_CAP_PADS; i++) 
  {
    filtered_data[i] = cap.filteredData(i);
    baseline_data[i] = cap.baselineData(i);
  }

  Serial.print("\n\nFiltered: ");
  for (uint8_t i = 0; i < NUM_CAP_PADS; i++) 
  {
    Serial.print(filtered_data[i]);Serial.print("  ");
  }
  Serial.print("\nBaseline: ");
  for (uint8_t i = 0; i < NUM_CAP_PADS; i++) 
  {
    Serial.print(baseline_data[i]);Serial.print("  ");
  }
  Serial.print("\nTouched: "); Serial.println(currtouched, BIN);
  delay(250);
#else
  for (uint8_t i = 0; i < NUM_CAP_PADS; i++) 
  {
    // rising edge: wasn't touched, now touched
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
      noteOn(0, note_pitches[i], 64);
    }
    
    // falling edge: was touched, now isn't touched
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
      noteOff(0, note_pitches[i], 64);
    }
  }
#endif

  // reset our state
  lasttouched = currtouched;
}
