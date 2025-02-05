#ifndef E24LC256_H
#define E24LC256_H

#include "Wire.h"

struct E24LC256 {

  public:
    enum Status {
      UNKNOWN,
      EEPROM_NOT_FOUND,
      EEPROM_FOUND
    } EEPROMStatus = UNKNOWN;

    E24LC256(uint8_t a = 0x50) {                              // Constructor - takes the I2C address of the EEPROM (default 0x50-0x57)
      I2CAddress = a;
    }

    void init() {
      Wire.begin();
      if (ackPolling()) {                                     // See whether the EEPROM responds. We're just starting up so it should be read
        EEPROMStatus = EEPROM_FOUND;
      }
      else {
        EEPROMStatus = EEPROM_NOT_FOUND;
      }
    }

    uint8_t read(uint16_t address) {                          // Read a single byte from the memory address given.
      if (ackPolling()) {                                     // Make sure the EEPROM is ready to communicate.
        Wire.beginTransmission(I2CAddress);
        Wire.write((byte) (address >> 8));
        Wire.write((byte) (address & 0xFF));
        Wire.endTransmission();
        Wire.requestFrom(I2CAddress, (uint8_t) 1);
        return Wire.read();
      }
    }

    void write(uint16_t address, uint8_t data) {              // Write a single byte to the memory address given.
      if (ackPolling()) {                                     // Make sure the EEPROM is ready to communicate.
        Wire.beginTransmission(I2CAddress);
        Wire.write((byte) (address >> 8));
        Wire.write((byte) (address & 0xFF));
        Wire.write((byte) data);
        Wire.endTransmission();
      }
    }
    template<typename T>
    void write(uint16_t address, T data) = delete;            // Prevent accidental use of types other than `uint8_t`

    void update(uint16_t address, uint8_t data) {             // Write a single byte to the memory address given
      // if it's different from the current value.
      uint8_t a = read(address);
      if (a != data) write(address, data);
    }
    template<typename T>
    void update(uint16_t address, T data) = delete;           // Prevent accidental use of types other than `uint8_t`

    Status getStatus() {
      return EEPROMStatus;
    }

    // Put complete stuctures to EEPROM - but only if the data has changed (determined on a per-page basis).
    //
    // TODO: compare 64-byte pages on Arduino.
    //
    template <typename T> T &put(uint16_t address, T &t) {
      const uint16_t initsize = sizeof(T);                    // Size of the object given: the number of bytes to write.
      uint8_t *ptr = (uint8_t*) &t;                           // Cast object to byte array for easier handling.

#ifdef ESP8266
      const uint16_t pageSize = 64;                           // Page size of the EEPROM (ESP's I2C buffer is 128 bytes).
#else
      const uint16_t pageSize = 32;                           // The size of the I2C buffer for AVR Arduinos, use that for page size.
#endif

        /* We have three potential numbers of bytes to write:
         * 1) The maximum we can fit in the buffer - although the buffer is `pageSize`,
         *    during read/write operations two of those bytes are just for the address,
         *    leaving `pageSize - 2` bytes for data.
         * 2) The number of bytes until the page boundary
         * 3) The remaining size of the data structure
         *
         * We can only ever write the smallest of these numbers. Once written,
         * update the address, ptr values and remaining size.
         * If remaining size is zero, we're done.
         */

        const uint16_t putSize = pageSize - 2;        // Max buffer size
        uint16_t remainingDataSize = initsize;        // Remaining number of bytes to write

      while (remainingDataSize > 0) {
        uint16_t remainingPageSize = pageSize * (address / pageSize + 1) - address; // Bytes until the next page boundary.

        uint16_t nbytes = putSize;
        if (remainingPageSize < nbytes) { nbytes = remainingPageSize; }
        if (remainingDataSize < nbytes) { nbytes = remainingDataSize; }

        if (!ackPolling()) {
          return t;
        }

        readBytes(address, readBuffer, nbytes);        // Read the first page, and compare it.
        if (compareBytes(readBuffer, ptr, nbytes) == false) {
          writeBytes(address, ptr, nbytes);            // If page different: write the new data to the EEPROM.
          ackPolling();                                       // Wait for EEPROM to finish writing before continuing with the next block.
        }
        address += nbytes;
        ptr += nbytes;
        remainingDataSize -= nbytes;
      }

      return t;
    }

    template <typename T> T &get(uint16_t address, T &t) {    // Get any type of data from the EEPROM.
      uint16_t size = sizeof(T);                              // The size of the type: amount of bytes to read.
      uint8_t *ptr = (uint8_t*) &t;                           // Cast object to byte array for easier handling.
#ifdef ESP8266
      const uint8_t bufferSize = 128;                         // ESP8266's default I2C buffer size - don't read more than that in one go.
#else
      const uint8_t bufferSize = 32;                          // Arduino's default I2C buffer size - don't read more than that in one go.
#endif

      if (ackPolling()) {                                     // Make sure the EEPROM is ready to communicate.
        for (uint16_t i = 0; i < size; i += bufferSize) {     // We have to read data bufferSize bytes (or less) at a time.
          uint8_t block = bufferSize;
          if (size - i < bufferSize) {                        // Calculate remainder, if less than bufferSize bytes left to read.
            block = size - i;
          }
          readBytes(address + i, ptr, block);
          ptr += block;
        }
      }
      return t;
    }

  private:
    uint8_t I2CAddress;
    uint8_t readBuffer[64];

    void writeBytes (uint16_t address, uint8_t *ptr, uint8_t nBytes) {
      Wire.beginTransmission(I2CAddress);
      Wire.write((uint8_t) (address >> 8));
      Wire.write((uint8_t) (address & 0xFF));
      for (uint16_t i = 0; i < nBytes; i++) {
        Wire.write((uint8_t) *ptr);
        ptr++;
      }
      Wire.endTransmission();
    }

    void readBytes (uint16_t address, uint8_t *ptr, uint8_t nBytes) {
      Wire.beginTransmission(I2CAddress);
      Wire.write((byte) (address >> 8));
      Wire.write((byte) (address & 0xFF));
      Wire.endTransmission();
      Wire.requestFrom(I2CAddress, nBytes);
      for (uint8_t j = 0; j < nBytes; j++) {
        *ptr = Wire.read();                                   // Read the bytes one by one, copy them to the data object.
        ptr++;                                                // Increment the pointer.
      }
    }

    bool compareBytes(uint8_t* a, uint8_t* b, uint8_t n) {
      for (uint8_t i = 0; i < n; i++) {
        if (*a != *b) {
          return false;
        }
        a++;
        b++;
      }
      return true;
    }

    bool ackPolling() {                                       // Poll the IC to make sure it's ready for communication.
      uint32_t startPolling = micros();
      uint8_t code = 1;
      while (code != 0                                        // Continue Until We Have A Successful Ack, Or
             && micros() - startPolling < 6000) {             // Timeout: Writing Should Not Take More Than 5 Ms, Normal Is ~4.5 Ms.
        Wire.beginTransmission(I2CAddress);
        Wire.write((uint8_t) 0);
        code = Wire.endTransmission();
      }
      return (code == 0);
    }
};
#endif
