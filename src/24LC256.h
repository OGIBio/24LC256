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

    E24LC256(uint8_t a = 0x50);  // Constructor - takes the I2C address of the EEPROM (default 0x50-0x57)

    void init();

    uint8_t read(uint16_t address);

    void write(uint16_t address, uint8_t data);

    template<typename T>
    void write(uint16_t address, T data) = delete;  // Prevent accidental use of types other than `uint8_t`

    void update(uint16_t address, uint8_t data);

    template<typename T>
    void update(uint16_t address, T data) = delete;  // Prevent accidental use of types other than `uint8_t`

    Status getStatus();

    // Put complete stuctures to EEPROM - but only if the data has changed (determined on a per-page basis).
    //
    // TODO: compare 64-byte pages on Arduino.
    //
    template<typename T> T &put(uint16_t address, T &t) {
      const uint16_t initsize = sizeof(T);      // Size of the object given: the number of bytes to write.
      uint8_t *ptr            = (uint8_t *)&t;  // Cast object to byte array for easier handling.
      _put(address, ptr, initsize);

      return t;
    }

    template<typename T> T &get(uint16_t address, T &t) {  // Get any type of data from the EEPROM.
      uint16_t size = sizeof(T);                           // The size of the type: amount of bytes to read.
      uint8_t *ptr  = (uint8_t *)&t;                       // Cast object to byte array for easier handling.
      _get(address, ptr, size);

      return t;
    }

  private:
    uint8_t I2CAddress;
    uint8_t readBuffer[64];

    void writeBytes(uint16_t address, uint8_t *ptr, uint8_t nBytes);

    void readBytes(uint16_t address, uint8_t *ptr, uint8_t nBytes);

    bool compareBytes(uint8_t *a, uint8_t *b, uint8_t n);

    bool ackPolling();

    void _put(uint16_t address, uint8_t *ptr, uint16_t nBytes);
    void _get(uint16_t address, uint8_t *ptr, uint16_t nBytes);
};
#endif
