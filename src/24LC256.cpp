#include "24LC256.h"

E24LC256::E24LC256(uint8_t a) : I2CAddress(a) {}

void E24LC256::init() {
  Wire.begin();
  if (ackPolling()) {  // See whether the EEPROM responds. We're just starting up so it should be read
    EEPROMStatus = EEPROM_FOUND;
  } else {
    EEPROMStatus = EEPROM_NOT_FOUND;
  }
}

E24LC256::Status E24LC256::getStatus() {
  return EEPROMStatus;
}

uint8_t E24LC256::read(uint16_t address) {  // Read a single byte from the memory address given.
  if (ackPolling()) {                       // Make sure the EEPROM is ready to communicate.
    Wire.beginTransmission(I2CAddress);
    Wire.write((byte)(address >> 8));
    Wire.write((byte)(address & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(I2CAddress, (uint8_t)1);
    return Wire.read();
  }
  return 0;
}

void E24LC256::write(uint16_t address, uint8_t data) {  // Write a single byte to the memory address given.
  if (ackPolling()) {                                   // Make sure the EEPROM is ready to communicate.
    Wire.beginTransmission(I2CAddress);
    Wire.write((byte)(address >> 8));
    Wire.write((byte)(address & 0xFF));
    Wire.write((byte)data);
    Wire.endTransmission();
  }
}

void E24LC256::update(uint16_t address, uint8_t data) {  // Write a single byte to the memory address given
  // if it's different from the current value.
  uint8_t a = read(address);
  if (a != data) write(address, data);
}

void E24LC256::writeBytes(uint16_t address, uint8_t *ptr, uint8_t nBytes) {
  Wire.beginTransmission(I2CAddress);
  Wire.write((uint8_t)(address >> 8));
  Wire.write((uint8_t)(address & 0xFF));
  for (uint16_t i = 0; i < nBytes; i++) {
    Wire.write((uint8_t)*ptr);
    ptr++;
  }
  Wire.endTransmission();
}

void E24LC256::readBytes(uint16_t address, uint8_t *ptr, uint8_t nBytes) {
  Wire.beginTransmission(I2CAddress);
  Wire.write((byte)(address >> 8));
  Wire.write((byte)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(I2CAddress, nBytes);
  for (uint8_t j = 0; j < nBytes; j++) {
    *ptr = Wire.read();  // Read the bytes one by one, copy them to the data object.
    ptr++;               // Increment the pointer.
  }
}

bool E24LC256::compareBytes(uint8_t *a, uint8_t *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (*a != *b) {
      return false;
    }
    a++;
    b++;
  }
  return true;
}

bool E24LC256::ackPolling() {  // Poll the IC to make sure it's ready for communication.
  uint32_t startPolling = micros();
  uint8_t code          = 1;
  while (code != 0                             // Continue Until We Have A Successful Ack, Or
         && micros() - startPolling < 6000) {  // Timeout: Writing Should Not Take More Than 5 Ms, Normal Is ~4.5 Ms.
    Wire.beginTransmission(I2CAddress);
    Wire.write((uint8_t)0);
    code = Wire.endTransmission();
  }
  return (code == 0);
}

void E24LC256::_put(uint16_t address, uint8_t *ptr, uint16_t nBytes) {
#ifdef ESP8266
  const uint16_t pageSize = 64;  // Page size of the EEPROM (ESP's I2C buffer is 128 bytes).
#else
  const uint16_t pageSize = 32;  // The size of the I2C buffer for AVR Arduinos, use that for page size.
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

  const uint16_t putSize     = pageSize - 2;  // Max buffer size
  uint16_t remainingDataSize = nBytes;        // Remaining number of bytes to write

  while (remainingDataSize > 0) {
    uint16_t remainingPageSize = pageSize * (address / pageSize + 1) - address;  // Bytes until the next page boundary.

    uint16_t nbytes            = putSize;
    if (remainingPageSize < nbytes) { nbytes = remainingPageSize; }
    if (remainingDataSize < nbytes) { nbytes = remainingDataSize; }

    if (!ackPolling()) {
      return;
    }

    readBytes(address, readBuffer, nbytes);  // Read the first page, and compare it.
    if (compareBytes(readBuffer, ptr, nbytes) == false) {
      writeBytes(address, ptr, nbytes);  // If page different: write the new data to the EEPROM.
      ackPolling();                      // Wait for EEPROM to finish writing before continuing with the next block.
    }
    address += nbytes;
    ptr += nbytes;
    remainingDataSize -= nbytes;
  }
}

void E24LC256::_get(uint16_t address, uint8_t *ptr, uint16_t nBytes) {
#ifdef ESP8266
  const uint8_t bufferSize = 128;  // ESP8266's default I2C buffer size - don't read more than that in one go.
#else
  const uint8_t bufferSize = 32;  // Arduino's default I2C buffer size - don't read more than that in one go.
#endif

  if (ackPolling()) {                                    // Make sure the EEPROM is ready to communicate.
    for (uint16_t i = 0; i < nBytes; i += bufferSize) {  // We have to read data bufferSize bytes (or less) at a time.
      uint8_t block = bufferSize;
      if (nBytes - i < bufferSize) {  // Calculate remainder, if less than bufferSize bytes left to read.
        block = nBytes - i;
      }
      readBytes(address + i, ptr, block);
      ptr += block;
    }
  }
}
