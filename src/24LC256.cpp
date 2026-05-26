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
