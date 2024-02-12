/*
  Based on https://github.com/LucasEtchezuri/Arduino-ADS131M04/blob/ed879ecd0da5ac0d1dfa0494ff65b1e7ad4ae6d5/ADS131M04.cpp
  We cannot use it as is because it is not correctly compiled on default Arduino IDE targeting original Arduinos (e.g., Arduino Uno, Nano):
  - It passes arguments to SPI.begin(), while original SPI.begin() does not accept arguments
    - https://github.com/LucasEtchezuri/Arduino-ADS131M04/issues/1
    - https://www.arduino.cc/reference/en/language/functions/communication/spi/begin/
  - It operates bit shift greater than variable width. This causes undefined behavior
    - https://github.com/LucasEtchezuri/Arduino-ADS131M04/issues/1
    - https://mfumi.hatenadiary.org/entry/20111217/1324137736
  Additional changes:
  - Call SPI.beginTransaction() and SPI.endTransaction() every time to accept other SPI devices
    - https://stupiddog.jp/note/archives/976
  - Set chip select (CS) high in begin() to accept other SPI devices
  - Fix some typos & Spanish comments
  - Use CRC to detect SPI communication error
  - Enable to change SPI speed
*/

#include "Arduino.h"
#include "ADS131M04.h"
#include "SPI.h"
#include "CRC.h"  // Search and install CRC on library manager of Arduino IDE
// https://www.arduino.cc/reference/en/libraries/crc/

ADS131M04::ADS131M04()
{
}

uint8_t ADS131M04::writeRegister(uint8_t address, uint16_t value)
{
  uint16_t res;
  uint8_t addressRcv;
  uint8_t bytesRcv;
  uint16_t cmd = 0;

  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  cmd = (CMD_WRITE_REG) | ((uint16_t)address << 7) | 0;

  //res = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(value);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  res = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);
  SPI.endTransaction();

  addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
  bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

  if (addressRcv == address)
  {
    return bytesRcv + 1;
  }
  return 0;
}

void ADS131M04::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  // Write a value to the register, applying the mask to touch only the necessary bits.
  // It does not perform the bit shift, the shifted value must be moved to the correct position.

  // Read the current contents of the register
  uint16_t register_contents = readRegister(address);

  // Bit by bit change of the mask (1 in the bits not to be touched and 0 in the bits to be modified).
  // AND is performed on the current contents of the register. There are "0" in the part to be modified.
  register_contents = register_contents & ~mask;

  // OR is performed with the value to be loaded into the register. Note that the value must be in the correct position (shitf).
  register_contents = register_contents | value;

  // Rewrite the register
  writeRegister(address, register_contents);
}

uint16_t ADS131M04::readRegister(uint8_t address)
{
  uint16_t cmd;
  uint16_t data;

  cmd = CMD_READ_REG | ((uint16_t)address << 7 | 0);

  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  //data = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  data = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);
  SPI.endTransaction();
  return data;
}

void ADS131M04::begin(uint8_t cs_pin, uint8_t drdy_pin, uint32_t spi_speed)
{
  // Set pins up
  ADS131M04_CS_PIN = cs_pin;
  ADS131M04_DRDY_PIN = drdy_pin;

  SPI.begin();
  // Configure chip select as an output
  pinMode(ADS131M04_CS_PIN, OUTPUT);
  // Set chip select high to accept other SPI devices
  digitalWrite(ADS131M04_CS_PIN, HIGH);
  delayMicroseconds(1);  // Should be longer than t_w(CSH)?
  // Configure DRDY as an input
  pinMode(ADS131M04_DRDY_PIN, INPUT);

  // Store SPI settings
  SPI_SETTINGS = SPISettings(spi_speed, MSBFIRST, SPI_MODE1);
}

int8_t ADS131M04::isDataReadySoft(byte channel)
{
  if (channel == 0)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }
  else if (channel == 2)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY2);
  }
  else if (channel == 3)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY3);
  }
  else
  {
    return -1;
  }
}

bool ADS131M04::isResetStatus(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

bool ADS131M04::isLockSPI(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M04::setDrdyFormat(uint8_t drdyFormat)
{
  if (drdyFormat > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
    return true;
  }
}

bool ADS131M04::setDrdyStateWhenUnavailable(uint8_t drdyState)
{
  if (drdyState > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
  }
}

bool ADS131M04::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
    return true;
  }
}

bool ADS131M04::setOsr(uint16_t osr)
{
  if (osr > 7)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
    return true;
  }
}

bool ADS131M04::setChannelEnable(uint8_t channel, uint16_t enable)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
    return true;
  }
}

bool ADS131M04::setChannelPGA(uint8_t channel, uint16_t pga)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
    return true;
  }
}

void ADS131M04::setGlobalChop(uint16_t global_chop)
{
  writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M04::setGlobalChopDelay(uint16_t delay)
{
  writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M04::setInputChannelSelection(uint8_t channel, uint8_t input)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
}

bool ADS131M04::setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_OCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_OCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_OCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_OCAL_LSB, (uint16_t)LSB << 8 , REGMASK_CHX_OCAL0_LSB);
    return true;
  }
}

bool ADS131M04::setChannelGainCalibration(uint8_t channel, uint32_t gain)
{

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_GCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_GCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_GCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_GCAL_LSB, (uint16_t)LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
}

bool ADS131M04::isDataReady()
{
  if (digitalRead(ADS131M04_DRDY_PIN) == HIGH)
  {
    return false;
  }
  return true;
}

bool ADS131M04::readADC(adcOutput* reading)
{
  uint32_t x = 0;
  uint32_t x2 = 0;
  uint32_t x3 = 0;
  uint8_t crc_inputs[15];
  uint16_t crc_received = 0;
  int32_t aux;

  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(ADS131M04_CS_PIN, LOW);
  delayMicroseconds(1);

  x = crc_inputs[0] = SPI.transfer(0x00);
  x2 = crc_inputs[1]  = SPI.transfer(0x00);
  crc_inputs[2] = SPI.transfer(0x00);

  reading->status = ((x << 8) | x2);

  x = crc_inputs[3] = SPI.transfer(0x00);
  x2 = crc_inputs[4] = SPI.transfer(0x00);
  x3 = crc_inputs[5] = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    reading->ch0 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    reading->ch0 = aux;
  }

  x = crc_inputs[6] = SPI.transfer(0x00);
  x2 = crc_inputs[7] = SPI.transfer(0x00);
  x3 = crc_inputs[8] = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    reading->ch1 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    reading->ch1 = aux;
  }

  x = crc_inputs[9] = SPI.transfer(0x00);
  x2 = crc_inputs[10] = SPI.transfer(0x00);
  x3 = crc_inputs[11] = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    reading->ch2 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    reading->ch2 = aux;
  }

  x = crc_inputs[12] = SPI.transfer(0x00);
  x2 = crc_inputs[13] = SPI.transfer(0x00);
  x3 = crc_inputs[14] = SPI.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    reading->ch3 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    reading->ch3 = aux;
  }

  x = SPI.transfer(0x00);
  x2 = SPI.transfer(0x00);
  SPI.transfer(0x00);
  crc_received = ((x << 8) | x2);

  delayMicroseconds(1);
  digitalWrite(ADS131M04_CS_PIN, HIGH);
  SPI.endTransaction();

  if (crc_received != crc16_CCITT(crc_inputs, 15))
  {
    // CRC error (SPI communication error)
    return false;
  }

  return true;
}
