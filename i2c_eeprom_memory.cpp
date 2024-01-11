#include "i2c_eeprom_memory.h"
#include <Wire.h>

#define I2C_SDA_PIN_NUM     21
#define I2C_SCL_PIN_NUM     22
#define I2C_CLK_FREQ        1000000
#define I2C_SLV_DEV_ADDR    0x50//hex
#define BYTE_SIZE           8//bits
#define MEM_MAX_WRITE_TIME  5//ms
#define CLEAR               0xFF//hex
#define I2C_MEM_PAGE_SIZE   64//bytes
#define I2C_MAX_READ_SIZE   32//bytes
#define KB                  1024//bits
#define OK                  0
#define FAIL                1

void i2c_mem_init(void)
{
  Wire.begin(I2C_SDA_PIN_NUM, I2C_SCL_PIN_NUM);
  Wire.setClock(I2C_CLK_FREQ);
}

uint8_t i2c_mem_write(uint16_t addr2write, uint8_t val)
{
  if (addr2write > I2C_MEM_SIZE) {
    Serial.println("DATA NOT WRITTEN, INCORRECT ADDRESS");
    return FAIL;
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2write >> BYTE_SIZE);
  Wire.write(addr2write);
  Wire.write(val);
  Wire.endTransmission();
  delay(MEM_MAX_WRITE_TIME);
  return OK;
}

uint8_t i2c_mem_read(uint16_t addr2read, uint8_t *data)
{
  if (addr2read > I2C_MEM_SIZE) {
    Serial.println("DATA NOT READ, INCORRECT ADDRESS");
    return FAIL;
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2read >> BYTE_SIZE);
  Wire.write(addr2read);
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLV_DEV_ADDR, 1);
  data[0] = Wire.read();
  return OK;
}

uint8_t i2c_mem_erase(uint16_t addr2erase)
{
  if (addr2erase > I2C_MEM_SIZE) {
    Serial.println("DATA NOT ERASED, INCORRECT ADDRESS");
    return FAIL;
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2erase >> BYTE_SIZE);
  Wire.write(addr2erase);
  Wire.write(CLEAR);
  Wire.endTransmission();
  delay(MEM_MAX_WRITE_TIME);
  return OK;
}

uint8_t i2c_mem_seq_write(uint16_t addr2write, uint8_t *data, uint16_t data_size)
{
  if (data_size > I2C_MEM_PAGE_SIZE) {
    Serial.println("DATA NOT WRITTEN TO I2C MEMORY");
    return FAIL;
  }
  if (addr2write > I2C_MEM_SIZE) {
    Serial.println("DATA NOT WRITTEN, INCORRECT ADDRESS");
    return FAIL;
  }
  uint8_t data2write[data_size];
  for (uint8_t i = 0; i < data_size; i++)
  {
    data2write[i] = data[i];
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2write >> BYTE_SIZE);
  Wire.write(addr2write);
  for (uint16_t i = 0; i < data_size; i++)
  {
    Wire.write(data2write[i]);
  }
  Wire.endTransmission();
  delay(MEM_MAX_WRITE_TIME);
  return OK;
}

uint8_t i2c_mem_seq_read(uint16_t addr2read, uint8_t *data, uint16_t data_size)
{
  if (data_size > I2C_MAX_READ_SIZE) {
    Serial.println("DATA NOT READ FROM I2C MEMORY");
    return FAIL;
  }
  if (addr2read + data_size > I2C_MEM_SIZE) {
    Serial.println("DATA NOT READ, INCORRECT ADDRESS");
    return FAIL;
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2read >> BYTE_SIZE);
  Wire.write(addr2read);
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLV_DEV_ADDR, data_size);
  for (uint16_t i = 0; i < data_size; i++)
  {
    if (Wire.available()) {
      data[i] = Wire.read();
    }
  }
  return OK;
}

uint8_t i2c_mem_seq_erase(uint16_t addr2erase, uint16_t data_size)
{
  if (data_size > I2C_MEM_PAGE_SIZE) {
    Serial.println("DATA NOT ERASED AT I2C MEMORY");
    return FAIL;
  }
  if (addr2erase > I2C_MEM_SIZE) {
    Serial.println("DATA NOT ERASED, INCORRECT ADDRESS");
    return FAIL;
  }
  Wire.beginTransmission(I2C_SLV_DEV_ADDR);
  Wire.write(addr2erase >> BYTE_SIZE);
  Wire.write(addr2erase);
  for (uint16_t i = 0; i < data_size; i++)
  {
    Wire.write(CLEAR);
  }
  Wire.endTransmission();
  delay(MEM_MAX_WRITE_TIME);
  return OK;
}

void i2c_mem_erase_all(void)
{
  for (uint16_t addr2erase = 0; addr2erase < I2C_MEM_SIZE; addr2erase += I2C_MEM_PAGE_SIZE)
  {
    Wire.beginTransmission(I2C_SLV_DEV_ADDR);
    Wire.write(addr2erase >> BYTE_SIZE);
    Wire.write(addr2erase);
    for (uint16_t i = 0; i < I2C_MEM_PAGE_SIZE; i++)
    {
      Wire.write(CLEAR);
    }
    Wire.endTransmission();
    delay(MEM_MAX_WRITE_TIME);
  }
}

uint16_t i2c_mem_seek_empty_position(uint16_t skip_positions)
{
  for (uint16_t addr2read = 0; addr2read < I2C_MEM_SIZE; addr2read += skip_positions)
  {
    Wire.beginTransmission(I2C_SLV_DEV_ADDR);
    Wire.write(addr2read >> BYTE_SIZE);
    Wire.write(addr2read);
    Wire.endTransmission();
    Wire.requestFrom(I2C_SLV_DEV_ADDR, 1);
    uint8_t data = Wire.read();
    if (data == CLEAR) {
      return addr2read;
    }
  }
  Serial.println("NOT FREE SPACE IN I2C MEMORY");
  return -1;
}

uint16_t i2c_mem_count_not_empty_position(uint16_t skip_positions)
{
  uint16_t count = 0;
  for (uint16_t addr2read = 0; addr2read < I2C_MEM_SIZE; addr2read += skip_positions)
  {
    Wire.beginTransmission(I2C_SLV_DEV_ADDR);
    Wire.write(addr2read >> BYTE_SIZE);
    Wire.write(addr2read);
    Wire.endTransmission();
    Wire.requestFrom(I2C_SLV_DEV_ADDR, 1);
    uint8_t data = Wire.read();
    if (data != CLEAR) {
      count++;
    }
  }
  return count;
}
