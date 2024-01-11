#ifndef i2c_eeprom_h
#define i2c_eeprom_h

#include <Arduino.h>

#define I2C_MEM_SIZE        32768//bytes

void      i2c_mem_init(void);
uint8_t   i2c_mem_write(uint16_t addr2write, uint8_t val);
uint8_t   i2c_mem_read(uint16_t addr2read, uint8_t *data);
uint8_t   i2c_mem_erase(uint16_t addr2erase);
uint8_t   i2c_mem_seq_write(uint16_t addr2write, uint8_t *data, uint16_t data_size);
uint8_t   i2c_mem_seq_read(uint16_t addr2read, uint8_t *data, uint16_t data_size);
uint8_t   i2c_mem_seq_erase(uint16_t addr2erase, uint16_t data_size);
void      i2c_mem_erase_all(void);
uint16_t  i2c_mem_seek_empty_position(uint16_t skip_positions);
uint16_t  i2c_mem_count_not_empty_position(uint16_t skip_positions);

#endif
