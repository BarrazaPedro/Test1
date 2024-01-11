#ifndef nvs_h
#define nvs_h

#include <EEPROM.h>

#define CLEAR 0xFF

void      nvs_init(void);
void      nvs_write(int addr, uint8_t val);
uint8_t   nvs_read(int addr);
void      nvs_erase(int addr);
void      nvs_write_uint16(int addr, uint16_t val);
uint16_t  nvs_read_uint16(int addr);
void      nvs_write_str(int addr, const char* buf_cstr);
void      nvs_write_str(int addr, String buf_str);
void      nvs_read_str(int addr, char* buf_cstr);
String    nvs_read_str(int addr);
void      nvs_erase_partition(void);
void      nvs_erase_all(void);

#endif
