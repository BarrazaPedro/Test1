#include "nvs.h"

#define NVS_MAX_SIZE    4096//bytes
#define NVS_SIZE        (NVS_MAX_SIZE/4)//1024bytes
#define STRING_MAX_SIZE 24//bytes

void nvs_init(void)
{
  EEPROM.begin(NVS_SIZE);
}

void nvs_write(int addr, uint8_t val)
{
  EEPROM.write(addr, val);
  EEPROM.commit();
}

uint8_t nvs_read(int addr)
{
  return EEPROM.read(addr);
}

void nvs_erase(int addr)
{
  EEPROM.write(addr, CLEAR);
  EEPROM.commit();
}

void nvs_write_uint16(int addr, uint16_t val)
{
  for (int i = 0; i < 2; i++)
  {
    EEPROM.write(addr + i, val >> (i * 8));
  }
  EEPROM.commit();
}

uint16_t nvs_read_uint16(int addr)
{
  uint16_t val_u16 = 0;
  for (int i = 0; i < 2; i++)
  {
    uint8_t byte_val = EEPROM.read(addr + i);
    val_u16 |= ((uint16_t)byte_val << (i * 8));
  }
  return val_u16;
}

void nvs_write_str(int addr, const char* buf_cstr)
{
  for (int i = 0; i < STRING_MAX_SIZE; i++)
  {
    char char_val = buf_cstr[i];
    EEPROM.write(addr + i, char_val);
    if (char_val == NULL) {
      break;
    }
  }
  EEPROM.commit();
}

void nvs_write_str(int addr, String buf_str)
{
  for (int i = 0; i < STRING_MAX_SIZE; i++)
  {
    char char_val = buf_str.charAt(i);
    EEPROM.write(addr + i, char_val);
    if (char_val == NULL) {
      break;
    }
  }
  EEPROM.commit();
}

void nvs_read_str(int addr, char* buf_cstr)
{
  for (int i = 0; i < STRING_MAX_SIZE; i++)
  {
    char char_val = EEPROM.read(addr + i);
    buf_cstr[i] = char_val;
    if (char_val == NULL) {
      break;
    }
  }
}

String nvs_read_str(int addr)
{
  String buf_str = "";
  for (int i = 0; i < STRING_MAX_SIZE; i++)
  {
    char char_val = EEPROM.read(addr + i);
    if (char_val == NULL) {
      break;
    }
    buf_str += char_val;
  }
  return buf_str;
}

void nvs_erase_partition(void)
{
  for (int i = 0; i < NVS_SIZE; i++)
  {
    EEPROM.write(i, CLEAR);
  }
  EEPROM.commit();
}

void nvs_erase_all(void)
{
  for (int i = 0; i < NVS_MAX_SIZE; i++)
  {
    EEPROM.write(i, CLEAR);
  }
  EEPROM.commit();
}
