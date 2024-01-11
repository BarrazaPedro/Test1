#ifndef mac_manager_h
#define mac_manager_h

#include <Arduino.h>

void mac_str2uint(const String &mac_string, uint8_t* mac_uint8);
void mac_uint2cstr(uint8_t* mac_uint8, char* mac_cstr);
void print_mac(uint8_t* mac_address);

#endif
