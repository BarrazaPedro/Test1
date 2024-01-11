#include "mac_manager.h"

#define MAC_SIZE  6//bytes

void mac_str2uint(const String &mac_string, uint8_t* mac_uint8)
{
  for (int i = 0; i < MAC_SIZE; i++)
  {
    mac_uint8[i] = strtol(mac_string.substring(3 * i, 3 * i + 2).c_str(), NULL, 16);
  }
}

void mac_uint2cstr(uint8_t* mac_uint8, char* mac_cstr)
{
  sprintf(mac_cstr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac_uint8[0], mac_uint8[1], mac_uint8[2], mac_uint8[3], mac_uint8[4], mac_uint8[5]);
}

void print_mac(uint8_t* mac_address)
{
  for (int i = 0; i < MAC_SIZE; i++)
  {
    Serial.printf("%02X", mac_address[i]);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}
