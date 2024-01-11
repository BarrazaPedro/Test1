#include "old_commands.h"
#include "control_lbox.h"
#include <SPIFFS.h>
#include <Update.h>

static void update_mode_on(void);

old_commands::old_commands() {
}

int old_commands::process_commands(String command, int start_index)
{
  if (command.startsWith("**", start_index)) {
    return process_asterisk_command(command, start_index);
  } else if (command.indexOf("g", start_index) != -1 && command.indexOf("k", start_index) != -1) {
    return process_gk_command(command, start_index);
  } else {
    return process_odd_command(command, start_index);
  }
}

int old_commands::process_asterisk_command(String command, int start_index)
{
  String content;
  while (start_index < command.length())
  {
    if (command.startsWith("**", start_index)) {
      content = command.substring(start_index + 2, start_index + 3);
      handle_asterisk_command(content);
      start_index += 3;
      return start_index;
    } else {
      start_index++;
    }
  }
  return start_index;
}

void old_commands::handle_asterisk_command(const String & content)
{
  char letter = content.charAt(0);
  switch (letter) {
    case 'L':
      ESP.restart();
      break;
    case 'Q': {
        print_pswd_mode();
        print_inputs_disabled();
        print_doors_closed();
        print_safe_inputs();
        print_geos_enabled();
        uint16_t geofences_num = geofence.container_count();
        SerialBT.printf("NumeroGeo:%d|", geofences_num);
        Serial.printf("NumeroGeo:%d|", geofences_num);
        print_enable_prints();
        SerialBT.println();
        Serial.println();
        print_gps_disabled();
        print_panic_mode();
        print_panic_timer();
        print_output_delay();
        print_output_mode();
        print_output_config();
        print_serial_enabled();
        break;
      }
    case 'R':
      print_references();
      break;
    case 'U':
      update_mode_on();
      break;
    case 'W': {
        uint8_t geo_status = geofence.return_geostatus();
        double nearest_dist = geofence.return_nearest_distance();
        uint16_t nearest_geo = geofence.return_nearest_geofence();
        SerialBT.printf("*GeoS|%d|%0.2f|%d|*", geo_status, nearest_dist, nearest_geo);
        Serial.printf("*GeoS|%d|%0.2f|%d|*", geo_status, nearest_dist, nearest_geo);
      }
      break;
    case 'b':
      uint8_t local_mac[6];
      get_local_addr(local_mac);
      SerialBT.printf("Addr:%02X:%02X:%02X:%02X:%02X:%02X|",
                      local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4], local_mac[5]);
      Serial.printf("Addr:%02X:%02X:%02X:%02X:%02X:%02X|",
                    local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4], local_mac[5]);
      SerialBT.printf("btName:LBOXV5%02X%02X%02X|", local_mac[3], local_mac[4], local_mac[5]);
      Serial.printf("btName:LBOXV5%02X%02X%02X|", local_mac[3], local_mac[4], local_mac[5]);
      SerialBT.printf("btPin:%02X%02X|", local_mac[4], local_mac[5]);
      Serial.printf("btPin:%02X%02X|", local_mac[4], local_mac[5]);
      break;
    case 'e':
      print_io();
      break;
    case 'h':
      gps_module.print_time(SerialBT);
      gps_module.print_location(SerialBT);
      gps_module.print_time(Serial);
      gps_module.print_location(Serial);
      break;
    case 'i':
      SerialBT.print("i");
      Serial.print("i");
      break;
    case 'm':
      print_lbox_status();
      break;
    case 'p':
      print_panic_pswd();
      break;
    case 'r':
      print_analog_inputs();
      break;
    case 's':
      gps_module.print_speed(SerialBT);
      gps_module.print_speed(Serial);
      break;
    case 'v':
      SerialBT.printf("version del firmware:%s\n", firmware_ver);
      Serial.printf("version del firmware:%s\n", firmware_ver);
      break;
    case 'w': {
        i2c_mem_erase_all();
        was_i2c_mem_modified = 1;
        SerialBT.print("Geocercas borradas|");
        Serial.print("Geocercas borradas|");
        break;
      }
    case 'z':
      gps_module.print_nmea(SerialBT);
      gps_module.print_nmea(Serial);
      break;
    default:
      break;
  }
}

static void update_mode_on(void)
{
  SPIFFS.begin();
  SerialBT.println("SetUpdateMode");
  Serial.println("SetUpdateMode");
  File update_file = SPIFFS.open("/update_mode_lbox.bin", "rb");
  if (!update_file) {
    SerialBT.println("UpdateModeFileError");
    Serial.println("UpdateModeFileError");
    update_file.close();
    SPIFFS.end();
    return;
  }
  SerialBT.println("EnablingUpdateMode...");
  Serial.println("EnablingUpdateMode...");
  if (Update.begin(update_file.size()) && Update.writeStream(update_file) && Update.end()) {
    SerialBT.println("UpdateModeSuccess, restarting...");
    Serial.println("UpdateModeSuccess, restarting...");
    delay(1000);
    ESP.restart();
  } else {
    SerialBT.println("UpdateModeError");
    Serial.println("UpdateModeError");
  }
  update_file.close();
  SPIFFS.end();
}

int old_commands::process_gk_command(String command, int start_index)
{
  String content;
  while (start_index < command.length())
  {
    int g_index = command.indexOf("g", start_index);
    if (g_index != -1) {
      int k_index = command.indexOf("k", g_index + 1);
      if (k_index != -1) {
        content = command.substring(g_index + 1, k_index);
        handle_gk_command(content);
        start_index = k_index + 1;
        return start_index;
      } else {
        start_index = g_index + 1;
        break;
      }
    } else {
      break;
    }
  }
  return start_index;
}

void old_commands::handle_gk_command(const String & content)
{
  int size = content.length();
  switch (size) {
    case 0: {
        uint16_t geofences_num = geofence.container_count();
        SerialBT.printf("GCC:%d|", geofences_num);
        Serial.printf("GCC:%d|", geofences_num);
        break;
      }
    case 2:
      change_configs(content);
      break;
    case 3:
      change_timers(content);
      break;
    case 4:
      change_references(content);
      break;
    case 5:
      change_panic_pswd(content);
      break;
    case 6:
      read_geofence(content);
      break;
    case 7: {
        erase_geofence(content);
        was_i2c_mem_modified = 1;
        break;
      }
    case 8:
      config_default(content);
      break;
    case 9:
      change_safe_inputs(content);
      break;
    case 10:
      change_output_delay(content);
      break;
    case 15:
      change_output_config(content);
      break;
    case 38: {
        write_geofence(content);
        was_i2c_mem_modified = 1;
        break;
      }
    default:
      break;
  }
}

void old_commands::change_configs(const String& content)
{
  char letter = content.charAt(0);
  uint8_t val = content.substring(1).toInt();
  switch (letter) {
    case 'B': {
        set_gps_disabled(val);
        if (return_gps_disabled()) {
          gps_module.suspend();
        } else {
          gps_module.resume();
        }
        break;
      }
    case 'D':
      set_pswd_mode(val);
      break;
    case 'G':
      set_geos_enabled(val);
      break;
    case 'O':
      set_output_mode(val);
      break;
    case 'P':
      set_panic_mode(val);
      break;
    case 'V':
      set_doors_closed(val);
      break;
    case 'X': {
        set_serial_enabled(val);
        if (return_serial_enabled()) {
          Serial.begin(9600);
        } else {
          Serial.end();
        }
        break;
      }
    case 'Y':
      set_inputs_disabled(val);
      break;
    default:
      break;
  }
}

void old_commands::change_timers(const String& content)
{
  char letter = content.charAt(0);
  uint8_t val = content.substring(1, 3).toInt();
  switch (letter) {
    case 'B':
      set_panic_timer(val);
      break;
    default:
      break;
  }
}

void old_commands::change_references(const String& content)
{
  uint8_t num = content.substring(0, 1).toInt();
  uint16_t val = content.substring(1).toInt();
  set_reference(num, val);
}

void old_commands::change_panic_pswd(const String& content)
{
  uint8_t val0 = content.substring(0, 1).toInt();
  uint8_t val1 = content.substring(1, 2).toInt();
  uint8_t val2 = content.substring(2, 3).toInt();
  set_panic_pswd(val0, val1, val2);
}

void old_commands::read_geofence(const String& content)
{
  uint16_t pos_container = content.substring(0).toInt();
  geofence_type geof2read[1];
  geofence.return_geofence(pos_container, geof2read);
  if (geof2read[0].type != CLEAR) {
    SerialBT.printf("g%011.6f%011.6f", geof2read[0].latitude, geof2read[0].longitude);
    SerialBT.printf("%06d%07d%d00k", geof2read[0].addr / GEOFENCE_SIZE, geof2read[0].radius, geof2read[0].type);
    Serial.printf("g%011.6f%011.6f", geof2read[0].latitude, geof2read[0].longitude);
    Serial.printf("%06d%07d%d00k", geof2read[0].addr / GEOFENCE_SIZE, geof2read[0].radius, geof2read[0].type);
  } else {
    SerialBT.printf("N/A|");
    Serial.printf("N/A|");
  }
}

void old_commands::erase_geofence(const String& content)
{
  uint16_t pos_container = content.substring(0, 6).toInt();
  geofence_type geof2erase[1];
  geofence.return_geofence(pos_container, geof2erase);
  if (geof2erase[0].type != CLEAR) {
    i2c_mem_seq_erase(pos_container * GEOFENCE_SIZE, GEOFENCE_SIZE);
    SerialBT.printf("Borrada:%d|", pos_container);
    Serial.printf("Borrada:%d|", pos_container);
  } else {
    SerialBT.printf("N/A|");
    Serial.printf("N/A|");
  }
}

void old_commands::config_default(const String & content)
{
  if (content == "resetall") {
    set_config_default();
    SerialBT.print("CONFIG HARD RESET|");
    Serial.print("CONFIG HARD RESET|");
    i2c_mem_erase_all();
    was_i2c_mem_modified = 1;
    SerialBT.print("Geocercas borradas|");
    Serial.print("Geocercas borradas|");
    ESP.restart();
  }
}

void old_commands::change_safe_inputs(const String & content)
{
  bool val0 = content.substring(5, 6).toInt();
  bool val1 = content.substring(6, 7).toInt();
  bool val2 = content.substring(7, 8).toInt();
  bool val3 = content.substring(8, 9).toInt();
  set_safe_inputs(val0, val1, val2, val3);
}

void old_commands::change_output_delay(const String & content)
{
  uint16_t delay_val = content.substring(6, 10).toInt();
  set_output_delay(delay_val);
}

void old_commands::change_output_config(const String & content)
{
  bool     val0 = content.substring(6, 7).toInt();
  uint32_t val1 = content.substring(7, 11).toInt();
  uint32_t val2 = content.substring(11, 15).toInt();
  set_output_config(val0, val1, val2);
}

void old_commands::write_geofence(const String & content)
{
  double lat_dbl = content.substring(0, 11).toDouble();
  double lng_dbl = content.substring(11, 22).toDouble();
  uint32_t addr2add = content.substring(22, 28).toInt();
  uint32_t rad2add = content.substring(28, 35).toInt();
  uint32_t type2add = content.substring(35, 36).toInt();
  uint32_t lat2add = geofence.convert_double_to_uint32(lat_dbl);
  uint32_t lng2add = geofence.convert_double_to_uint32(lng_dbl);
  uint8_t geof2add[16];
  geofence.convert_uint32_to_uint8(type2add, geof2add, 0);
  geofence.convert_uint32_to_uint8(rad2add, geof2add, 4);
  geofence.convert_uint32_to_uint8(lat2add, geof2add, 8);
  geofence.convert_uint32_to_uint8(lng2add, geof2add, 12);
  if (addr2add < CONTAINER_SIZE) {
    addr2add *= GEOFENCE_SIZE;
  } else {
    addr2add = i2c_mem_seek_empty_position(GEOFENCE_SIZE);
  }
  i2c_mem_seq_write(addr2add, geof2add, sizeof(geof2add));
  SerialBT.printf("*DA:%d*", addr2add / GEOFENCE_SIZE);
  Serial.printf("*DA:%d*", addr2add / GEOFENCE_SIZE);
}

int old_commands::process_odd_command(String command, int start_index)
{
  while (start_index < command.length())
  {
    int next_space_index = command.indexOf(' ', start_index);
    if (next_space_index != -1) {
      String odd_command = command.substring(start_index, next_space_index);
      handle_odd_command(odd_command);
      start_index = next_space_index + 1;
      return start_index;
    } else {
      String odd_command = command.substring(start_index);
      handle_odd_command(odd_command);
      start_index = command.length();
      return start_index;
    }
  }
  return start_index;
}

void old_commands::handle_odd_command(const String & content)
{
  if (content == "yyy") {
    set_lbox_status(ALARMED);
  }
  if (content == "ZZZ") {
    set_lbox_status(FREE);
  }
  if (content == "YYY") {
    set_lbox_status(SET);
  }
  if (content == "xa") {
    set_enable_prints(content[1]);
  }
  if (content == "xb") {
    set_enable_prints(content[1]);
  }
  if (content == "xc") {
    set_enable_prints(content[1]);
  }
  if (content == "xd") {
    set_enable_prints(content[1]);
  }
}
