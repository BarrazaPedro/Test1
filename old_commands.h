#ifndef old_commands_h
#define old_commands_h

#include "geofence_manager.h"
#include "i2c_eeprom_memory.h"
#include "custom_gps.h"

extern      geofence_manager geofence;
extern      custom_gps gps_module;
extern bool was_i2c_mem_modified;
extern char firmware_ver[7];

class old_commands {
  public:
    old_commands();
    int process_commands(String command, int start_index);
    int process_asterisk_command(String command, int start_index);
    int process_gk_command(String command, int start_index);
    int process_odd_command(String command, int start_index);

  private:
    void handle_asterisk_command(const String& content);
    void handle_gk_command(const String& content);
    void change_configs(const String& content);
    void change_timers(const String& content);
    void change_references(const String& content);
    void change_panic_pswd(const String& content);
    void read_geofence(const String& content);
    void erase_geofence(const String& content);
    void config_default(const String& content);
    void change_safe_inputs(const String& content);
    void change_output_delay(const String & content);
    void change_output_config(const String & content);
    void write_geofence(const String & content);
    void handle_odd_command(const String& content);
};

#endif
