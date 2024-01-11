#ifndef CONTROL_LBOX_H
#define CONTROL_LBOX_H

#include <BluetoothSerial.h>

extern BluetoothSerial SerialBT;
extern bool are_inputs_updated;

enum LBOX_STATUS {
  FREE    = 0,
  SET     = 1,
  ALARMED = 2
};

enum NVS_REFERENCE {
  LBOX_CONFIG   = 0,
  LBOX_STATUS   = 1,
  LOCAL_MAC_B0  = 2,
  LOCAL_MAC_B1  = 3,
  LOCAL_MAC_B2  = 4,
  LOCAL_MAC_B3  = 5,
  LOCAL_MAC_B4  = 6,
  LOCAL_MAC_B5  = 7,
  SAFE_INPUT0   = 8,
  SAFE_INPUT1   = 9,
  SAFE_INPUT2   = 10,
  SAFE_INPUT3   = 11,
  PANIC_PSWD0   = 12,
  PANIC_PSWD1   = 13,
  PANIC_PSWD2   = 14,
  MIN_REF_I0    = 15,
  MAX_REF_I0    = 17,
  MIN_REF_I1    = 19,
  MAX_REF_I1    = 21,
  MIN_REF_I2    = 23,
  MAX_REF_I2    = 25,
  MIN_REF_I3    = 27,
  MAX_REF_I3    = 29,
  IN_DEBOUNCE_T = 31,
  PANIC_TIMER   = 33,
  PSWD_MODE     = 34,
  PANIC_MODE    = 35,
  IN_DISABLED   = 36,
  DOORS_CLOSED  = 37,
  GEOSTATUS_EN  = 38,
  GPS_DIS       = 39,
  SERIAL_EN     = 40,
  OUTPUT_MODE   = 41,
  OUTPUT_DELAY  = 42,
  PRINTS_EN     = 44,
  BEF_CONNECTED = 45,
  CLIENT_MAC_B0 = 46,
  CLIENT_MAC_B1 = 47,
  CLIENT_MAC_B2 = 48,
  CLIENT_MAC_B3 = 49,
  CLIENT_MAC_B4 = 50,
  CLIENT_MAC_B5 = 51,
  ON_OFF_SELECT = 52,
  OUT_OFF_DELAY = 53,
  OUT_ON_DELAY  = 55
};

void    lbox_control_begin(void);
void    get_local_addr(uint8_t* local_mac);
void    red_led_on(void);
void    red_led_off(void);
void    blue_led_on(void);
void    blink_led(void);
void    blink_led_task(void *pvParameters);
void    set_output(bool val);
void    task_io(void *pvParameters);
bool    convert_analog2digital_input(bool digital_value, uint16_t analog_value, uint16_t on_value, uint16_t off_value);
void    print_inputs_changes(void);
uint8_t get_inputs_status(void);
void    set_safe_inputs(bool safe_input0, bool safe_input1, bool safe_input2, bool safe_input3);
void    print_safe_inputs(void);
void    set_reference(uint8_t ref_num, uint16_t val);
void    print_references(void);
void    print_io(void);
void    print_analog_inputs(void);
void    set_inputs_disabled(uint8_t new_val);
void    print_inputs_disabled(void);
void    set_doors_closed(uint8_t new_val);
void    print_doors_closed(void);
uint8_t get_lbox_status(uint8_t geostatus);
void    set_lbox_status(uint8_t status_val);
void    print_lbox_status(void);
void    set_pswd_mode(uint8_t new_val);
void    print_pswd_mode(void);
void    set_panic_mode(uint8_t new_val);
void    print_panic_mode(void);
void    set_panic_timer(uint8_t timer_val);
void    print_panic_timer(void);
void    set_geos_enabled(bool new_val);
bool    return_geos_enabled(void);
void    print_geos_enabled(void);
void    set_gps_disabled(bool new_val);
bool    return_gps_disabled(void);
void    print_gps_disabled(void);
void    set_serial_enabled(bool new_val);
bool    return_serial_enabled(void);
void    print_serial_enabled(void);
void    set_panic_pswd(uint8_t panic_pswd0, uint8_t panic_pswd1, uint8_t panic_pswd2);
void    print_panic_pswd(void);
void    set_output_mode(bool new_val);
void    print_output_mode(void);
void    set_output_delay(uint16_t new_val);
void    print_output_delay(void);
void    set_output_config(bool selector, uint32_t off_delay, uint32_t on_delay);
void    print_output_config(void);
void    set_enable_prints(uint8_t new_val);
void    print_enable_prints(void);
bool    get_enable_print_geos(void);
void    set_config_default(void);

#endif
