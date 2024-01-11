#include "control_lbox.h"
#include "nvs.h"

#define blue_led    12
#define red_led     14
#define input0_pin  36
#define input1_pin  39
#define input2_pin  34
#define input3_pin  35
#define panic_pin   02
#define output_pin  13

enum INPUT_STATUS {
  PSWD   = 0,
  SAFE   = 1,
  UNSAFE = 2,
  PANIC  = 3
};

uint8_t   inputs_status = 0;
uint8_t   lbox_status = 0;
int64_t   previous_blink_time = 0;
bool      b_led_state = 0;
uint16_t  inputs_analog[4];
bool      inputs_digital[4];
bool      prev_inputs_val[4] = {1, 1, 0, 0};
bool      safe_inputs[4];
bool      panic_value = 0;
bool      previous_panic_value = 0;
int64_t   prev_panic_long_pressed_time = INT64_MAX;
int64_t   previous_panic_time = INT64_MAX;
uint16_t  panic_long_pressed_timer = UINT16_MAX;
uint8_t   panic_digit = 0;
uint8_t   panic_pswd[3];
uint8_t   panic_code[3] = {0, 0, 0};
uint16_t  min_ref_inputs[4];
uint16_t  max_ref_inputs[4];
uint16_t  input_debounce_time;
uint8_t   pswd_mode;
uint8_t   panic_mode;
bool      inputs_disabled = 0;
bool      force_doors_closed = 0;
bool      geos_enabled;
bool      gps_disabled;
bool      serial_enabled;
bool      output_mode;
bool      output_value = 0;
uint32_t  output_delay = 0;
int64_t   prev_alarmed_time = INT64_MAX;
bool      enable_print_in;
bool      enable_print_geos;
bool      on_off_selector;
bool      output_status = !on_off_selector;
int64_t   prev_on_off_time = -INT32_MAX;
uint32_t  output_on_delay;
uint32_t  output_off_delay;

void lbox_control_begin(void)
{
  pinMode(blue_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(output_pin, OUTPUT);
  pinMode(panic_pin, INPUT);
  if (nvs_read(LBOX_CONFIG) != 0) {
    nvs_write(LBOX_STATUS, FREE);
    nvs_write(SAFE_INPUT0, 0);
    nvs_write(SAFE_INPUT1, 0);
    nvs_write(SAFE_INPUT2, 0);
    nvs_write(SAFE_INPUT3, 0);
    nvs_write(PANIC_PSWD0, 1);
    nvs_write(PANIC_PSWD1, 2);
    nvs_write(PANIC_PSWD2, 3);
    nvs_write_uint16(MIN_REF_I0, 200);
    nvs_write_uint16(MAX_REF_I0, 300);
    nvs_write_uint16(MIN_REF_I1, 200);
    nvs_write_uint16(MAX_REF_I1, 300);
    nvs_write_uint16(MIN_REF_I2, 200);
    nvs_write_uint16(MAX_REF_I2, 300);
    nvs_write_uint16(MIN_REF_I3, 200);
    nvs_write_uint16(MAX_REF_I3, 300);
    nvs_write_uint16(IN_DEBOUNCE_T, 1000);
    nvs_write(PANIC_TIMER, 5);
    nvs_write(PSWD_MODE, FREE);
    nvs_write(PANIC_MODE, ALARMED);
    nvs_write(IN_DISABLED, 0);
    nvs_write(DOORS_CLOSED, 0);
    nvs_write(GEOSTATUS_EN, 1);
    nvs_write(GPS_DIS, 0);
    nvs_write(SERIAL_EN, 1);
    nvs_write(OUTPUT_MODE, 1);
    nvs_write_uint16(OUTPUT_DELAY, 0);
    nvs_write(PRINTS_EN, 'd');
    nvs_write(ON_OFF_SELECT, 1);
    nvs_write_uint16(OUT_OFF_DELAY, 0);
    nvs_write_uint16(OUT_ON_DELAY, 9999);
    nvs_write(LBOX_CONFIG, 0);
  }
  lbox_status = nvs_read(LBOX_STATUS);
  safe_inputs[0] = nvs_read(SAFE_INPUT0);
  safe_inputs[1] = nvs_read(SAFE_INPUT1);
  safe_inputs[2] = nvs_read(SAFE_INPUT2);
  safe_inputs[3] = nvs_read(SAFE_INPUT3);
  panic_pswd[0] = nvs_read(PANIC_PSWD0);
  panic_pswd[1] = nvs_read(PANIC_PSWD1);
  panic_pswd[2] = nvs_read(PANIC_PSWD2);
  min_ref_inputs[0] = nvs_read_uint16(MIN_REF_I0);
  max_ref_inputs[0] = nvs_read_uint16(MAX_REF_I0);
  min_ref_inputs[1] = nvs_read_uint16(MIN_REF_I1);
  max_ref_inputs[1] = nvs_read_uint16(MAX_REF_I1);
  min_ref_inputs[2] = nvs_read_uint16(MIN_REF_I2);
  max_ref_inputs[2] = nvs_read_uint16(MAX_REF_I2);
  min_ref_inputs[3] = nvs_read_uint16(MIN_REF_I3);
  max_ref_inputs[3] = nvs_read_uint16(MAX_REF_I3);
  input_debounce_time = nvs_read_uint16(IN_DEBOUNCE_T);
  panic_long_pressed_timer = nvs_read(PANIC_TIMER) * 1000;
  pswd_mode = nvs_read(PSWD_MODE);
  panic_mode = nvs_read(PANIC_MODE);
  inputs_disabled = nvs_read(IN_DISABLED);
  force_doors_closed = nvs_read(DOORS_CLOSED);
  geos_enabled = nvs_read(GEOSTATUS_EN);
  gps_disabled = nvs_read(GPS_DIS);
  serial_enabled = nvs_read(SERIAL_EN);
  output_mode = nvs_read(OUTPUT_MODE);
  output_delay = nvs_read_uint16(OUTPUT_DELAY) * 1000;
  enable_print_in = nvs_read(PRINTS_EN) & 1;
  enable_print_geos = nvs_read(PRINTS_EN) >> 1 & 1;
  are_inputs_updated = false;
  if (lbox_status == ALARMED) {
    output_mode ? output_value = HIGH : output_value = LOW;
    prev_alarmed_time = 0;
  } else {
    output_mode ? output_value = LOW : output_value = HIGH;
  }
  set_output(output_value);
  on_off_selector = nvs_read(ON_OFF_SELECT);
  output_status = !on_off_selector;
  if (nvs_read_uint16(OUT_OFF_DELAY) >= 9999) {
    output_off_delay = UINT32_MAX;
  } else {
    output_off_delay = nvs_read_uint16(OUT_OFF_DELAY) * 1000;
  }
  if (nvs_read_uint16(OUT_ON_DELAY) >= 9999) {
    output_on_delay = UINT32_MAX;
  } else {
    output_on_delay = nvs_read_uint16(OUT_ON_DELAY) * 1000;
  }
}

void get_local_addr(uint8_t* local_mac)
{
  local_mac[0] = nvs_read(LOCAL_MAC_B0);
  local_mac[1] = nvs_read(LOCAL_MAC_B1);
  local_mac[2] = nvs_read(LOCAL_MAC_B2);
  local_mac[3] = nvs_read(LOCAL_MAC_B3);
  local_mac[4] = nvs_read(LOCAL_MAC_B4);
  local_mac[5] = nvs_read(LOCAL_MAC_B5);
}

void red_led_on(void)
{
  digitalWrite(red_led, HIGH);
}

void red_led_off(void)
{
  digitalWrite(red_led, LOW);
}

void blue_led_on(void)
{
  digitalWrite(blue_led, HIGH);
  b_led_state = 1;
}

void blink_led(void)
{
  int64_t current_time = millis();
  int blink_delay = 500;
  if (current_time - previous_blink_time >= blink_delay) {
    previous_blink_time = current_time;
    b_led_state = !b_led_state;
    digitalWrite(blue_led, b_led_state);
  }
}

void blink_led_task(void *pvParameters)
{
  while (1) {
    int64_t current_time = millis();
    int blink_delay = 500;
    if (current_time - previous_blink_time >= blink_delay) {
      previous_blink_time = current_time;
      b_led_state = !b_led_state;
      digitalWrite(blue_led, b_led_state);
    }
  }
}

void set_output(bool val)
{
  digitalWrite(output_pin, val);
}

void task_io(void *pvParameters)
{
  while (1) {
    inputs_analog[0] = analogRead(input0_pin);
    inputs_analog[1] = analogRead(input1_pin);
    inputs_analog[2] = analogRead(input2_pin);
    inputs_analog[3] = analogRead(input3_pin);
    inputs_digital[0] = convert_analog2digital_input(inputs_digital[0], inputs_analog[0], max_ref_inputs[0], min_ref_inputs[0]);
    inputs_digital[1] = convert_analog2digital_input(inputs_digital[1], inputs_analog[1], max_ref_inputs[1], min_ref_inputs[1]);
    inputs_digital[2] = convert_analog2digital_input(inputs_digital[2], inputs_analog[2], max_ref_inputs[2], min_ref_inputs[2]);
    inputs_digital[3] = convert_analog2digital_input(inputs_digital[3], inputs_analog[3], max_ref_inputs[3], min_ref_inputs[3]);
    if (enable_print_in) {
      print_inputs_changes();
    }
    are_inputs_updated = true;
    int64_t current_time = millis();
    if (lbox_status == ALARMED) {
      if (output_status && current_time - prev_on_off_time >= output_on_delay) {
        prev_on_off_time = current_time;
        output_status = LOW;
        output_value = LOW;
      }
      if (!output_status && current_time - prev_on_off_time >= output_off_delay) {
        prev_on_off_time = current_time;
        output_status = HIGH;
        output_value = HIGH;
      }
      set_output(output_status);
    } else {
      set_output(!output_mode);
      output_value = !output_mode;
      output_status = on_off_selector;
    }
    delay(input_debounce_time / 10);
  }
}

bool convert_analog2digital_input(bool digital_value, uint16_t analog_value, uint16_t on_value, uint16_t off_value)
{
  if (analog_value > on_value) {
    return 1;
  }
  if (analog_value < off_value) {
    return 0;
  }
  return digital_value;
}

void print_inputs_changes(void)
{
  for (int i = 0; i < sizeof(inputs_digital); i++)
  {
    if (inputs_digital[i] != prev_inputs_val[i]) {
      if (inputs_digital[i] == HIGH) {
        SerialBT.printf("Encendido %d|", i);
        Serial.printf("Encendido %d|", i);
      } else {
        SerialBT.printf("Apagado %d|", i);
        Serial.printf("Apagado %d|", i);
      }
      prev_inputs_val[i] = inputs_digital[i];
    }
  }
}

uint8_t get_inputs_status(void)
{
  //  0pswd,1safe,2unsafe,3panic
  int64_t current_time = millis();
  panic_value = digitalRead(panic_pin);
  are_inputs_updated = false;
  if (panic_value != previous_panic_value) {
    if (panic_value == 1) {
      prev_panic_long_pressed_time = current_time;
      if (enable_print_in) {
        SerialBT.printf("Encendido 4|");
        Serial.printf("Encendido 4|");
      }
    } else {
      prev_panic_long_pressed_time = INT64_MAX;
      if (enable_print_in) {
        SerialBT.printf("Apagado 4|");
        Serial.printf("Apagado 4|");
      }
    }
  } else {
    if (current_time - prev_panic_long_pressed_time >= panic_long_pressed_timer && lbox_status != panic_mode) {
      inputs_status = PANIC;
      SerialBT.printf("*BotonPanico*");
      Serial.printf("*BotonPanico*");
      return inputs_status;
    }
  }
  if (current_time - 1500 >= previous_panic_time) {
    panic_digit++;
    previous_panic_time = current_time;
  }
  if (panic_value != previous_panic_value) {
    if (panic_value == 1) {
      previous_panic_time = current_time;
    } else {
      if (current_time - previous_panic_time >= 100) {
        panic_code[panic_digit]++;
      }
    }
  }
  previous_panic_value = panic_value;
  if (panic_digit > 2) {
    if (panic_code[0] == panic_pswd[0] && panic_code[1] == panic_pswd[1] && panic_code[2] == panic_pswd[2]) {
      SerialBT.print("Liberado manual");
      Serial.print("Liberado manual");
      inputs_status = PSWD;
      panic_digit = 0;
      previous_panic_time = INT64_MAX;
      memset(panic_code, 0, sizeof(panic_code));
      return inputs_status;
    } else {
      if (enable_print_in) {
        SerialBT.printf("Code:%d%d%d|", panic_code[0], panic_code[1], panic_code[2]);
        Serial.printf("Code:%d%d%d|", panic_code[0], panic_code[1], panic_code[2]);
      }
      panic_digit = 0;
      previous_panic_time = INT64_MAX;
      memset(panic_code, 0, sizeof(panic_code));
    }
  }
  if (inputs_disabled) {
    if (force_doors_closed) {
      inputs_status = SAFE;
      return inputs_status;
    } else {
      inputs_status = UNSAFE;
      return inputs_status;
    }
  }
  for (int i = 0; i < sizeof(inputs_digital); i++)
  {
    if (inputs_digital[i] != safe_inputs[i]) {
      inputs_status = UNSAFE;
      return inputs_status;
    }
  }
  inputs_status = SAFE;
  return inputs_status;
}

void set_safe_inputs(bool safe_input0, bool safe_input1, bool safe_input2, bool safe_input3)
{
  safe_inputs[0] = safe_input0;
  nvs_write(SAFE_INPUT0, safe_input0);
  safe_inputs[1] = safe_input1;
  nvs_write(SAFE_INPUT1, safe_input1);
  safe_inputs[2] = safe_input2;
  nvs_write(SAFE_INPUT2, safe_input2);
  safe_inputs[3] = safe_input3;
  nvs_write(SAFE_INPUT3, safe_input3);
  print_safe_inputs();
}

void print_safe_inputs(void)
{
  SerialBT.printf("CondicionesEnt:%d|", safe_inputs[0] * 1 + safe_inputs[1] * 2 + safe_inputs[2] * 4 + safe_inputs[3] * 8);
  Serial.printf("CondicionesEnt:%d|", safe_inputs[0] * 1 + safe_inputs[1] * 2 + safe_inputs[2] * 4 + safe_inputs[3] * 8);
}

void set_reference(uint8_t ref_num, uint16_t val)
{
  uint8_t ref_num_nvs;
  uint16_t new_val = val * 4;
  switch (ref_num) {
    case 1:
      ref_num_nvs = MIN_REF_I0;
      min_ref_inputs[0] = new_val;
      break;
    case 2:
      ref_num_nvs = MAX_REF_I0;
      max_ref_inputs[0] = new_val;
      break;
    case 3:
      ref_num_nvs = MIN_REF_I1;
      min_ref_inputs[1] = new_val;
      break;
    case 4:
      ref_num_nvs = MAX_REF_I1;
      max_ref_inputs[1] = new_val;
      break;
    case 5:
      ref_num_nvs = MIN_REF_I2;
      min_ref_inputs[2] = new_val;
      break;
    case 6:
      ref_num_nvs = MAX_REF_I2;
      max_ref_inputs[2] = new_val;
      break;
    case 7:
      ref_num_nvs = MIN_REF_I3;
      min_ref_inputs[3] = new_val;
      break;
    case 8:
      ref_num_nvs = MAX_REF_I3;
      max_ref_inputs[3] = new_val;
      break;
    case 9:
      ref_num_nvs = IN_DEBOUNCE_T;
      input_debounce_time = new_val;
      break;
    default:
      return;
  }
  nvs_write_uint16(ref_num_nvs, new_val);
  print_references();
}

void print_references(void)
{
  SerialBT.printf("Refs:%d-%d-%d-%d-%d-%d-%d-%d-%d|", min_ref_inputs[0], max_ref_inputs[0],
                  min_ref_inputs[1], max_ref_inputs[1], min_ref_inputs[2], max_ref_inputs[2],
                  min_ref_inputs[3], max_ref_inputs[3], input_debounce_time);
  Serial.printf("Refs:%d-%d-%d-%d-%d-%d-%d-%d-%d|", min_ref_inputs[0], max_ref_inputs[0],
                min_ref_inputs[1], max_ref_inputs[1], min_ref_inputs[2], max_ref_inputs[2],
                min_ref_inputs[3], max_ref_inputs[3], input_debounce_time);
}

void print_io(void)
{
  SerialBT.printf("e%d%d%d%d%d%d|", inputs_digital[0], inputs_digital[1],
                  inputs_digital[2], inputs_digital[3], panic_value, output_value);
  Serial.printf("e%d%d%d%d%d%d|", inputs_digital[0], inputs_digital[1],
                inputs_digital[2], inputs_digital[3], panic_value, output_value);
}

void print_analog_inputs(void)
{
  SerialBT.printf("%d*%d*%d*%d*%d*", inputs_analog[0], inputs_analog[1],
                  inputs_analog[2], inputs_analog[3], panic_value);
  Serial.printf("%d*%d*%d*%d*%d*", inputs_analog[0], inputs_analog[1],
                inputs_analog[2], inputs_analog[3], panic_value);
}

void set_inputs_disabled(uint8_t new_val)
{
  nvs_write(IN_DISABLED, new_val);
  inputs_disabled = new_val;
  print_inputs_disabled();
}

void print_inputs_disabled(void)
{
  SerialBT.printf("BypassEnt:%d|", inputs_disabled);
  Serial.printf("BypassEnt:%d|", inputs_disabled);
}

void set_doors_closed(uint8_t new_val)
{
  nvs_write(DOORS_CLOSED, new_val);
  force_doors_closed = new_val;
  print_doors_closed();
}

void print_doors_closed(void)
{
  SerialBT.printf("Validity:%d|", force_doors_closed);
  Serial.printf("Validity:%d|", force_doors_closed);
}

uint8_t get_lbox_status(uint8_t geostatus)
{
  //  0free,1set,2alarmed
  uint8_t previous_lbox_status = lbox_status;
  if (geostatus != 8) {
    if (lbox_status == SET) {
      if (inputs_status == UNSAFE && geostatus == 0) {
        lbox_status = ALARMED;
      }
      if (geostatus == 2) {
        lbox_status = ALARMED;
        SerialBT.printf("*Geocerca Insegura*");
        Serial.printf("*Geocerca Insegura*");
      }
    }
    if (geostatus == 5) {
      lbox_status = FREE;
    }
    if (geostatus == 3) {
      lbox_status = SET;
    }
  }
  if (geostatus == 8) {
    if (inputs_status == SAFE) {
      lbox_status = FREE;
    }
    if (inputs_status == UNSAFE) {
      lbox_status = ALARMED;
    }
  }
  if (inputs_status == PSWD) {
    lbox_status = pswd_mode;
  }
  if (inputs_status == PANIC) {
    lbox_status = panic_mode;
  }
  if (lbox_status != previous_lbox_status) {
    nvs_write(LBOX_STATUS, lbox_status);
    if (lbox_status == ALARMED) {
      prev_alarmed_time = (int64_t)millis();
    }
    print_lbox_status();
  }
  return lbox_status;
}

void set_lbox_status(uint8_t status_val)
{
  uint8_t previous_lbox_status = lbox_status;
  if (status_val != previous_lbox_status) {
    nvs_write(LBOX_STATUS, status_val);
    lbox_status = status_val;
    if (lbox_status == ALARMED) {
      prev_alarmed_time = (int64_t)millis();
      prev_on_off_time = (int64_t)millis();
    }
  }
  print_lbox_status();
}

void print_lbox_status(void)
{
  SerialBT.printf("*Modo %d*", lbox_status);
  Serial.printf("*Modo %d*", lbox_status);
}

void set_pswd_mode(uint8_t new_val)
{
  nvs_write(PSWD_MODE, new_val);
  pswd_mode = new_val;
  print_pswd_mode();
}

void print_pswd_mode(void)
{
  SerialBT.printf("MDesbloqueo:%d|", pswd_mode);
  Serial.printf("MDesbloqueo:%d|*", pswd_mode);
}

void set_panic_mode(uint8_t new_val)
{
  nvs_write(PANIC_MODE, new_val);
  panic_mode = new_val;
  print_panic_mode();
}

void print_panic_mode(void)
{
  SerialBT.printf("ModoPCab:%d|", panic_mode);
  Serial.printf("ModoPCab:%d|", panic_mode);
}

void set_panic_timer(uint8_t timer_val)
{
  nvs_write(PANIC_TIMER, timer_val);
  panic_long_pressed_timer = timer_val * 1000;
  print_panic_timer();
}

void print_panic_timer(void)
{
  SerialBT.printf("TimerBP:%d|", panic_long_pressed_timer);
  Serial.printf("TimerBP:%d|", panic_long_pressed_timer);
}

void set_geos_enabled(bool new_val)
{
  nvs_write(GEOSTATUS_EN, new_val);
  geos_enabled = new_val;
  print_geos_enabled();
}

bool return_geos_enabled(void)
{
  return geos_enabled;
}

void print_geos_enabled(void)
{
  SerialBT.printf("ConsultarGeo:%d|", geos_enabled);
  Serial.printf("ConsultarGeo:%d|", geos_enabled);
}

void set_gps_disabled(bool new_val)
{
  nvs_write(GPS_DIS, new_val);
  gps_disabled = new_val;
  nvs_write(GEOSTATUS_EN, !new_val);
  geos_enabled = !new_val;
  print_gps_disabled();
  print_geos_enabled();
}

bool return_gps_disabled(void)
{
  return gps_disabled;
}

void print_gps_disabled(void)
{
  SerialBT.printf("BloqueoTransfer:%d|", gps_disabled);
  Serial.printf("BloqueoTransfer:%d|", gps_disabled);
}

void set_serial_enabled(bool new_val)
{
  nvs_write(SERIAL_EN, new_val);
  serial_enabled = new_val;
  print_serial_enabled();
}

bool return_serial_enabled(void)
{
  return serial_enabled;
}

void print_serial_enabled(void)
{
  SerialBT.printf("EnSerial:%d|", serial_enabled);
  Serial.printf("EnSerial:%d|", serial_enabled);
}

void set_panic_pswd(uint8_t panic_pswd0, uint8_t panic_pswd1, uint8_t panic_pswd2)
{
  panic_pswd[0] = panic_pswd0;
  nvs_write(PANIC_PSWD0, panic_pswd0);
  panic_pswd[1] = panic_pswd1;
  nvs_write(PANIC_PSWD1, panic_pswd1);
  panic_pswd[2] = panic_pswd2;
  nvs_write(PANIC_PSWD2, panic_pswd2);
  print_panic_pswd();
}

void print_panic_pswd(void)
{
  SerialBT.printf("Pin%d%d%d|", panic_pswd[0], panic_pswd[1], panic_pswd[2]);
  Serial.printf("Pin%d%d%d|", panic_pswd[0], panic_pswd[1], panic_pswd[2]);
}

void set_output_mode(bool new_val)
{
  nvs_write(OUTPUT_MODE, new_val);
  output_mode = new_val;
  print_output_mode();
}

void print_output_mode(void)
{
  SerialBT.printf("MSalida:%d|", output_mode);
  Serial.printf("MSalida:%d|", output_mode);
}

void set_output_delay(uint16_t new_val)
{
  nvs_write_uint16(OUTPUT_DELAY, new_val);
  output_delay = new_val * 1000;
  print_output_delay();
}

void print_output_delay(void)
{
  SerialBT.printf("TimerSalida:%d|", output_delay);
  Serial.printf("TimerSalida:%d|", output_delay);
}

void set_output_config(bool selector, uint32_t off_delay, uint32_t on_delay)
{
  on_off_selector = selector;
  nvs_write(ON_OFF_SELECT, selector);
  output_status = !on_off_selector;
  output_off_delay = off_delay * 1000;
  if (off_delay >= 9999) {
    output_off_delay = UINT32_MAX;
  }
  nvs_write_uint16(OUT_OFF_DELAY, off_delay);
  output_on_delay = on_delay * 1000;
  if (on_delay >= 9999) {
    output_on_delay = UINT32_MAX;
  }
  nvs_write_uint16(OUT_ON_DELAY, on_delay);
  print_output_config();
}

void print_output_config(void)
{
  if (on_off_selector) {
    SerialBT.printf("ONtime:%d|OFFtime:%d|", output_on_delay / 1000, output_off_delay / 1000);
    Serial.printf("ONtime:%d|OFFtime:%d|", output_on_delay / 1000, output_off_delay / 1000);
  } else {
    SerialBT.printf("OFFtime:%d|ONtime:%d|", output_off_delay / 1000, output_on_delay / 1000);
    Serial.printf("OFFtime:%d|ONtime:%d|", output_off_delay / 1000, output_on_delay / 1000);
  }
}

void set_enable_prints(uint8_t new_val)
{
  nvs_write(PRINTS_EN, new_val);
  enable_print_in = new_val & 1;
  enable_print_geos = new_val >> 1 & 1;
  print_enable_prints();
}

void print_enable_prints(void)
{
  uint8_t enable_prints = nvs_read(PRINTS_EN);
  SerialBT.printf("ImpresionX:%d|", enable_prints);
  Serial.printf("ImpresionX:%d|", enable_prints);
}

bool get_enable_print_geos(void)
{
  return enable_print_geos;
}

void set_config_default(void)
{
  nvs_write(LBOX_CONFIG, CLEAR);
}
