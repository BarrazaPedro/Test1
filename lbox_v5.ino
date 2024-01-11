#include "bt_serial.h"
#include "old_commands.h"
#include "control_lbox.h"
#include "nvs.h"
#include <esp_task_wdt.h>

#define WDT_TIME  70
#define NUM_TRIES 3
#define BAUDRATE  9600
#define GPS_RX    19
#define GPS_TX    23

BluetoothSerial   SerialBT;
BTSerial          serial_bt(SerialBT);
old_commands      old_commands;
geofence_manager  geofence;
custom_gps        gps_module;
geofence_type*    geofence_container;

TaskHandle_t task_bt_handle;
TaskHandle_t task_led_handle;
TaskHandle_t task_io_handle;
TaskHandle_t task_gps_handle;

char      firmware_ver[7] = "V5.2.0";
bool      was_i2c_mem_modified = 0;
uint8_t   local_mac[6];
uint8_t   client_mac[6];
bool      it_was_bt_connected_last_time;
char      lbox_name[13];
char      bt_pin[5];
bool      bt_state = 0;
uint64_t  previous_geostatus_time = 0;
uint64_t  previous_status_time = 0;
uint16_t  geostatus_delay = 1000;
uint8_t   geo_status = 0;
uint8_t   prev_geostatus_val = 0;
bool      are_inputs_updated;
bool      gps_has_updated = 0;
uint8_t   lbox_state = 0;
uint8_t   prev_lbox_state = 0;

void setup()
{
  i2c_mem_init();
  nvs_init();
  esp_task_wdt_init(WDT_TIME, true);
  esp_task_wdt_add(NULL);
  geofence_container = geofence.create_container();
  gps_module.begin(BAUDRATE, GPS_RX, GPS_TX);
  Serial.begin(BAUDRATE);
  delay(100);
  if (nvs_read(LBOX_CONFIG) != 0) {
    serial_bt.get_local_addr(local_mac);
    nvs_write(LOCAL_MAC_B0, local_mac[0]);
    nvs_write(LOCAL_MAC_B1, local_mac[1]);
    nvs_write(LOCAL_MAC_B2, local_mac[2]);
    nvs_write(LOCAL_MAC_B3, local_mac[3]);
    nvs_write(LOCAL_MAC_B4, local_mac[4]);
    nvs_write(LOCAL_MAC_B5, local_mac[5]);
  }
  local_mac[0] = nvs_read(LOCAL_MAC_B0);
  local_mac[1] = nvs_read(LOCAL_MAC_B1);
  local_mac[2] = nvs_read(LOCAL_MAC_B2);
  local_mac[3] = nvs_read(LOCAL_MAC_B3);
  local_mac[4] = nvs_read(LOCAL_MAC_B4);
  local_mac[5] = nvs_read(LOCAL_MAC_B5);
  serial_bt.get_pin(local_mac, bt_pin);
  lbox_control_begin();
  if (return_serial_enabled()) {
    Serial.begin(BAUDRATE);
  } else {
    Serial.end();
  }
  if (return_gps_disabled()) {
    gps_module.suspend();
  } else {
    gps_module.resume();
  }
  sprintf(lbox_name, "LBOXV5%02X%02X%02X", local_mac[3], local_mac[4], local_mac[5]);
  Serial.println();
  Serial.print("btName:");
  Serial.println(lbox_name);
  serial_bt.begin_mst(lbox_name);
  Serial.print("btPin:");
  Serial.println(bt_pin);
  serial_bt.set_pin(bt_pin);
  xTaskCreate(blink_led_task, "blink_led", 4096, NULL, 1, &task_led_handle);
  xTaskCreate(task_gps, "gps", 4096, NULL, 1, &task_gps_handle);
  Serial.printf("version del firmware:%s\n", firmware_ver);
  update_container();
  Serial.printf("numero de geocercas:%d\n", geofence.container_count());
  it_was_bt_connected_last_time = nvs_read(BEF_CONNECTED);
  if (it_was_bt_connected_last_time) {
    client_mac[0] = nvs_read(CLIENT_MAC_B0);
    client_mac[1] = nvs_read(CLIENT_MAC_B1);
    client_mac[2] = nvs_read(CLIENT_MAC_B2);
    client_mac[3] = nvs_read(CLIENT_MAC_B3);
    client_mac[4] = nvs_read(CLIENT_MAC_B4);
    client_mac[5] = nvs_read(CLIENT_MAC_B5);
    bt_state = serial_bt.connect_address(client_mac);
  }
  if (!bt_state) {
    nvs_write(BEF_CONNECTED, 0);
    if (it_was_bt_connected_last_time) {
      ESP.restart();
    }
    bt_state = serial_bt.iterative_scan_and_connect(NUM_TRIES);
  }
  if (bt_state) {
    nvs_write(BEF_CONNECTED, 1);
    serial_bt.get_client_addr(client_mac);
    nvs_write(CLIENT_MAC_B0, client_mac[0]);
    nvs_write(CLIENT_MAC_B1, client_mac[1]);
    nvs_write(CLIENT_MAC_B2, client_mac[2]);
    nvs_write(CLIENT_MAC_B3, client_mac[3]);
    nvs_write(CLIENT_MAC_B4, client_mac[4]);
    nvs_write(CLIENT_MAC_B5, client_mac[5]);
  }
  vTaskDelete(task_led_handle);
  vTaskDelete(task_gps_handle);
  xTaskCreate(task_bt, "bt", 4096, NULL, 1, &task_bt_handle);
  xTaskCreate(task_io, "io", 4096, NULL, 1, &task_io_handle);
}

void loop()
{
  if (bt_state) {
    blue_led_on();
  } else {
    blink_led();
  }
  gps_has_updated = gps_module.update();
  if (gps_module.is_valid()) {
    red_led_on();
  } else {
    red_led_off();
  }
  int64_t current_time = millis();
  if (current_time - previous_geostatus_time >= geostatus_delay) {
    double gps_latitude;
    double gps_longitude;
    gps_module.get_location(gps_latitude, gps_longitude);
    geo_status = geofence.get_geostatus(return_geos_enabled(), gps_latitude, gps_longitude);
    if (geo_status != prev_geostatus_val && get_enable_print_geos())
    {
      gps_module.print_location(SerialBT);
      gps_module.print_location(Serial);
      double nearest_dist = geofence.return_nearest_distance();
      uint16_t nearest_geo = geofence.return_nearest_geofence();
      SerialBT.printf("*GeoS|%d|%0.2f|%d|*", geo_status, nearest_dist, nearest_geo);
      Serial.printf("*GeoS|%d|%0.2f|%d|*", geo_status, nearest_dist, nearest_geo);
    }
    previous_geostatus_time = current_time;
    prev_geostatus_val = geo_status;
  }
  if (are_inputs_updated) {
    get_inputs_status();
    lbox_state = get_lbox_status(geo_status);
    if (prev_lbox_state != lbox_state) {
      gps_module.print_location(SerialBT);
      gps_module.print_location(Serial);
      double nearest_dist = geofence.return_nearest_distance();
      uint16_t nearest_geo = geofence.return_nearest_geofence();
      SerialBT.printf("*GeoS|%d|%0.2f|%d|*\n", geo_status, nearest_dist, nearest_geo);
      Serial.printf("*GeoS|%d|%0.2f|%d|*\n", geo_status, nearest_dist, nearest_geo);
    }
    prev_lbox_state = lbox_state;
  }
  String received_command = serial_bt.receive_data();
  if (received_command != "") {
    received_command.trim();
    int start_index = 0;
    while (start_index < received_command.length()) {
      start_index = old_commands.process_commands(received_command, start_index);
      esp_task_wdt_reset();
    }
    if (was_i2c_mem_modified) {
      update_container();
      was_i2c_mem_modified = 0;
    }
  }
  delay(5);
  esp_task_wdt_reset();
}

void task_gps(void *pvParameters)
{
  while (1) {
    gps_has_updated = gps_module.update();
    if (gps_module.is_valid()) {
      red_led_on();
    } else {
      red_led_off();
    }
    delay(5);
    esp_task_wdt_reset();
  }
}

void task_bt(void *pvParameters)
{
  while (1) {
    bt_state = serial_bt.reconnecting(NUM_TRIES);
    delay(5);
    esp_task_wdt_reset();
  }
}

void update_container(void)
{
  uint8_t buf_data[32];
  for (int i = 0; i < I2C_MEM_SIZE; i += sizeof(buf_data))
  {
    i2c_mem_seq_read(i, buf_data, sizeof(buf_data));
    geofence.fill_container(i / GEOFENCE_SIZE, buf_data, sizeof(buf_data));
  }
}
