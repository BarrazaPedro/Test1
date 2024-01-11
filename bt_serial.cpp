#include "bt_serial.h"
#include <esp_bt_device.h>
#include "mac_manager.h"

#define SLAVE   0
#define MASTER  1
#define SCAN_TIME 5000
#define COD_WF 4326144//Webfleet
#define COD_TK 6357532//Teltonika
//#define DIAGNOSTIC

bool is_bt_connected;
int reconnecting_tries;
uint8_t valid_server_address[6];
bool is_server_address_registered;
bool is_connection_busy;

void check_connection_mst(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client connected");
  }
  else if (event == ESP_SPP_CLOSE_EVT) {
    is_bt_connected = false;
  }
  else if (event == ESP_SPP_DISCOVERY_COMP_EVT) {
    if (reconnecting_tries == 0) {
      ESP.restart();
    }
    reconnecting_tries--;
  }
}

void check_connection_slv(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT && !is_bt_connected  && !is_connection_busy) {
    is_connection_busy = true;
    if (param->srv_open.status == ESP_SPP_SUCCESS) {
      char server_address[18];
      sprintf(server_address, "%02X:%02X:%02X:%02X:%02X:%02X",
              param->srv_open.rem_bda[0], param->srv_open.rem_bda[1], param->srv_open.rem_bda[2],
              param->srv_open.rem_bda[3], param->srv_open.rem_bda[4], param->srv_open.rem_bda[5]);
      Serial.println("A device is trying to connect");
      Serial.print("Device address: ");
      Serial.println(server_address);
      char valid_server_addr_str[18];
      mac_uint2cstr(valid_server_address, valid_server_addr_str);
      if (strcmp(server_address, valid_server_addr_str) == 0 || !is_server_address_registered) {
        is_bt_connected = true;
        SerialBT.println("Server connected");
        Serial.println("Server connected");
      } else {
        SerialBT.println("Device not allowed");
        Serial.println("Device not allowed");
        SerialBT.disconnect();
      }
    }
    is_connection_busy = false;
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    bool was_server_disconnected = param->close.async;
    if (was_server_disconnected) {
      is_bt_connected = false;
      Serial.println("Server disconnected");
    } else {
      Serial.println("Invader disconnected");
    }
  }
}

BTSerial::BTSerial(BluetoothSerial& serial_bt) : _serial_bt(serial_bt)
{
  is_bt_connected = false;
  reconnecting_tries = 1000;
}

void BTSerial::get_local_addr(uint8_t* local_mac)
{
  _serial_bt.begin("ESP32", MASTER);
  const uint8_t* const_local_mac = esp_bt_dev_get_address();
  memcpy(local_mac, const_local_mac, 6);
}

void BTSerial::begin_mst(String device_name)
{
  _serial_bt.begin(device_name, MASTER);
  _serial_bt.register_callback(check_connection_mst);
}

void BTSerial::get_pin(const uint8_t* mac_uint8, char* pin)
{
  sprintf(pin, "%02X%02X", mac_uint8[4], mac_uint8[5]);
}

void BTSerial::set_pin(const char *pin)
{
  _serial_bt.setPin(pin);
}

bool BTSerial::connect_address(uint8_t* client_address)
{
  Serial.print("Connecting to previous address: ");
  print_mac(client_address);
  is_bt_connected = _serial_bt.connect(client_address);
  is_bt_connected = _serial_bt.connected(1000);
  if (is_bt_connected) {
    memcpy(_connected_address, client_address, sizeof(_connected_address));
    Serial.println("Client connected succesfully!");
  }
  return is_bt_connected;
}

bool BTSerial::scan_and_connect(int connecting_tries)
{
  Serial.print("Scanning bluetooth... ");
  BTScanResults *devices_bt = _serial_bt.discover(SCAN_TIME * (1 + connecting_tries));
  int num_devices = devices_bt->getCount();
  Serial.print(num_devices);
  Serial.print(" bluetooth devices detected, ");
  bool is_device_compatible = false;
  int compatible_devices_num = 0;
  BTAdvertisedDevice *device2connect;
  uint8_t mac_addr[6];
  BTAdvertisedDevice *devices[num_devices];
  for (int i = 0; i < num_devices; i++)
  {
    devices[i] = devices_bt->getDevice(i);
#ifdef DIAGNOSTIC
    Serial.println();
    Serial.print("Device ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(devices[i]->toString().c_str());
    Serial.println(devices[i]->getCOD());//full cod
#endif
    //    if (devices[i]->getCOD() == COD_WF || devices[i]->getCOD() == COD_TK) {
    if (devices[i]->getCOD() == COD_WF) {
      compatible_devices_num++;
      if (is_device_compatible == false) {
        device2connect = devices[i];
        is_device_compatible = true;
      }
      else if (devices[i]->getRSSI() > device2connect->getRSSI()) {
        device2connect = devices[i];
      }
    }
  }
  BTAdvertisedDevice *compatible_devices[compatible_devices_num];
  uint8_t compatible_devices_counter = 0;
  for (int i = 0; i < num_devices; i++)
  {
#ifdef DIAGNOSTIC
    Serial.println("---");
    Serial.print("Device ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(devices[i]->toString().c_str());
    Serial.println(devices[i]->getCOD());
#endif
    //    if (devices[i]->getCOD() == COD_WF || devices[i]->getCOD() == COD_TK) {
    if (devices[i]->getCOD() == COD_WF) {
      compatible_devices[compatible_devices_counter] = devices[i];
      compatible_devices_counter++;
    }
  }
  for (int i = 0; i < compatible_devices_num; i++)
  {
#ifdef DIAGNOSTIC
    Serial.println("///");
    Serial.print("Device ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(compatible_devices[i]->toString().c_str());
    Serial.println(compatible_devices[i]->getCOD());
#endif
  }
  Serial.printf("%d compatible\n", compatible_devices_num);
  if (is_device_compatible) {
#ifdef DIAGNOSTIC
    Serial.println(device2connect->getName().c_str());
    Serial.println(device2connect->getAddress().toString().c_str());
    Serial.println(device2connect->getCOD());
    Serial.println(device2connect->getRSSI());
#endif
    for (int i = 0; i < compatible_devices_num; i++)
    {
      String mac_string = compatible_devices[i]->getAddress().toString().c_str();
      const char* client_name = compatible_devices[i]->getName().c_str();
      mac_str2uint(mac_string, mac_addr);
      Serial.printf("Connecting to device: %s, ", client_name);
      print_mac(mac_addr);
      is_bt_connected = _serial_bt.connect(mac_addr);
      is_bt_connected = _serial_bt.connected(1000);
      if (is_bt_connected) {
        compatible_devices[i]->getCOD() == COD_WF ? Serial.print("Webfleet ") : Serial.print("Teltonika ");
        Serial.print("client connected succesfully,");
        print_mac(mac_addr);
        _serial_bt.println("Client connected succesfully!");
        memcpy(_connected_address, mac_addr, sizeof(mac_addr));
        _connected_client_name = String(client_name);
        break;
      }
    }
  }
  if (!is_bt_connected) {
    ESP.restart();
  }
  return is_bt_connected;
}

bool BTSerial::iterative_scan_and_connect(int connecting_tries)
{
  for (int num_try = 0; num_try < connecting_tries; num_try++)
  {
    if (scan_and_connect(num_try)) {
      is_bt_connected = true;
      break;
    }
  }
  if (!is_bt_connected) {
    ESP.restart();
  }
  return is_bt_connected;
}

bool BTSerial::reconnecting(int tries)
{
  if (!is_bt_connected) {
    Serial.println("Trying to reconnect client...");
    is_bt_connected = _serial_bt.connect(_connected_address);
    is_bt_connected = _serial_bt.connected(1000);
    if (is_bt_connected) {
      Serial.print(_connected_client_name);
      Serial.print(" connected succesfully, has address: ");
      print_mac(_connected_address);
      _serial_bt.println("Client connected succesfully!");
    }
  } else {
    reconnecting_tries = tries;
  }
  return is_bt_connected;
}

void BTSerial::get_client_addr(uint8_t* client_mac)
{
  memcpy(client_mac, _connected_address, 6);
}

void BTSerial::begin_slv(String device_name)
{
  _serial_bt.begin(device_name, SLAVE);
  _serial_bt.register_callback(check_connection_slv);
  is_server_address_registered = false;
  is_connection_busy = false;
}

void BTSerial::register_server(uint8_t* server_address)
{
  is_server_address_registered = true;
  memcpy(valid_server_address, server_address, sizeof(valid_server_address));
  Serial.print("Server registered succesfully, address: ");
  print_mac(valid_server_address);
}

void BTSerial::remove_server(void)
{
  is_server_address_registered = false;
  uint8_t blank_mac[6] = {0, 0, 0, 0, 0, 0};
  memcpy(valid_server_address, blank_mac, sizeof(valid_server_address));
  Serial.println("Server removed");
}

String BTSerial::receive_data(void)
{
  String received_data = "";
  while (_serial_bt.available())
  {
    received_data = SerialBT.readStringUntil('\n');
  }
  return received_data;
}

void BTSerial::print_data(String data)
{
  _serial_bt.print(data);
}
