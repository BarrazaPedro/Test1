#ifndef bt_serial_h
#define bt_serial_h

#include <BluetoothSerial.h>

extern BluetoothSerial SerialBT;

void check_connection_mst(esp_spp_cb_event_t event, esp_spp_cb_param_t* param);
void check_connection_slv(esp_spp_cb_event_t event, esp_spp_cb_param_t* param);

class BTSerial {
  public:
    BTSerial(BluetoothSerial& serial_bt);
    void    get_local_addr(uint8_t* local_mac);
    void    begin_mst(String device_name);
    void    get_pin(const uint8_t* mac_uint8, char* pin);
    void    set_pin(const char *pin);
    bool    connect_address(uint8_t* client_address);
    bool    scan_and_connect(int connecting_tries);
    bool    iterative_scan_and_connect(int connecting_tries);
    bool    reconnecting(int tries);
    void    get_client_addr(uint8_t* client_mac);
    void    begin_slv(String device_name);
    void    register_server(uint8_t* server_address);
    void    remove_server(void);
    String  receive_data(void);
    void    print_data(String data);

  private:
    BluetoothSerial& _serial_bt;
    uint8_t _connected_address[6];
    String _connected_client_name;
};

#endif
