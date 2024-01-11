#ifndef custom_gps_h
#define custom_gps_h

#include <TinyGPS++.h>

class custom_gps {
  public:
    custom_gps();
    void begin(int baudrate, int pin_rx, int pin_tx);
    void suspend(void);
    void resume(void);
    bool is_valid(void);
    bool update(void);
    void get_location(double &latitude, double &longitude);
    void print_location(Stream &outputStream);
    void print_nmea(Stream &outputStream);
    void print_info(Stream &outputStream);
    void print_speed(Stream &outputStream);
    void print_time(Stream &outputStream);

  private:
    TinyGPSPlus     _gps;
    HardwareSerial *_gps_serial;
    int64_t         _previous_time;
    bool            _save_gps_data;
    String          _nmea_str;
};

#endif
