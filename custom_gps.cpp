#include "custom_gps.h"

custom_gps::custom_gps() {
}

void custom_gps::begin(int baudrate, int pin_rx, int pin_tx)
{
  _gps_serial = new HardwareSerial(1);
  _gps_serial->begin(baudrate, SERIAL_8N1, pin_rx, pin_tx);
  _previous_time = INT64_MAX;
  _save_gps_data = true;
  _nmea_str;
}

void custom_gps::suspend(void)
{
  _save_gps_data = false;
}

void custom_gps::resume(void)
{
  _save_gps_data = true;
}

bool custom_gps::is_valid(void)
{
  return _gps.location.age() < 1500;
}

bool custom_gps::update(void)
{
  bool gps_has_updated = false;
  _nmea_str = "";
  while (_gps_serial->available())
  {
    if (_save_gps_data) {
      char nmea_char = _gps_serial->read();
      _nmea_str += nmea_char;
      _gps.encode(nmea_char);
    } else {
      _gps_serial->read();
    }
  }
  if (_previous_time != _gps.time.second()) {
    _previous_time = _gps.time.second();
    gps_has_updated = true;
  }
  return gps_has_updated;
}

void custom_gps::get_location(double &latitude, double &longitude)
{
  if (_gps.location.isValid()) {
    latitude = _gps.location.lat();
    longitude = _gps.location.lng();
  } else {
    latitude = 0.0;
    longitude = 0.0;
  }
}

void custom_gps::print_location(Stream &outputStream)
{
  if (_gps.location.isValid()) {
    outputStream.printf("Latitude:%0.6f|Longitude:%0.6f|", _gps.location.lat(), _gps.location.lng());
  } else {
    outputStream.printf("Latitude:%0.6f|Longitude:%0.6f|", 0.0, 0.0);
  }
}

void custom_gps::print_nmea(Stream &outputStream)
{
  int64_t init_time = millis();
  while (_nmea_str == "") {
    update();
    if (millis() >= init_time + 1000) {
      outputStream.print("No GPS signal detected, check wiring or config");
      break;
    }
  }
  outputStream.println(_nmea_str);
}

void custom_gps::print_info(Stream &outputStream)
{
  outputStream.println(F("*GPS Info*"));
  outputStream.printf("Sats:%d|HDOP:%0.2f|\n", _gps.satellites.value(), _gps.hdop.hdop());
  outputStream.printf("Latitude:%0.6f|Longitude:%0.6f|\n", _gps.location.lat(), _gps.location.lng());
  outputStream.printf("Speed:%0.2f|Course:%0.2f|\n", _gps.speed.kmph(), _gps.course.deg());
  outputStream.printf("CharP:%d|FailCh:%d|\n", _gps.charsProcessed(), _gps.failedChecksum());
  outputStream.printf("Fix Age:%d|\n", _gps.location.age());
}

void custom_gps::print_speed(Stream &outputStream)
{
  if (_gps.speed.isValid()) {
    outputStream.printf("Speed:%0.2fKm/h|", _gps.speed.kmph());
  } else {
    outputStream.printf("Speed:Invalid|");
  }
}

void custom_gps::print_time(Stream &outputStream)
{
  if (_gps.date.isValid()) {
    outputStream.printf("Date:%02d/%02d/%04d|", _gps.date.day(), _gps.date.month(), _gps.date.year());
  } else {
    outputStream.printf("Date:Invalid|");
  }
  if (_gps.time.isValid()) {
    outputStream.printf("Time:%02d:%02d:%02d|", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
  } else {
    outputStream.printf("Time:Invalid|");
  }
}
