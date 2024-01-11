#ifndef GEOFENCE_MANAGER_H
#define GEOFENCE_MANAGER_H

#include <Arduino.h>

#define CLEAR           0xFF
#define CONTAINER_SIZE  2048//max number of geofences
#define GEOFENCE_SIZE   16  //bytes
#define KB              1024//bits

struct geofence_type {
  uint16_t  addr;
  uint8_t   type = CLEAR;
  uint32_t  radius;
  double    latitude;
  double    longitude;
};

class geofence_manager {
  public:
    geofence_manager();
    geofence_type* create_container(void);
    void      delete_container(void);
    void      fill_container(uint16_t offset, const uint8_t* data, uint16_t data_size);
    double    convert_uint32_to_double(uint32_t uint_value);
    uint32_t  convert_double_to_uint32(double double_value);
    void      convert_uint32_to_uint8(uint32_t uint32_val, uint8_t* uint8_val, uint16_t offset);
    uint16_t  container_count(void);
    uint16_t  seek_geofence_empty_pos(void);
    uint16_t  seek_geofence_mem_pos(double lat, double lng);
    void      print_geofence(uint16_t position);
    void      return_geofence(uint16_t position, geofence_type* geofence2return);
    void      print_all_geofences(void);
    double    calculate_distance(double lat1, double lng1, double lat2, double lng2);
    uint16_t  compare_distance(double lat, double lng);
    double    return_nearest_distance(void);
    uint16_t  return_nearest_geofence(void);
    uint8_t   get_geostatus(bool enabler, double lat, double lng);
    uint8_t   return_geostatus(void);
    void      check_heap_size(void);
    uint8_t*  create_raw_array(void);
    void      delete_raw_array(uint8_t* &raw_array);
    void      fill_raw_array(uint32_t uint32_val);
    void      clear_raw_array(void);
    void      copy_raw_array(uint8_t* array_copy);
    void      read_raw_array(uint16_t addr2read);
    uint16_t  seek_raw_array_empty_pos(void);

  private:
    geofence_type*  geofence_container;
    uint8_t*        geofence_raw_array;
};

#endif
